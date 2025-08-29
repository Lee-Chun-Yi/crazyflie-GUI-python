from __future__ import annotations
import socket, struct, logging, time, math
from time import perf_counter_ns
from threading import Thread, Event, Lock
from typing import Optional, List, Callable
from queue import Queue
from collections import deque

from cflib.crtp.crtpstack import CRTPPacket

from .models import SharedState, set_last_stream_xyz, get_last_stream_xyz as _get_last_stream_xyz
from . import models
from .utils import set_realtime_priority
from .realtime import Realtime
from .config import RT, Safety, Landing, Rates, PreviewCfg
from .link import LinkManager

logger = logging.getLogger(__name__)


def get_last_stream_xyz():
    """Expose last streamed XYZ for read-only access."""
    return _get_last_stream_xyz()

# Global loop references to ensure only one active at a time
_setpoint_loop: Optional['SetpointLoop'] = None
_pwm_loop: Optional['PWMSetpointLoop'] = None
_pwm_udp: Optional['PWMUDPReceiver'] = None
_path_loop: Optional['FlightPathLoop'] = None
# Track UDPInput instances using port 8888
_udp_inputs: List['UDPInput'] = []

# --- 4-PWM CRTP configuration -------------------------------------------
PORT_PWM = 0x0A
CHAN_PWM = 0
ENABLE_PARAM_CANDIDATES = ["crtp_pwm.enable", "pwm.enable", "motorPowerSet.enable"]

# Runtime state for the new 4-PWM loop
_pwm_thread: Optional[Thread] = None
_pwm_stop_evt = Event()
_pwm_enable_param_name = ""


def clamp_u16(v: int) -> int:
    return 0 if v < 0 else (65535 if v > 65535 else int(v))


def send_packet_compat(cf, pkt) -> None:
    """Best-effort send_packet with multiple fallbacks."""
    for attr in ("link", "cf.link", "_link"):
        obj = cf
        try:
            for part in attr.split('.'):
                obj = getattr(obj, part)
            obj.send_packet(pkt)
            return
        except Exception:
            continue
    raise RuntimeError("No send_packet available on CF object")


def send_4pwm_packet(cf, m1, m2, m3, m4) -> None:
    pkt = CRTPPacket()
    pkt.port = PORT_PWM
    pkt.channel = CHAN_PWM
    pkt.data = struct.pack("<HHHH", m1, m2, m3, m4)
    send_packet_compat(cf, pkt)


def _parse_bool(val: object) -> bool:
    s = str(val).strip().lower()
    if s in ("1", "true", "t", "yes", "on"):
        return True
    if s in ("0", "false", "f", "no", "off"):
        return False
    try:
        return bool(int(s))
    except Exception:
        return False


def try_set_enable_param(cf, state: int) -> str:
    target = "1" if state else "0"
    expected = bool(state)
    for name in ENABLE_PARAM_CANDIDATES:
        try:
            cf.param.set_value(name, target)
            time.sleep(0.05)
            for _ in range(3):
                try:
                    val = cf.param.get_value(name)
                    if _parse_bool(val) == expected:
                        return name
                except Exception:
                    pass
                time.sleep(0.05)
            logging.warning("PWM enable param %s could not be verified", name)
            return ""
        except Exception:
            continue
    logging.warning("No PWM enable param could be set")
    return ""


def start_4pwm_loop_async(
    state: SharedState,
    link_mgr: LinkManager,
    rate_hz: float,
    pwm_mode: str = "manual",
    manual_pwm: List[int] | None = None,
    udp_port: int = 8888,
    on_success: Optional[Callable[[], None]] = None,
    on_error: Optional[Callable[[str], None]] = None,
) -> None:
    """Kick off the 4×PWM start sequence in a background thread and return immediately."""

    def _starter() -> None:
        global _pwm_thread, _pwm_stop_evt, _pwm_udp, _pwm_enable_param_name
        try:
            if _pwm_thread and _pwm_thread.is_alive():
                if on_success:
                    on_success()
                return
            if not link_mgr:
                raise RuntimeError("No link manager")
            # quick connection attempts
            for _ in range(3):
                if link_mgr.ensure_connected():
                    break
                time.sleep(0.1)
            else:
                raise RuntimeError("No link")
            cf = link_mgr.get_cf()
            if cf is None:
                raise RuntimeError("No Crazyflie handle")
            for _ in range(3):
                if link_mgr.send_arming_request(True):
                    break
                time.sleep(0.05)
            models.clear_stop_flags(state)
            _pwm_enable_param_name = try_set_enable_param(cf, 1)
            _pwm_stop_evt.clear()
            rate = float(rate_hz) if rate_hz else float(Rates.SETPOINT_HZ)
            interval = 1.0 / rate if rate > 0 else 0.01
            if pwm_mode == "udp":
                if _pwm_udp is None:
                    _pwm_udp = PWMUDPReceiver(port=udp_port)
                _pwm_udp.start()
                models.set_accept_udp_8888(True)
                try:
                    with state.lock:
                        state.using_udp_8888 = True
                except Exception:
                    pass
            else:
                vals = [clamp_u16(v) for v in (manual_pwm or [0, 0, 0, 0])[:4]]
                models.set_desired_pwm_tuple(*vals)

            def _worker():
                last_t = time.perf_counter()
                next_t = time.monotonic()
                models.set_pwm_running(True)
                while (
                    not _pwm_stop_evt.is_set()
                    and not state.stop_all.is_set()
                    and not state.stop_flight.is_set()
                ):
                    if pwm_mode == "udp" and _pwm_udp:
                        vals = _pwm_udp.get_last()
                    else:
                        vals = models.get_desired_pwm_tuple()
                    m1, m2, m3, m4 = [clamp_u16(v) for v in vals[:4]]
                    try:
                        send_4pwm_packet(cf, m1, m2, m3, m4)
                    except Exception:
                        logger.exception("send_4pwm_packet failed")
                        break
                    models.set_last_pwm((m1, m2, m3, m4))
                    now = time.perf_counter()
                    dt = now - last_t
                    if dt > 0:
                        models.set_pwm_actual_rate(1.0 / dt)
                    last_t = now
                    next_t += interval
                    sleep = next_t - time.monotonic()
                    if sleep > 0:
                        time.sleep(sleep)
                models.set_pwm_running(False)

            _pwm_thread = Thread(target=_worker, daemon=True)
            _pwm_thread.start()
            with state.lock:
                state.active_mode = "pwm"
                state.is_flying.set()
            if on_success:
                on_success()
        except Exception as e:
            if on_error:
                on_error(str(e))

    Thread(target=_starter, daemon=True).start()


def preview_points_none(x: float, y: float, z: float):
    return [(float(x), float(y), float(z))]


def preview_points_circle(
    cx: float,
    cy: float,
    z0: float,
    radius: float,
    clockwise: bool,
    hold_z: bool,
    z_amp: float = 0.0,
    z_period: float = 1.0,
    n: int = PreviewCfg.CIRCLE_SAMPLES,
):
    pts = []
    n = max(1, int(n))
    for i in range(n + 1):
        frac = i / n
        ang = 2 * math.pi * frac
        if clockwise:
            ang = -ang
        x = cx + radius * math.cos(ang)
        y = cy + radius * math.sin(ang)
        if hold_z or z_amp <= 0.0 or z_period <= 0.0:
            z = z0
        else:
            z = z0 + z_amp * math.sin(2 * math.pi * frac)
        pts.append((x, y, z))
    return pts


def preview_points_square(
    cx: float,
    cy: float,
    z0: float,
    side: float,
    clockwise: bool,
    hold_z: bool,
    z_amp: float = 0.0,
    z_period: float = 1.0,
    n_edge: int = PreviewCfg.SQUARE_EDGE_SAMPLES,
):
    n_edge = max(1, int(n_edge))
    half = side / 2.0
    if clockwise:
        verts = [
            (cx + half, cy + half),
            (cx + half, cy - half),
            (cx - half, cy - half),
            (cx - half, cy + half),
        ]
    else:
        verts = [
            (cx + half, cy + half),
            (cx - half, cy + half),
            (cx - half, cy - half),
            (cx + half, cy - half),
        ]
    pts = []
    total = 4 * n_edge
    for i in range(4):
        sx, sy = verts[i]
        ex, ey = verts[(i + 1) % 4]
        for j in range(n_edge):
            frac = (i * n_edge + j) / total
            ratio = j / n_edge
            x = sx + (ex - sx) * ratio
            y = sy + (ey - sy) * ratio
            if hold_z or z_amp <= 0.0 or z_period <= 0.0:
                z = z0
            else:
                z = z0 + z_amp * math.sin(2 * math.pi * frac)
            pts.append((x, y, z))
    # close loop
    pts.append(pts[0])
    return pts


class UDPInput:
    """Non-blocking UDP listener for RPYT with adjustable rate.
    Packet: little-endian 4*float32 => (roll, pitch, yawrate, throttle_raw)
    """
    def __init__(self, state: SharedState, port: int = 8888, rate_hz: int = 100):
        self.state = state
        self.port = port
        self._rate_hz = max(1, int(rate_hz))
        self._rate_lock = Lock()
        self._running = Event()
        self._thread: Optional[Thread] = None

    # --- public API ---
    def start(self):
        if self._thread:
            if self._thread.is_alive():
                return
            # cleanup stale thread reference
            self._thread = None
        self._running.set()
        self._thread = Thread(target=self._run, daemon=True)
        self._thread.start()
        set_realtime_priority(self._thread.ident)  # reduce jitter; may require admin rights
        # register active listener
        if self not in _udp_inputs:
            _udp_inputs.append(self)
        try:
            with self.state.lock:
                self.state.using_udp_8888 = True
        except Exception:
            pass

    def stop(self):
        self._running.clear()
        if self._thread:
            self._thread.join(timeout=1.0)
            self._thread = None
        if self in _udp_inputs:
            _udp_inputs.remove(self)
        try:
            with self.state.lock:
                self.state.using_udp_8888 = False
        except Exception:
            pass

    def set_rate(self, hz: int):
        with self._rate_lock:
            self._rate_hz = max(1, int(hz))

    def get_rate(self) -> int:
        with self._rate_lock:
            return self._rate_hz

    # --- worker ---
    def _run(self):
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.bind(("127.0.0.1", self.port))
        sock.setblocking(False)
        wait = Event()
        t_ns = perf_counter_ns()
        # use ns resolution and manual wait loop to reduce jitter (~2ms worst case)
        while self._running.is_set() and not self.state.stop_all.is_set():
            try:
                while True:
                    data, _ = sock.recvfrom(1024)
                    if len(data) == 16:
                        r, p, y, thr_f = struct.unpack("<4f", data)
                        r = max(-5.0, min(5.0, float(r)))
                        p = max(-5.0, min(5.0, float(p)))
                        y = max(-360.0, min(360.0, float(y)))
                        th_raw = max(0, min(20000, int(thr_f)))
                        with self.state.lock:
                            offset = int(getattr(self.state, "throttle_offset", 40000))
                        th = th_raw + offset
                        th = max(0, min(65535, th))
                        if not models.get_accept_udp_8888():
                            continue
                        with self.state.lock:
                            self.state.rpyth.update({
                                "roll": r, "pitch": p, "yaw": y, "thrust": th
                            })
                        models.set_last_rpyt((r, p, y, th))
            except BlockingIOError:
                pass
            except Exception:
                # swallow and keep running
                pass
            # timing
            with self._rate_lock:
                dt_ns = int(1_000_000_000 / float(self._rate_hz))
            t_ns += dt_ns
            while True:
                remaining = t_ns - perf_counter_ns()
                if remaining <= 0:
                    t_ns = perf_counter_ns()
                    break
                wait.wait(remaining / 1e9)


class PWMUDPReceiver:
    """Background UDP listener for 4 PWM values (m1–m4) as little-endian <4H> (8 bytes).
    Atomic update: self._last is replaced ONLY when a full, valid 8-byte packet is parsed.
    """

    def __init__(self, port: int = 8888):
        self.port = port
        self._running = Event()
        self._thread: Optional[Thread] = None
        self._last: List[int] = [0, 0, 0, 0]
        self._packets_ok = 0
        self._packets_bad = 0

    # --- public API ---
    def start(self):
        if self._thread and self._thread.is_alive():
            return
        self._running.set()
        self._thread = Thread(target=self._run, daemon=True)
        self._thread.start()

    def stop(self):
        self._running.clear()

    def get_last(self) -> List[int]:
        return list(self._last)

    def get_stats(self) -> dict:
        return {"packets_ok": self._packets_ok, "packets_bad": self._packets_bad}

    # --- internals ---
    @staticmethod
    def _clamp_u16(v: int) -> int:
        return 0 if v < 0 else (65535 if v > 65535 else v)

    def _run(self):
        import socket, struct
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            sock.bind(("127.0.0.1", self.port))  # keep current behavior
        except Exception:
            sock.bind(("", self.port))
        sock.settimeout(0.2)

        while self._running.is_set():
            try:
                data, _ = sock.recvfrom(1024)
            except socket.timeout:
                continue
            except Exception:
                self._packets_bad += 1
                continue

            # Expect exactly 8 bytes: <4H> little-endian
            if len(data) != 8:
                self._packets_bad += 1
                continue

            try:
                m1, m2, m3, m4 = struct.unpack("<4H", data)
                vals = [self._clamp_u16(int(m1)),
                        self._clamp_u16(int(m2)),
                        self._clamp_u16(int(m3)),
                        self._clamp_u16(int(m4))]
            except struct.error:
                self._packets_bad += 1
                continue

            # Atomic update on successful parse only
            self._last = vals
            self._packets_ok += 1


class _SendQueue:
    """Worker thread processing queued send operations."""

    def __init__(self):
        self.queue: Queue = Queue()
        self._worker: Optional[Thread] = None

    def start(self):
        if self._worker and self._worker.is_alive():
            return
        self._worker = Thread(target=self._run, daemon=True)
        self._worker.start()

    def stop(self):
        if not self._worker:
            return
        # wait for queued messages to be sent
        self.queue.join()
        # signal the worker to exit
        self.queue.put(None)
        self._worker.join()

    def enqueue(self, func, *args, **kwargs):
        self.queue.put((func, args, kwargs))

    def _run(self):
        while True:
            item = self.queue.get()
            if item is None:
                self.queue.task_done()
                break
            func, args, kwargs = item
            try:
                func(*args, **kwargs)
            except Exception:
                pass
            finally:
                self.queue.task_done()


class BaseLoop:
    """Common loop functionality for rate management and timing stats."""

    def __init__(self, rate_hz: int = 100):
        self._rate_hz = max(1, int(rate_hz))
        self._rate_lock = Lock()
        self._run_flag = Event()
        self._thread: Optional[Thread] = None
        self._sender = _SendQueue()
        # timing stats
        self._jitter_buf = deque(maxlen=300)
        self._miss_count = 0
        self._total_count = 0
        self._timing_lock = Lock()
        self._last_send_time: Optional[float] = None
        self._stats_cache = (float("nan"), float("nan"), float("nan"))
        self._stats_time = 0.0
        # actual rate measurement
        self._count = 0
        self._t_rate = 0.0
        self._actual_rate_hz = 0.0
        # realtime helper
        self._rt = Realtime(
            cpu=RT.PIN_CPU,
            fifo_priority=RT.FIFO_PRIORITY,
            spin_ns=RT.SPIN_NS,
        )

    # --- public API ---
    def start(self, rate_hz: Optional[int] = None):
        if rate_hz is not None:
            self.set_rate(rate_hz)
        else:
            self._rt.set_rate(self.get_rate())
        if self._thread and self._thread.is_alive():
            return
        with self._timing_lock:
            self._jitter_buf.clear()
            self._miss_count = 0
            self._total_count = 0
        self._last_send_time = None
        self._sender.start()
        self._run_flag.set()
        self._thread = Thread(target=self._run, daemon=True)
        self._thread.start()

    def stop(self):
        self._run_flag.clear()
        if self._thread:
            self._thread.join()
        self._sender.stop()

    def is_running(self) -> bool:
        return self._run_flag.is_set()

    def set_rate(self, hz: int):
        with self._rate_lock:
            self._rate_hz = max(1, int(hz))
            self._rt.set_rate(self._rate_hz)

    def get_rate(self) -> int:
        with self._rate_lock:
            return self._rate_hz

    def get_actual_rate(self) -> float:
        return float(self._actual_rate_hz)

    def get_timing_snapshot(self):
        with self._timing_lock:
            return list(self._jitter_buf), int(self._miss_count), int(self._total_count)

    def get_cached_stats(self, now: float):
        with self._timing_lock:
            if now - self._stats_time >= 1.0:
                j_ms = [v * 1000.0 for v in self._jitter_buf]
                n = len(j_ms)
                if n >= 5:
                    vals = sorted(j_ms)
                    p95 = vals[int(0.95 * (n - 1))]
                    p99 = vals[int(0.99 * (n - 1))]
                else:
                    p95 = float("nan")
                    p99 = float("nan")
                miss_pct = 100.0 * self._miss_count / self._total_count if self._total_count > 0 else float("nan")
                self._stats_cache = (p95, p99, miss_pct)
                self._stats_time = now
            return self._stats_cache

    # --- hooks for subclasses ---
    def _should_run(self) -> bool:
        return self._run_flag.is_set()

    def _step(self):
        raise NotImplementedError

    # --- worker ---
    def _run(self):
        self._rt.start()
        try:
            t = self._rt.now()
            with self._rate_lock:
                dt = self._rt._dt
            t_target = t + dt
            self._t_rate = t
            while self._should_run():
                self._step()
                now = self._rt.now()
                dt_real = now - self._last_send_time if self._last_send_time is not None else dt
                jitter_abs = abs(dt_real - dt)
                lateness = max(0.0, now - t_target)
                miss = lateness > 0.5 * dt
                with self._timing_lock:
                    self._jitter_buf.append(jitter_abs)
                    self._total_count += 1
                    if miss:
                        self._miss_count += 1
                self._last_send_time = now
                self._count += 1
                elapsed = now - self._t_rate
                if elapsed >= 1.0:
                    self._actual_rate_hz = self._count / elapsed
                    self._count = 0
                    self._t_rate = now
                with self._rate_lock:
                    dt = self._rt._dt
                t_target += dt
                self._rt.sleep_until(t_target)
        finally:
            self._rt.stop()


class SetpointLoop(BaseLoop):
    """Send RPYT to Crazyflie at a fixed (adjustable) frequency."""

    def __init__(
        self,
        state: SharedState,
        link,
        rate_hz: int = 100,
        min_thrust: int = Safety.TAKEOFF_THRUST,
    ):
        super().__init__(rate_hz)
        self.state = state
        self.link = link
        self.min_thrust = int(min_thrust)

    def _should_run(self) -> bool:
        return self._run_flag.is_set() and not self.state.stop_all.is_set()

    def _step(self):
        with self.state.lock:
            vbat = float(self.state.vbat or 0.0)
            r = float(self.state.rpyth.get("roll", 0.0))
            p = float(self.state.rpyth.get("pitch", 0.0))
            y = float(self.state.rpyth.get("yaw", 0.0))
            th = int(self.state.rpyth.get("thrust", 0))
            if vbat and vbat < 3.7:
                r = p = y = 0.0
                th = 0
            if th <= self.min_thrust:
                r = p = y = 0.0
        models.set_last_rpyt((r, p, y, th))

        cf = getattr(self.link, "cf", None)
        if cf is not None:
            self._sender.enqueue(cf.commander.send_setpoint, r, p, y, th)
        elif hasattr(self.link, "send_setpoint"):
            self._sender.enqueue(self.link.send_setpoint, r, p, y, th)



class PWMSetpointLoop(BaseLoop):
    """Send PWM values to Crazyflie using either manual or UDP input."""

    def __init__(self, link, rate_hz: int = 100):
        super().__init__(rate_hz)
        self.link = link
        self.last_pwm = [0, 0, 0, 0]
        self._mode = "manual"
        self._manual_pwm = [0, 0, 0, 0]
        self._udp: Optional[PWMUDPReceiver] = None

    def set_mode(self, mode: str):
        if mode not in ("manual", "udp"):
            raise ValueError("mode must be 'manual' or 'udp'")
        self._mode = mode

    def set_manual_pwm(self, pwm: List[int]):
        self._manual_pwm = [int(max(0, min(65535, v))) for v in pwm[:4]] + [0] * (4 - len(pwm))

    def attach_udp(self, receiver: PWMUDPReceiver):
        self._udp = receiver

    def _dispatch_pwm(self, pwm: List[int]):
        cf = getattr(self.link, "cf", None)
        if cf is None:
            return
        motors = ("m1", "m2", "m3", "m4")
        for i, m in enumerate(motors):
            try:
                cf.param.set_value(f"motorPowerSet.{m}", str(int(pwm[i])))
            except Exception:
                pass

    def _step(self):
        if self._mode == "udp" and self._udp:
            pwm = self._udp.get_last()
        else:
            pwm = list(self._manual_pwm)
        self.last_pwm[:] = pwm[:4]
        self._sender.enqueue(self._dispatch_pwm, pwm)


class FlightPathLoop(BaseLoop):
    """Background generator streaming XYZ targets to MATLAB via UDP."""

    def __init__(self, state: SharedState, rate_hz: int, base_xyz: tuple[float, float, float],
                 path_type: str, params: dict):
        super().__init__(rate_hz)
        self.state = state
        self._cfg_lock = Lock()
        self.base_xyz = tuple(float(v) for v in base_xyz)
        self.path_type = path_type
        self.params = dict(params)
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._addr = ("127.0.0.1", 51002)
        self._t0: float | None = None

    def update_config(self, cfg: dict):
        """Hot-swap path configuration atomically."""
        with self._cfg_lock:
            if "xyz" in cfg:
                self.base_xyz = tuple(float(v) for v in cfg["xyz"])
            if "path_type" in cfg:
                self.path_type = str(cfg["path_type"])
            if "path_params" in cfg:
                self.params = dict(cfg["path_params"])
            if "rate_hz" in cfg:
                self.set_rate(int(cfg["rate_hz"]))

    def stop(self):  # override to close socket
        super().stop()
        try:
            self._sock.close()
        except Exception:
            pass
        set_last_stream_xyz(None)

    def _should_run(self) -> bool:
        return self._run_flag.is_set() and not self.state.stop_all.is_set()

    def _compute_circle(self, t: float, base: tuple[float, float, float], params: dict) -> tuple[float, float, float, float]:
        cx = float(params.get("center_x", base[0]))
        cy = float(params.get("center_y", base[1]))
        R = float(params.get("radius", 0.0))
        v = float(params.get("speed", 0.0))
        cw = bool(params.get("clockwise", True))
        holdz = bool(params.get("hold_z", True))
        amp = float(params.get("z_amp", 0.0))
        per = float(params.get("z_period", 1.0))
        ang = (v / R if R > 0 else 0.0) * t
        if cw:
            ang = -ang
        x = cx + R * math.cos(ang)
        y = cy + R * math.sin(ang)
        if holdz or amp <= 0.0 or per <= 0.0:
            z = base[2]
        else:
            z = base[2] + amp * math.sin(2 * math.pi * t / per)
        return x, y, z, ang

    def _compute_square(self, t: float, base: tuple[float, float, float], params: dict) -> tuple[float, float, float, float]:
        cx = float(params.get("center_x", base[0]))
        cy = float(params.get("center_y", base[1]))
        L = float(params.get("side", params.get("side_length", 0.0)))
        v = float(params.get("speed", 0.0))
        cw = bool(params.get("clockwise", True))
        dwell = float(params.get("dwell", 0.0))
        holdz = bool(params.get("hold_z", True))
        amp = float(params.get("z_amp", 0.0))
        per = float(params.get("z_period", 1.0))
        edge_time = L / v if v > 0 else 0.0
        cycle = 4 * (edge_time + dwell)
        t_mod = t % cycle if cycle > 0 else 0.0
        half = L / 2.0
        if cw:
            verts = [
                (cx + half, cy + half),
                (cx + half, cy - half),
                (cx - half, cy - half),
                (cx - half, cy + half),
            ]
        else:
            verts = [
                (cx + half, cy + half),
                (cx - half, cy + half),
                (cx - half, cy - half),
                (cx + half, cy - half),
            ]
        idx = 0.0
        x = y = 0.0
        for i in range(4):
            seg_start = i * (edge_time + dwell)
            t_in = t_mod - seg_start
            if t_in < 0:
                continue
            if t_in < edge_time:
                sx, sy = verts[i]
                ex, ey = verts[(i + 1) % 4]
                ratio = t_in / edge_time if edge_time > 0 else 0.0
                x = sx + (ex - sx) * ratio
                y = sy + (ey - sy) * ratio
                idx = i + ratio
                break
            if t_in < edge_time + dwell:
                x, y = verts[(i + 1) % 4]
                idx = float(i + 1)
                break
        if holdz or amp <= 0.0 or per <= 0.0:
            z = base[2]
        else:
            z = base[2] + amp * math.sin(2 * math.pi * t / per)
        return x, y, z, idx

    def _step(self):
        if self._t0 is None:
            self._t0 = self._rt.now()
        t = self._rt.now() - self._t0
        with self._cfg_lock:
            base_xyz = self.base_xyz
            ptype = self.path_type
            params = dict(self.params)
        try:
            if ptype == "circle":
                x, y, z, ang = self._compute_circle(t, base_xyz, params)
                idx = ang
            elif ptype == "square":
                x, y, z, idx = self._compute_square(t, base_xyz, params)
                ang = 0.0
            else:
                x, y, z = base_xyz
                ang = 0.0
                idx = 0.0
            pkt = struct.pack("<3f", float(x), float(y), float(z))
            set_last_stream_xyz((x, y, z))
            self._sender.enqueue(self._sock.sendto, pkt, self._addr)
            with self.state.lock:
                self.state.path_last_xyz = (x, y, z)
                self.state.path_elapsed = t
                self.state.path_angle = ang
                self.state.path_index = idx
                self.state.path_actual_rate = self.get_actual_rate()
        except Exception as e:
            with self.state.lock:
                self.state.path_error = str(e)
            self._run_flag.clear()



# ---- helpers & public API ----

def zero_output(mode: str, link: LinkManager) -> None:
    """Send a neutral frame depending on control mode."""
    cf = link.get_cf() if link else None
    if cf is None:
        return
    if mode == "rpyt":
        try:
            cf.commander.send_setpoint(0.0, 0.0, 0.0, 0)
        except Exception:
            pass
    else:
        motors = ("m1", "m2", "m3", "m4")
        for m in motors:
            try:
                cf.param.set_value(f"motorPowerSet.{m}", "0")
            except Exception:
                pass


def send_rpyt_zero(commander):
    try:
        commander.send_setpoint(0.0, 0.0, 0.0, 0)
    except Exception:
        logger.exception("Failed to send neutral RPYT in Land")


def smooth_landing(target, mode: str = "rpyt", *, start_thrust: int | None = None,
                   mid_thrust: int = Landing.MID_THRUST,
                   ramp1_time: float = Landing.RAMP1_TIME,
                   ramp2_time: float = Landing.RAMP2_TIME,
                   steps: int = Landing.STEPS) -> None:
    """Ramp down thrust/PWM smoothly."""
    if steps <= 0 or target is None:
        return
    # Phase 0: neutral frame
    if mode == "rpyt":
        try:
            target.send_setpoint(0.0, 0.0, 0.0, 0)
        except Exception:
            pass
    else:
        for m in ("m1", "m2", "m3", "m4"):
            try:
                target.param.set_value(f"motorPowerSet.{m}", "0")
            except Exception:
                pass
    if start_thrust is None:
        start_thrust = mid_thrust
    dt1 = ramp1_time / steps if steps else 0
    dt2 = ramp2_time / steps if steps else 0
    if mode == "rpyt":
        cmd = target
        for i in range(steps):
            level = int(start_thrust - (start_thrust - mid_thrust) * (i + 1) / steps)
            cmd.send_setpoint(0.0, 0.0, 0.0, max(level, 0))
            time.sleep(dt1)
        for i in range(steps):
            level = int(mid_thrust - mid_thrust * (i + 1) / steps)
            cmd.send_setpoint(0.0, 0.0, 0.0, max(level, 0))
            time.sleep(dt2)
    else:
        cf = target
        for i in range(steps):
            level = int(start_thrust - (start_thrust - mid_thrust) * (i + 1) / steps)
            for m in ("m1", "m2", "m3", "m4"):
                try:
                    cf.param.set_value(f"motorPowerSet.{m}", str(max(level, 0)))
                except Exception:
                    pass
            time.sleep(dt1)
        for i in range(steps):
            level = int(mid_thrust - mid_thrust * (i + 1) / steps)
            for m in ("m1", "m2", "m3", "m4"):
                try:
                    cf.param.set_value(f"motorPowerSet.{m}", str(max(level, 0)))
                except Exception:
                    pass
            time.sleep(dt2)


def start_4pwm_loop(state: SharedState, link_mgr: LinkManager, rate_hz: float,
                    pwm_mode: str = "manual", manual_pwm: List[int] | None = None,
                    udp_port: int = 8888) -> bool:
    """Start background loop sending 4×PWM via CRTP port 0x0A."""
    global _pwm_thread, _pwm_stop_evt, _pwm_udp, _pwm_enable_param_name
    if _pwm_thread and _pwm_thread.is_alive():
        return True
    if not link_mgr or not link_mgr.ensure_connected():
        logging.error("No link; cannot start PWM loop")
        return False
    cf = link_mgr.get_cf()
    if cf is None:
        logging.error("No Crazyflie handle")
        return False
    try:
        link_mgr.send_arming_request(True)
    except Exception:
        pass
    # Clear any lingering stop flags from previous runs before starting
    # a new PWM loop. This allows the loop to restart after an emergency
    # stop or other abort that left the flags set.
    models.clear_stop_flags(state)
    _pwm_enable_param_name = try_set_enable_param(cf, 1)

    _pwm_stop_evt.clear()
    rate = float(rate_hz) if rate_hz else float(Rates.SETPOINT_HZ)
    interval = 1.0 / rate if rate > 0 else 0.01

    if pwm_mode == "udp":
        if _pwm_udp is None:
            _pwm_udp = PWMUDPReceiver(port=udp_port)
        _pwm_udp.start()
        models.set_accept_udp_8888(True)
        try:
            with state.lock:
                state.using_udp_8888 = True
        except Exception:
            pass
    else:
        vals = [clamp_u16(v) for v in (manual_pwm or [0, 0, 0, 0])[:4]]
        models.set_desired_pwm_tuple(*vals)

    def _worker():
        last_t = time.perf_counter()
        next_t = time.monotonic()
        models.set_pwm_running(True)
        while (not _pwm_stop_evt.is_set() and
               not state.stop_all.is_set() and
               not state.stop_flight.is_set()):
            if pwm_mode == "udp" and _pwm_udp:
                vals = _pwm_udp.get_last()
            else:
                vals = models.get_desired_pwm_tuple()
            m1, m2, m3, m4 = [clamp_u16(v) for v in vals[:4]]
            try:
                send_4pwm_packet(cf, m1, m2, m3, m4)
            except Exception:
                logger.exception("send_4pwm_packet failed")
                break
            models.set_last_pwm((m1, m2, m3, m4))
            now = time.perf_counter()
            dt = now - last_t
            if dt > 0:
                models.set_pwm_actual_rate(1.0 / dt)
            last_t = now
            next_t += interval
            sleep = next_t - time.monotonic()
            if sleep > 0:
                time.sleep(sleep)
        models.set_pwm_running(False)

    _pwm_thread = Thread(target=_worker, daemon=True)
    _pwm_thread.start()
    with state.lock:
        state.active_mode = "pwm"
        state.is_flying.set()
    return True


def stop_4pwm_loop(state: SharedState) -> None:
    global _pwm_thread, _pwm_stop_evt, _pwm_udp
    _pwm_stop_evt.set()
    if _pwm_thread:
        _pwm_thread.join(timeout=1.0)
        _pwm_thread = None
    if _pwm_udp:
        _pwm_udp.stop()
        _pwm_udp = None
    models.set_pwm_running(False)
    models.set_accept_udp_8888(False)
    try:
        with state.lock:
            state.using_udp_8888 = False
    except Exception:
        pass


def land_4pwm(state: SharedState, link_mgr: LinkManager) -> None:
    """Ramp down motors symmetrically then stop the PWM loop."""
    cf = link_mgr.get_cf() if link_mgr else None
    last = models.get_last_pwm()
    stop_4pwm_loop(state)
    if cf is None:
        return
    start_vals = [clamp_u16(v) for v in last]
    start = max(start_vals)
    steps = Landing.STEPS
    total_time = Landing.RAMP1_TIME + Landing.RAMP2_TIME
    dt = total_time / steps if steps else 0
    for i in range(steps):
        frac = 1.0 - (i + 1) / steps
        vals = [int(v * frac) for v in start_vals]
        try:
            send_4pwm_packet(cf, *vals)
        except Exception:
            pass
        time.sleep(dt)
    try:
        send_4pwm_packet(cf, 0, 0, 0, 0)
    except Exception:
        pass
    models.set_last_pwm((0, 0, 0, 0))
    global _pwm_enable_param_name
    if _pwm_enable_param_name:
        try:
            cf.param.set_value(_pwm_enable_param_name, "0")
            time.sleep(0.05)
        except Exception:
            pass
        _pwm_enable_param_name = ""
    with state.lock:
        state.active_mode = None
        state.is_flying.clear()


def emergency_stop_4pwm(state: SharedState, link_mgr: LinkManager) -> None:
    cf = link_mgr.get_cf() if link_mgr else None
    stop_4pwm_loop(state)
    if cf:
        try:
            send_4pwm_packet(cf, 0, 0, 0, 0)
        except Exception:
            pass
    models.set_last_pwm((0, 0, 0, 0))
    with state.lock:
        state.active_mode = None
        state.is_flying.clear()


def clear_udp_8888_with_zero(link_mgr: LinkManager | None, state: SharedState) -> None:
    """Send zero PWM and neutral RPYT once, then free UDP 8888."""
    models.set_accept_udp_8888(False)
    cf = link_mgr.get_cf() if link_mgr else None
    try:
        if cf:
            send_4pwm_packet(cf, 0, 0, 0, 0)
    except Exception:
        pass
    try:
        cmdr = link_mgr.get_commander() if link_mgr else None
        if cmdr:
            cmdr.send_setpoint(0.0, 0.0, 0.0, 0)
    except Exception:
        pass
    stop_4pwm_loop(state)
    stop_udp8888_receivers_if_any()
    close_udp8888_sockets_if_any()
    models.set_last_rpyt((0.0, 0.0, 0.0, 0.0))
    models.set_last_pwm((0, 0, 0, 0))
    try:
        with state.lock:
            state.rpyth.update({"roll": 0.0, "pitch": 0.0, "yaw": 0.0, "thrust": 0.0})
            state.using_udp_8888 = False
    except Exception:
        pass
    try:
        from .utils import clear_udp_ports_windows
        clear_udp_ports_windows([8888])
    except Exception:
        pass


def start_mode(mode: str, state: SharedState, link: LinkManager, *, rate_hz: int | None = None,
               pwm_mode: str = "manual", manual_pwm: List[int] | None = None,
               udp_port: int = 8888):
    """Ensure connection, send neutral, arm, and start the requested loop."""
    global _setpoint_loop, _pwm_loop, _pwm_udp
    if not link or not link.ensure_connected():
        logging.error("No link; cannot start mode")
        return None
    zero_output(mode, link)
    link.send_arming_request(True)

    if mode == "rpyt":
        if _pwm_loop and _pwm_loop.is_running():
            _pwm_loop.stop()
            _pwm_loop = None
        if _setpoint_loop is None:
            _setpoint_loop = SetpointLoop(state, link, rate_hz=rate_hz or Rates.SETPOINT_HZ)
        _setpoint_loop.set_rate(rate_hz or _setpoint_loop.get_rate())
        _setpoint_loop.start()
        loop = _setpoint_loop
    else:
        if _setpoint_loop and _setpoint_loop.is_running():
            _setpoint_loop.stop()
            _setpoint_loop = None
        if _pwm_loop is None:
            _pwm_loop = PWMSetpointLoop(link, rate_hz=rate_hz or Rates.SETPOINT_HZ)
        _pwm_loop.set_rate(rate_hz or _pwm_loop.get_rate())
        _pwm_loop.set_mode(pwm_mode)
        if pwm_mode == "manual" and manual_pwm is not None:
            _pwm_loop.set_manual_pwm(manual_pwm)
        elif pwm_mode == "udp":
            if _pwm_udp is None:
                _pwm_udp = PWMUDPReceiver(port=udp_port)
            _pwm_udp.start()
            _pwm_loop.attach_udp(_pwm_udp)
            try:
                with state.lock:
                    state.using_udp_8888 = True
            except Exception:
                pass
        _pwm_loop.start()
        loop = _pwm_loop
    with state.lock:
        state.active_mode = mode
        state.is_flying.set()
    models.set_accept_udp_8888(True)
    return loop


def land(mode: str, state: SharedState, link: LinkManager):
    """Perform smooth landing then send final zero and stop loops."""
    global _setpoint_loop, _pwm_loop, _pwm_udp
    models.set_accept_udp_8888(False)
    models.set_last_rpyt((0.0, 0.0, 0.0, 0.0))
    with state.lock:
        state.rpyth.update({"roll": 0.0, "pitch": 0.0, "yaw": 0.0, "thrust": 0.0})
        state.using_udp_8888 = False
    cf = link.get_cf() if link else None
    if cf is None:
        logging.error("No CF for landing")
        return False
    if mode == "rpyt":
        commander = link.get_commander() if link else None
        if commander is None:
            logging.error("No commander for landing")
            return False
        with state.lock:
            start_thr = int(state.rpyth.get("thrust", 0))
        # immediate neutral frame before stopping loop
        try:
            cf.commander.send_setpoint(0.0, 0.0, 0.0, 0)
        except Exception:
            logger.exception("Failed to send neutral RPYT in Land")
        if _setpoint_loop and _setpoint_loop.is_running():
            _setpoint_loop.stop()
            _setpoint_loop = None
        # ramp down thrust with zeros for roll/pitch/yaw
        smooth_landing(commander, "rpyt", start_thrust=start_thr)
    else:
        last = [0, 0, 0, 0]
        if _pwm_loop and _pwm_loop.is_running():
            last = list(getattr(_pwm_loop, "last_pwm", [0, 0, 0, 0]))
            _pwm_loop.stop()
        if _pwm_udp:
            _pwm_udp.stop(); _pwm_udp = None
        avg = int(sum(last)/4) if last else 0
        smooth_landing(cf, "pwm", start_thrust=avg)
        _pwm_loop = None

    # final safety zero
    zero_output(mode, link)
    try:
        link.send_arming_request(False)
    except Exception:
        pass
    with state.lock:
        state.active_mode = None
        state.is_flying.clear()
    clear_udp_8888()
    send_udp_rpyt_zero()
    if mode == "rpyt":
        logging.info("Landing complete, zero RPYT packet dispatched, port 8888 cleared.")
    else:
        logging.info("Landing complete (PWM), zero RPYT packet dispatched, port 8888 cleared.")
    return True


# ---- Flight path public API ----

def start_path(state: SharedState):
    """Start streaming XYZ targets to MATLAB based on current model state."""
    global _path_loop
    if _path_loop and _path_loop.is_running():
        _path_loop.stop()
    with state.lock:
        base_xyz = tuple(state.xyz_target)
        path_type = state.path_type
        params = dict(state.path_params)
        rate_hz = int(state.rate_hz)
    loop = FlightPathLoop(state, rate_hz, base_xyz, path_type, params)
    loop.start()
    _path_loop = loop
    with state.lock:
        state.stream_running = True
        state.path_error = ""
    set_last_stream_xyz(base_xyz)
    return loop


def hot_update(config: dict) -> None:
    """Update running path loop configuration without restart."""
    loop = _path_loop
    if loop and loop.is_running():
        loop.update_config(config)


def stop_path(state: SharedState):
    """Stop the active flight path stream."""
    global _path_loop
    if _path_loop:
        _path_loop.stop()
        _path_loop = None
    with state.lock:
        state.stream_running = False
    set_last_stream_xyz(None)


def stop_all_control_loops_if_any() -> None:
    """Stop any running control/path loops and UDP receivers."""
    global _setpoint_loop, _pwm_loop, _path_loop
    for name in ("_setpoint_loop", "_pwm_loop", "_path_loop"):
        loop = globals().get(name)
        try:
            if loop and getattr(loop, "is_running", lambda: False)():
                loop.stop()
        except Exception:
            pass
        finally:
            globals()[name] = None
    stop_udp8888_receivers_if_any()
    set_last_stream_xyz(None)


def send_xyz_once(x: float, y: float, z: float):
    """Send a single XYZ packet to MATLAB (non-blocking best effort)."""
    try:
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.setblocking(False)
        x, y, z = float(x), float(y), float(z)
        set_last_stream_xyz((x, y, z))
        sock.sendto(struct.pack("<3f", x, y, z), ("127.0.0.1", 51002))
    except Exception:
        pass
    finally:
        try:
            sock.close()
        except Exception:
            pass


def stop_udp8888_receivers_if_any() -> None:
    """Stop any internal threads reading from UDP port 8888."""
    global _pwm_udp
    try:
        for udp in list(_udp_inputs):
            try:
                th = getattr(udp, "_thread", None)
                udp.stop()
                if th:
                    th.join(timeout=0.5)
            except Exception:
                pass
            finally:
                if udp in _udp_inputs:
                    _udp_inputs.remove(udp)
        if _pwm_udp:
            try:
                th = getattr(_pwm_udp, "_thread", None)
                _pwm_udp.stop()
                if th:
                    th.join(timeout=0.5)
            except Exception:
                pass
            finally:
                _pwm_udp = None
    except Exception:
        pass


def close_udp8888_sockets_if_any() -> None:
    """Bind and close port 8888 to ensure it's free."""
    import socket, contextlib
    with contextlib.suppress(Exception):
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.bind(("127.0.0.1", 8888))
        s.close()


def clear_udp_8888(force: bool = True) -> None:
    """Best-effort: stop any receivers/sockets bound to UDP 8888 and free it."""
    try:
        stop_udp8888_receivers_if_any()
        close_udp8888_sockets_if_any()
        try:
            from .utils import clear_udp_ports_windows
            clear_udp_ports_windows([8888])
        except Exception:
            pass
    except Exception:
        logger.exception("clear_udp_8888 failed")


def clear_udp_8888_with_neutral(link_mgr: LinkManager | None) -> None:
    """Send a neutral RPYT frame once, then clear UDP port 8888."""
    models.set_accept_udp_8888(False)

    # 1) immediate neutral frame
    try:
        cmdr = link_mgr.get_commander() if link_mgr else None
        if cmdr:
            cmdr.send_setpoint(0.0, 0.0, 0.0, 0)
    except Exception:
        pass

    # 2) stop listeners and free port
    stop_udp8888_receivers_if_any()
    close_udp8888_sockets_if_any()

    # 3) force-clear last values and state
    models.set_last_rpyt((0.0, 0.0, 0.0, 0.0))
    if link_mgr and getattr(link_mgr, "state", None):
        try:
            with link_mgr.state.lock:
                link_mgr.state.rpyth.update({"roll": 0.0, "pitch": 0.0, "yaw": 0.0, "thrust": 0.0})
                link_mgr.state.using_udp_8888 = False
        except Exception:
            pass

    # 4) actively free the port and sweep on Windows
    try:
        from .utils import clear_udp_ports_windows
        clear_udp_ports_windows([8888])
    except Exception:
        pass


def emergency_stop(link_mgr: LinkManager | None, state: SharedState) -> None:
    """Immediate hard stop: kill loops and send neutral RPYT once."""
    try:
        # 1) cut all loops
        state.stop_all.set()
        state.stop_flight.set()
        state.stream_running = False
        stop_all_control_loops_if_any()

        # 2) neutral RPYT
        cmdr = link_mgr.get_commander() if link_mgr else None
        if cmdr:
            cmdr.send_setpoint(0.0, 0.0, 0.0, 0)

        # 3) reset model/state
        state.active_mode = None
        state.is_flying.clear()
        models.set_accept_udp_8888(False)
        models.set_last_rpyt((0.0, 0.0, 0.0, 0.0))
        models.clear_stop_flags(state)

        print("[SAFETY] Emergency stop: control loops terminated, RPYT=0 sent.")
    except Exception as e:
        print(f"[SAFETY] Emergency stop failed: {e}")


def send_udp_rpyt_zero():
    """Send a single zeroed RPYT packet to localhost UDP port 8888."""
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        pkt = struct.pack("<4f", 0.0, 0.0, 0.0, 0.0)
        sock.sendto(pkt, ("127.0.0.1", 8888))
        logging.debug("[UDP] Sent zero RPYT packet to port 8888")
    finally:
        sock.close()
