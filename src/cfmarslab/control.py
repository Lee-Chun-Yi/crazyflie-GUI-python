from __future__ import annotations
import socket, struct, logging, time, math
from time import perf_counter_ns
from threading import Thread, Event, Lock
from typing import Optional, List
from queue import Queue
from collections import deque

from .models import SharedState
from .utils import set_realtime_priority
from .realtime import Realtime
from .config import RT, Safety, Landing, Rates, PreviewCfg
from .link import LinkManager

logger = logging.getLogger(__name__)

# Global loop references to ensure only one active at a time
_setpoint_loop: Optional['SetpointLoop'] = None
_pwm_loop: Optional['PWMSetpointLoop'] = None
_pwm_udp: Optional['PWMUDPReceiver'] = None
_path_loop: Optional['FlightPathLoop'] = None


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

    def stop(self):
        self._running.clear()
        if self._thread:
            self._thread.join(timeout=1.0)
            self._thread = None

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
                        with self.state.lock:
                            self.state.rpyth.update({
                                "roll": r, "pitch": p, "yaw": y, "thrust": th
                            })
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
        _pwm_loop.start()
        loop = _pwm_loop
    with state.lock:
        state.active_mode = mode
        state.is_flying.set()
    return loop


def land(mode: str, state: SharedState, link: LinkManager):
    """Perform smooth landing then send final zero and stop loops."""
    global _setpoint_loop, _pwm_loop, _pwm_udp
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


def send_xyz_once(x: float, y: float, z: float):
    """Send a single XYZ packet to MATLAB (non-blocking best effort)."""
    try:
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.setblocking(False)
        sock.sendto(struct.pack("<3f", float(x), float(y), float(z)), ("127.0.0.1", 51002))
    except Exception:
        pass
    finally:
        try:
            sock.close()
        except Exception:
            pass


def clear_udp_8888():
    """Release localhost UDP port 8888 if held by this process."""
    try:
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.bind(("127.0.0.1", 8888))
        sock.close()
        logging.debug("[UDP] Cleared port 8888 after Stop")
    except OSError:
        pass


def send_udp_rpyt_zero():
    """Send a single zeroed RPYT packet to localhost UDP port 8888."""
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        pkt = struct.pack("<4f", 0.0, 0.0, 0.0, 0.0)
        sock.sendto(pkt, ("127.0.0.1", 8888))
        logging.debug("[UDP] Sent zero RPYT packet to port 8888")
    finally:
        sock.close()
