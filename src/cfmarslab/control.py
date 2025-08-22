from __future__ import annotations
import socket, struct
from time import perf_counter_ns
from threading import Thread, Event, Lock
from typing import Optional, List
from queue import Queue
from collections import deque

from .models import SharedState
from .utils import set_realtime_priority
from .realtime import Realtime
from .config import RT


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

    def __init__(self, state: SharedState, link, rate_hz: int = 100):
        super().__init__(rate_hz)
        self.state = state
        self.link = link

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
            if th <= 48000:
                r = p = y = 0.0
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
