from __future__ import annotations
import socket, struct
from time import perf_counter, sleep
from threading import Thread, Event, Lock
from typing import Optional, List

from .models import SharedState
from .utils import set_realtime_priority


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
        if self._thread and self._thread.is_alive():
            return
        self._running.set()
        self._thread = Thread(target=self._run, daemon=True)
        self._thread.start()
        set_realtime_priority(self._thread.ident)  # reduce jitter; may require admin rights

    def stop(self):
        self._running.clear()

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
        t = perf_counter()
        while self._running.is_set() and not self.state.stop_all.is_set():
            try:
                while True:
                    data, _ = sock.recvfrom(1024)
                    if len(data) == 16:
                        r, p, y, thr_f = struct.unpack("<4f", data)
                        r = max(-5.0, min(5.0, float(r)))
                        p = max(-5.0, min(5.0, float(p)))
                        y = max(-360.0, min(360.0, float(y)))
                        th = max(0, min(20000, int(thr_f))) + 40000
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
                dt = 1.0 / float(self._rate_hz)
            t += dt
            d = t - perf_counter()
            if d > 0:
                sleep(d)
            else:
                t = perf_counter()


class PWMUDPReceiver:
    """Background UDP listener for 4 PWM values (m1–m4).

    Accepts either little-endian floats (``<4f``) or uint16 (``<4H``) and
    keeps the last successfully received set. Designed to be lightweight and
    non-blocking so it can be polled from another thread."""

    def __init__(self, port: int = 8888):
        self.port = port
        self._running = Event()
        self._thread: Optional[Thread] = None
        self._last = [0, 0, 0, 0]

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

    # --- worker ---
    def _run(self):
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.bind(("127.0.0.1", self.port))
        sock.settimeout(0.2)
        while self._running.is_set():
            try:
                data, _ = sock.recvfrom(1024)
            except socket.timeout:
                continue
            except Exception:
                continue
            if len(data) != 16:
                continue
            vals: List[int] = []
            try:
                floats = struct.unpack("<4f", data)
                vals = [int(max(0, min(65535, v))) for v in floats]
            except Exception:
                try:
                    ushorts = struct.unpack("<4H", data)
                    vals = [int(max(0, min(65535, v))) for v in ushorts]
                except Exception:
                    continue
            if vals:
                self._last = vals


class SetpointLoop:
    """Send RPYT to Crazyflie at a fixed (adjustable) frequency, and
    measure the *actual* send rate averaged over ~1s windows."""
    def __init__(self, state: SharedState, link, rate_hz: int = 100):
        self.state = state
        self.link = link
        self._rate_hz = max(1, int(rate_hz))
        self._rate_lock = Lock()
        self._run_flag = Event()
        self._thread: Optional[Thread] = None
        # actual rate measurement
        self._count = 0
        self._t_rate = perf_counter()
        self._actual_rate_hz = 0.0

    # --- public API ---
    def start(self, rate_hz: Optional[int] = None):
        if rate_hz is not None:
            self.set_rate(rate_hz)
        if self._thread and self._thread.is_alive():
            return
        self._run_flag.set()
        self._thread = Thread(target=self._run, daemon=True)
        self._thread.start()
        set_realtime_priority(self._thread.ident)  # reduce jitter; may require admin rights

    def stop(self):
        self._run_flag.clear()

    def is_running(self) -> bool:
        return self._run_flag.is_set()

    def set_rate(self, hz: int):
        with self._rate_lock:
            self._rate_hz = max(1, int(hz))

    def get_rate(self) -> int:
        with self._rate_lock:
            return self._rate_hz

    def get_actual_rate(self) -> float:
        """Return the last computed actual send rate in Hz (1s average)."""
        return float(self._actual_rate_hz)

    # --- worker ---
    def _run(self):
        t = perf_counter()
        while self._run_flag.is_set() and not self.state.stop_all.is_set():
            # fetch once per tick
            with self.state.lock:
                vbat = float(self.state.vbat or 0.0)
                r = float(self.state.rpyth.get("roll", 0.0))
                p = float(self.state.rpyth.get("pitch", 0.0))
                y = float(self.state.rpyth.get("yaw", 0.0))
                th = int(self.state.rpyth.get("thrust", 0))
                if vbat and vbat < 3.7:
                    r = p = y = 0.0; th = 0
                if th <= 48000:
                    r = p = y = 0.0
            try:
                self.link.send_setpoint(r, p, y, th)
            except Exception:
                pass
            # --- actual rate accounting ---
            self._count += 1
            now = perf_counter()
            elapsed = now - self._t_rate
            if elapsed >= 1.0:
                self._actual_rate_hz = self._count / elapsed
                self._count = 0
                self._t_rate = now

            # timing
            with self._rate_lock:
                dt = 1.0 / float(self._rate_hz)
            t += dt
            d = t - perf_counter()
            if d > 0:
                sleep(d)
            else:
                t = perf_counter()


class PWMSetpointLoop:
    """Send PWM values to Crazyflie using either manual or UDP input."""

    def __init__(self, link, rate_hz: int = 100):
        self.link = link
        self._rate_hz = max(1, int(rate_hz))
        self._rate_lock = Lock()
        self._run_flag = Event()
        self._thread: Optional[Thread] = None
        self.last_pwm = [0, 0, 0, 0]
        self._mode = "manual"
        self._manual_pwm = [0, 0, 0, 0]
        self._udp: Optional[PWMUDPReceiver] = None

    # --- public API ---
    def start(self, rate_hz: Optional[int] = None):
        if rate_hz is not None:
            self.set_rate(rate_hz)
        if self._thread and self._thread.is_alive():
            return
        self._run_flag.set()
        self._thread = Thread(target=self._run, daemon=True)
        self._thread.start()
        set_realtime_priority(self._thread.ident)  # reduce jitter; may require admin rights

    def stop(self):
        self._run_flag.clear()

    def is_running(self) -> bool:
        return self._run_flag.is_set()

    def set_rate(self, hz: int):
        with self._rate_lock:
            self._rate_hz = max(1, int(hz))

    def get_rate(self) -> int:
        with self._rate_lock:
            return self._rate_hz

    def set_mode(self, mode: str):
        if mode not in ("manual", "udp"):
            raise ValueError("mode must be 'manual' or 'udp'")
        self._mode = mode

    def set_manual_pwm(self, pwm: List[int]):
        self._manual_pwm = [int(max(0, min(65535, v))) for v in pwm[:4]] + [0]*(4-len(pwm))

    def attach_udp(self, receiver: PWMUDPReceiver):
        self._udp = receiver

    # --- worker ---
    def _run(self):
        motors = ("m1", "m2", "m3", "m4")
        t = perf_counter()
        while self._run_flag.is_set():
            if self._mode == "udp" and self._udp:
                pwm = self._udp.get_last()
            else:
                pwm = list(self._manual_pwm)
            cf = getattr(self.link, "cf", None)
            if cf is not None:
                for i, m in enumerate(motors):
                    try:
                        cf.param.set_value(f"motorPowerSet.{m}", str(int(pwm[i])))
                    except Exception:
                        pass
            self.last_pwm[:] = pwm[:4]
            with self._rate_lock:
                dt = 1.0 / float(self._rate_hz)
            t += dt
            d = t - perf_counter()
            if d > 0:
                sleep(d)
            else:
                t = perf_counter()
