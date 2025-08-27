from dataclasses import dataclass, field
from threading import Event, Lock
from collections import deque
from typing import Dict, Deque, Tuple, Optional
import threading

from .config import PathCfg


@dataclass
class SharedState:
    # Control inputs (deg, deg/s, thrust)
    rpyth: Dict[str, float] = field(default_factory=lambda: {
        "roll": 0.0, "pitch": 0.0, "yaw": 0.0, "thrust": 0.0
    })
    # User target coords (m)
    xyz_target: Tuple[float, float, float] = PathCfg.DEFAULT_TARGET
    # Throttle offset applied to incoming RPYT throttle
    throttle_offset: int = 40000
    # Telemetry
    vbat: float = 0.0
    rssi: float = float('nan')       # last received RSSI in dBm
    latency_ms: float = float('nan') # ms between log callbacks

    # Concurrency primitives
    stop_all: Event = field(default_factory=Event)
    stop_flight: Event = field(default_factory=Event)
    lock: Lock = field(default_factory=Lock)
    # Active control mode and running state
    active_mode: str | None = None
    is_flying: Event = field(default_factory=Event)

    # Ring buffer for logs (UI plots)
    log_buf: Deque = field(default_factory=lambda: deque(maxlen=2000))

    # --- XYZ path streaming state ---
    stream_running: bool = False
    # Last path selection applied from the UI
    path_type: str = "none"      # "none" | "circle" | "square"
    path_params: Dict[str, float] = field(default_factory=dict)
    rate_hz: int = PathCfg.DEFAULT_HZ
    path_elapsed: float = 0.0
    path_last_xyz: Tuple[float, float, float] = (0.0, 0.0, 0.0)
    path_angle: float = 0.0      # for circle
    path_index: float = 0.0      # for square
    path_actual_rate: float = 0.0
    path_error: str = ""

    # Whether any internal receiver is bound to localhost UDP port 8888
    using_udp_8888: bool = False


# --- last streamed XYZ for UI marker -------------------------------------
_last_stream_xyz: Optional[Tuple[float, float, float]] = None
_stream_lock = Lock()

# --- UDP 8888 gating and last RPYT --------------------------------------
_lock = threading.Lock()
_accept_udp_8888 = False
_last_rpyt = (0.0, 0.0, 0.0, 0.0)


def set_accept_udp_8888(v: bool):
    global _accept_udp_8888
    with _lock:
        _accept_udp_8888 = bool(v)


def get_accept_udp_8888():
    with _lock:
        return _accept_udp_8888


def set_last_rpyt(rpyt_tuple):
    global _last_rpyt
    with _lock:
        _last_rpyt = tuple(float(x) for x in rpyt_tuple)


def get_last_rpyt():
    with _lock:
        return _last_rpyt


def set_last_stream_xyz(v: Optional[Tuple[float, float, float]]) -> None:
    """Atomically store the most recent XYZ sent to MATLAB."""
    with _stream_lock:
        globals()["_last_stream_xyz"] = v


def get_last_stream_xyz() -> Optional[Tuple[float, float, float]]:
    """Return the last streamed XYZ, or None if unavailable."""
    with _stream_lock:
        return _last_stream_xyz
