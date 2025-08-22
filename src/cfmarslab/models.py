from dataclasses import dataclass, field
from threading import Event, Lock
from collections import deque
from typing import Dict, Deque, Tuple


@dataclass
class SharedState:
    # Control inputs (deg, deg/s, thrust)
    rpyth: Dict[str, float] = field(default_factory=lambda: {
        "roll": 0.0, "pitch": 0.0, "yaw": 0.0, "thrust": 0.0
    })
    # User target coords (m)
    user_xyz: Tuple[float, float, float] = (0.0, 0.0, 0.0)
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

    # Ring buffer for logs (UI plots)
    log_buf: Deque = field(default_factory=lambda: deque(maxlen=2000))
