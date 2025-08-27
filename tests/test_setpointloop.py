import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))

from cfmarslab.control import SetpointLoop
from cfmarslab.models import SharedState


class DummyLink:
    def __init__(self):
        self.sent = []

    def send_setpoint(self, r, p, y, th):
        self.sent.append((r, p, y, th))


def test_setpointloop_min_thrust():
    state = SharedState()
    link = DummyLink()
    loop = SetpointLoop(state, link, min_thrust=38000)
    loop._sender.enqueue = lambda func, *args, **kwargs: func(*args, **kwargs)

    with state.lock:
        state.vbat = 4.0
        state.rpyth.update({"roll": 1.0, "pitch": 2.0, "yaw": 3.0, "thrust": 37000})
    loop._step()
    assert link.sent[-1] == (0.0, 0.0, 0.0, 37000)

    with state.lock:
        state.rpyth.update({"roll": 1.0, "pitch": 2.0, "yaw": 3.0, "thrust": 39000})
    loop._step()
    assert link.sent[-1] == (1.0, 2.0, 3.0, 39000)
