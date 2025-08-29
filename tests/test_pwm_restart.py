import time
import sys
from pathlib import Path

# Ensure src directory on path
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))

from cfmarslab.models import SharedState, get_last_pwm
from cfmarslab.control import (
    start_4pwm_loop,
    stop_4pwm_loop,
    emergency_stop_4pwm,
)


class DummyParam:
    def __init__(self):
        self.values = {}

    def set_value(self, name, value):
        self.values[name] = value

    def get_value(self, name):
        return self.values.get(name, "0")


class DummyCF:
    def __init__(self, param):
        self.param = param
        self.sent_packets = []
        # send_packet_compat expects a 'link' attribute with send_packet
        self.link = self

    def send_packet(self, pkt):
        self.sent_packets.append(pkt)


class DummyLinkManager:
    def __init__(self, cf):
        self._cf = cf

    def ensure_connected(self):
        return True

    def get_cf(self):
        return self._cf

    def send_arming_request(self, state):
        pass

    def get_commander(self):
        return None


def test_pwm_loop_restarts_after_emergency_stop():
    state = SharedState()
    cf = DummyCF(DummyParam())
    link = DummyLinkManager(cf)

    # First, ensure any previous loop is stopped and stop flags are set
    emergency_stop_4pwm(state, link)
    state.stop_all.set()
    state.stop_flight.set()

    pwm_vals = [100, 200, 300, 400]
    assert start_4pwm_loop(state, link, rate_hz=50, pwm_mode="manual", manual_pwm=pwm_vals)

    # Allow the worker thread to run at least once
    time.sleep(0.1)
    stop_4pwm_loop(state)

    # The PWM loop should have sent our manual values
    assert get_last_pwm() == tuple(pwm_vals)
    assert cf.sent_packets, "No PWM packets were sent"
