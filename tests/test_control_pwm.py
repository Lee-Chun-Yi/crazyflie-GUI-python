import logging
import sys
from pathlib import Path

# Ensure src directory on path
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))

from cfmarslab.control import try_set_enable_param


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


def test_try_set_enable_param_success():
    cf = DummyCF(DummyParam())
    name = try_set_enable_param(cf, 1)
    assert name == "crtp_pwm.enable"


def test_try_set_enable_param_failure(caplog):
    class FailingParam(DummyParam):
        def get_value(self, name):
            return "0"

    cf = DummyCF(FailingParam())
    with caplog.at_level(logging.WARNING):
        name = try_set_enable_param(cf, 1)
    assert name == ""
    assert any("could not be verified" in rec.message for rec in caplog.records)
