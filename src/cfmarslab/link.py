# src/cfmarslab/link.py
from __future__ import annotations
import time
from typing import Optional

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.utils.power_switch import PowerSwitch

from .models import SharedState

cflib.crtp.init_drivers()

class LinkManager:
    """Manage Crazyflie link + minimal telemetry (VBAT, optional RSSI/latency)."""
    def __init__(self, state: SharedState, uri: str):
        self.state = state
        self.uri = uri
        self.scf: Optional[SyncCrazyflie] = None
        self.cf: Optional[Crazyflie] = None
        self._lg: Optional[LogConfig] = None
        self._last_cb: Optional[float] = None

    def connect(self):
        # establish link
        self.scf = SyncCrazyflie(self.uri, cf=Crazyflie(rw_cache="./cache"))
        self.scf.__enter__()  # manual context enter to keep handle
        self.cf = self.scf.cf

        # minimal telemetry log
        lg = LogConfig(name="Battery", period_in_ms=100)
        lg.add_variable("pm.vbat", "FP16")
        # try to add RSSI if available (not all platforms expose it)
        try:
            lg.add_variable("radio.rssi", "float")
        except Exception:
            pass

        self.cf.log.add_config(lg)
        lg.data_received_cb.add_callback(self._on_log)
        lg.start()
        self._lg = lg

    def _on_log(self, ts, data, logconf):
        # update shared state
        with self.state.lock:
            self.state.vbat = float(data.get("pm.vbat", 0.0))
            if "radio.rssi" in data:
                try:
                    self.state.rssi = float(data["radio.rssi"])
                except Exception:
                    pass
            # simple latency proxy: interval between callbacks (ms)
            now = time.perf_counter()
            if self._last_cb is not None:
                self.state.latency_ms = (now - self._last_cb) * 1000.0
            self._last_cb = now

    def send_setpoint(self, roll: float, pitch: float, yawrate: float, thrust: int):
        if self.cf is not None:
            self.cf.commander.send_setpoint(roll, pitch, yawrate, thrust)

    def power_cycle(self):
        ps = PowerSwitch(self.uri)
        ps.stm_power_down()
        time.sleep(2.5)
        ps.stm_power_up()
        time.sleep(2.5)

    def disconnect(self):
        try:
            if self.cf is not None:
                # send one last zero setpoint for safety
                self.cf.commander.send_setpoint(0.0, 0.0, 0.0, 0)
            # stop logging if started
            if self._lg is not None:
                try:
                    self._lg.stop()
                except Exception:
                    pass
                self._lg = None
        finally:
            if self.scf is not None:
                # close link
                self.scf.__exit__(None, None, None)
            self.scf = None
            self.cf = None
            self._last_cb = None
