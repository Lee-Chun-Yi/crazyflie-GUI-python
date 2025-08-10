# src/cfmarslab/link.py
from __future__ import annotations
import time
from typing import Optional
from threading import Event, Thread

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.utils.power_switch import PowerSwitch

from .models import SharedState

cflib.crtp.init_drivers()

class LinkManager:
    """Manage Crazyflie link + minimal telemetry.

    Features:
    - Auto-reconnect when battery log stops updating for a while.
    - RSSI/Latency sampling into SharedState for UI to display.
    """
    STALE_SEC = 1.5           # no log callback for this long => stale
    BACKOFF_SEC = 2.0         # wait before reconnect attempt

    def __init__(self, state: SharedState, uri: str, auto_reconnect: bool = True):
        self.state = state
        self.uri = uri
        self.auto_reconnect = auto_reconnect
        self.scf: Optional[SyncCrazyflie] = None
        self.cf: Optional[Crazyflie] = None
        self._lg: Optional[LogConfig] = None
        self._last_cb: Optional[float] = None
        # monitor
        self._mon_flag = Event()
        self._mon_thread: Optional[Thread] = None
        self._reconnect_attempts = 0

    # --- lifecycle ---
    def connect(self):
        self._open_link()
        self._start_monitor()

    def _open_link(self):
        # establish link
        self.scf = SyncCrazyflie(self.uri, cf=Crazyflie(rw_cache="./cache"))
        self.scf.__enter__()  # manual context enter to keep handle
        self.cf = self.scf.cf

        # minimal telemetry log
        lg = LogConfig(name="Battery", period_in_ms=100)
        lg.add_variable("pm.vbat", "FP16")
        try:
            lg.add_variable("radio.rssi", "float")
        except Exception:
            pass
        self.cf.log.add_config(lg)
        lg.data_received_cb.add_callback(self._on_log)
        lg.start()
        self._lg = lg
        self._last_cb = time.perf_counter()
        self._reconnect_attempts = 0

    def _start_monitor(self):
        if self._mon_thread and self._mon_thread.is_alive():
            return
        self._mon_flag.set()
        self._mon_thread = Thread(target=self._monitor, daemon=True)
        self._mon_thread.start()

    def _stop_monitor(self):
        self._mon_flag.clear()

    def _monitor(self):
        # watch for stale telemetry
        while self._mon_flag.is_set():
            try:
                if self.auto_reconnect and self._last_cb is not None:
                    age = time.perf_counter() - self._last_cb
                    if age > self.STALE_SEC:
                        self._reconnect_attempts += 1
                        self._reconnect()
                        # after reconnect attempt, wait a bit before re-evaluating
                        time.sleep(self.BACKOFF_SEC)
                        continue
            except Exception:
                pass
            time.sleep(0.25)

    def _reconnect(self):
        # best effort: safe zero setpoint then reconnect
        try:
            if self.cf is not None:
                try:
                    self.cf.commander.send_setpoint(0.0, 0.0, 0.0, 0)
                except Exception:
                    pass
        finally:
            try:
                self._close_link()
            except Exception:
                pass
            try:
                self._open_link()
            except Exception:
                # optional power-cycle after multiple failures
                if self._reconnect_attempts % 3 == 0:
                    try:
                        ps = PowerSwitch(self.uri)
                        ps.stm_power_down(); time.sleep(2.0); ps.stm_power_up(); time.sleep(2.0)
                    except Exception:
                        pass

    def disconnect(self):
        self._stop_monitor()
        try:
            if self.cf is not None:
                # send one last zero setpoint for safety
                self.cf.commander.send_setpoint(0.0, 0.0, 0.0, 0)
            if self._lg is not None:
                try:
                    self._lg.stop()
                except Exception:
                    pass
                self._lg = None
        finally:
            self._close_link()

    def _close_link(self):
        if self.scf is not None:
            self.scf.__exit__(None, None, None)
        self.scf = None
        self.cf = None
        self._last_cb = None

    # --- callbacks & commands ---
    def _on_log(self, ts, data, logconf):
        # update state
        now = time.perf_counter()
        with self.state.lock:
            self.state.vbat = float(data.get("pm.vbat", 0.0))
            if "radio.rssi" in data:
                try:
                    self.state.rssi = float(data["radio.rssi"])
                except Exception:
                    pass
            if getattr(self, "_last_cb", None) is not None:
                self.state.latency_ms = (now - self._last_cb) * 1000.0
        self._last_cb = now

    def send_setpoint(self, roll: float, pitch: float, yawrate: float, thrust: int):
        if self.cf is not None:
            self.cf.commander.send_setpoint(roll, pitch, yawrate, thrust)

    def power_cycle(self):
        ps = PowerSwitch(self.uri)
        ps.stm_power_down(); time.sleep(2.5); ps.stm_power_up(); time.sleep(2.5)
