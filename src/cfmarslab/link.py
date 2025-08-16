# src/cfmarslab/link.py
from __future__ import annotations
import time
from typing import Optional, Iterable

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.utils.power_switch import PowerSwitch

from .models import SharedState
from .config import Controls

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
        # platform/arming detection
        self.is_bolt: bool = False
        self._param_toc: set[str] | None = None
        self._arm_param: str | None = None

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

    # --- parameter helpers ---
    @property
    def arm_param(self) -> str | None:
        return self._arm_param

    def _build_param_toc(self) -> set[str]:
        toc: set[str] = set()
        if self.cf is None:
            return toc
        try:
            for grp, params in getattr(self.cf.param.toc, "toc", {}).items():
                for name in params:
                    toc.add(f"{grp}.{name}")
        except Exception:
            pass
        self._param_toc = toc
        return toc

    def find_param(self, names: Iterable[str]) -> str | None:
        toc = self._param_toc or self._build_param_toc()
        for name in names:
            if name in toc:
                return name
        return None

    def get_bool_param(self, name: str, default: bool = False) -> bool:
        if self.cf is None:
            return default
        for _ in range(3):
            try:
                val = self.cf.param.get_value(name)
                s = str(val).strip().lower()
                if s in ("1", "true", "t", "yes", "on"):
                    return True
                if s in ("0", "false", "f", "no", "off"):
                    return False
                return bool(int(s))
            except Exception:
                time.sleep(0.05)
        return default

    def set_bool_param(self, name: str, value: bool) -> bool:
        if self.cf is None:
            return False
        target = "1" if value else "0"
        for _ in range(3):
            try:
                self.cf.param.set_value(name, target)
                time.sleep(0.05)
                if self.get_bool_param(name, not value) == value:
                    return True
            except Exception:
                pass
            time.sleep(0.05)
        return False

    def detect_platform_and_arm_param(self) -> None:
        cfg = Controls()
        toc = self._build_param_toc()
        # platform detection
        self.is_bolt = False
        if cfg.PLATFORM_HINT == "bolt":
            self.is_bolt = True
        elif "system.isBolt" in toc:
            self.is_bolt = self.get_bool_param("system.isBolt", False)
        elif "system.board" in toc:
            try:
                val = self.cf.param.get_value("system.board")
                self.is_bolt = str(val).strip().lower() == "bolt"
            except Exception:
                pass
        elif any(k.startswith("bolt.") for k in toc):
            self.is_bolt = True

        # arming parameter selection
        self._arm_param = self.find_param(cfg.ARM_PARAM_CANDIDATES)

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
