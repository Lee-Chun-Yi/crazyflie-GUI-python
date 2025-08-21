from __future__ import annotations
from dataclasses import dataclass, asdict
from pathlib import Path
import json

CONFIG_DIR = Path.home() / ".crazygui"
CONFIG_PATH = CONFIG_DIR / "config.json"

@dataclass(frozen=True)
class Rates:
    UDP_HZ: int = 100          # UDP 接收節拍
    SETPOINT_HZ: int = 100     # RPYT 發送節拍
    LOG_UI_HZ: int = 15        # UI 更新率

@dataclass(frozen=True)
class Safety:
    VBAT_BLOCK: float = 3.7    # 低於此電壓禁止起飛
    VBAT_AUTO_LAND: float = 3.5

@dataclass(frozen=True)
class Controls:
    ARM_PARAM_CANDIDATES: tuple[str, ...] = (
        "stabilizer.armed",
        "ctrlCmd.armed",
        "motorPowerSet.enable",
    )
    PLATFORM_HINT: str | None = None  # "bolt" / None


@dataclass(frozen=True)
class RT:
    FIFO_PRIORITY: int = 50    # Linux only
    SPIN_NS: int = 150_000     # busy-wait window (nanoseconds)
    PIN_CPU: int | None = None # set e.g. 2 to bind to core 2

@dataclass(frozen=True)
class Vicon:
    PORT: int = 8889
    # Preferred format; thread will still auto-fallback by packet length:
    PREFERRED: str = ">6d"     # big-endian 6 doubles
    # Valid alternatives: ">6f"

@dataclass
class AppConfig:
    recent_uris: list[str]
    auto_reconnect: bool

    @staticmethod
    def default() -> "AppConfig":
        return AppConfig(
            recent_uris=["radio://0/99/2M/E7E7E7E7E7"],
            auto_reconnect=True,
        )

def load_config() -> AppConfig:
    try:
        CONFIG_DIR.mkdir(parents=True, exist_ok=True)
        if CONFIG_PATH.exists():
            data = json.loads(CONFIG_PATH.read_text(encoding="utf-8"))
            return AppConfig(
                recent_uris=list(data.get("recent_uris", [])) or AppConfig.default().recent_uris,
                auto_reconnect=bool(data.get("auto_reconnect", True)),
            )
    except Exception:
        pass
    return AppConfig.default()

def save_config(cfg: AppConfig) -> None:
    try:
        CONFIG_DIR.mkdir(parents=True, exist_ok=True)
        CONFIG_PATH.write_text(json.dumps(asdict(cfg), indent=2), encoding="utf-8")
    except Exception:
        pass
