import json
import sys
from pathlib import Path

# Ensure the src directory is on the Python path
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))

from cfmarslab.config import AppConfig, load_config, save_config  # type: ignore


def test_load_config_returns_default(tmp_path, monkeypatch):
    monkeypatch.setattr("cfmarslab.config.CONFIG_DIR", tmp_path)
    config_path = tmp_path / "config.json"
    monkeypatch.setattr("cfmarslab.config.CONFIG_PATH", config_path)

    # Missing file should yield default configuration
    cfg = load_config()
    assert cfg == AppConfig.default()

    # Corrupted JSON should also yield default configuration
    config_path.write_text("{bad json}", encoding="utf-8")
    cfg = load_config()
    assert cfg == AppConfig.default()

    # Clean up
    config_path.unlink(missing_ok=True)


def test_save_and_load_config(tmp_path, monkeypatch):
    monkeypatch.setattr("cfmarslab.config.CONFIG_DIR", tmp_path)
    config_path = tmp_path / "config.json"
    monkeypatch.setattr("cfmarslab.config.CONFIG_PATH", config_path)

    cfg = AppConfig(recent_uris=["radio://1/1/2M/test"], auto_reconnect=False)
    save_config(cfg)

    # Ensure the file was written correctly
    assert json.loads(config_path.read_text(encoding="utf-8")) == {
        "recent_uris": ["radio://1/1/2M/test"],
        "auto_reconnect": False,
    }

    # Loading should return the same configuration
    loaded = load_config()
    assert loaded == cfg

    # Clean up
    config_path.unlink(missing_ok=True)
