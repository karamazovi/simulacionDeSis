import os
import json
from pathlib import Path


def as_bool(value: str, default: bool = False) -> bool:
    if value is None:
        return default
    return value.strip().lower() in {"1", "true", "yes", "on"}


class Settings:
    _base = Path(__file__).resolve().parent
    _settings_file = Path(os.getenv("MQTT_WEB_SETTINGS_FILE", _base / "settings.json"))
    _json = {}
    if _settings_file.exists():
        with _settings_file.open("r", encoding="utf-8") as f:
            _json = json.load(f)

    _dashboard = _json.get("dashboard") or _json.get("flask", {})
    DASHBOARD_HOST = os.getenv("DASHBOARD_HOST", _dashboard.get("host", "127.0.0.1"))
    DASHBOARD_PORT = int(os.getenv("DASHBOARD_PORT", str(_dashboard.get("port", 8501))))
    DASHBOARD_DEBUG = as_bool(os.getenv("DASHBOARD_DEBUG", str(_dashboard.get("debug", False))), False)

    # Backward-compat names for existing scripts that still read FLASK_*.
    FLASK_HOST = DASHBOARD_HOST
    FLASK_PORT = DASHBOARD_PORT
    FLASK_DEBUG = DASHBOARD_DEBUG

    MQTT_HOST = os.getenv("MQTT_HOST", _json.get("mqtt", {}).get("host", "localhost"))
    MQTT_PORT = int(os.getenv("MQTT_PORT", str(_json.get("mqtt", {}).get("port", 1883))))
    MQTT_USERNAME = os.getenv("MQTT_USERNAME", _json.get("mqtt", {}).get("username", ""))
    MQTT_PASSWORD = os.getenv("MQTT_PASSWORD", _json.get("mqtt", {}).get("password", ""))
    MQTT_CLIENT_ID = os.getenv("MQTT_CLIENT_ID", _json.get("mqtt", {}).get("client_id", "robotarm-web-controller"))

    TOPIC_COMMAND = os.getenv("MQTT_TOPIC_COMMAND", _json.get("topics", {}).get("command", "robotarm/command"))
    TOPIC_PID = os.getenv("MQTT_TOPIC_PID", _json.get("topics", {}).get("pid", "robotarm/pid"))
    TOPIC_MODE = os.getenv("MQTT_TOPIC_MODE", _json.get("topics", {}).get("mode", "robotarm/mode"))
    TOPIC_TELEMETRY = os.getenv("MQTT_TOPIC_TELEMETRY", _json.get("topics", {}).get("telemetry", "robotarm/telemetry"))
    TOPIC_ACK = os.getenv("MQTT_TOPIC_ACK", _json.get("topics", {}).get("ack", "robotarm/ack"))


JOINT_LIMITS = {
    "JointBase": {"min": -182.0, "max": -2.0, "default": -2.0},
    "JointHombro": {"min": -90.0, "max": 90.0, "default": 0.0},
    "JointCodo": {"min": -135.0, "max": 135.0, "default": 0.0},
}

PID_DEFAULTS = {
    "JointBase": {"kp": 2.0, "ki": 0.15, "kd": 0.08},
    "JointHombro": {"kp": 2.5, "ki": 0.2, "kd": 0.09},
    "JointCodo": {"kp": 2.3, "ki": 0.18, "kd": 0.08},
}
