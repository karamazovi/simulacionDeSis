"""Configuracion central del control del brazo robotico."""

JOINTS_CONFIG = {
    "JointBase": {
        "min": -180,
        "max": 180,
        "default": 0,
        "label": "Base",
        "color": "#89b4fa",
        # Angulo de referencia respecto a la linea roja: ref = joint + offset.
        "reference_offset": 0.0,
    },
    "JointHombro": {
        "min": -90,
        "max": 90,
        "default": 0,
        "label": "Hombro",
        "color": "#a6e3a1",
        "reference_offset": 0.0,
    },
    "JointCodo": {
        "min": -135,
        "max": 135,
        "default": 0,
        "label": "Codo",
        "color": "#fab387",
        "reference_offset": 0.0,
    },
}

PID_DEFAULTS = {
    "JointBase": {"kp": 2.0, "ki": 0.15, "kd": 0.08},
    "JointHombro": {"kp": 2.5, "ki": 0.2, "kd": 0.09},
    "JointCodo": {"kp": 2.3, "ki": 0.18, "kd": 0.08},
}

PID_LOOP_SECONDS = 0.05

IK_CONFIG = {
    # Longitudes aproximadas de eslabones para control por punto XY (mm).
    "link_1": 300.0,
    "link_2": 200.0,
    "elbow_up": True,
}

MQTT_CONFIG = {
    "enabled": True,
    "host": "127.0.0.1",
    "port": 1883,
    "username": "",
    "password": "",
    "client_id": "fusion-robotarm-control",
    "topic_command": "robotarm/command",
    "topic_pid": "robotarm/pid",
    "topic_mode": "robotarm/mode",
}
