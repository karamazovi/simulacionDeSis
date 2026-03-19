"""Interfaz de control del brazo robótico para Fusion 360."""

import adsk.core
import adsk.fusion
import traceback
import threading
import math
import time
import queue
import json
import os
import sys

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
if SCRIPT_DIR not in sys.path:
    sys.path.insert(0, SCRIPT_DIR)

MODULE_IMPORT_ERROR = None
try:
    from robot_arm_config import JOINTS_CONFIG, PID_DEFAULTS, PID_LOOP_SECONDS, IK_CONFIG, MQTT_CONFIG
    from robot_arm_pid1 import JointPIDManager
    from robot_arm_kinematics import solve_planar_2link
except Exception:
    MODULE_IMPORT_ERROR = traceback.format_exc()
    JOINTS_CONFIG = {}
    PID_DEFAULTS = {}
    PID_LOOP_SECONDS = 0.05
    IK_CONFIG = {}
    MQTT_CONFIG = {
        "enabled": False,
        "host": "127.0.0.1",
        "port": 1883,
        "username": "",
        "password": "",
        "client_id": "fusion-robotarm-control",
        "topic_command": "robotarm/command",
        "topic_pid": "robotarm/pid",
        "topic_mode": "robotarm/mode",
    }
    JointPIDManager = None
    solve_planar_2link = None

try:
    import serial
    import serial.tools.list_ports
    SERIAL_AVAILABLE = True
except ImportError:
    SERIAL_AVAILABLE = False

try:
    import paho.mqtt.client as mqtt
    MQTT_AVAILABLE = True
except ImportError:
    MQTT_AVAILABLE = False


app = None
ui = None
root_tk = None
tk = None
ttk = None
slider_vars = {}
current_joint_angles = {}

serial_conn = None
serial_thread = None
serial_running = False
mqtt_client = None
mqtt_thread = None
mqtt_running = False
serial_queue = queue.Queue()
safety_last_message = ""
safety_last_time = 0.0

pid_manager = JointPIDManager(JOINTS_CONFIG, PID_DEFAULTS) if JointPIDManager else None
last_pid_ts = None
pid_target_ramps = {}


def _ensure_tk():
    global tk, ttk
    if tk is not None and ttk is not None:
        return True
    try:
        import tkinter as _tk
        from tkinter import ttk as _ttk

        tk = _tk
        ttk = _ttk
        return True
    except Exception as exc:
        if ui:
            ui.messageBox(
                "No fue posible cargar tkinter en el Python de Fusion.\n\n"
                f"Detalle: {exc}\n\n"
                "Si esto falla en tu instalación, la UI debe migrarse a comandos nativos de Fusion.",
                "RobotArmControl - Error de UI",
            )
        return False


def _push_safety_alert(message):
    global safety_last_message, safety_last_time
    now = time.monotonic()
    if message == safety_last_message and (now - safety_last_time) < 1.0:
        return
    safety_last_message = message
    safety_last_time = now
    serial_queue.put(("safety", message))



# Limita el valor de la junta a sus rangos permitidos
def limitarJunta(nombre, valor, fuente="control"):
    cfg = JOINTS_CONFIG[nombre]
    bruto = float(valor)
    limitado = max(cfg["min"], min(cfg["max"], bruto))
    if nombre == "JointBase" and bruto != limitado:
        _push_safety_alert(
            f"Seguridad: Base limitada por {fuente}. Pedido={bruto:.1f}deg, aplicado={limitado:.1f}deg"
        )
    return limitado


def _joint_to_reference(joint_name, joint_deg):
    offset = JOINTS_CONFIG[joint_name].get("reference_offset", 0.0)
    return float(joint_deg) + float(offset)


def _reference_to_joint(joint_name, ref_deg):
    offset = JOINTS_CONFIG[joint_name].get("reference_offset", 0.0)
    return float(ref_deg) - float(offset)


def _clamp_reference_to_limits(joint_name, ref_deg, source="setpoint"):
    joint_deg = _reference_to_joint(joint_name, ref_deg)
    joint_clamped = limitarJunta(joint_name, joint_deg, fuente=source)
    return _joint_to_reference(joint_name, joint_clamped)


def get_joint(name):
    try:
        design = app.activeProduct
        for joint in design.rootComponent.joints:
            if joint.name == name:
                return joint
    except Exception:
        pass
    return None


def update_joint(joint_name, degrees, source="manual"):
    try:
        joint = get_joint(joint_name)
        if joint is None:
            return
        clamped = limitarJunta(joint_name, degrees, fuente=source)
        joint.jointMotion.rotationValue = math.radians(clamped)
        current_joint_angles[joint_name] = clamped
    except Exception:
        pass


def update_joint_from_reference(joint_name, ref_degrees, source="referencia"):
    joint_deg = _reference_to_joint(joint_name, ref_degrees)
    clamped = limitarJunta(joint_name, joint_deg, fuente=source)
    if joint_name in slider_vars:
        slider_vars[joint_name].set(clamped)
    else:
        update_joint(joint_name, clamped, source=source)


def list_all_joints():
    try:
        design = app.activeProduct
        names = [j.name for j in design.rootComponent.joints]
        return "\n".join(names) if names else "(sin uniones)"
    except Exception:
        return "(error al leer uniones)"


def list_serial_ports():
    if not SERIAL_AVAILABLE:
        return []
    return [p.device for p in serial.tools.list_ports.comports()]


def serial_reader_worker(port, baud, joint_names):
    global serial_running, serial_conn
    try:
        serial_conn = serial.Serial(port, baud, timeout=1)
        serial_conn.reset_input_buffer()
        time.sleep(2.0)
        serial_queue.put(("connected", port))

        while serial_running:
            try:
                raw = serial_conn.readline()
                if not raw:
                    continue
                line = raw.decode("ascii", errors="ignore").strip()
                if not line or line.startswith("#"):
                    continue

                serial_queue.put(("line", line))
                parts = line.split(",")
                if len(parts) != len(joint_names):
                    continue

                angles = [float(p) for p in parts]
                clamped_angles = []
                for name, angle in zip(joint_names, angles):
                    clamped_angles.append((name, limitarJunta(name, angle, fuente="serial")))
                serial_queue.put(("angles", clamped_angles))
            except (ValueError, UnicodeDecodeError):
                continue

    except Exception as exc:
        serial_queue.put(("error", str(exc)))
    finally:
        if serial_conn and serial_conn.is_open:
            serial_conn.close()
        serial_conn = None
        serial_queue.put(("stopped", None))


def mqtt_reader_worker():
    global mqtt_running, mqtt_client

    if not MQTT_AVAILABLE:
        serial_queue.put(("mqtt_status", "MQTT no disponible (instala paho-mqtt)"))
        return

    cfg = MQTT_CONFIG
    topic_command = cfg.get("topic_command", "robotarm/command")
    topic_pid = cfg.get("topic_pid", "robotarm/pid")
    topic_mode = cfg.get("topic_mode", "robotarm/mode")

    try:
        mqtt_client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2, client_id=cfg.get("client_id", "fusion-robotarm-control"))
    except Exception:
        mqtt_client = mqtt.Client(client_id=cfg.get("client_id", "fusion-robotarm-control"))

    username = cfg.get("username", "")
    password = cfg.get("password", "")
    if username:
        mqtt_client.username_pw_set(username, password)

    def _on_connect(client, userdata, flags, reason_code, properties=None):
        ok = reason_code == 0
        if ok:
            client.subscribe(topic_command)
            client.subscribe(topic_pid)
            client.subscribe(topic_mode)
            serial_queue.put(("mqtt_status", f"MQTT conectado ({cfg.get('host')}:{cfg.get('port')})"))
        else:
            serial_queue.put(("mqtt_status", f"MQTT error de conexion rc={reason_code}"))

    def _on_disconnect(client, userdata, disconnect_flags, reason_code, properties=None):
        if mqtt_running:
            serial_queue.put(("mqtt_status", "MQTT desconectado"))

    def _on_message(client, userdata, msg):
        decoded = msg.payload.decode("utf-8", errors="ignore")
        payload = None
        try:
            payload = json.loads(decoded)
        except Exception:
            serial_queue.put(("mqtt_status", f"MQTT payload invalido en {msg.topic}"))
            return

        body = payload.get("body") if isinstance(payload, dict) else None
        cmd_type = payload.get("type") if isinstance(payload, dict) else None
        if body is None and isinstance(payload, dict):
            body = payload

        if msg.topic == topic_command or cmd_type == "joint_command":
            joints = body.get("joints") if isinstance(body, dict) else None
            if isinstance(joints, dict):
                serial_queue.put(("mqtt_joint_command", joints))
        elif msg.topic == topic_pid or cmd_type == "pid_command":
            if isinstance(body, dict):
                serial_queue.put(("mqtt_pid_command", body))
        elif msg.topic == topic_mode or cmd_type == "mode_command":
            mode_val = body.get("mode") if isinstance(body, dict) else None
            if mode_val is not None:
                serial_queue.put(("mqtt_mode_command", str(mode_val)))

    mqtt_client.on_connect = _on_connect
    mqtt_client.on_disconnect = _on_disconnect
    mqtt_client.on_message = _on_message

    host = cfg.get("host", "127.0.0.1")
    port = int(cfg.get("port", 1883))
    while mqtt_running:
        try:
            mqtt_client.connect(host, port, keepalive=30)
            mqtt_client.loop_forever(retry_first_connection=True)
        except Exception as exc:
            serial_queue.put(("mqtt_status", f"MQTT error: {exc}"))
            time.sleep(2.0)
        finally:
            try:
                mqtt_client.disconnect()
            except Exception:
                pass


def on_close():
    global root_tk, serial_running, mqtt_running
    serial_running = False
    mqtt_running = False
    if pid_manager:
        pid_manager.set_enabled(False)
    if serial_thread and serial_thread.is_alive():
        serial_thread.join(timeout=1.5)
    if mqtt_client is not None:
        try:
            mqtt_client.disconnect()
        except Exception:
            pass
    if mqtt_thread and mqtt_thread.is_alive():
        mqtt_thread.join(timeout=1.5)
    if root_tk:
        try:
            root_tk.destroy()
        except Exception:
            pass
        root_tk = None


def create_gui_window():
    global root_tk, slider_vars, serial_running, serial_thread, mqtt_running, mqtt_thread, last_pid_ts, pid_target_ramps

    if not _ensure_tk():
        return False

    if pid_manager is None:
        if ui:
            ui.messageBox(
                "No se pudieron cargar los módulos locales del script.\n\n"
                f"Detalle:\n{MODULE_IMPORT_ERROR}",
                "RobotArmControl - Error de importación",
            )
        return False

    try:
        root_tk = tk.Tk()
    except Exception as exc:
        if ui:
            ui.messageBox(f"No se pudo crear la ventana tkinter.\n\n{exc}", "RobotArmControl - Error de UI")
        return False
    root_tk.title("Control Brazo Robótico - Fusion 360")
    root_tk.geometry("620x980")
    root_tk.minsize(560, 860)
    root_tk.resizable(True, True)
    root_tk.configure(bg="#1e1e2e")
    root_tk.protocol("WM_DELETE_WINDOW", on_close)

    tk.Label(
        root_tk,
        text="Control Brazo Robótico",
        bg="#1e1e2e",
        fg="#cdd6f4",
        font=("Segoe UI", 14, "bold"),
    ).pack(pady=(14, 2))

    tk.Label(
        root_tk,
        text="Referencia roja + objetivo por punto + ajuste PID",
        bg="#1e1e2e",
        fg="#6c7086",
        font=("Segoe UI", 9),
    ).pack(pady=(0, 10))

    safety_var = tk.StringVar(value="Seguridad: sin alertas")
    safety_lbl = tk.Label(
        root_tk,
        textvariable=safety_var,
        bg="#1e1e2e",
        fg="#a6e3a1",
        font=("Segoe UI", 8, "bold"),
        anchor="w",
    )
    safety_lbl.pack(fill=tk.X, padx=20, pady=(0, 6))

    safety_btn_row = tk.Frame(root_tk, bg="#1e1e2e")
    safety_btn_row.pack(fill=tk.X, padx=20, pady=(0, 6))

    def _clear_safety_alert():
        global safety_last_message
        safety_last_message = ""
        safety_var.set("Seguridad: sin alertas")
        safety_lbl.config(fg="#a6e3a1")

    tk.Button(
        safety_btn_row,
        text="Limpiar alerta",
        command=_clear_safety_alert,
        bg="#313244",
        fg="#cdd6f4",
        relief=tk.FLAT,
        font=("Segoe UI", 8),
        padx=8,
        pady=3,
        cursor="hand2",
        bd=0,
    ).pack(side=tk.LEFT)

    tk.Frame(root_tk, bg="#313244", height=1).pack(fill=tk.X, padx=20, pady=(0, 8))

    serial_card = tk.Frame(root_tk, bg="#181825", bd=0)
    serial_card.pack(fill=tk.X, padx=20, pady=(0, 8))

    tk.Label(
        serial_card,
        text="  Puerto Serial (Arduino / Emulador)",
        bg="#181825",
        fg="#89b4fa",
        font=("Segoe UI", 9, "bold"),
        anchor="w",
    ).pack(fill=tk.X, padx=8, pady=(8, 4))

    serial_row = tk.Frame(serial_card, bg="#181825")
    serial_row.pack(fill=tk.X, padx=8, pady=(0, 6))

    available_ports = list_serial_ports()
    port_var = tk.StringVar(value=available_ports[0] if available_ports else "COM3")
    port_menu = ttk.Combobox(serial_row, textvariable=port_var, values=available_ports, width=9, state="normal")
    port_menu.pack(side=tk.LEFT, padx=(0, 6))

    baud_var = tk.StringVar(value="9600")
    ttk.Combobox(serial_row, textvariable=baud_var, values=["9600", "115200"], width=8, state="readonly").pack(
        side=tk.LEFT, padx=(0, 8)
    )

    status_var = tk.StringVar(value="Desconectado")
    status_lbl = tk.Label(serial_row, textvariable=status_var, bg="#181825", fg="#f38ba8", font=("Segoe UI", 9, "bold"))
    status_lbl.pack(side=tk.LEFT, padx=(0, 8))

    last_line_var = tk.StringVar(value="Sin datos recibidos")
    mqtt_status_var = tk.StringVar(value="MQTT: deshabilitado")

    def _refresh_ports():
        ports = list_serial_ports()
        port_menu["values"] = ports
        if ports and port_var.get() not in ports:
            port_var.set(ports[0])

    def _toggle_serial():
        global serial_running, serial_thread

        if serial_running:
            serial_running = False
            if serial_thread and serial_thread.is_alive():
                serial_thread.join(timeout=2)
            status_var.set("Desconectado")
            status_lbl.config(fg="#f38ba8")
            connect_btn.config(text="Conectar", bg="#45475a")
            return

        if not SERIAL_AVAILABLE:
            ui.messageBox(
                "pyserial no esta instalado. Ejecuta: pip install pyserial",
                "RobotArmControl - pyserial no encontrado",
            )
            return

        port = port_var.get().strip()
        baud = int(baud_var.get())
        joint_names = list(JOINTS_CONFIG.keys())
        serial_running = True
        status_var.set("Conectando...")
        status_lbl.config(fg="#f9e2af")
        last_line_var.set("Esperando datos...")
        serial_thread = threading.Thread(target=serial_reader_worker, args=(port, baud, joint_names), daemon=True)
        serial_thread.start()

    button_row = tk.Frame(serial_card, bg="#181825")
    button_row.pack(fill=tk.X, padx=8, pady=(0, 8))

    tk.Button(
        button_row,
        text="Refrescar",
        command=_refresh_ports,
        bg="#313244",
        fg="#cdd6f4",
        relief=tk.FLAT,
        font=("Segoe UI", 9),
        padx=10,
        pady=4,
        cursor="hand2",
        bd=0,
    ).pack(side=tk.LEFT, padx=(0, 6))

    connect_btn = tk.Button(
        button_row,
        text="Conectar",
        command=_toggle_serial,
        bg="#45475a",
        fg="#cdd6f4",
        relief=tk.FLAT,
        font=("Segoe UI", 9, "bold"),
        padx=12,
        pady=4,
        cursor="hand2",
        bd=0,
    )
    connect_btn.pack(side=tk.LEFT)

    tk.Label(serial_card, textvariable=last_line_var, bg="#181825", fg="#cdd6f4", font=("Consolas", 8), anchor="w").pack(
        fill=tk.X, padx=8, pady=(0, 6)
    )
    tk.Label(serial_card, textvariable=mqtt_status_var, bg="#181825", fg="#89b4fa", font=("Consolas", 8), anchor="w").pack(
        fill=tk.X, padx=8, pady=(0, 6)
    )

    if MQTT_CONFIG.get("enabled", True):
        if MQTT_AVAILABLE:
            mqtt_running = True
            mqtt_status_var.set(f"MQTT: conectando {MQTT_CONFIG.get('host')}:{MQTT_CONFIG.get('port')}")
            mqtt_thread = threading.Thread(target=mqtt_reader_worker, daemon=True)
            mqtt_thread.start()
        else:
            mqtt_status_var.set("MQTT: paho-mqtt no instalado")
    else:
        mqtt_status_var.set("MQTT: deshabilitado en config")

    reference_card = tk.Frame(root_tk, bg="#181825", bd=0)
    reference_card.pack(fill=tk.X, padx=20, pady=(0, 8))

    tk.Label(
        reference_card,
        text="  Referencia línea roja (Base)",
        bg="#181825",
        fg="#f9e2af",
        font=("Segoe UI", 9, "bold"),
        anchor="w",
    ).pack(fill=tk.X, padx=8, pady=(8, 6))

    def _base_ref_limits():
        cfg = JOINTS_CONFIG["JointBase"]
        offset = float(cfg.get("reference_offset", 0.0))
        return cfg["min"] + offset, cfg["max"] + offset

    base_min_ref, base_max_ref = _base_ref_limits()
    base_ref_var = tk.DoubleVar(value=max(base_min_ref, min(base_max_ref, 0.0)))
    base_offset_var = tk.StringVar(value=str(JOINTS_CONFIG["JointBase"].get("reference_offset", 0.0)))
    base_ref_txt = tk.StringVar(value=f"Objetivo base: {base_ref_var.get():.1f} deg")
    tk.Label(reference_card, textvariable=base_ref_txt, bg="#181825", fg="#cdd6f4", font=("Segoe UI", 9)).pack(
        anchor="w", padx=8, pady=(0, 4)
    )

    def _on_base_ref_change(*_):
        base_ref_txt.set(f"Objetivo base: {base_ref_var.get():.1f} deg")

    base_ref_var.trace_add("write", _on_base_ref_change)
    base_limits_txt = tk.StringVar(value=f"Límites base: {base_min_ref:.1f} a {base_max_ref:.1f} deg")
    tk.Label(reference_card, textvariable=base_limits_txt, bg="#181825", fg="#6c7086", font=("Segoe UI", 8)).pack(
        anchor="w", padx=8, pady=(0, 2)
    )

    base_ref_scale = ttk.Scale(reference_card, from_=base_min_ref, to=base_max_ref, variable=base_ref_var, orient=tk.HORIZONTAL)
    base_ref_scale.pack(
        fill=tk.X, padx=8, pady=(0, 6)
    )

    base_ticks_row = tk.Frame(reference_card, bg="#181825")
    base_ticks_row.pack(fill=tk.X, padx=8, pady=(0, 6))
    base_min_txt = tk.StringVar(value=f"{base_min_ref:.1f} deg")
    base_max_txt = tk.StringVar(value=f"{base_max_ref:.1f} deg")
    tk.Label(base_ticks_row, textvariable=base_min_txt, bg="#181825", fg="#6c7086", font=("Segoe UI", 8)).pack(side=tk.LEFT)
    tk.Label(base_ticks_row, textvariable=base_max_txt, bg="#181825", fg="#6c7086", font=("Segoe UI", 8)).pack(side=tk.RIGHT)

    def _apply_base_reference():
        update_joint_from_reference("JointBase", base_ref_var.get(), source="referencia")

    offset_row = tk.Frame(reference_card, bg="#181825")
    offset_row.pack(fill=tk.X, padx=8, pady=(0, 6))
    tk.Label(offset_row, text="Offset base:", bg="#181825", fg="#cdd6f4", font=("Segoe UI", 8)).pack(side=tk.LEFT)
    tk.Entry(offset_row, textvariable=base_offset_var, width=8).pack(side=tk.LEFT, padx=(6, 8))

    def _apply_base_offset():
        try:
            JOINTS_CONFIG["JointBase"]["reference_offset"] = float(base_offset_var.get())
            new_min_ref, new_max_ref = _base_ref_limits()
            base_ref_scale.configure(from_=new_min_ref, to=new_max_ref)
            base_ref_var.set(max(new_min_ref, min(new_max_ref, base_ref_var.get())))
            base_limits_txt.set(f"Límites base: {new_min_ref:.1f} a {new_max_ref:.1f} deg")
            base_min_txt.set(f"{new_min_ref:.1f} deg")
            base_max_txt.set(f"{new_max_ref:.1f} deg")
            ui.messageBox(
                f"Offset base actualizado a {JOINTS_CONFIG['JointBase']['reference_offset']:.2f}deg",
                "Calibración línea roja",
            )
        except ValueError:
            ui.messageBox("Offset inválido. Usa un número.", "Calibración línea roja")

    tk.Button(
        offset_row,
        text="Aplicar offset",
        command=_apply_base_offset,
        bg="#313244",
        fg="#cdd6f4",
        relief=tk.FLAT,
        font=("Segoe UI", 8),
        padx=8,
        pady=3,
        cursor="hand2",
        bd=0,
    ).pack(side=tk.LEFT)

    tk.Button(
        reference_card,
        text="Ir a ángulo base",
        command=_apply_base_reference,
        bg="#45475a",
        fg="#cdd6f4",
        relief=tk.FLAT,
        font=("Segoe UI", 9, "bold"),
        padx=12,
        pady=4,
        cursor="hand2",
        bd=0,
    ).pack(anchor="w", padx=8, pady=(0, 8))

    ik_card = tk.Frame(root_tk, bg="#181825", bd=0)
    ik_card.pack(fill=tk.X, padx=20, pady=(0, 8))

    tk.Label(
        ik_card,
        text="  Objetivo por punto XY (Hombro + Codo)",
        bg="#181825",
        fg="#a6e3a1",
        font=("Segoe UI", 9, "bold"),
        anchor="w",
    ).pack(fill=tk.X, padx=8, pady=(8, 6))

    x_var = tk.DoubleVar(value=140.0)
    y_var = tk.DoubleVar(value=80.0)

    xy_txt = tk.StringVar(value="X=140.0 mm, Y=80.0 mm")
    tk.Label(ik_card, textvariable=xy_txt, bg="#181825", fg="#cdd6f4", font=("Segoe UI", 9)).pack(anchor="w", padx=8)

    def _xy_refresh(*_):
        xy_txt.set(f"X={x_var.get():.1f} mm, Y={y_var.get():.1f} mm")

    x_var.trace_add("write", _xy_refresh)
    y_var.trace_add("write", _xy_refresh)

    ttk.Scale(ik_card, from_=-220, to=220, variable=x_var, orient=tk.HORIZONTAL).pack(fill=tk.X, padx=8, pady=(4, 2))
    ttk.Scale(ik_card, from_=-220, to=220, variable=y_var, orient=tk.HORIZONTAL).pack(fill=tk.X, padx=8, pady=(2, 6))

    entry_row = tk.Frame(ik_card, bg="#181825")
    entry_row.pack(fill=tk.X, padx=8, pady=(0, 6))
    tk.Label(entry_row, text="X:", bg="#181825", fg="#cdd6f4").pack(side=tk.LEFT)
    x_entry = tk.Entry(entry_row, width=8)
    x_entry.insert(0, "140")
    x_entry.pack(side=tk.LEFT, padx=(4, 10))
    tk.Label(entry_row, text="Y:", bg="#181825", fg="#cdd6f4").pack(side=tk.LEFT)
    y_entry = tk.Entry(entry_row, width=8)
    y_entry.insert(0, "80")
    y_entry.pack(side=tk.LEFT, padx=(4, 0))

    def _sync_entries_to_sliders():
        try:
            x_var.set(float(x_entry.get().strip()))
            y_var.set(float(y_entry.get().strip()))
        except ValueError:
            ui.messageBox("Valores XY inválidos.", "Control por punto")

    def _go_to_xy_target():
        _sync_entries_to_sliders()
        try:
            shoulder_ref, elbow_ref = solve_planar_2link(
                x_var.get(),
                y_var.get(),
                IK_CONFIG["link_1"],
                IK_CONFIG["link_2"],
                elbow_up=IK_CONFIG.get("elbow_up", True),
            )
            update_joint_from_reference("JointHombro", shoulder_ref, source="IK")
            update_joint_from_reference("JointCodo", elbow_ref, source="IK")
        except ValueError as exc:
            ui.messageBox(str(exc), "Control por punto")

    tk.Button(
        ik_card,
        text="Ir al punto XY",
        command=_go_to_xy_target,
        bg="#45475a",
        fg="#cdd6f4",
        relief=tk.FLAT,
        font=("Segoe UI", 9, "bold"),
        padx=12,
        pady=4,
        cursor="hand2",
        bd=0,
    ).pack(anchor="w", padx=8, pady=(0, 8))

    pid_card = tk.Frame(root_tk, bg="#181825", bd=0)
    pid_card.pack(fill=tk.X, padx=20, pady=(0, 8))

    tk.Label(
        pid_card,
        text="  PID por junta (referencia línea roja)",
        bg="#181825",
        fg="#fab387",
        font=("Segoe UI", 9, "bold"),
        anchor="w",
    ).pack(fill=tk.X, padx=8, pady=(8, 6))

    pid_enabled_var = tk.BooleanVar(value=False)
    selected_joint_var = tk.StringVar(value="JointBase")
    kp_var = tk.StringVar(value=str(PID_DEFAULTS["JointBase"]["kp"]))
    ki_var = tk.StringVar(value=str(PID_DEFAULTS["JointBase"]["ki"]))
    kd_var = tk.StringVar(value=str(PID_DEFAULTS["JointBase"]["kd"]))
    target_var = tk.StringVar(value="0.0")
    ramp_time_var = tk.StringVar(value="0.0")
    ramp_cycles_var = tk.StringVar(value="0")
    tol_var = tk.StringVar(value="0.6")
    settle_var = tk.StringVar(value="3")
    pid_runtime_target_var = tk.StringVar(value="Setpoint actual: --")
    pid_ramp_progress_var = tk.DoubleVar(value=0.0)

    def _set_pid_runtime_status(text, color="#f9e2af"):
        pid_runtime_target_var.set(text)
        pid_runtime_target_lbl.config(fg=color)

    def _toggle_pid_enabled():
        pid_manager.set_enabled(pid_enabled_var.get())
        if not pid_enabled_var.get():
            pid_target_ramps.clear()
            _set_pid_runtime_status("Setpoint actual: --", "#6c7086")
            pid_ramp_progress_var.set(0.0)

    top_row = tk.Frame(pid_card, bg="#181825")
    top_row.pack(fill=tk.X, padx=8, pady=(0, 4))
    tk.Checkbutton(
        top_row,
        text="Activar PID",
        variable=pid_enabled_var,
        bg="#181825",
        fg="#cdd6f4",
        selectcolor="#313244",
        activebackground="#181825",
        activeforeground="#cdd6f4",
        command=_toggle_pid_enabled,
    ).pack(side=tk.LEFT)

    ttk.Combobox(
        top_row,
        textvariable=selected_joint_var,
        values=list(JOINTS_CONFIG.keys()),
        width=14,
        state="readonly",
    ).pack(side=tk.LEFT, padx=(8, 0))

    gains_row = tk.Frame(pid_card, bg="#181825")
    gains_row.pack(fill=tk.X, padx=8, pady=(0, 4))
    for label, var in (("Kp", kp_var), ("Ki", ki_var), ("Kd", kd_var), ("Setpoint", target_var), ("Tiempo(s)", ramp_time_var)):
        tk.Label(gains_row, text=label, bg="#181825", fg="#cdd6f4", font=("Segoe UI", 8)).pack(side=tk.LEFT)
        tk.Entry(gains_row, textvariable=var, width=6).pack(side=tk.LEFT, padx=(4, 8))

    settle_row = tk.Frame(pid_card, bg="#181825")
    settle_row.pack(fill=tk.X, padx=8, pady=(0, 4))
    tk.Label(settle_row, text="CiclosSP", bg="#181825", fg="#cdd6f4", font=("Segoe UI", 8)).pack(side=tk.LEFT)
    tk.Entry(settle_row, textvariable=ramp_cycles_var, width=6).pack(side=tk.LEFT, padx=(4, 8))
    tk.Label(settle_row, text="Tol(deg)", bg="#181825", fg="#cdd6f4", font=("Segoe UI", 8)).pack(side=tk.LEFT)
    tk.Entry(settle_row, textvariable=tol_var, width=6).pack(side=tk.LEFT, padx=(4, 8))
    tk.Label(settle_row, text="CiclosSettle", bg="#181825", fg="#cdd6f4", font=("Segoe UI", 8)).pack(side=tk.LEFT)
    tk.Entry(settle_row, textvariable=settle_var, width=6).pack(side=tk.LEFT, padx=(4, 8))

    pid_runtime_target_lbl = tk.Label(
        pid_card,
        textvariable=pid_runtime_target_var,
        bg="#181825",
        fg="#f9e2af",
        font=("Segoe UI", 8, "bold"),
        anchor="w",
    )
    pid_runtime_target_lbl.pack(fill=tk.X, padx=8, pady=(0, 4))

    ttk.Progressbar(
        pid_card,
        variable=pid_ramp_progress_var,
        maximum=100.0,
        mode="determinate",
    ).pack(fill=tk.X, padx=8, pady=(0, 6))

    def _load_joint_pid(*_):
        joint_name = selected_joint_var.get()
        kp, ki, kd = pid_manager.get_gains(joint_name)
        kp_var.set(f"{kp:.3f}")
        ki_var.set(f"{ki:.3f}")
        kd_var.set(f"{kd:.3f}")

    selected_joint_var.trace_add("write", _load_joint_pid)

    def _apply_pid_values(
        joint_name,
        kp,
        ki,
        kd,
        setpoint,
        ramp_time,
        ramp_cycles,
        tolerance,
        settle_cycles,
        enabled=True,
        notify=True,
        source="UI",
    ):
        if joint_name not in JOINTS_CONFIG:
            if notify:
                ui.messageBox(f"Junta inválida: {joint_name}", "PID")
            return

        if ramp_time < 0 or ramp_cycles < 0:
            if notify:
                ui.messageBox("Tiempo o ciclos inválidos para PID.", "PID")
            return

        # Modo PID seleccionado: solo una junta activa por vez para evitar conflictos.
        for other_joint in JOINTS_CONFIG.keys():
            if other_joint != joint_name:
                pid_manager.clear_target(other_joint)
                pid_target_ramps.pop(other_joint, None)

        if not enabled:
            pid_manager.clear_target(joint_name)
            pid_target_ramps.pop(joint_name, None)
            active_targets = any(target is not None for target in pid_manager.targets_ref.values())
            if not active_targets:
                pid_manager.set_enabled(False)
                pid_enabled_var.set(False)
            _set_pid_runtime_status(f"PID desactivado para {joint_name}", "#6c7086")
            pid_ramp_progress_var.set(0.0)
            return

        pid_manager.set_enabled(True)
        pid_enabled_var.set(True)
        pid_manager.set_gains(joint_name, kp, ki, kd)
        pid_manager.set_settling(error_tolerance_deg=tolerance, settle_cycles_required=settle_cycles)
        current_joint = current_joint_angles.get(joint_name, JOINTS_CONFIG[joint_name]["default"])
        current_ref = _joint_to_reference(joint_name, current_joint)
        requested_setpoint = setpoint
        setpoint = _clamp_reference_to_limits(joint_name, setpoint, source=f"PID-{source}")
        if ramp_time > 0 or ramp_cycles > 0:
            if ramp_cycles > 0:
                steps_total = ramp_cycles
                approx_time = steps_total * PID_LOOP_SECONDS
            else:
                steps_total = max(1, int(round(ramp_time / PID_LOOP_SECONDS)))
                approx_time = ramp_time
            delta_ref = (setpoint - current_ref) / float(steps_total)
            pid_target_ramps[joint_name] = {
                "start_ref": current_ref,
                "end_ref": setpoint,
                "steps_total": steps_total,
                "steps_done": 0,
                "delta_ref": delta_ref,
                "last_ref": current_ref,
                "start_ts": time.time(),
                "max_time_s": max(approx_time * 1.5, PID_LOOP_SECONDS * 2.0),
                "mode": "incremental_direct",
            }
            pid_manager.clear_target(joint_name)
            _set_pid_runtime_status(
                f"Setpoint actual ({joint_name}): {current_ref:.2f} deg -> {setpoint:.2f} deg en {steps_total} pasos (~{approx_time:.2f}s)",
                "#f9e2af",
            )
            pid_ramp_progress_var.set(0.0)
        else:
            pid_target_ramps.pop(joint_name, None)
            pid_manager.set_target(joint_name, setpoint)
            _set_pid_runtime_status(f"Setpoint actual ({joint_name}): {setpoint:.2f} deg", "#89b4fa")
            pid_ramp_progress_var.set(100.0)

        if notify:
            ui.messageBox(
                f"PID aplicado a {joint_name}.\n"
                f"Kp={kp:.3f}, Ki={ki:.3f}, Kd={kd:.3f}, Setpoint={setpoint:.2f}deg, Tiempo={ramp_time:.2f}s, CiclosSP={ramp_cycles}\n"
                f"Tol={tolerance:.2f}deg, CiclosSettle={settle_cycles}\n"
                f"Setpoint pedido={requested_setpoint:.2f}deg, aplicado={setpoint:.2f}deg\n"
                "Con CiclosSP>0: moverá en pasos incrementales directos (sin oscilación de ida y vuelta).\n"
                "Al completar pasos/tiempo, se detiene la rampa y queda en el valor objetivo.",
                "PID actualizado",
            )

    def _apply_pid_settings():
        joint_name = selected_joint_var.get()
        try:
            kp = float(kp_var.get())
            ki = float(ki_var.get())
            kd = float(kd_var.get())
            setpoint = float(target_var.get())
            ramp_time = float(ramp_time_var.get())
            ramp_cycles = int(ramp_cycles_var.get())
            tolerance = float(tol_var.get())
            settle_cycles = int(settle_var.get())
        except ValueError:
            ui.messageBox("PID inválido: usa números en Kp/Ki/Kd/Setpoint/Tiempo/CiclosSP/Tol/CiclosSettle.", "PID")
            return

        _apply_pid_values(
            joint_name=joint_name,
            kp=kp,
            ki=ki,
            kd=kd,
            setpoint=setpoint,
            ramp_time=ramp_time,
            ramp_cycles=ramp_cycles,
            tolerance=tolerance,
            settle_cycles=settle_cycles,
            enabled=pid_enabled_var.get(),
            notify=True,
            source="UI",
        )

    def _show_pid_strategy():
        ui.messageBox(
            "Estrategia recomendada para mejorar PID:\n\n"
            "1) Arranca con Ki=0 y Kd=0.\n"
            "2) Sube Kp hasta que oscile suave alrededor del objetivo.\n"
            "3) Agrega Kd para amortiguar sobreimpulso.\n"
            "4) Agrega Ki poco a poco para eliminar error en estado estable.\n"
            "5) Si vibra: baja Kp 10-20% o sube Kd.\n"
            "6) Si tarda mucho: sube Kp o Ki ligeramente.\n\n"
            "Prueba con escalones de 10deg y registra tiempo de establecimiento.",
            "Estrategia de ajuste PID",
        )

    pid_btn_row = tk.Frame(pid_card, bg="#181825")
    pid_btn_row.pack(fill=tk.X, padx=8, pady=(0, 8))

    tk.Button(
        pid_btn_row,
        text="Aplicar PID",
        command=_apply_pid_settings,
        bg="#45475a",
        fg="#cdd6f4",
        relief=tk.FLAT,
        font=("Segoe UI", 9, "bold"),
        padx=12,
        pady=4,
        cursor="hand2",
        bd=0,
    ).pack(side=tk.LEFT, padx=(0, 6))

    tk.Button(
        pid_btn_row,
        text="Estrategia",
        command=_show_pid_strategy,
        bg="#313244",
        fg="#cdd6f4",
        relief=tk.FLAT,
        font=("Segoe UI", 9),
        padx=10,
        pady=4,
        cursor="hand2",
        bd=0,
    ).pack(side=tk.LEFT)

    tk.Frame(root_tk, bg="#313244", height=1).pack(fill=tk.X, padx=20, pady=(0, 8))

    main_frame = tk.Frame(root_tk, bg="#1e1e2e")
    main_frame.pack(fill=tk.BOTH, expand=True, padx=20)

    for joint_name, cfg in JOINTS_CONFIG.items():
        card = tk.Frame(main_frame, bg="#313244")
        card.pack(fill=tk.X, pady=4)
        inner = tk.Frame(card, bg="#313244")
        inner.pack(fill=tk.X, padx=14, pady=10)

        var = tk.DoubleVar(value=cfg["default"])
        val_str = tk.StringVar(value=f"{cfg['default']:.1f}deg")
        slider_vars[joint_name] = var
        current_joint_angles[joint_name] = cfg["default"]

        header = tk.Frame(inner, bg="#313244")
        header.pack(fill=tk.X)
        tk.Label(header, text=f"{cfg['label']}", bg="#313244", fg=cfg["color"], font=("Segoe UI", 10, "bold")).pack(side=tk.LEFT)
        tk.Label(header, textvariable=val_str, bg="#313244", fg="#cdd6f4", font=("Segoe UI", 10)).pack(side=tk.RIGHT)

        def make_cb(jname=joint_name, v=var, vs=val_str):
            def cb(*_):
                val = v.get()
                vs.set(f"{val:.1f}deg")
                update_joint(jname, val)

            return cb

        var.trace_add("write", make_cb())
        ttk.Scale(inner, from_=cfg["min"], to=cfg["max"], variable=var, orient=tk.HORIZONTAL).pack(fill=tk.X, pady=(6, 2))

        rf = tk.Frame(inner, bg="#313244")
        rf.pack(fill=tk.X)
        tk.Label(rf, text=f"{cfg['min']}deg", bg="#313244", fg="#6c7086", font=("Segoe UI", 8)).pack(side=tk.LEFT)
        tk.Label(rf, text=f"{cfg['max']}deg", bg="#313244", fg="#6c7086", font=("Segoe UI", 8)).pack(side=tk.RIGHT)

    btn_frame = tk.Frame(root_tk, bg="#1e1e2e")
    btn_frame.pack(pady=12)

    def reset_all():
        for jname, cfg in JOINTS_CONFIG.items():
            slider_vars[jname].set(cfg["default"])
        if pid_manager:
            pid_manager.reset_all()
        pid_target_ramps.clear()
        _set_pid_runtime_status("Setpoint actual: --", "#6c7086")
        pid_ramp_progress_var.set(0.0)

    def show_joints():
        ui.messageBox(f"Uniones del diseño activo:\n\n{list_all_joints()}", "Uniones")

    tk.Button(
        btn_frame,
        text="Reset",
        command=reset_all,
        bg="#45475a",
        fg="#cdd6f4",
        relief=tk.FLAT,
        font=("Segoe UI", 10),
        padx=14,
        pady=6,
        cursor="hand2",
        bd=0,
    ).pack(side=tk.LEFT, padx=5)

    tk.Button(
        btn_frame,
        text="Ver uniones",
        command=show_joints,
        bg="#45475a",
        fg="#cdd6f4",
        relief=tk.FLAT,
        font=("Segoe UI", 10),
        padx=14,
        pady=6,
        cursor="hand2",
        bd=0,
    ).pack(side=tk.LEFT, padx=5)

    tk.Button(
        btn_frame,
        text="Cerrar",
        command=on_close,
        bg="#f38ba8",
        fg="#1e1e2e",
        relief=tk.FLAT,
        font=("Segoe UI", 10, "bold"),
        padx=14,
        pady=6,
        cursor="hand2",
        bd=0,
    ).pack(side=tk.LEFT, padx=5)

    def _poll_and_control():
        global last_pid_ts

        while not serial_queue.empty():
            event, payload = serial_queue.get_nowait()
            if event == "connected":
                status_var.set(payload)
                status_lbl.config(fg="#a6e3a1")
                connect_btn.config(text="Desconectar", bg="#f38ba8")
            elif event == "line":
                last_line_var.set(payload)
            elif event == "mqtt_status":
                mqtt_status_var.set(f"MQTT: {payload}")
            elif event == "angles":
                for name, angle in payload:
                    if name not in slider_vars:
                        continue

                    # Evita conflicto: si una junta esta bajo PID/rampa activa, no sobrescribirla con serial.
                    pid_target_active = False
                    if pid_manager and pid_manager.enabled:
                        pid_target_active = pid_manager.targets_ref.get(name) is not None
                    if name in pid_target_ramps or pid_target_active:
                        continue

                    slider_vars[name].set(angle)
            elif event == "mqtt_joint_command":
                if isinstance(payload, dict):
                    for name, angle in payload.items():
                        if name not in JOINTS_CONFIG:
                            continue
                        try:
                            update_joint(name, float(angle), source="mqtt")
                            if name in slider_vars:
                                slider_vars[name].set(current_joint_angles.get(name, JOINTS_CONFIG[name]["default"]))
                        except Exception:
                            continue
            elif event == "mqtt_pid_command":
                if isinstance(payload, dict):
                    joint_name = str(payload.get("joint", "JointBase"))
                    if joint_name in JOINTS_CONFIG:
                        selected_joint_var.set(joint_name)
                        kp = float(payload.get("kp", PID_DEFAULTS[joint_name]["kp"]))
                        ki = float(payload.get("ki", PID_DEFAULTS[joint_name]["ki"]))
                        kd = float(payload.get("kd", PID_DEFAULTS[joint_name]["kd"]))
                        setpoint = float(payload.get("setpoint", 0.0))
                        ramp_time = float(payload.get("ramp_time_s", 0.0))
                        ramp_cycles = int(payload.get("ramp_cycles", 0))
                        tolerance = float(payload.get("tolerance_deg", 0.6))
                        settle_cycles = int(payload.get("settle_cycles", 3))
                        enabled = bool(payload.get("enabled", True))

                        kp_var.set(f"{kp:.3f}")
                        ki_var.set(f"{ki:.3f}")
                        kd_var.set(f"{kd:.3f}")
                        target_var.set(f"{setpoint:.3f}")
                        ramp_time_var.set(f"{ramp_time:.3f}")
                        ramp_cycles_var.set(str(ramp_cycles))
                        tol_var.set(f"{tolerance:.3f}")
                        settle_var.set(str(settle_cycles))

                        _apply_pid_values(
                            joint_name=joint_name,
                            kp=kp,
                            ki=ki,
                            kd=kd,
                            setpoint=setpoint,
                            ramp_time=ramp_time,
                            ramp_cycles=ramp_cycles,
                            tolerance=tolerance,
                            settle_cycles=settle_cycles,
                            enabled=enabled,
                            notify=False,
                            source="MQTT",
                        )
            elif event == "mqtt_mode_command":
                mode_cmd = str(payload).strip().lower()
                if mode_cmd == "stop":
                    pid_target_ramps.clear()
                    if pid_manager:
                        pid_manager.set_enabled(False)
                        pid_manager.reset_all()
                    pid_enabled_var.set(False)
                    _set_pid_runtime_status("Modo STOP recibido por MQTT", "#f38ba8")
            elif event == "error":
                status_var.set("Error COM")
                status_lbl.config(fg="#f38ba8")
                connect_btn.config(text="Conectar", bg="#45475a")
                ui.messageBox(f"Error serial:\n{payload}", "RobotArmControl - Error serial")
            elif event == "safety":
                safety_var.set(payload)
                safety_lbl.config(fg="#f38ba8")
            elif event == "stopped" and not serial_running:
                status_var.set("Desconectado")
                status_lbl.config(fg="#f38ba8")
                connect_btn.config(text="Conectar", bg="#45475a")

        now = time.time()
        if last_pid_ts is None:
            last_pid_ts = now
        dt = now - last_pid_ts
        if dt >= PID_LOOP_SECONDS:
            last_pid_ts = now

            # Trayectoria de setpoint: mueve referencia objetivo de forma gradual en el tiempo indicado.
            ramp_joints = set()
            for joint_name, ramp in list(pid_target_ramps.items()):
                ramp_joints.add(joint_name)
                steps_total = max(1, int(ramp.get("steps_total", 1)))
                next_step = min(steps_total, int(ramp.get("steps_done", 0)) + 1)
                target_ref = ramp["start_ref"] + (ramp["delta_ref"] * next_step)
                if next_step >= steps_total:
                    target_ref = ramp["end_ref"]

                # Monotono incremental: evita devolverse durante los pasos salvo necesidad por limites.
                last_ref = float(ramp.get("last_ref", ramp["start_ref"]))
                if ramp["delta_ref"] >= 0:
                    target_ref = max(last_ref, target_ref)
                    target_ref = min(target_ref, ramp["end_ref"])
                else:
                    target_ref = min(last_ref, target_ref)
                    target_ref = max(target_ref, ramp["end_ref"])

                # Aplica limites de junta tambien en referencia (incluye base).
                target_ref = _clamp_reference_to_limits(joint_name, target_ref, source="rampa-setpoint")
                ramp["last_ref"] = target_ref

                elapsed = now - float(ramp.get("start_ts", now))
                timed_out = elapsed >= float(ramp.get("max_time_s", 0.0))

                # Durante la rampa, aplica trayectoria temporal directa para garantizar avance al objetivo en tiempo x.
                update_joint_from_reference(joint_name, target_ref, source="rampa-setpoint")
                remaining_steps = max(0, steps_total - next_step)
                _set_pid_runtime_status(
                    f"Setpoint actual ({joint_name}): {target_ref:.2f} deg (paso {next_step}/{steps_total}, faltan {remaining_steps})",
                    "#f9e2af",
                )
                pid_ramp_progress_var.set((next_step / float(steps_total)) * 100.0)
                ramp["steps_done"] = next_step
                if next_step >= steps_total or timed_out:
                    final_ref = _clamp_reference_to_limits(joint_name, ramp["end_ref"], source="rampa-setpoint-final")
                    # Cierre exacto del objetivo para evitar residuo de redondeo y bucles de aproximacion.
                    update_joint_from_reference(joint_name, final_ref, source="rampa-setpoint-final")

                    # Al completar N pasos/tiempo, detener objetivo PID para que no siga iterando en bucle.
                    pid_manager.clear_target(joint_name)
                    pid_target_ramps.pop(joint_name, None)
                    _set_pid_runtime_status(
                        f"Setpoint actual ({joint_name}): {final_ref:.2f} deg (rampa completada)",
                        "#a6e3a1",
                    )
                    pid_ramp_progress_var.set(100.0)

            current_refs = {}
            for joint_name, joint_deg in current_joint_angles.items():
                current_refs[joint_name] = _joint_to_reference(joint_name, joint_deg)

            outputs = pid_manager.step(current_refs, dt)
            for joint_name, ref_target in outputs.items():
                if joint_name in ramp_joints:
                    continue
                update_joint_from_reference(joint_name, ref_target, source="PID")

            # Si no hay objetivos ni rampas activas, apagar PID para evitar iteración innecesaria.
            active_targets = any(target is not None for target in pid_manager.targets_ref.values())
            if pid_manager.enabled and (not pid_target_ramps) and (not active_targets):
                pid_manager.set_enabled(False)
                pid_enabled_var.set(False)
                _set_pid_runtime_status("Setpoint actual: finalizado (PID detenido)", "#a6e3a1")

        if root_tk is not None:
            root_tk.after(40, _poll_and_control)

    _poll_and_control()
    return True


def run(context):
    global app, ui

    try:
        app = adsk.core.Application.get()
        ui = app.userInterface

        if MODULE_IMPORT_ERROR:
            ui.messageBox(
                "El script no pudo cargar sus dependencias locales.\n\n"
                f"Detalle:\n{MODULE_IMPORT_ERROR}",
                "RobotArmControl - Error de importación",
            )
            return

        if not app.activeProduct or not isinstance(app.activeProduct, adsk.fusion.Design):
            ui.messageBox(
                "No se encontró un diseño activo. Abre un ensamblaje antes de ejecutar el script.",
                "RobotArmControl",
            )
            return

        if not create_gui_window():
            return

        while root_tk is not None:
            try:
                root_tk.update()
            except Exception:
                break
            adsk.doEvents()

    except Exception:
        if ui:
            ui.messageBox(f"Error inesperado:\n\n{traceback.format_exc()}", "RobotArmControl - Error")
    finally:
        global serial_running, mqtt_running
        serial_running = False
        mqtt_running = False
        if pid_manager:
            pid_manager.set_enabled(False)
        if serial_thread and serial_thread.is_alive():
            serial_thread.join(timeout=1.5)
        if mqtt_client is not None:
            try:
                mqtt_client.disconnect()
            except Exception:
                pass
        if mqtt_thread and mqtt_thread.is_alive():
            mqtt_thread.join(timeout=1.5)
        if root_tk is not None:
            try:
                root_tk.destroy()
            except Exception:
                pass
