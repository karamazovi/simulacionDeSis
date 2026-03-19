import json
import math
import os
import threading
import time
import traceback

import adsk.core
import adsk.fusion

import paho.mqtt.client as mqtt

# Variables globales
_app = None
_ui = None
_config = {}
_mqtt_client = None
_custom_event = None
_custom_event_handler = None
_handlers = []
_pid_controllers = {}   # {nombre_junta: PidController}
_pid_lock = threading.Lock()

CUSTOM_EVENT_ID = "FusionMqttBridgeEvent"
LOCK_FILE = os.path.join(os.path.dirname(__file__), ".running")


# ── Configuracion ─────────────────────────────────────────────────────────────

def cargar_config():
    ruta = os.path.join(os.path.dirname(__file__), "FusionBridge.settings.json")
    with open(ruta, encoding="utf-8") as f:
        return json.load(f)


# ── Juntas ────────────────────────────────────────────────────────────────────

def set_joint(name, rad):
    try:
        design = _app.activeProduct
        if not design or not hasattr(design, "rootComponent"):
            return False
        if not isinstance(rad, (int, float)) or math.isnan(rad) or math.isinf(rad):
            print(f"Valor invalido para junta {name}: {rad}")
            return False
        for j in design.rootComponent.joints:
            if j.name == name:
                if hasattr(j.jointMotion, "rotationValue"):
                    j.jointMotion.rotationValue = rad
                    return True
        return False
    except Exception as e:
        print(f"Error en set_joint({name}, {rad}): {e}")
        return False


def get_joint(name):
    """Lee la posicion actual de una junta en radianes. Retorna None si falla."""
    try:
        design = _app.activeProduct
        if not design or not hasattr(design, "rootComponent"):
            return None
        for j in design.rootComponent.joints:
            if j.name == name:
                if hasattr(j.jointMotion, "rotationValue"):
                    return j.jointMotion.rotationValue
        return None
    except Exception as e:
        print(f"Error en get_joint({name}): {e}")
        return None


# ── Controlador PID ───────────────────────────────────────────────────────────

class PidController:
    """
    Controlador PID por junta. Corre en su propio hilo.
    Lee rotationValue, calcula correccion y escribe via CustomEvent.
    """

    def __init__(self, nombre, kp, ki, kd, setpoint_rad, dt=0.05,
                 tolerancia_rad=0.01, ciclos_estabilizacion=3):
        self.nombre = nombre
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.setpoint = setpoint_rad
        self.dt = dt
        self.tolerancia = tolerancia_rad
        self.ciclos_estabilizacion = ciclos_estabilizacion

        self._activo = True
        self._integral = 0.0
        self._error_anterior = 0.0
        self._hilo = threading.Thread(target=self._loop, daemon=True)
        self._hilo.start()

    def detener(self):
        self._activo = False

    def _loop(self):
        ciclos_dentro = 0
        print(f"[PID] {self.nombre} iniciado — setpoint={math.degrees(self.setpoint):.1f}°")

        while self._activo and os.path.exists(LOCK_FILE):
            posicion_actual = get_joint(self.nombre)
            if posicion_actual is None:
                time.sleep(self.dt)
                continue

            error = self.setpoint - posicion_actual
            self._integral += error * self.dt
            derivada = (error - self._error_anterior) / self.dt
            self._error_anterior = error

            salida = (self.kp * error) + (self.ki * self._integral) + (self.kd * derivada)
            nueva_posicion = posicion_actual + salida * self.dt

            # Enviar corrección via CustomEvent al hilo principal
            if _custom_event and _app:
                event_data = {"tipo": "joints", "body": {"joints": {self.nombre: nueva_posicion}}}
                try:
                    _app.fireCustomEvent(CUSTOM_EVENT_ID, json.dumps(event_data))
                except Exception:
                    pass

            # Verificar convergencia
            if abs(error) < self.tolerancia:
                ciclos_dentro += 1
                if ciclos_dentro >= self.ciclos_estabilizacion:
                    print(f"[PID] {self.nombre} convergió en {math.degrees(self.setpoint):.1f}°")
                    self._activo = False
                    break
            else:
                ciclos_dentro = 0

            time.sleep(self.dt)

        print(f"[PID] {self.nombre} detenido")


# ── Manejador de eventos ──────────────────────────────────────────────────────

class MqttEventHandler(adsk.core.CustomEventHandler):
    def __init__(self):
        super().__init__()

    def notify(self, args):
        try:
            event_args = adsk.core.CustomEventArgs.cast(args)
            data = json.loads(event_args.additionalInfo)
            tipo = data.get("tipo")
            body = data.get("body", {})

            if tipo == "joints":
                for nombre, valor in body.get("joints", {}).items():
                    set_joint(nombre, valor)
                adsk.core.Application.get().activeViewport.refresh()

            elif tipo == "mode":
                if body.get("mode") == "stop":
                    # Detener todos los PID activos
                    with _pid_lock:
                        for ctrl in _pid_controllers.values():
                            ctrl.detener()
                        _pid_controllers.clear()
                    print("Modo STOP — todos los PID detenidos")

        except Exception:
            print(f"Error en evento: {traceback.format_exc()}")


# ── MQTT callbacks ────────────────────────────────────────────────────────────

def on_connect(client, userdata, flags, rc, props=None):
    if rc == 0:
        client.subscribe(_config["temas"]["comando"])
        client.subscribe(_config["temas"]["modo"])
        client.subscribe(_config["temas"].get("pid", "robotarm/pid"))
        print("MQTT conectado — escuchando topics")


def on_disconnect(client, userdata, rc, props=None):
    print(f"MQTT desconectado (rc={rc})")


def on_message(client, userdata, msg):
    if not _custom_event:
        return
    try:
        payload = json.loads(msg.payload.decode())
    except json.JSONDecodeError:
        return

    tipo = payload.get("type") or payload.get("tipo")
    body = payload.get("body") or payload.get("cuerpo") or {}

    # ── Comando de juntas (movimiento directo) ────────────────────────────────
    if tipo in ["joint_command", "comando_juntas"]:
        # Detener PID de las juntas que se muevan manualmente
        joints = body.get("joints") or body.get("juntas") or {}
        with _pid_lock:
            for nombre in joints:
                if nombre in _pid_controllers:
                    _pid_controllers[nombre].detener()
                    del _pid_controllers[nombre]
        event_data = {"tipo": "joints", "body": {"joints": joints}}
        _app.fireCustomEvent(CUSTOM_EVENT_ID, json.dumps(event_data))

    # ── Comando de modo ───────────────────────────────────────────────────────
    elif tipo in ["mode_command", "comando_modo"]:
        modo = body.get("mode") or body.get("modo")
        _app.fireCustomEvent(CUSTOM_EVENT_ID, json.dumps({"tipo": "mode", "body": {"mode": modo}}))

    # ── Comando PID ───────────────────────────────────────────────────────────
    elif tipo in ["pid_command", "comando_pid"]:
        nombre  = body.get("junta") or body.get("joint")
        enabled = body.get("habilitado", body.get("enabled", True))
        kp      = float(body.get("kp", 2.0))
        ki      = float(body.get("ki", 0.1))
        kd      = float(body.get("kd", 0.05))
        sp_deg  = float(body.get("setpoint", 0.0))
        sp_rad  = math.radians(sp_deg)
        tol_deg = float(body.get("tolerancia", body.get("tolerance_deg", 0.6)))
        tol_rad = math.radians(tol_deg)
        ciclos  = int(body.get("ciclosEstabilizacion", body.get("settle_cycles", 3)))

        with _pid_lock:
            # Detener PID previo de esta junta si existe
            if nombre in _pid_controllers:
                _pid_controllers[nombre].detener()
                del _pid_controllers[nombre]

            if enabled and nombre:
                ctrl = PidController(
                    nombre=nombre,
                    kp=kp, ki=ki, kd=kd,
                    setpoint_rad=sp_rad,
                    tolerancia_rad=tol_rad,
                    ciclos_estabilizacion=ciclos,
                )
                _pid_controllers[nombre] = ctrl
                print(f"[PID] {nombre} configurado: Kp={kp} Ki={ki} Kd={kd} SP={sp_deg}°")


# ── Loop MQTT ─────────────────────────────────────────────────────────────────

def iniciar_mqtt():
    global _mqtt_client
    backoff = 1
    while os.path.exists(LOCK_FILE):
        try:
            _mqtt_client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2)
            _mqtt_client.on_connect = on_connect
            _mqtt_client.on_disconnect = on_disconnect
            _mqtt_client.on_message = on_message
            _mqtt_client.connect(_config["mqtt"]["host"], _config["mqtt"]["port"])
            backoff = 1
            while os.path.exists(LOCK_FILE):
                _mqtt_client.loop(timeout=0.1)
        except Exception as e:
            print(f"Error MQTT: {e}. Reintentando en {backoff}s...")
            if os.path.exists(LOCK_FILE):
                time.sleep(backoff)
                backoff = min(backoff * 2, 60)
        finally:
            if _mqtt_client:
                try:
                    _mqtt_client.disconnect()
                except Exception:
                    pass


# ── Entry points ──────────────────────────────────────────────────────────────

def run(context):
    global _app, _ui, _config, _custom_event, _custom_event_handler, _handlers

    _app = adsk.core.Application.get()
    _ui = _app.userInterface

    try:
        # Toggle: segunda ejecucion detiene el bridge
        if os.path.exists(LOCK_FILE):
            os.remove(LOCK_FILE)
            print("FusionMqttBridge DETENIDO")
            return

        _config = cargar_config()

        _custom_event = _app.registerCustomEvent(CUSTOM_EVENT_ID)
        _custom_event_handler = MqttEventHandler()
        _custom_event.add(_custom_event_handler)
        _handlers.append(_custom_event_handler)

        open(LOCK_FILE, "w").close()
        threading.Thread(target=iniciar_mqtt, daemon=True).start()

        print(f"FusionMqttBridge ACTIVO — {_config['mqtt']['host']}:{_config['mqtt']['port']}")
        print(f"Topics: {_config['temas']['comando']} | {_config['temas']['modo']} | {_config['temas'].get('pid','robotarm/pid')}")
        print("Vuelve a ejecutar el script para detenerlo.")

        while os.path.exists(LOCK_FILE):
            adsk.doEvents()
            time.sleep(0.05)

    except Exception:
        if os.path.exists(LOCK_FILE):
            os.remove(LOCK_FILE)
        if _ui:
            _ui.messageBox(f"Error: {traceback.format_exc()}")
    finally:
        if _custom_event and _custom_event_handler:
            try:
                _custom_event.remove(_custom_event_handler)
                _app.unregisterCustomEvent(CUSTOM_EVENT_ID)
            except Exception:
                pass
        _handlers.clear()
        with _pid_lock:
            for ctrl in _pid_controllers.values():
                ctrl.detener()
            _pid_controllers.clear()


def stop(context):
    if os.path.exists(LOCK_FILE):
        os.remove(LOCK_FILE)
