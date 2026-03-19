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
_handlers = []  # evitar garbage collection de handlers

CUSTOM_EVENT_ID = "FusionMqttBridgeEvent"
LOCK_FILE = os.path.join(os.path.dirname(__file__), ".running")


def cargar_config():
    ruta = os.path.join(os.path.dirname(__file__), "FusionMqttBridge.settings.json")
    with open(ruta, encoding="utf-8") as f:
        return json.load(f)


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
                    print("Modo STOP recibido")

        except Exception:
            print(f"Error en evento MQTT: {traceback.format_exc()}")


def on_connect(client, userdata, flags, rc, props=None):
    if rc == 0:
        client.subscribe(_config["temas"]["comando"])
        client.subscribe(_config["temas"]["modo"])
        print(f"MQTT conectado — escuchando topics")


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
    event_data = None

    if tipo in ["joint_command", "comando_juntas"]:
        event_data = {"tipo": "joints", "body": {"joints": body.get("joints", {})}}
    elif tipo in ["mode_command", "comando_modo"]:
        event_data = {"tipo": "mode", "body": {"mode": body.get("mode")}}

    if event_data:
        _app.fireCustomEvent(CUSTOM_EVENT_ID, json.dumps(event_data))


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


def run(context):
    global _app, _ui, _config, _custom_event, _custom_event_handler, _handlers

    _app = adsk.core.Application.get()
    _ui = _app.userInterface

    try:
        # ── MODO TOGGLE: segunda ejecucion detiene el bridge ──────────────────
        if os.path.exists(LOCK_FILE):
            os.remove(LOCK_FILE)
            print("FusionMqttBridge DETENIDO")
            return

        # ── INICIO ────────────────────────────────────────────────────────────
        _config = cargar_config()

        # Registrar evento personalizado
        _custom_event = _app.registerCustomEvent(CUSTOM_EVENT_ID)
        _custom_event_handler = MqttEventHandler()
        _custom_event.add(_custom_event_handler)
        _handlers.append(_custom_event_handler)  # evitar GC

        # Crear lock file y arrancar hilo MQTT
        open(LOCK_FILE, "w").close()
        threading.Thread(target=iniciar_mqtt, daemon=True).start()

        print(f"FusionMqttBridge ACTIVO — {_config['mqtt']['host']}:{_config['mqtt']['port']}")
        print(f"Topics: {_config['temas']['comando']} | {_config['temas']['modo']}")
        print("Vuelve a ejecutar el script para detenerlo.")

        # Loop ligero: mantiene run() vivo para que stop() no limpie todo
        while os.path.exists(LOCK_FILE):
            adsk.doEvents()
            time.sleep(0.05)

    except Exception:
        if os.path.exists(LOCK_FILE):
            os.remove(LOCK_FILE)
        if _ui:
            _ui.messageBox(f"Error: {traceback.format_exc()}")
    finally:
        # Limpiar evento personalizado
        if _custom_event and _custom_event_handler:
            try:
                _custom_event.remove(_custom_event_handler)
                _app.unregisterCustomEvent(CUSTOM_EVENT_ID)
            except Exception:
                pass
        _handlers.clear()


def stop(context):
    # Asegurar limpieza si Fusion fuerza el stop
    if os.path.exists(LOCK_FILE):
        os.remove(LOCK_FILE)
