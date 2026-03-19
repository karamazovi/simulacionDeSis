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
_mqtt_thread = None
_custom_event = None
_custom_event_handler = None
_stop_flag = False

CUSTOM_EVENT_ID = "FusionMqttBridgeEvent"


def cargar_config():
    """Carga la configuracion desde el archivo JSON."""
    ruta = os.path.join(os.path.dirname(__file__), "FusionMqttBridge.settings.json")
    with open(ruta, encoding="utf-8") as f:
        return json.load(f)


def set_joint(name, rad):
    """Mueve una junta del diseño activo al valor especificado en radianes."""
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
    """Manejador de eventos personalizados para procesar mensajes MQTT en el hilo principal."""

    def __init__(self):
        super().__init__()

    def notify(self, args):
        try:
            event_args = adsk.core.CustomEventArgs.cast(args)
            data = json.loads(event_args.additionalInfo)

            tipo = data.get("tipo")
            body = data.get("body", {})

            if tipo == "joints":
                joints = body.get("joints", {})
                for nombre, valor in joints.items():
                    set_joint(nombre, valor)
                adsk.core.Application.get().activeViewport.refresh()

            elif tipo == "mode":
                modo = body.get("mode")
                if modo == "stop":
                    _ui.messageBox("Modo STOP recibido")

        except Exception:
            if _ui:
                _ui.messageBox(f"Error en evento: {traceback.format_exc()}")


def on_connect(client, userdata, flags, rc, props=None):
    """Callback cuando se conecta al broker MQTT."""
    if rc == 0:
        client.subscribe(_config["temas"]["comando"])
        client.subscribe(_config["temas"]["modo"])
        print(f"Conectado a MQTT y suscrito a topics")


def on_disconnect(client, userdata, rc, props=None):
    """Callback cuando se desconecta del broker MQTT."""
    print(f"Desconectado de MQTT con codigo: {rc}")


def on_message(client, userdata, msg):
    """Callback cuando llega un mensaje MQTT. Dispara un evento personalizado."""
    global _custom_event, _stop_flag

    if _stop_flag or not _custom_event:
        return

    try:
        payload = json.loads(msg.payload.decode())
    except json.JSONDecodeError:
        return

    body = payload.get("body") or payload.get("cuerpo") or {}
    tipo = payload.get("type") or payload.get("tipo")

    event_data = None

    if tipo in ["joint_command", "comando_juntas"]:
        event_data = {"tipo": "joints", "body": {"joints": body.get("joints", {})}}

    elif tipo in ["mode_command", "comando_modo"]:
        event_data = {"tipo": "mode", "body": {"mode": body.get("mode")}}

    if event_data:
        _app.fireCustomEvent(CUSTOM_EVENT_ID, json.dumps(event_data))


def mqtt_loop():
    """Bucle principal de MQTT que corre en un hilo separado."""
    global _mqtt_client, _stop_flag

    backoff = 1
    while not _stop_flag:
        try:
            _mqtt_client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2)
            _mqtt_client.on_connect = on_connect
            _mqtt_client.on_disconnect = on_disconnect
            _mqtt_client.on_message = on_message

            _mqtt_client.connect(_config["mqtt"]["host"], _config["mqtt"]["port"])
            backoff = 1  # reset al conectar con exito

            while not _stop_flag:
                _mqtt_client.loop(timeout=0.1)

        except Exception as e:
            print(f"Error en hilo MQTT: {e}. Reintentando en {backoff}s...")
            if not _stop_flag:
                time.sleep(backoff)
                backoff = min(backoff * 2, 60)
        finally:
            if _mqtt_client:
                try:
                    _mqtt_client.disconnect()
                except Exception:
                    pass


def run(context):
    """Punto de entrada del script de Fusion 360."""
    global _app, _ui, _config, _custom_event, _custom_event_handler, _mqtt_thread, _stop_flag

    try:
        _app = adsk.core.Application.get()
        _ui = _app.userInterface
        _stop_flag = False

        # Cargar configuracion
        _config = cargar_config()

        # Registrar evento personalizado
        _custom_event = _app.registerCustomEvent(CUSTOM_EVENT_ID)
        _custom_event_handler = MqttEventHandler()
        _custom_event.add(_custom_event_handler)

        # Iniciar hilo MQTT
        _mqtt_thread = threading.Thread(target=mqtt_loop, daemon=True)
        _mqtt_thread.start()

        print(f"FusionMqttBridge ACTIVO — {_config['mqtt']['host']}:{_config['mqtt']['port']}")
        print(f"Escuchando: {_config['temas']['comando']} | {_config['temas']['modo']}")
        print("Para detener: vuelve a ejecutar el script.")

        # Mantener el script vivo sin bloquear el hilo principal de Fusion
        while not _stop_flag:
            adsk.doEvents()
            time.sleep(0.1)

    except Exception:
        if _ui:
            _ui.messageBox(f"Error al iniciar: {traceback.format_exc()}")


def stop(context):
    """Funcion de limpieza llamada cuando el script se detiene."""
    global _app, _ui, _custom_event, _custom_event_handler, _mqtt_client, _stop_flag

    try:
        _stop_flag = True

        # Esperar a que el hilo MQTT termine su propio cleanup
        if _mqtt_thread and _mqtt_thread.is_alive():
            _mqtt_thread.join(timeout=3.0)
        _mqtt_client = None

        # Desregistrar evento personalizado
        if _custom_event and _custom_event_handler:
            _custom_event.remove(_custom_event_handler)

        if _custom_event:
            _app.unregisterCustomEvent(CUSTOM_EVENT_ID)
            _custom_event = None

        _custom_event_handler = None

        if _ui:
            _ui.messageBox("FusionMqttBridge detenido.")

    except Exception:
        if _ui:
            _ui.messageBox(f"Error al detener: {traceback.format_exc()}")