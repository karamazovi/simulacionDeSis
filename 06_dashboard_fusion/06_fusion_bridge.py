"""
EJEMPLO 06 - Script Fusion 360
Recibe comandos del dashboard (dashboard.py) via MQTT y mueve las juntas.
"""
import adsk.core, adsk.fusion
import json, math, threading, time, traceback
import paho.mqtt.client as mqtt

BROKER   = "127.0.0.1"
PUERTO   = 1883
TOPIC_CMD  = "robotarm/command"
TOPIC_MODE = "robotarm/mode"

_app       = None
_corriendo = True
_evento    = None
_handler   = None
EVENT_ID   = "Ej06Event"


class Manejador(adsk.core.CustomEventHandler):
    def __init__(self):
        super().__init__()

    def notify(self, args):
        try:
            datos = json.loads(adsk.core.CustomEventArgs.cast(args).additionalInfo)
            design = _app.activeProduct
            for nombre, rad in datos.get("joints", {}).items():
                for j in design.rootComponent.joints:
                    if j.name == nombre:
                        j.jointMotion.rotationValue = rad
            _app.activeViewport.refresh()
        except Exception:
            print(traceback.format_exc())


def hilo_mqtt():
    def al_conectar(client, userdata, flags, rc, props=None):
        if rc == 0:
            client.subscribe(TOPIC_CMD)
            client.subscribe(TOPIC_MODE)
            print("MQTT conectado")

    def al_recibir(client, userdata, msg):
        if not _corriendo or not _evento:
            return
        try:
            payload = json.loads(msg.payload.decode())
            tipo  = payload.get("type", "")
            body  = payload.get("body", {})
            if tipo == "joint_command":
                _app.fireCustomEvent(EVENT_ID, json.dumps(body.get("joints", {})))
        except Exception as e:
            print(f"Error mensaje: {e}")

    c = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2, client_id="fusion-ej06")
    c.on_connect = al_conectar
    c.on_message = al_recibir
    try:
        c.connect(BROKER, PUERTO)
        while _corriendo:
            c.loop(timeout=0.1)
    except Exception as e:
        print(f"Error MQTT: {e}")
    finally:
        c.disconnect()


def run(context):
    global _app, _corriendo, _evento, _handler
    _app       = adsk.core.Application.get()
    _corriendo = True

    _evento  = _app.registerCustomEvent(EVENT_ID)
    _handler = Manejador()
    _evento.add(_handler)

    threading.Thread(target=hilo_mqtt, daemon=True).start()
    print("Ejemplo 06 activo — mueve los sliders en el dashboard")

    while _corriendo:
        adsk.doEvents()
        time.sleep(0.05)


def stop(context):
    global _corriendo
    _corriendo = False
    if _evento and _handler:
        _evento.remove(_handler)
        _app.unregisterCustomEvent(EVENT_ID)
    print("Ejemplo 06 detenido.")
