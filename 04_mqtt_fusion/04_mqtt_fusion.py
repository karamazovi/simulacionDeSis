"""
EJEMPLO 04 - HORA 4: MQTT + Fusion 360
Escucha mensajes MQTT y mueve JointBase al angulo recibido.

Formato del mensaje esperado (JSON):
  {"angulo": 45.0}

Para probar: ejecuta test_enviar.py desde otra terminal.
"""
import adsk.core, adsk.fusion
import json, math, threading, time, traceback
import paho.mqtt.client as mqtt

BROKER = "127.0.0.1"
PUERTO = 1883
TOPIC  = "clase/junta"
JUNTA  = "JointBase"

_app    = None
_ui     = None
_corriendo = True


def mover_junta(grados):
    try:
        design = _app.activeProduct
        for j in design.rootComponent.joints:
            if j.name == JUNTA:
                j.jointMotion.rotationValue = math.radians(grados)
                _app.activeViewport.refresh()
                print(f"JointBase → {grados:.1f}°")
                return True
        print(f"Junta '{JUNTA}' no encontrada")
        return False
    except Exception as e:
        print(f"Error: {e}")
        return False


def hilo_mqtt():
    def al_conectar(client, userdata, flags, rc, props=None):
        if rc == 0:
            client.subscribe(TOPIC)
            print(f"MQTT conectado — escuchando '{TOPIC}'")

    def al_recibir(client, userdata, msg):
        try:
            datos = json.loads(msg.payload.decode())
            angulo = float(datos.get("angulo", 0.0))
            mover_junta(angulo)
        except Exception as e:
            print(f"Mensaje invalido: {e}")

    cliente = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2, client_id="fusion-ej04")
    cliente.on_connect = al_conectar
    cliente.on_message = al_recibir
    try:
        cliente.connect(BROKER, PUERTO)
        while _corriendo:
            cliente.loop(timeout=0.1)
    except Exception as e:
        print(f"Error MQTT: {e}")
    finally:
        cliente.disconnect()


def run(context):
    global _app, _ui, _corriendo
    _app = adsk.core.Application.get()
    _ui  = _app.userInterface
    _corriendo = True

    threading.Thread(target=hilo_mqtt, daemon=True).start()
    print("Ejemplo 04 activo. Enviando mensajes con test_enviar.py")
    print("Presiona Stop en Fusion para detener.")

    while _corriendo:
        adsk.doEvents()
        time.sleep(0.05)


def stop(context):
    global _corriendo
    _corriendo = False
    print("Ejemplo 04 detenido.")
