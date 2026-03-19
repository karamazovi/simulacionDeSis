import adsk.core
import adsk.fusion
import json
import time
import paho.mqtt.client as mqtt

def run(context):
    app = adsk.core.Application.get()
    ui = app.userInterface

    lineas = []

    # 1. Listar joints
    design = app.activeProduct
    if not design or not hasattr(design, "rootComponent"):
        ui.messageBox("No hay diseño activo con componentes.")
        return

    joints = design.rootComponent.joints
    lineas.append(f"Joints encontrados: {joints.count}")
    for j in joints:
        tiene_rot = hasattr(j.jointMotion, "rotationValue")
        lineas.append(f"  '{j.name}'  |  rotationValue: {tiene_rot}")

    # 2. Probar conexion MQTT
    recibido = {"msg": None, "conectado": False, "error": None}

    def on_connect(client, userdata, flags, rc, props=None):
        if rc == 0:
            recibido["conectado"] = True
            client.subscribe("robotarm/command")
        else:
            recibido["error"] = f"rc={rc}"

    def on_message(client, userdata, msg):
        recibido["msg"] = msg.payload.decode()

    try:
        cliente = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2, client_id="diagnostico")
        cliente.on_connect = on_connect
        cliente.on_message = on_message
        cliente.connect("127.0.0.1", 1883)
        cliente.loop_start()
        time.sleep(2)
        cliente.loop_stop()
        cliente.disconnect()

        lineas.append(f"\nMQTT conectado: {recibido['conectado']}")
        if recibido["error"]:
            lineas.append(f"Error MQTT: {recibido['error']}")
        if recibido["msg"]:
            lineas.append(f"Mensaje recibido:\n{recibido['msg']}")
        else:
            lineas.append("Sin mensajes en 2s (envia desde dashboard mientras espera)")

    except Exception as e:
        lineas.append(f"\nError MQTT: {e}")

    ui.messageBox("\n".join(lineas), "Diagnostico FusionMqttBridge")
