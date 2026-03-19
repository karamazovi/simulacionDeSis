"""
Script de diagnostico - ejecutar desde Fusion 360 Scripts.
Muestra los nombres exactos de joints y prueba recibir un mensaje MQTT.
"""
import adsk.core
import adsk.fusion
import json
import threading
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
        lineas.append(f"  - '{j.name}'  |  rotationValue: {tiene_rot}")

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
        time.sleep(2)  # esperar conexion y posibles mensajes
        cliente.loop_stop()
        cliente.disconnect()

        lineas.append(f"\nMQTT conectado: {recibido['conectado']}")
        if recibido["error"]:
            lineas.append(f"Error MQTT: {recibido['error']}")
        if recibido["msg"]:
            lineas.append(f"Mensaje recibido:\n{recibido['msg']}")
        else:
            lineas.append("Sin mensajes recibidos en 2s (normal si no enviaste desde dashboard)")

    except Exception as e:
        lineas.append(f"\nError conectando MQTT: {e}")

    ui.messageBox("\n".join(lineas), "Diagnostico FusionMqttBridge")
