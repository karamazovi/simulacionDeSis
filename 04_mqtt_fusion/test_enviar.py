"""
EJEMPLO 04 - Test: envia angulos a JointBase via MQTT.
Ejecutar: python test_enviar.py
"""
import paho.mqtt.client as mqtt
import json, time

BROKER = "127.0.0.1"
PUERTO = 1883
TOPIC  = "clase/junta"

cliente = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2, client_id="test-ej04")
cliente.connect(BROKER, PUERTO)
cliente.loop_start()
time.sleep(0.5)

angulos = [0, 30, 60, 90, 45, -30, 0]
for angulo in angulos:
    mensaje = json.dumps({"angulo": angulo})
    cliente.publish(TOPIC, mensaje)
    print(f"Enviado: {angulo}°")
    time.sleep(2)

cliente.loop_stop()
cliente.disconnect()
print("Test completado.")
