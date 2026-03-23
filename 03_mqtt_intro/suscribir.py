"""
EJEMPLO 03 - HORA 3: Introduccion a MQTT
Suscribe a un topic y muestra los mensajes recibidos.
Corre este archivo EN OTRA TERMINAL mientras publicar.py esta corriendo.

Ejecutar: python suscribir.py
"""
import paho.mqtt.client as mqtt

BROKER = "127.0.0.1"
PUERTO = 1883
TOPIC  = "clase/saludo"


def al_conectar(client, userdata, flags, rc, props=None):
    if rc == 0:
        print(f"Conectado al broker {BROKER}:{PUERTO}")
        client.subscribe(TOPIC)
        print(f"Escuchando topic: {TOPIC}")
        print("Esperando mensajes... (Ctrl+C para detener)\n")
    else:
        print(f"Error de conexion rc={rc}")


def al_recibir(client, userdata, msg):
    print(f"Recibido en '{msg.topic}': {msg.payload.decode()}")


cliente = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2, client_id="suscriptor")
cliente.on_connect = al_conectar
cliente.on_message = al_recibir

try:
    cliente.connect(BROKER, PUERTO)
    cliente.loop_forever()
except KeyboardInterrupt:
    print("\nDetenido.")
finally:
    cliente.disconnect()
