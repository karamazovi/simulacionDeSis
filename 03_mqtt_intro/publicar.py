"""
EJEMPLO 03 - HORA 3: Introduccion a MQTT
Publica mensajes en un topic. Corre PRIMERO este archivo.
Requiere broker externo corriendo en 127.0.0.1:1883
(corre basic_dashboard.py de FusionMqttBridge para levantar el broker)

Ejecutar: python publicar.py
"""
import paho.mqtt.client as mqtt
import time

BROKER = "127.0.0.1"
PUERTO = 1883
TOPIC  = "clase/saludo"

cliente = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2, client_id="publicador")
cliente.connect(BROKER, PUERTO)
cliente.loop_start()

print(f"Conectado al broker {BROKER}:{PUERTO}")
print(f"Publicando en topic: {TOPIC}")
print("Presiona Ctrl+C para detener\n")

contador = 1
try:
    while True:
        mensaje = f"Hola desde Python #{contador}"
        cliente.publish(TOPIC, mensaje)
        print(f"Enviado: {mensaje}")
        contador += 1
        time.sleep(2)
except KeyboardInterrupt:
    print("\nDetenido.")
finally:
    cliente.loop_stop()
    cliente.disconnect()
