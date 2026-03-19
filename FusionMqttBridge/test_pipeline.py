"""
Test de pipeline completo: simula lo que hace el dashboard y verifica
que el mensaje llega en el formato que espera FusionMqttBridge.py
Correr con: python test_pipeline.py
"""
import json
import math
import time
import threading
import paho.mqtt.client as mqtt

BROKER_HOST = "127.0.0.1"
BROKER_PORT = 1883
TOPIC_COMANDO = "robotarm/command"

resultados = {"enviado": False, "recibido": None, "error": None, "conectado_pub": False, "conectado_sub": False}

# ── Suscriptor (simula lo que hace FusionMqttBridge.py) ──────────────────────
def on_connect_sub(client, userdata, flags, rc, props=None):
    if rc == 0:
        resultados["conectado_sub"] = True
        client.subscribe(TOPIC_COMANDO)
    else:
        resultados["error"] = f"Suscriptor no pudo conectar: rc={rc}"

def on_message(client, userdata, msg):
    try:
        payload = json.loads(msg.payload.decode())
        tipo = payload.get("type") or payload.get("tipo")
        body = payload.get("body") or payload.get("cuerpo") or {}
        joints = body.get("joints", {})
        resultados["recibido"] = {"tipo": tipo, "joints": joints}
    except Exception as e:
        resultados["error"] = f"Error parseando mensaje: {e}"

sub = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2, client_id="test-sub")
sub.on_connect = on_connect_sub
sub.on_message = on_message

# ── Publicador (simula lo que hace basic_dashboard.py) ──────────────────────
def on_connect_pub(client, userdata, flags, rc, props=None):
    resultados["conectado_pub"] = (rc == 0)

pub = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2, client_id="test-pub")
pub.on_connect = on_connect_pub

# ── Ejecutar test ─────────────────────────────────────────────────────────────
print("=" * 55)
print("TEST PIPELINE FusionMqttBridge")
print("=" * 55)

# Conectar ambos clientes
try:
    sub.connect(BROKER_HOST, BROKER_PORT)
    sub.loop_start()
    time.sleep(0.5)
except Exception as e:
    print(f"[FALLO] No se pudo conectar suscriptor: {e}")
    print("  → ¿Está corriendo el broker? Inicia el dashboard primero.")
    exit(1)

try:
    pub.connect(BROKER_HOST, BROKER_PORT)
    pub.loop_start()
    time.sleep(0.5)
except Exception as e:
    print(f"[FALLO] No se pudo conectar publicador: {e}")
    exit(1)

print(f"[{'OK' if resultados['conectado_sub'] else 'FALLO'}] Suscriptor MQTT conectado")
print(f"[{'OK' if resultados['conectado_pub'] else 'FALLO'}] Publicador MQTT conectado")

if not resultados["conectado_sub"] or not resultados["conectado_pub"]:
    print("\nNo se pudo conectar. Verifica que el broker esté activo.")
    exit(1)

# Construir payload exactamente como lo hace basic_dashboard.py
valores_grados = {"JointBase": 45.0, "JointHombro": -30.0, "JointCodo": 60.0}
cuerpo = {k: math.radians(v) for k, v in valores_grados.items()}
payload = json.dumps({
    "type": "joint_command",
    "timestamp": "2026-01-01T00:00:00+00:00",
    "source": "test_pipeline",
    "body": {"joints": cuerpo},
})

pub.publish(TOPIC_COMANDO, payload)
resultados["enviado"] = True
print(f"\n[OK] Mensaje enviado al topic '{TOPIC_COMANDO}'")
print(f"     Valores en grados: {valores_grados}")

# Esperar recepcion
time.sleep(1.5)
sub.loop_stop()
pub.loop_stop()

# ── Resultados ────────────────────────────────────────────────────────────────
print("\n" + "=" * 55)
print("RESULTADO")
print("=" * 55)

if resultados["recibido"]:
    r = resultados["recibido"]
    print(f"[OK] Mensaje recibido correctamente")
    print(f"     tipo  : {r['tipo']}")
    print(f"     joints:")
    for nombre, rad in r["joints"].items():
        grados = math.degrees(rad)
        print(f"       {nombre}: {rad:.4f} rad  ({grados:.1f}°)")

    # Validar que los nombres coinciden con los del settings.json
    esperados = {"JointBase", "JointHombro", "JointCodo"}
    recibidos = set(r["joints"].keys())
    if recibidos == esperados:
        print(f"\n[OK] Nombres de joints correctos: {recibidos}")
    else:
        print(f"\n[AVISO] Joints recibidos : {recibidos}")
        print(f"         Joints esperados: {esperados}")
        print("  → Verifica que los nombres en settings.json coincidan con Fusion 360")
else:
    print("[FALLO] No se recibió ningún mensaje")
    if resultados["error"]:
        print(f"  Error: {resultados['error']}")

print("\n[INFO] Si todo sale OK aquí pero Fusion no mueve las juntas:")
print("  1. Verifica que FusionMqttBridge.py esté corriendo (diálogo abierto)")
print("  2. Verifica que el diseño activo tenga joints con esos nombres exactos")
print("  3. Corre diagnostico.py desde Fusion para ver los nombres reales")
