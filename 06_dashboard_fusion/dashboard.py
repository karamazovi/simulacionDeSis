"""
EJEMPLO 06 - HORA 6: Dashboard Streamlit → MQTT → Fusion 360
Envia comandos de juntas via MQTT. Requiere:
  1. Broker corriendo en 127.0.0.1:1883
  2. Script 06_fusion_bridge.py corriendo en Fusion 360

Ejecutar: streamlit run dashboard.py
"""
import json, math, os, threading, time
import paho.mqtt.client as mqtt
import streamlit as st

BROKER = "127.0.0.1"
PUERTO = 1883
TOPIC  = "robotarm/command"

LIMITES = {
    "JointBase":   {"min": -180, "max": 180, "default": 0},
    "JointHombro": {"min":  -90, "max":  90, "default": 0},
    "JointCodo":   {"min": -135, "max": 135, "default": 0},
}


@st.cache_resource
def obtener_cliente():
    estado = {"conectado": False, "error": None}

    def al_conectar(c, u, f, rc, p=None):
        estado["conectado"] = rc == 0

    def al_desconectar(c, u, rc, p=None):
        estado["conectado"] = False

    c = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2, client_id=f"ej06-{os.getpid()}")
    c.on_connect    = al_conectar
    c.on_disconnect = al_desconectar

    def conectar():
        try:
            c.connect(BROKER, PUERTO)
            c.loop_start()
        except Exception as e:
            estado["error"] = str(e)

    threading.Thread(target=conectar, daemon=True).start()
    return c, estado


st.set_page_config(page_title="Ejemplo 06 - Dashboard + Fusion", layout="wide")
st.title("Dashboard → MQTT → Fusion 360")

cliente, estado = obtener_cliente()

# Estado de conexion
if estado["conectado"]:
    st.success("MQTT conectado")
else:
    st.warning("MQTT desconectado — espera un momento o verifica el broker")
    if estado.get("error"):
        st.caption(f"Error: {estado['error']}")

st.subheader("Control de Juntas")
col_sliders, col_botones = st.columns([3, 1])

valores = {}
with col_sliders:
    for nombre, lim in LIMITES.items():
        valores[nombre] = st.slider(
            nombre,
            float(lim["min"]), float(lim["max"]), float(lim["default"]),
            step=1.0
        )

with col_botones:
    st.write("")
    st.write("")
    if st.button("Enviar", use_container_width=True, type="primary"):
        if not estado["conectado"]:
            st.error("Sin conexion MQTT")
        else:
            payload = json.dumps({
                "type": "joint_command",
                "body": {"joints": {k: math.radians(v) for k, v in valores.items()}}
            })
            cliente.publish(TOPIC, payload)
            st.success("Enviado")

    if st.button("STOP", use_container_width=True):
        if estado["conectado"]:
            cliente.publish("robotarm/mode", json.dumps({"type": "mode_command", "body": {"mode": "stop"}}))
            st.warning("STOP enviado")
