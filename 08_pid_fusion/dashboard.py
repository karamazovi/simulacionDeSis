"""
EJEMPLO 08 - HORA 8: PID controlando Fusion 360
Dashboard con control PID para una sola junta.
Requiere:
  1. Broker en 127.0.0.1:1883
  2. Script 08_pid_fusion.py corriendo en Fusion 360

Ejecutar: streamlit run dashboard.py
"""
import json, math, os, threading, time
import paho.mqtt.client as mqtt
import streamlit as st

BROKER     = "127.0.0.1"
PUERTO     = 1883
TOPIC_PID  = "robotarm/pid"
TOPIC_MODE = "robotarm/mode"

LIMITES = {
    "JointBase":   {"min": -180, "max": 180},
    "JointHombro": {"min":  -90, "max":  90},
    "JointCodo":   {"min": -135, "max": 135},
}


@st.cache_resource
def obtener_cliente():
    estado = {"conectado": False}

    def al_conectar(c, u, f, rc, p=None):
        estado["conectado"] = rc == 0

    def al_desconectar(c, u, rc, p=None):
        estado["conectado"] = False

    c = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2, client_id=f"ej08-{os.getpid()}")
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


st.set_page_config(page_title="Ejemplo 08 - PID Fusion", layout="wide")
st.title("Control PID → Fusion 360")

cliente, estado = obtener_cliente()

st.metric("MQTT", "Conectado" if estado["conectado"] else "Desconectado")

st.subheader("Parametros PID")
col1, col2 = st.columns(2)

with col1:
    junta   = st.selectbox("Junta", list(LIMITES.keys()))
    kp      = st.number_input("Kp", value=2.0,  min_value=0.0, step=0.1, format="%.3f")
    ki      = st.number_input("Ki", value=0.1,  min_value=0.0, step=0.01, format="%.3f")
    kd      = st.number_input("Kd", value=0.05, min_value=0.0, step=0.01, format="%.3f")

with col2:
    lim      = LIMITES[junta]
    setpoint = st.number_input(
        "Setpoint (grados)",
        min_value=float(lim["min"]),
        max_value=float(lim["max"]),
        value=0.0, step=1.0
    )
    tolerancia = st.number_input("Tolerancia (grados)", value=0.5, min_value=0.01, step=0.1)
    ciclos     = st.number_input("Ciclos estabilizacion", value=3, min_value=1, step=1)
    habilitado = st.checkbox("Habilitado", value=True)

col_a, col_b = st.columns(2)
with col_a:
    if st.button("Enviar PID", use_container_width=True, type="primary"):
        if not estado["conectado"]:
            st.error("Sin conexion MQTT")
        else:
            payload = json.dumps({
                "type": "pid_command",
                "body": {
                    "joint":       junta,
                    "kp":          kp,
                    "ki":          ki,
                    "kd":          kd,
                    "setpoint":    setpoint,
                    "tolerance_deg": tolerancia,
                    "settle_cycles": int(ciclos),
                    "enabled":     habilitado,
                }
            })
            cliente.publish(TOPIC_PID, payload)
            st.success(f"PID enviado a {junta} → {setpoint}°")

with col_b:
    if st.button("STOP", use_container_width=True):
        if estado["conectado"]:
            cliente.publish(TOPIC_MODE, json.dumps({
                "type": "mode_command",
                "body": {"mode": "stop"}
            }))
            st.warning("STOP enviado")

st.divider()
with st.expander("Como funciona el PID"):
    st.markdown("""
    1. El dashboard envía **Kp, Ki, Kd y setpoint** por MQTT
    2. El script de Fusion recibe los parametros
    3. Cada 50ms lee la posicion actual de la junta (`rotationValue`)
    4. Calcula el error = setpoint − posicion
    5. Aplica la correccion PID a `rotationValue`
    6. Repite hasta converger

    **Ajuste rapido:**
    - Sube **Kp** hasta que oscile
    - Agrega **Kd** para amortiguar
    - Agrega **Ki** para eliminar error residual
    """)
