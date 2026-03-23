"""
EJEMPLO 09 - HORA 9: Sistema autosuficiente con broker embebido
Todo en un solo dashboard: broker + control de juntas + PID.
NO requiere broker externo.

Ejecutar: streamlit run dashboard.py
Luego correr 09_fusion_bridge.py en Fusion 360.
"""
import asyncio, json, math, os, threading, time
import paho.mqtt.client as mqtt
import streamlit as st

BROKER = "127.0.0.1"
PUERTO = 1883

LIMITES = {
    "JointBase":   {"min": -180, "max": 180, "default": 0},
    "JointHombro": {"min":  -90, "max":  90, "default": 0},
    "JointCodo":   {"min": -135, "max": 135, "default": 0},
}
PID_DEFAULTS = {
    "JointBase":   {"kp": 2.0, "ki": 0.15, "kd": 0.08},
    "JointHombro": {"kp": 2.5, "ki": 0.20, "kd": 0.09},
    "JointCodo":   {"kp": 2.3, "ki": 0.18, "kd": 0.08},
}

# ── Broker embebido ───────────────────────────────────────────────────────────
try:
    from amqtt.broker import Broker as _AmqttBroker
    _BROKER_OK = True
except ImportError:
    _BROKER_OK = False

_CONFIG_BROKER = {
    "listeners": {"default": {"type": "tcp", "bind": f"0.0.0.0:{PUERTO}"}},
    "auth": {"allow-anonymous": True},
    "sys_interval": 0,
}


class BrokerEmbebido:
    _inst = None
    _lock = threading.Lock()

    def __new__(cls):
        with cls._lock:
            if cls._inst is None:
                cls._inst = super().__new__(cls)
                cls._inst._corriendo = False
        return cls._inst

    def iniciar(self):
        if self._corriendo or not _BROKER_OK:
            return self._corriendo
        listo = threading.Event()

        def run():
            loop = asyncio.new_event_loop()
            asyncio.set_event_loop(loop)
            async def main():
                b = _AmqttBroker(_CONFIG_BROKER)
                await b.start()
                self._corriendo = True
                listo.set()
                while self._corriendo:
                    await asyncio.sleep(0.5)
                await b.shutdown()
            try:
                loop.run_until_complete(main())
            except Exception as e:
                print(f"Error broker: {e}")
                listo.set()
            finally:
                loop.close()

        threading.Thread(target=run, daemon=True).start()
        listo.wait(timeout=5)
        return self._corriendo


# ── Cliente MQTT ──────────────────────────────────────────────────────────────
@st.cache_resource
def obtener_cliente():
    estado = {"conectado": False}

    def al_conectar(c, u, f, rc, p=None):
        estado["conectado"] = rc == 0
    def al_desconectar(c, u, rc, p=None):
        estado["conectado"] = False

    c = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2, client_id=f"ej09-{os.getpid()}")
    c.on_connect    = al_conectar
    c.on_disconnect = al_desconectar

    def conectar():
        for _ in range(10):
            try:
                c.connect(BROKER, PUERTO)
                c.loop_start()
                return
            except Exception:
                time.sleep(1)

    threading.Thread(target=conectar, daemon=True).start()
    return c, estado


# ── UI ────────────────────────────────────────────────────────────────────────
st.set_page_config(page_title="Ejemplo 09 - Sistema Completo", layout="wide")
st.title("Sistema Autosuficiente — Broker + Control + PID")

# Iniciar broker
if "broker" not in st.session_state:
    b = BrokerEmbebido()
    st.session_state["broker_activo"] = b.iniciar()

cliente, estado = obtener_cliente()

col1, col2, col3 = st.columns(3)
col1.metric("Broker", "Activo" if st.session_state.get("broker_activo") else "No disponible")
col2.metric("MQTT",   "Conectado" if estado["conectado"] else "Desconectado")
col3.caption(f"127.0.0.1:{PUERTO}")

tab1, tab2 = st.tabs(["Control de Juntas", "Control PID"])

# ── Tab 1: Control directo ────────────────────────────────────────────────────
with tab1:
    st.subheader("Mover juntas directamente")
    col_s, col_b = st.columns([3, 1])
    valores = {}
    with col_s:
        for nombre, lim in LIMITES.items():
            valores[nombre] = st.slider(
                nombre,
                float(lim["min"]), float(lim["max"]), float(lim["default"]), step=1.0
            )
    with col_b:
        st.write(""); st.write("")
        if st.button("Enviar", use_container_width=True, type="primary"):
            if estado["conectado"]:
                cliente.publish("robotarm/command", json.dumps({
                    "type": "joint_command",
                    "body": {"joints": {k: math.radians(v) for k, v in valores.items()}}
                }))
                st.success("Enviado")
            else:
                st.error("Sin conexion")

        if st.button("Home", use_container_width=True):
            if estado["conectado"]:
                home = {k: math.radians(lim["default"]) for k, lim in LIMITES.items()}
                cliente.publish("robotarm/command", json.dumps({
                    "type": "joint_command", "body": {"joints": home}
                }))
                st.info("Home enviado")

        if st.button("STOP", use_container_width=True):
            if estado["conectado"]:
                cliente.publish("robotarm/mode", json.dumps({
                    "type": "mode_command", "body": {"mode": "stop"}
                }))
                st.warning("STOP enviado")

# ── Tab 2: PID ────────────────────────────────────────────────────────────────
with tab2:
    st.subheader("Control PID por junta")
    nombres = list(LIMITES.keys())
    junta   = st.selectbox("Junta", nombres, key="pid_junta")
    defs    = PID_DEFAULTS[junta]
    lim     = LIMITES[junta]

    c1, c2 = st.columns(2)
    with c1:
        kp = st.number_input("Kp", value=float(defs["kp"]), step=0.01, format="%.3f")
        ki = st.number_input("Ki", value=float(defs["ki"]), step=0.01, format="%.3f")
        kd = st.number_input("Kd", value=float(defs["kd"]), step=0.01, format="%.3f")
    with c2:
        setpoint   = st.number_input("Setpoint (grados)", float(lim["min"]), float(lim["max"]), 0.0, step=1.0)
        tolerancia = st.number_input("Tolerancia (grados)", value=0.5, step=0.1)
        ciclos     = st.number_input("Ciclos estabilizacion", value=3, min_value=1, step=1)
        habilitado = st.checkbox("Habilitado", value=True)

    ca, cb = st.columns(2)
    with ca:
        if st.button("Enviar PID", use_container_width=True, type="primary"):
            if estado["conectado"]:
                cliente.publish("robotarm/pid", json.dumps({
                    "type": "pid_command",
                    "body": {
                        "joint": junta, "enabled": habilitado,
                        "kp": kp, "ki": ki, "kd": kd,
                        "setpoint": setpoint,
                        "tolerance_deg": tolerancia,
                        "settle_cycles": int(ciclos),
                    }
                }))
                st.success(f"PID enviado → {junta} {setpoint}°")
            else:
                st.error("Sin conexion")
    with cb:
        if st.button("Detener PID", use_container_width=True):
            if estado["conectado"]:
                cliente.publish("robotarm/pid", json.dumps({
                    "type": "pid_command",
                    "body": {"joint": junta, "enabled": False, "kp": kp, "ki": ki, "kd": kd, "setpoint": setpoint}
                }))
                st.info("PID detenido")


if __name__ == "__main__":
    principal()
