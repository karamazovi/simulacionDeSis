"""
Dashboard para RobotArmControl1.
Broker MQTT embebido incluido - no requiere software externo.

Ejecutar: streamlit run dashboard.py
Luego en Fusion 360: Scripts -> RobotArmControl1 -> Run
"""

import asyncio
import json
import math
import os
import threading
import time
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Dict

import paho.mqtt.client as mqtt
import streamlit as st

# ── Broker embebido ───────────────────────────────────────────────────────────
try:
    from amqtt.broker import Broker
    BROKER_DISPONIBLE = True
except ImportError:
    BROKER_DISPONIBLE = False

CONFIG_BROKER = {
    "listeners": {"default": {"type": "tcp", "bind": "0.0.0.0:1883"}},
    "auth": {"allow-anonymous": True},
    "sys_interval": 0,
}

# ── Configuracion de juntas (debe coincidir con robot_arm_config.py) ──────────
JUNTAS = {
    "JointBase":   {"min": -180.0, "max": 180.0, "default": 0.0},
    "JointHombro": {"min": -90.0,  "max": 90.0,  "default": 0.0},
    "JointCodo":   {"min": -135.0, "max": 135.0, "default": 0.0},
}

PID_DEFAULTS = {
    "JointBase":   {"kp": 2.0,  "ki": 0.15, "kd": 0.08},
    "JointHombro": {"kp": 2.5,  "ki": 0.2,  "kd": 0.09},
    "JointCodo":   {"kp": 2.3,  "ki": 0.18, "kd": 0.08},
}

# Eslabones para cinematica inversa (mm)
LINK_1 = 300.0
LINK_2 = 200.0

TOPICS = {
    "comando": "robotarm/command",
    "pid":     "robotarm/pid",
    "modo":    "robotarm/mode",
}


# ── Broker ────────────────────────────────────────────────────────────────────
class BrokerEmbebido:
    _instancia = None
    _lock = threading.Lock()

    def __new__(cls):
        with cls._lock:
            if cls._instancia is None:
                cls._instancia = super().__new__(cls)
                cls._instancia._init_done = False
            return cls._instancia

    def __init__(self):
        if self._init_done:
            return
        self._init_done = True
        self.corriendo = False

    def iniciar(self):
        if self.corriendo or not BROKER_DISPONIBLE:
            return self.corriendo
        listo = threading.Event()

        def run():
            loop = asyncio.new_event_loop()
            asyncio.set_event_loop(loop)

            async def main():
                broker = Broker(CONFIG_BROKER)
                await broker.start()
                self.corriendo = True
                listo.set()
                while self.corriendo:
                    await asyncio.sleep(0.5)
                await broker.shutdown()

            try:
                loop.run_until_complete(main())
            except Exception as e:
                print(f"Error broker: {e}")
                listo.set()
            finally:
                loop.close()

        threading.Thread(target=run, daemon=True).start()
        listo.wait(timeout=5)
        return self.corriendo


# ── Cliente MQTT ──────────────────────────────────────────────────────────────
class ClienteMqtt:
    def __init__(self):
        self.estado = {"conectado": False, "ultimo_error": None}
        self.cliente = mqtt.Client(
            mqtt.CallbackAPIVersion.VERSION2,
            client_id=f"dashboard-rac1-{os.getpid()}",
        )
        self.cliente.on_connect = self._on_connect
        self.cliente.on_disconnect = self._on_disconnect
        self._conectar()

    def _on_connect(self, client, userdata, flags, rc, props=None):
        self.estado["conectado"] = rc == 0
        if rc != 0:
            self.estado["ultimo_error"] = f"rc={rc}"

    def _on_disconnect(self, client, userdata, rc, props=None):
        self.estado["conectado"] = False

    def _conectar(self):
        def loop():
            backoff = 1
            while True:
                try:
                    self.cliente.connect("127.0.0.1", 1883)
                    self.cliente.loop_start()
                    return
                except Exception as e:
                    self.estado["ultimo_error"] = str(e)
                    time.sleep(backoff)
                    backoff = min(backoff * 2, 30)
        threading.Thread(target=loop, daemon=True).start()

    def _payload(self, tipo: str, cuerpo: Dict[str, Any]) -> str:
        mapa = {"juntas": "joint_command", "pid": "pid_command", "modo": "mode_command"}
        return json.dumps({
            "type": mapa.get(tipo, tipo),
            "timestamp": datetime.now(timezone.utc).isoformat(),
            "source": "dashboard_rac1",
            "body": cuerpo,
        })

    def publicar(self, tema: str, payload: str):
        if not self.estado["conectado"]:
            raise RuntimeError("MQTT no conectado")
        self.cliente.publish(tema, payload)

    def enviar_juntas(self, valores: Dict[str, float]):
        # RobotArmControl1 espera grados (no radianes)
        payload = self._payload("juntas", {"joints": valores})
        self.publicar(TOPICS["comando"], payload)

    def enviar_pid(self, valores: Dict[str, Any]):
        payload = self._payload("pid", {
            "joint":          str(valores["joint"]),
            "enabled":        bool(valores.get("enabled", True)),
            "kp":             float(valores["kp"]),
            "ki":             float(valores["ki"]),
            "kd":             float(valores["kd"]),
            "setpoint":       float(valores["setpoint"]),
            "ramp_time_s":    float(valores.get("ramp_time_s", 0.0)),
            "ramp_cycles":    int(valores.get("ramp_cycles", 0)),
            "tolerance_deg":  max(0.01, float(valores.get("tolerance_deg", 0.6))),
            "settle_cycles":  max(1, int(valores.get("settle_cycles", 3))),
        })
        self.publicar(TOPICS["pid"], payload)

    def enviar_modo(self, modo: str):
        payload = self._payload("modo", {"mode": modo})
        self.publicar(TOPICS["modo"], payload)


# ── IK ────────────────────────────────────────────────────────────────────────
def ik_2link(x, y, link_1=LINK_1, link_2=LINK_2, elbow_up=True):
    dist2 = x*x + y*y
    dist = math.sqrt(dist2)
    if dist > (link_1 + link_2) or dist < abs(link_1 - link_2):
        raise ValueError(f"Punto ({x:.1f}, {y:.1f}) fuera del alcance del brazo")
    cos_codo = (dist2 - link_1**2 - link_2**2) / (2.0 * link_1 * link_2)
    cos_codo = max(-1.0, min(1.0, cos_codo))
    ang_codo = math.acos(cos_codo)
    if elbow_up:
        ang_codo = -ang_codo
    k1 = link_1 + link_2 * math.cos(ang_codo)
    k2 = link_2 * math.sin(ang_codo)
    ang_hombro = math.atan2(y, x) - math.atan2(k2, k1)
    return math.degrees(ang_hombro), math.degrees(ang_codo)


# ── UI Principal ──────────────────────────────────────────────────────────────
def principal():
    st.set_page_config(page_title="RobotArmControl1 Dashboard", layout="wide")
    st.title("RobotArmControl1 — Dashboard")

    # Broker
    if "broker" not in st.session_state:
        b = BrokerEmbebido()
        st.session_state["broker_activo"] = b.iniciar()

    # Cliente
    if "cliente" not in st.session_state:
        st.session_state["cliente"] = ClienteMqtt()
    cliente: ClienteMqtt = st.session_state["cliente"]

    # Sidebar
    with st.sidebar:
        st.header("Estado")
        if st.session_state.get("broker_activo"):
            st.success("Broker embebido activo")
        elif not BROKER_DISPONIBLE:
            st.error("amqtt no instalado")
            st.code("pip install amqtt")
        else:
            st.warning("Broker iniciando...")

        if cliente.estado["conectado"]:
            st.success("MQTT conectado")
        else:
            st.error("MQTT desconectado")
            if cliente.estado["ultimo_error"]:
                st.caption(f"Error: {cliente.estado['ultimo_error']}")

        st.divider()
        st.caption("Puerto: 1883 | Topics:")
        for k, v in TOPICS.items():
            st.caption(f"  {k}: {v}")

    tab1, tab2, tab3 = st.tabs(["Juntas", "PID", "Cinematica Inversa (IK)"])

    # ── Tab 1: Juntas ─────────────────────────────────────────────────────────
    with tab1:
        st.subheader("Control Manual de Juntas")
        valores = {}
        for nombre, lim in JUNTAS.items():
            valores[nombre] = st.slider(
                nombre,
                float(lim["min"]), float(lim["max"]), float(lim["default"]),
                step=1.0,
                help=f"Rango: {lim['min']}° a {lim['max']}°",
            )

        c1, c2, c3 = st.columns(3)
        with c1:
            if st.button("Enviar", use_container_width=True, type="primary"):
                try:
                    cliente.enviar_juntas(valores)
                    st.success("Comando enviado")
                except Exception as e:
                    st.error(f"Error: {e}")
        with c2:
            if st.button("Home", use_container_width=True):
                try:
                    cliente.enviar_juntas({k: v["default"] for k, v in JUNTAS.items()})
                    st.success("Home enviado")
                except Exception as e:
                    st.error(f"Error: {e}")
        with c3:
            if st.button("STOP", use_container_width=True, type="secondary"):
                try:
                    cliente.enviar_modo("stop")
                    st.warning("STOP enviado")
                except Exception as e:
                    st.error(f"Error: {e}")

    # ── Tab 2: PID ────────────────────────────────────────────────────────────
    with tab2:
        st.subheader("Control PID por Junta")
        nombres = list(JUNTAS.keys())
        junta = st.selectbox("Junta", options=nombres)
        defs = PID_DEFAULTS[junta]
        lim = JUNTAS[junta]

        c1, c2 = st.columns(2)
        with c1:
            kp = st.number_input("Kp", value=float(defs["kp"]), step=0.01, format="%.3f")
            ki = st.number_input("Ki", value=float(defs["ki"]), step=0.01, format="%.3f")
            kd = st.number_input("Kd", value=float(defs["kd"]), step=0.01, format="%.3f")
        with c2:
            setpoint = st.number_input(
                "Setpoint (grados)",
                min_value=float(lim["min"]), max_value=float(lim["max"]),
                value=float(lim["default"]), step=1.0,
            )
            tolerancia = st.number_input("Tolerancia (grados)", min_value=0.01, value=0.6, step=0.1)
            ciclos = st.number_input("Ciclos estabilizacion", min_value=1, value=3, step=1)
            ramp_time = st.number_input("Tiempo rampa (s)", min_value=0.0, value=0.0, step=0.1)
            ramp_cycles = st.number_input("Ciclos rampa", min_value=0, value=0, step=1)

        habilitado = st.checkbox("Habilitado", value=True)

        c1, c2 = st.columns(2)
        with c1:
            if st.button("Enviar PID", use_container_width=True, type="primary"):
                try:
                    cliente.enviar_pid({
                        "joint": junta, "enabled": habilitado,
                        "kp": kp, "ki": ki, "kd": kd,
                        "setpoint": setpoint,
                        "ramp_time_s": ramp_time, "ramp_cycles": int(ramp_cycles),
                        "tolerance_deg": tolerancia, "settle_cycles": int(ciclos),
                    })
                    st.success(f"PID enviado a {junta}")
                except Exception as e:
                    st.error(f"Error: {e}")
        with c2:
            if st.button("Detener PID", use_container_width=True):
                try:
                    cliente.enviar_pid({"joint": junta, "enabled": False,
                        "kp": kp, "ki": ki, "kd": kd, "setpoint": setpoint,
                        "tolerance_deg": tolerancia, "settle_cycles": int(ciclos),
                        "ramp_time_s": 0.0, "ramp_cycles": 0})
                    st.info(f"PID detenido en {junta}")
                except Exception as e:
                    st.error(f"Error: {e}")

        with st.expander("Estrategia de ajuste PID"):
            st.markdown("""
            1. Empieza con `Ki=0` y `Kd=0`
            2. Sube `Kp` hasta que oscile suavemente alrededor del objetivo
            3. Agrega `Kd` para reducir el sobreimpulso
            4. Agrega `Ki` poco a poco para eliminar el error en estado estable
            5. Si vibra: baja `Kp` 10-20% o sube `Kd`
            """)

    # ── Tab 3: IK ─────────────────────────────────────────────────────────────
    with tab3:
        st.subheader("Cinematica Inversa — Objetivo XY")
        st.caption(f"Eslabones: Link1={LINK_1}mm | Link2={LINK_2}mm | Alcance max: {LINK_1+LINK_2:.0f}mm")

        c1, c2 = st.columns(2)
        with c1:
            x = st.number_input("X (mm)", value=140.0, step=1.0,
                                min_value=float(-(LINK_1+LINK_2)), max_value=float(LINK_1+LINK_2))
            y = st.number_input("Y (mm)", value=80.0, step=1.0,
                                min_value=float(-(LINK_1+LINK_2)), max_value=float(LINK_1+LINK_2))
            elbow_up = st.checkbox("Codo arriba", value=True)

        with c2:
            try:
                hombro_deg, codo_deg = ik_2link(x, y, elbow_up=elbow_up)
                st.metric("JointHombro", f"{hombro_deg:.1f}°")
                st.metric("JointCodo",   f"{codo_deg:.1f}°")
                alcance = math.sqrt(x**2 + y**2)
                st.caption(f"Distancia al objetivo: {alcance:.1f}mm")
                ik_valido = True
            except ValueError as e:
                st.error(str(e))
                ik_valido = False
                hombro_deg, codo_deg = 0.0, 0.0

        if st.button("Ir al punto XY", use_container_width=True,
                     type="primary", disabled=not ik_valido):
            try:
                cliente.enviar_juntas({
                    "JointHombro": hombro_deg,
                    "JointCodo":   codo_deg,
                })
                st.success(f"Enviado: Hombro={hombro_deg:.1f}° Codo={codo_deg:.1f}°")
            except Exception as e:
                st.error(f"Error: {e}")


if __name__ == "__main__":
    principal()
