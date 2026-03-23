"""
Dashboard basico para controlar brazo robotico en Fusion 360 via MQTT.
Incluye broker MQTT embebido - no requiere broker externo.

Ejecutar con: streamlit run basic_dashboard.py --server.port 8500
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

# Broker MQTT embebido
try:
    from amqtt.broker import Broker
    BROKER_DISPONIBLE = True
except ImportError:
    BROKER_DISPONIBLE = False

# Configuracion
DIRECTORIO_ACTUAL = Path(__file__).resolve().parent
ARCHIVO_CONFIG = DIRECTORIO_ACTUAL / "FusionMqttBridge.settings.json"

CONFIG_BROKER = {
    "listeners": {
        "default": {
            "type": "tcp",
            "bind": "0.0.0.0:1883",
        },
    },
    "auth": {
        "allow-anonymous": True,
    },
    "sys_interval": 0,
}


def cargar_configuracion() -> Dict[str, Any]:
    """Carga configuracion desde archivo JSON o usa valores por defecto."""
    valores_defecto: Dict[str, Any] = {
        "mqtt": {
            "host": "127.0.0.1",
            "port": 1883,
            "username": "",
            "password": "",
            "client_id": "dashboard-basico",
        },
        "temas": {
            "comando": "robotarm/command",
            "modo": "robotarm/mode",
            "telemetria": "robotarm/telemetry",
            "ack": "robotarm/ack",
        },
        "juntas": {
            "base": {"min": -180.0, "max": 180.0, "default": 0.0},
            "hombro": {"min": -90.0, "max": 90.0, "default": 0.0},
            "codo": {"min": -135.0, "max": 135.0, "default": 0.0},
        },
    }
    if ARCHIVO_CONFIG.exists():
        with ARCHIVO_CONFIG.open("r", encoding="utf-8") as f:
            cargado = json.load(f)
        for seccion in ("mqtt", "temas"):
            if isinstance(cargado.get(seccion), dict):
                valores_defecto[seccion].update(cargado[seccion])
        # juntas se reemplaza completa para no mezclar con los defaults
        if isinstance(cargado.get("juntas"), dict):
            valores_defecto["juntas"] = cargado["juntas"]
    return valores_defecto


# Cargar configuracion global
configuracion = cargar_configuracion()
config_mqtt = configuracion["mqtt"]
temas = configuracion["temas"]
limites_juntas = configuracion["juntas"]


class BrokerEmbebido:
    """Broker MQTT embebido que corre en un hilo separado."""

    _instancia = None
    _lock = threading.Lock()

    def __new__(cls):
        with cls._lock:
            if cls._instancia is None:
                cls._instancia = super().__new__(cls)
                cls._instancia._iniciado = False
            return cls._instancia

    def __init__(self):
        if hasattr(self, "_init_done"):
            return
        self._init_done = True
        self.broker = None
        self.loop = None
        self.hilo = None
        self.corriendo = False

    def iniciar(self):
        """Inicia el broker en un hilo separado."""
        if self.corriendo:
            return True

        if not BROKER_DISPONIBLE:
            return False

        listo = threading.Event()

        def run_broker():
            self.loop = asyncio.new_event_loop()
            asyncio.set_event_loop(self.loop)

            async def broker_main():
                self.broker = Broker(CONFIG_BROKER)
                await self.broker.start()
                self.corriendo = True
                listo.set()
                while self.corriendo:
                    await asyncio.sleep(0.5)
                await self.broker.shutdown()

            try:
                self.loop.run_until_complete(broker_main())
            except Exception as e:
                print(f"Error en broker: {e}")
                listo.set()  # desbloquear aunque haya error
            finally:
                self.loop.close()

        self.hilo = threading.Thread(target=run_broker, daemon=True)
        self.hilo.start()
        listo.wait(timeout=5)
        return self.corriendo

    def detener(self):
        """Detiene el broker."""
        self.corriendo = False


def id_cliente(base: str) -> str:
    """Genera un ID de cliente unico."""
    return f"{base}-{os.getpid()}-{int(time.time())}"


def ahora_utc() -> str:
    """Retorna timestamp UTC en formato ISO."""
    return datetime.now(timezone.utc).isoformat()


def construir_payload(tipo_comando: str, cuerpo: Dict[str, Any]) -> str:
    """Construye el payload JSON para enviar via MQTT."""
    mapa_tipos = {
        "comando_juntas": "joint_command",
        "comando_modo": "mode_command",
    }
    return json.dumps({
        "type": mapa_tipos.get(tipo_comando, tipo_comando),
        "timestamp": ahora_utc(),
        "source": "dashboard_basico",
        "body": cuerpo,
    })


class ClienteMqtt:
    """Cliente MQTT para enviar comandos al brazo robotico."""

    def __init__(self):
        self.estado = {"conectado": False, "ultimo_error": None}
        self.cliente = mqtt.Client(
            mqtt.CallbackAPIVersion.VERSION2,
            client_id=id_cliente("dashboard"),
        )
        self.cliente.on_connect = self._on_connect
        self.cliente.on_disconnect = self._on_disconnect
        self._conectar_async()

    def _on_connect(self, client, userdata, flags, rc, props=None):
        self.estado["conectado"] = rc == 0
        if rc != 0:
            self.estado["ultimo_error"] = f"Codigo de conexion: {rc}"

    def _on_disconnect(self, client, userdata, rc, props=None):
        self.estado["conectado"] = False

    def _conectar_async(self):
        """Intenta conectar al broker en un hilo separado."""
        def conectar():
            intentos = 0
            while intentos < 10:
                try:
                    self.cliente.connect(config_mqtt["host"], config_mqtt["port"])
                    self.cliente.loop_start()
                    return
                except Exception as e:
                    self.estado["ultimo_error"] = str(e)
                    intentos += 1
                    time.sleep(1)

        threading.Thread(target=conectar, daemon=True).start()

    def publicar(self, tema: str, payload: str):
        """Publica un mensaje en el tema especificado."""
        if not self.estado["conectado"]:
            raise RuntimeError("MQTT no conectado")
        self.cliente.publish(tema, payload)

    def enviar_juntas(self, valores: Dict[str, float]):
        """Envia comandos de juntas (convierte grados a radianes)."""
        cuerpo = {k: math.radians(v) for k, v in valores.items()}
        payload = construir_payload("comando_juntas", {"joints": cuerpo})
        self.publicar(temas["comando"], payload)

    def enviar_modo(self, modo: str):
        """Envia comando de modo (stop, home, etc.)."""
        payload = construir_payload("comando_modo", {"mode": modo})
        self.publicar(temas["modo"], payload)

    def enviar_pid(self, valores: Dict[str, Any]):
        """Envia parametros PID para una junta."""
        payload = construir_payload("comando_pid", {
            "joint":      str(valores["joint"]),
            "enabled":    bool(valores.get("enabled", True)),
            "kp":         float(valores["kp"]),
            "ki":         float(valores["ki"]),
            "kd":         float(valores["kd"]),
            "setpoint":   float(valores["setpoint"]),
            "tolerance_deg":  max(0.01, float(valores.get("tolerance_deg", 0.6))),
            "settle_cycles":  max(1, int(valores.get("settle_cycles", 3))),
        })
        self.publicar(temas.get("pid", "robotarm/pid"), payload)


def principal():
    """Funcion principal del dashboard Streamlit."""
    st.set_page_config(page_title="Robot Arm Control", page_icon="🤖", layout="wide")
    st.title("Robot Arm Dashboard")

    # Iniciar broker embebido (singleton)
    if "broker" not in st.session_state:
        broker = BrokerEmbebido()
        if BROKER_DISPONIBLE:
            broker.iniciar()
            st.session_state["broker"] = broker
            st.session_state["broker_activo"] = True
        else:
            st.session_state["broker_activo"] = False

    # Crear cliente MQTT (el broker ya señalizó que está listo via threading.Event)
    if "cliente" not in st.session_state:
        st.session_state["cliente"] = ClienteMqtt()

    cliente = st.session_state["cliente"]

    # Sidebar con estado
    with st.sidebar:
        st.header("Estado")

        if st.session_state.get("broker_activo"):
            st.success("Broker embebido activo")
        else:
            st.warning("Broker embebido no disponible")
            st.info("Instala amqtt: `pip install amqtt`")
            st.info("O usa un broker externo en 127.0.0.1:1883")

        if cliente.estado["conectado"]:
            st.success("Cliente MQTT conectado")
        else:
            st.error("Cliente MQTT desconectado")
            if cliente.estado["ultimo_error"]:
                st.caption(f"Error: {cliente.estado['ultimo_error']}")

        st.divider()
        st.subheader("Configuracion")
        st.text(f"Host: {config_mqtt['host']}")
        st.text(f"Puerto: {config_mqtt['port']}")
        st.text(f"Topic cmd: {temas['comando']}")

    # ── Control de Juntas ────────────────────────────────────────────────────
    st.subheader("Control de Juntas")

    col1, col2 = st.columns([3, 1])

    with col1:
        valores = {}
        for nombre, lim in limites_juntas.items():
            valores[nombre] = st.slider(
                f"🔧 {nombre.capitalize()}",
                float(lim["min"]),
                float(lim["max"]),
                float(lim["default"]),
                step=1.0,
                help=f"Rango: {lim['min']}° a {lim['max']}°",
            )

    with col2:
        st.write("")  # Espaciador
        st.write("")

        if st.button("📤 Enviar", use_container_width=True, type="primary"):
            try:
                cliente.enviar_juntas(valores)
                st.success("Comando enviado")
            except Exception as e:
                st.error(f"Error: {e}")

        if st.button("🏠 Home", use_container_width=True):
            try:
                home = {k: lim["default"] for k, lim in limites_juntas.items()}
                cliente.enviar_juntas(home)
                st.success("Enviado a home")
            except Exception as e:
                st.error(f"Error: {e}")

        if st.button("🛑 STOP", use_container_width=True, type="secondary"):
            try:
                cliente.enviar_modo("stop")
                st.warning("STOP enviado")
            except Exception as e:
                st.error(f"Error: {e}")

    st.divider()

    # ── Control PID ──────────────────────────────────────────────────────────
    st.subheader("Control PID")
    nombres_juntas = list(limites_juntas.keys())

    pid_col1, pid_col2 = st.columns(2)
    with pid_col1:
        junta_pid = st.selectbox("Junta", options=nombres_juntas)
        kp = st.number_input("Kp", value=2.0, step=0.01, format="%.3f")
        ki = st.number_input("Ki", value=0.1, step=0.01, format="%.3f")
        kd = st.number_input("Kd", value=0.05, step=0.01, format="%.3f")
    with pid_col2:
        lim = limites_juntas[junta_pid]
        setpoint = st.number_input(
            "Setpoint (grados)",
            min_value=float(lim["min"]),
            max_value=float(lim["max"]),
            value=float(lim["default"]),
            step=1.0,
        )
        tolerancia = st.number_input("Tolerancia (grados)", min_value=0.01, value=0.6, step=0.1)
        ciclos = st.number_input("Ciclos estabilizacion", min_value=1, value=3, step=1)
        pid_habilitado = st.checkbox("Habilitado", value=True)

    pid_btn_col1, pid_btn_col2 = st.columns(2)
    with pid_btn_col1:
        if st.button("Enviar PID", use_container_width=True, type="primary"):
            try:
                cliente.enviar_pid({
                    "joint": junta_pid, "enabled": pid_habilitado,
                    "kp": kp, "ki": ki, "kd": kd,
                    "setpoint": setpoint,
                    "tolerance_deg": tolerancia, "settle_cycles": int(ciclos),
                })
                st.success(f"PID enviado a {junta_pid}")
            except Exception as e:
                st.error(f"Error: {e}")
    with pid_btn_col2:
        if st.button("Detener PID", use_container_width=True):
            try:
                cliente.enviar_pid({
                    "joint": junta_pid, "enabled": False,
                    "kp": kp, "ki": ki, "kd": kd, "setpoint": setpoint,
                    "tolerance_deg": tolerancia, "settle_cycles": int(ciclos),
                })
                st.info(f"PID detenido en {junta_pid}")
            except Exception as e:
                st.error(f"Error: {e}")

    st.divider()

    # Info de uso
    with st.expander("ℹ️ Instrucciones de uso"):
        st.markdown("""
        ### Uso con Fusion 360

        1. **Ejecuta este dashboard**: `streamlit run basic_dashboard.py --server.port 8500`
        2. **Abre Fusion 360** con un modelo que tenga juntas (joints)
        3. **Ejecuta el script** `FusionMqttBridge.py` desde Fusion 360
        4. **Mueve los sliders** y presiona "Enviar" para controlar las juntas

        ### Requisitos

        - Python con `streamlit` y `paho-mqtt`
        - Opcional: `amqtt` para broker embebido (`pip install amqtt`)
        - Sin `amqtt`: usar broker externo como Mosquitto

        ### Configuracion

        Edita `FusionMqttBridge.settings.json` para cambiar:
        - Host/puerto del broker MQTT
        - Nombres y limites de las juntas
        - Topics MQTT
        """)


if __name__ == "__main__":
    principal()
