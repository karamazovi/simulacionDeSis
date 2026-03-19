
# Panel de control MQTT para el robot
import asyncio
import json
import os
import threading
import time
from datetime import datetime, timezone
from typing import Any, Dict

import paho.mqtt.client as mqtt
import streamlit as st

from config import JOINT_LIMITS as limitesJuntas, PID_DEFAULTS as pidPorDefecto, Settings as Configuracion

# ── Broker MQTT embebido ──────────────────────────────────────────────────────
try:
    from amqtt.broker import Broker as _MqttBroker
    _BROKER_DISPONIBLE = True
except ImportError:
    _BROKER_DISPONIBLE = False

_CONFIG_BROKER = {
    "listeners": {"default": {"type": "tcp", "bind": f"0.0.0.0:{Configuracion.MQTT_PORT}"}},
    "timeout-disconnect-delay": 2,
    "auth": {"allow-anonymous": True},
}


class _BrokerEmbebido:
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
        self.corriendo = False
        self.hilo = None

    def iniciar(self):
        if self.corriendo or not _BROKER_DISPONIBLE:
            return self.corriendo
        listo = threading.Event()

        def run():
            loop = asyncio.new_event_loop()
            asyncio.set_event_loop(loop)

            async def main():
                broker = _MqttBroker(_CONFIG_BROKER)
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

        self.hilo = threading.Thread(target=run, daemon=True)
        self.hilo.start()
        listo.wait(timeout=5)
        return self.corriendo



# Devuelve la hora actual en formato UTC
def ahoraUtc() -> str:
    return datetime.now(timezone.utc).isoformat()



# Limita el valor de la junta a sus rangos permitidos
def limitarJunta(nombreJunta: str, valor: float) -> float:
    cfg = limitesJuntas[nombreJunta]
    return max(cfg["min"], min(cfg["max"], float(valor)))



# Construye el payload JSON para enviar comandos
def construirPayload(tipoComando: str, cuerpo: Dict[str, Any]) -> str:
    mapa_tipos = {
        "comando_juntas": "joint_command",
        "comando_modo": "mode_command",
        "comando_pid": "pid_command",
    }
    payload = {
        "type": mapa_tipos.get(tipoComando, tipoComando),
        "timestamp": ahoraUtc(),
        "source": "panel_mqtt",
        "body": cuerpo,
    }
    return json.dumps(payload, ensure_ascii=True)



# Genera un client_id único para MQTT
def idClienteMqtt(base: str) -> str:
    return f"{base}-{os.getpid()}"



# Clase principal para manejar la conexión y comunicación MQTT
class RuntimePuenteMqtt:
    def __init__(self) -> None:
        self.bloqueoEstado = threading.Lock()
        self.estado: Dict[str, Any] = {
            "mqttConectado": False,
            "ultimaTelemetria": None,
            "ultimoAck": None,
            "ultimoError": None,
            "ultimaPublicacion": None,
        }

        self.clienteMqtt = mqtt.Client(
            mqtt.CallbackAPIVersion.VERSION2,
            client_id=idClienteMqtt(Configuracion.MQTT_CLIENT_ID),
        )
        if Configuracion.MQTT_USERNAME:
            self.clienteMqtt.username_pw_set(Configuracion.MQTT_USERNAME, Configuracion.MQTT_PASSWORD)

        self.clienteMqtt.on_connect = self.alConectar
        self.clienteMqtt.on_disconnect = self.alDesconectar
        self.clienteMqtt.on_message = self.alMensaje

        self.hilo = threading.Thread(target=self.trabajadorMqtt, daemon=True)
        self.hilo.start()

    def actualizarEstado(self, **kwargs: Any) -> None:
        with self.bloqueoEstado:
            self.estado.update(kwargs)

    def obtenerEstado(self) -> Dict[str, Any]:
        with self.bloqueoEstado:
            return dict(self.estado)

    def publicar(self, tema: str, payload: str) -> None:
        resultado = self.clienteMqtt.publish(tema, payload=payload, qos=1, retain=False)
        resultado.wait_for_publish(timeout=2.0)
        if resultado.rc != mqtt.MQTT_ERR_SUCCESS:
            raise RuntimeError(f"Publicación MQTT falló rc={resultado.rc}")
        if not resultado.is_published():
            raise RuntimeError("Publicación MQTT no fue confirmada por el broker")
        self.actualizarEstado(ultimaPublicacion={"tema": tema, "payload": json.loads(payload), "timestamp": ahoraUtc()})

    def alConectar(self, cliente: mqtt.Client, datosUsuario: Any, banderas: Dict[str, Any], codigoRazon: int, propiedades: Any) -> None:
        conectado = codigoRazon == 0
        self.actualizarEstado(mqttConectado=conectado, ultimoError=None if conectado else f"Error de conexión rc={codigoRazon}")
        if conectado:
            cliente.subscribe(Configuracion.TOPIC_TELEMETRY)
            cliente.subscribe(Configuracion.TOPIC_ACK)

    def alDesconectar(self, cliente: mqtt.Client, datosUsuario: Any, banderasDesconexion: Any, codigoRazon: int, propiedades: Any) -> None:
        self.actualizarEstado(mqttConectado=False)

    def alMensaje(self, cliente: mqtt.Client, datosUsuario: Any, mensaje: mqtt.MQTTMessage) -> None:
        decodificado = mensaje.payload.decode("utf-8", errors="ignore")
        parseado: Any = decodificado
        try:
            parseado = json.loads(decodificado)
        except json.JSONDecodeError:
            pass

        if mensaje.topic == Configuracion.TOPIC_TELEMETRY:
            self.actualizarEstado(ultimaTelemetria={"tema": mensaje.topic, "payload": parseado, "timestamp": ahoraUtc()})
        elif mensaje.topic == Configuracion.TOPIC_ACK:
            self.actualizarEstado(ultimoAck={"tema": mensaje.topic, "payload": parseado, "timestamp": ahoraUtc()})

    def trabajadorMqtt(self) -> None:
        while True:
            try:
                self.clienteMqtt.connect(Configuracion.MQTT_HOST, Configuracion.MQTT_PORT, keepalive=30)
                self.clienteMqtt.loop_forever(retry_first_connection=True)
            except Exception as exc:
                self.actualizarEstado(mqttConectado=False, ultimoError=f"Error en hilo MQTT: {exc}")
                time.sleep(2.0)



# Instancia única del runtime para Streamlit
@st.cache_resource
def obtenerRuntime() -> RuntimePuenteMqtt:
    return RuntimePuenteMqtt()



# Envía los valores de las juntas
def enviarJuntas(runtime: RuntimePuenteMqtt, valores: Dict[str, float]) -> None:
    import math
    cuerpo = {nombreJunta: math.radians(limitarJunta(nombreJunta, valor)) for nombreJunta, valor in valores.items()}
    payload = construirPayload("comando_juntas", {"joints": cuerpo})
    runtime.publicar(Configuracion.TOPIC_COMMAND, payload)



# Envía los parámetros PID
def enviarPid(runtime: RuntimePuenteMqtt, valores: Dict[str, Any]) -> None:
    nombreJunta = str(valores["joint"])
    setpoint = limitarJunta(nombreJunta, float(valores["setpoint"]))

    cuerpo = {
        "junta": nombreJunta,
        "habilitado": bool(valores["enabled"]),
        "kp": float(valores["kp"]),
        "ki": float(valores["ki"]),
        "kd": float(valores["kd"]),
        "setpoint": setpoint,
        "tiempoRampa": max(0.0, float(valores["ramp_time_s"])),
        "ciclosRampa": max(0, int(valores["ramp_cycles"])),
        "tolerancia": max(0.01, float(valores["tolerance_deg"])),
        "ciclosEstabilizacion": max(1, int(valores["settle_cycles"])),
    }

    payload = construirPayload("comando_pid", cuerpo)
    runtime.publicar(Configuracion.TOPIC_PID, payload)



# Envía el modo de operación
def enviarModo(runtime: RuntimePuenteMqtt, nombreModo: str) -> None:
    payload = construirPayload("comando_modo", {"modo": nombreModo})
    runtime.publicar(Configuracion.TOPIC_MODE, payload)



# Interfaz principal de Streamlit
def principal() -> None:
    st.set_page_config(page_title="Panel MQTT Robot", layout="wide")

    # Broker embebido (singleton)
    if "broker" not in st.session_state:
        broker = _BrokerEmbebido()
        activo = broker.iniciar()
        st.session_state["broker"] = broker
        st.session_state["broker_activo"] = activo

    runtime = obtenerRuntime()

    st.title("Panel MQTT Robot")
    st.caption("Panel 100% Python con Streamlit. No HTML/CSS manual.")

    c1, c2, c3 = st.columns(3)
    estado = runtime.obtenerEstado()
    c1.metric("MQTT Cliente", "Conectado" if estado.get("mqttConectado") else "Desconectado")
    c2.metric("Broker", "Activo" if st.session_state.get("broker_activo") else ("No disponible" if not _BROKER_DISPONIBLE else "Iniciando"))
    c3.caption(f"Broker: {Configuracion.MQTT_HOST}:{Configuracion.MQTT_PORT}")

    if estado.get("ultimoError"):
        st.warning(str(estado["ultimoError"]))

    st.subheader("Enviar Grados De Juntas")
    with st.form("formularioJuntas"):
        valoresJuntas: Dict[str, float] = {}
        columnas = st.columns(len(limitesJuntas))
        for idx, (nombreJunta, limites) in enumerate(limitesJuntas.items()):
            with columnas[idx]:
                valoresJunta = st.number_input(
                    label=f"{nombreJunta} ({limites['min']} a {limites['max']})",
                    min_value=float(limites["min"]),
                    max_value=float(limites["max"]),
                    value=float(limites["default"]),
                    step=0.1,
                    key=f"juntas_{nombreJunta}",
                )
                valoresJuntas[nombreJunta] = valoresJunta
        enviarJuntasBtn = st.form_submit_button("Enviar Juntas")

    if enviarJuntasBtn:
        if not runtime.obtenerEstado().get("mqttConectado"):
            st.error("Sin conexión MQTT. Espera a que el broker responda e intenta de nuevo.")
        else:
            try:
                enviarJuntas(runtime, valoresJuntas)
                st.success("Comando de juntas enviado")
            except Exception as exc:
                runtime.actualizarEstado(ultimoError=f"Error al publicar: {exc}")
                st.error(f"Error MQTT al enviar juntas: {exc}")

    st.subheader("Enviar PID")
    nombresJuntas = list(limitesJuntas.keys())
    with st.form("formularioPid"):
        juntaSeleccionada = st.selectbox("Junta", options=nombresJuntas, index=0)
        valoresPorDefecto = pidPorDefecto[juntaSeleccionada]

        p1, p2, p3 = st.columns(3)
        kp = p1.number_input("Kp", value=float(valoresPorDefecto["kp"]), step=0.01, format="%.4f")
        ki = p2.number_input("Ki", value=float(valoresPorDefecto["ki"]), step=0.01, format="%.4f")
        kd = p3.number_input("Kd", value=float(valoresPorDefecto["kd"]), step=0.01, format="%.4f")

        s1, s2, s3 = st.columns(3)
        setpoint = s1.number_input(
            "Setpoint (deg)",
            min_value=float(limitesJuntas[juntaSeleccionada]["min"]),
            max_value=float(limitesJuntas[juntaSeleccionada]["max"]),
            value=float(limitesJuntas[juntaSeleccionada]["default"]),
            step=0.1,
        )
        tiempoRampa = s2.number_input("Tiempo de rampa (s)", min_value=0.0, value=0.0, step=0.1)
        ciclosRampa = s3.number_input("Ciclos de rampa", min_value=0, value=0, step=1)

        t1, t2, t3 = st.columns(3)
        habilitado = t1.checkbox("Habilitado", value=True)
        tolerancia = t2.number_input("Tolerancia (deg)", min_value=0.01, value=0.6, step=0.01)
        ciclosEstabilizacion = t3.number_input("Ciclos de estabilización", min_value=1, value=3, step=1)

        enviarPidBtn = st.form_submit_button("Enviar PID")

    if enviarPidBtn:
        if not runtime.obtenerEstado().get("mqttConectado"):
            st.error("Sin conexión MQTT. Espera a que el broker responda e intenta de nuevo.")
        else:
            try:
                enviarPid(
                    runtime,
                    {
                        "joint": juntaSeleccionada,
                        "enabled": habilitado,
                        "kp": kp,
                        "ki": ki,
                        "kd": kd,
                        "setpoint": setpoint,
                        "ramp_time_s": tiempoRampa,
                        "ramp_cycles": ciclosRampa,
                        "tolerance_deg": tolerancia,
                        "settle_cycles": ciclosEstabilizacion,
                    },
                )
                st.success("Comando PID enviado")
            except Exception as exc:
                runtime.actualizarEstado(ultimoError=f"Error al publicar: {exc}")
                st.error(f"Error MQTT al enviar PID: {exc}")

    st.subheader("Control De Modo")
    stopDeshabilitado = not runtime.obtenerEstado().get("mqttConectado", False)
    if st.button("STOP", type="primary", disabled=stopDeshabilitado):
        if not runtime.obtenerEstado().get("mqttConectado"):
            st.error("Sin conexión MQTT. Espera a que el broker responda e intenta de nuevo.")
        else:
            try:
                enviarModo(runtime, "stop")
                st.success("STOP enviado")
            except Exception as exc:
                runtime.actualizarEstado(ultimoError=f"Error al publicar: {exc}")
                st.error(f"Error MQTT al enviar STOP: {exc}")
    if stopDeshabilitado:
        st.caption("Botón deshabilitado: sin conexión MQTT.")

    st.subheader("Estado")
    st.json(runtime.obtenerEstado(), expanded=True)
    if st.button("Refrescar estado"):
        st.rerun()



if __name__ == "__main__":
    import subprocess
    import sys

    # Detecta si este archivo ya está corriendo dentro del runtime de Streamlit.
    enStreamlit = False
    try:
        from streamlit.runtime.scriptrunner import get_script_run_ctx
        enStreamlit = get_script_run_ctx() is not None
    except Exception:
        pass

    if enStreamlit:
        principal()
    else:
        subprocess.run(
            [
                sys.executable,
                "-m",
                "streamlit",
                "run",
                __file__,
                "--server.address",
                "127.0.0.1",
                "--server.port",
                "8501",
            ],
            check=False,
        )
