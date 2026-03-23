"""
EJEMPLO 08 - Script Fusion 360
Recibe parametros PID del dashboard y controla las juntas.
"""
import adsk.core, adsk.fusion
import json, math, os, threading, time, traceback
import paho.mqtt.client as mqtt

BROKER     = "127.0.0.1"
PUERTO     = 1883
TOPIC_PID  = "robotarm/pid"
TOPIC_MODE = "robotarm/mode"
EVENT_ID   = "Ej08Event"

_app       = None
_corriendo = True
_evento    = None
_handler   = None
_pids      = {}   # {nombre_junta: PID activo}
_pid_lock  = threading.Lock()


class PID:
    def __init__(self, kp, ki, kd, setpoint, dt=0.05, tolerancia=0.01, ciclos=3):
        self.kp = kp; self.ki = ki; self.kd = kd
        self.setpoint   = setpoint
        self.dt         = dt
        self.tolerancia = tolerancia
        self.ciclos_ok  = ciclos
        self._integral  = 0.0
        self._error_ant = 0.0
        self.activo     = True
        self._hilo      = threading.Thread(target=self._loop, daemon=True)
        self._hilo.start()

    def detener(self):
        self.activo = False

    def _loop(self):
        convergido = 0
        nombre = self._nombre
        print(f"[PID] {nombre} iniciado → {math.degrees(self.setpoint):.1f}°")

        while self.activo and _corriendo:
            pos = _leer_junta(nombre)
            if pos is None:
                time.sleep(self.dt)
                continue

            error          = self.setpoint - pos
            self._integral += error * self.dt
            derivada        = (error - self._error_ant) / self.dt
            self._error_ant = error
            salida          = (self.kp * error) + (self.ki * self._integral) + (self.kd * derivada)

            nueva_pos = pos + salida * self.dt
            if _evento and _app:
                try:
                    _app.fireCustomEvent(EVENT_ID, json.dumps({nombre: nueva_pos}))
                except Exception:
                    pass

            if abs(error) < self.tolerancia:
                convergido += 1
                if convergido >= self.ciclos_ok:
                    print(f"[PID] {nombre} convergido")
                    self.activo = False
                    break
            else:
                convergido = 0

            time.sleep(self.dt)

        print(f"[PID] {nombre} detenido")


def _leer_junta(nombre):
    try:
        design = _app.activeProduct
        for j in design.rootComponent.joints:
            if j.name == nombre:
                return j.jointMotion.rotationValue
    except Exception:
        pass
    return None


class Manejador(adsk.core.CustomEventHandler):
    def __init__(self):
        super().__init__()

    def notify(self, args):
        try:
            datos  = json.loads(adsk.core.CustomEventArgs.cast(args).additionalInfo)
            design = _app.activeProduct
            for nombre, rad in datos.items():
                for j in design.rootComponent.joints:
                    if j.name == nombre:
                        j.jointMotion.rotationValue = rad
            _app.activeViewport.refresh()
        except Exception:
            print(traceback.format_exc())


def hilo_mqtt():
    def al_conectar(client, userdata, flags, rc, props=None):
        if rc == 0:
            client.subscribe(TOPIC_PID)
            client.subscribe(TOPIC_MODE)
            print("MQTT conectado")

    def al_recibir(client, userdata, msg):
        try:
            payload = json.loads(msg.payload.decode())
            tipo    = payload.get("type", "")
            body    = payload.get("body", {})

            if tipo == "pid_command":
                nombre    = body.get("joint")
                enabled   = body.get("enabled", True)
                kp        = float(body.get("kp", 2.0))
                ki        = float(body.get("ki", 0.1))
                kd        = float(body.get("kd", 0.05))
                sp_rad    = math.radians(float(body.get("setpoint", 0.0)))
                tol_rad   = math.radians(float(body.get("tolerance_deg", 0.5)))
                ciclos    = int(body.get("settle_cycles", 3))

                with _pid_lock:
                    if nombre in _pids:
                        _pids[nombre].detener()
                    if enabled and nombre:
                        p = PID(kp, ki, kd, sp_rad, tolerancia=tol_rad, ciclos=ciclos)
                        p._nombre = nombre
                        _pids[nombre] = p
                        print(f"PID {nombre}: Kp={kp} Ki={ki} Kd={kd} SP={math.degrees(sp_rad):.1f}°")

            elif tipo == "mode_command" and body.get("mode") == "stop":
                with _pid_lock:
                    for p in _pids.values():
                        p.detener()
                    _pids.clear()
                print("STOP recibido")

        except Exception as e:
            print(f"Error: {e}")

    c = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2, client_id="fusion-ej08")
    c.on_connect = al_conectar
    c.on_message = al_recibir
    try:
        c.connect(BROKER, PUERTO)
        while _corriendo:
            c.loop(timeout=0.1)
    except Exception as e:
        print(f"Error MQTT: {e}")
    finally:
        c.disconnect()


def run(context):
    global _app, _corriendo, _evento, _handler
    _app       = adsk.core.Application.get()
    _corriendo = True

    _evento  = _app.registerCustomEvent(EVENT_ID)
    _handler = Manejador()
    _evento.add(_handler)

    threading.Thread(target=hilo_mqtt, daemon=True).start()
    print("Ejemplo 08 activo — configura PID en el dashboard")

    while _corriendo:
        adsk.doEvents()
        time.sleep(0.05)


def stop(context):
    global _corriendo
    _corriendo = False
    with _pid_lock:
        for p in _pids.values():
            p.detener()
        _pids.clear()
    if _evento and _handler:
        _evento.remove(_handler)
        _app.unregisterCustomEvent(EVENT_ID)
    print("Ejemplo 08 detenido.")
