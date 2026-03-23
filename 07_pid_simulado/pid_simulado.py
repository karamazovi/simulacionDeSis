"""
EJEMPLO 07 - HORA 7: PID simulado en Python puro
NO requiere Fusion 360 ni MQTT. Solo Python.

Simula una junta que tiene inercia y muestra como el PID la lleva al setpoint.
Ejecutar: python pid_simulado.py
"""
import time
import math


class ControladorPID:
    def __init__(self, kp, ki, kd, dt=0.05):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.dt = dt
        self._integral       = 0.0
        self._error_anterior = 0.0

    def calcular(self, setpoint, medicion):
        error     = setpoint - medicion
        self._integral        += error * self.dt
        derivada               = (error - self._error_anterior) / self.dt
        self._error_anterior   = error
        return (self.kp * error) + (self.ki * self._integral) + (self.kd * derivada)

    def reset(self):
        self._integral       = 0.0
        self._error_anterior = 0.0


class JuntaSimulada:
    """Simula una junta con inercia (sistema de primer orden)."""
    def __init__(self, posicion_inicial=0.0, inercia=0.8):
        self.posicion = posicion_inicial
        self.inercia  = inercia          # 0 = sin inercia, 1 = mucha inercia

    def aplicar(self, salida_pid, dt):
        # La junta no sigue instantaneamente — tiene inercia
        self.posicion += salida_pid * dt * (1.0 - self.inercia)
        return self.posicion


def barra_progreso(actual, setpoint, rango=90, ancho=30):
    """Visualizacion simple en terminal."""
    pct = max(0.0, min(1.0, (actual + rango) / (2 * rango)))
    lleno = int(pct * ancho)
    barra = "=" * lleno + "-" * (ancho - lleno)
    sp_pct = max(0.0, min(1.0, (setpoint + rango) / (2 * rango)))
    sp_pos = int(sp_pct * ancho)
    barra_list = list(barra)
    if 0 <= sp_pos < ancho:
        barra_list[sp_pos] = "|"
    return "[" + "".join(barra_list) + "]"


# Parametros
KP = 3.0
KI = 0.5
KD = 0.2
DT = 0.05

SETPOINTS = [45.0, -30.0, 90.0, 0.0]   # secuencia de objetivos

pid   = ControladorPID(kp=KP, ki=KI, kd=KD, dt=DT)
junta = JuntaSimulada(posicion_inicial=0.0, inercia=0.7)

print("=" * 60)
print("SIMULACION PID - Junta con inercia")
print(f"Kp={KP}  Ki={KI}  Kd={KD}  dt={DT}s")
print("| = setpoint,  = = posicion actual")
print("=" * 60)

for setpoint in SETPOINTS:
    pid.reset()
    print(f"\nSetpoint: {setpoint:.1f} grados")
    pasos = 0
    convergido = 0

    while True:
        salida   = pid.calcular(setpoint, junta.posicion)
        posicion = junta.aplicar(salida, DT)
        error    = abs(setpoint - posicion)

        barra = barra_progreso(posicion, setpoint)
        print(f"\r  {barra}  pos={posicion:7.2f}°  err={error:.2f}°", end="", flush=True)

        if error < 0.5:
            convergido += 1
            if convergido >= 5:
                print(f"\n  Convergido en {pasos * DT:.2f}s")
                break
        else:
            convergido = 0

        pasos += 1
        if pasos > 500:
            print(f"\n  Timeout — error final: {error:.2f}°")
            break

        time.sleep(DT)

print("\nSimulacion completada.")
