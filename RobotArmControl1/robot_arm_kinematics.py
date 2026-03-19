"""Funciones de cinemática para control de objetivo XY."""

import math


def solve_planar_2link(x, y, link_1, link_2, elbow_up=True):
    """
    Resuelve la cinemática inversa (IK) de un brazo plano de 2 eslabones.
    Devuelve (hombroGrados, codoGradosRelativo).
    """
    posicionX = float(x)
    posicionY = float(y)
    longitudEslabon1 = float(link_1)
    longitudEslabon2 = float(link_2)

    distanciaCuadrada = (posicionX * posicionX) + (posicionY * posicionY)
    distancia = math.sqrt(distanciaCuadrada)

    if distancia > (longitudEslabon1 + longitudEslabon2) or distancia < abs(longitudEslabon1 - longitudEslabon2):
        raise ValueError("Punto fuera del alcance del brazo")

    cosenoCodo = (distanciaCuadrada - (longitudEslabon1 * longitudEslabon1) - (longitudEslabon2 * longitudEslabon2)) / (2.0 * longitudEslabon1 * longitudEslabon2)
    cosenoCodo = max(-1.0, min(1.0, cosenoCodo))
    anguloCodoRad = math.acos(cosenoCodo)
    if elbow_up:
        anguloCodoRad = -anguloCodoRad

    k1 = longitudEslabon1 + (longitudEslabon2 * math.cos(anguloCodoRad))
    k2 = longitudEslabon2 * math.sin(anguloCodoRad)
    anguloHombroRad = math.atan2(posicionY, posicionX) - math.atan2(k2, k1)

    hombroGrados = math.degrees(anguloHombroRad)
    codoGradosRelativo = math.degrees(anguloCodoRad)
    return hombroGrados, codoGradosRelativo
