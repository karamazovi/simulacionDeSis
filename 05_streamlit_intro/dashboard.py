"""
EJEMPLO 05 - HORA 5: Introduccion a Streamlit
Dashboard basico con sliders. NO requiere MQTT ni Fusion.
Solo muestra como funciona Streamlit.

Ejecutar: streamlit run dashboard.py
"""
import streamlit as st
import math

st.set_page_config(page_title="Ejemplo 05 - Streamlit Intro", layout="centered")
st.title("Control de Juntas (sin MQTT)")
st.caption("Mueve los sliders y observa los valores en tiempo real.")

st.subheader("Posicion de juntas")

base   = st.slider("JointBase",   -180, 180,  0, step=1, help="Rotacion base en grados")
hombro = st.slider("JointHombro",  -90,  90,  0, step=1, help="Rotacion hombro en grados")
codo   = st.slider("JointCodo",   -135, 135,  0, step=1, help="Rotacion codo en grados")

st.divider()
st.subheader("Valores calculados")

col1, col2, col3 = st.columns(3)
col1.metric("Base (rad)",   f"{math.radians(base):.3f}")
col2.metric("Hombro (rad)", f"{math.radians(hombro):.3f}")
col3.metric("Codo (rad)",   f"{math.radians(codo):.3f}")

st.divider()
st.subheader("JSON que se enviaria por MQTT")
import json
payload = {
    "type": "joint_command",
    "body": {
        "joints": {
            "JointBase":   round(math.radians(base),   4),
            "JointHombro": round(math.radians(hombro), 4),
            "JointCodo":   round(math.radians(codo),   4),
        }
    }
}
st.json(payload)
