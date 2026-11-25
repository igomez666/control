"""Control PID: ejemplos con python-control (0.10) y numpy.

Planta de ejemplo: G(s) = 1 / (s + 1)^3
"""
import numpy as np
import control as ct

s = ct.tf('s')
G = 1 / (s + 1)**3


def pid(Kp, Ki=0.0, Kd=0.0, N=10):
    """PID en paralelo con la derivada filtrada: Kp + Ki/s + Kd s / (1 + Td/N s)."""
    C = ct.tf([Kp], [1])
    if Ki:
        C = C + Ki / s
    if Kd:
        Td = Kd / Kp
        C = C + Kd * s / (1 + Td / N * s)
    return C


def resumen(C, nombre):
    T = ct.feedback(C * G, 1)
    i = ct.step_info(T, T=np.linspace(0, 60, 60001))
    print(f"{nombre:<26} Mp = {i['Overshoot']:5.1f} %   ts = {i['SettlingTime']:5.2f} s   "
          f"y(∞) = {i['SteadyStateValue']:.3f}")


# --- 1. Efecto de cada acción ---------------------------------------------------
resumen(pid(2), "P   (Kp=2)")
resumen(pid(2, 0.6), "PI  (Kp=2, Ki=0.6)")
resumen(pid(2, 0.6, 1), "PID (Kp=2, Ki=0.6, Kd=1)")

# --- 2. Ziegler–Nichols en bucle cerrado ------------------------------------------
Ku, _, w180, _ = ct.margin(G)          # ganancia crítica y frecuencia de oscilación
Tu = 2 * np.pi / w180
Kp, Ti, Td = 0.6 * Ku, Tu / 2, Tu / 8
print(f"\nKu = {Ku:.2f}   Tu = {Tu:.2f} s   ->   Kp = {Kp:.2f}, Ti = {Ti:.2f} s, Td = {Td:.2f} s")
resumen(pid(Kp, Kp / Ti, Kp * Td), "Ziegler–Nichols")
resumen(pid(3, 1.2, 2.5), "ZN retocado")

# --- 3. Windup: PI con actuador saturado (simulación de Euler) ------------------
def sim_windup(modo, Kp=4, Ki=4, umax=1.5, Tt=0.5, dt=1e-3, tf=12):
    """Planta 1/(2s+1). modo: 'sin' (sin anti-windup), 'clamp' o 'back'."""
    n = int(tf / dt); y = I = 0.0; Y = np.zeros(n); U = np.zeros(n)
    for k in range(n):
        r = 1.0 if k * dt < 8 else 0.0
        e = r - y
        v = Kp * e + I                      # salida del PID sin saturar
        u = np.clip(v, -umax, umax)         # el actuador satura
        if modo == "sin":
            I += Ki * e * dt
        elif modo == "clamp":               # integración condicional
            if v == u or np.sign(e) != np.sign(v):
                I += Ki * e * dt
        else:                               # back-calculation
            I += (Ki * e + (u - v) / Tt) * dt
        y += (u - y) / 2 * dt               # planta 1/(2s+1)
        Y[k], U[k] = y, u
    return Y, U

for modo in ["sin", "clamp", "back"]:
    Y, _ = sim_windup(modo)
    print(f"windup ({modo:<5}): sobreoscilación = {100 * (Y.max() - 1):.1f} %")
