"""Trimado y linealización de un péndulo con python-control (probado con la versión 0.10.2).

Péndulo con un par de entrada en el eje:
    m L² θ'' = τ − m g L sin θ − b θ'
Estados x = [θ, ω], entrada u = τ, salida y = θ.
Instalación:  pip install control
"""
import numpy as np
import matplotlib.pyplot as plt
import control as ct

# --- 1. Modelo no lineal -----------------------------------------------------
params = {"m": 1.0, "L": 1.0, "g": 9.81, "b": 0.5}


def pendulo_rhs(t, x, u, params):
    m, L, g, b = params["m"], params["L"], params["g"], params["b"]
    theta, omega = x
    domega = (u[0] - m*g*L*np.sin(theta) - b*omega) / (m*L**2)
    return [omega, domega]


def pendulo_out(t, x, u, params):
    return [x[0]]


pendulo = ct.nlsys(pendulo_rhs, pendulo_out, name="pendulo",
                   states=["theta", "omega"], inputs=["tau"],
                   outputs=["theta"], params=params)

# --- 2. Trimado: equilibrio con θ = π/4 --------------------------------------
theta0 = np.pi / 4
op = ct.find_operating_point(
    pendulo,
    initial_state=[theta0, 0],  # estimación del estado
    inputs=[0],                 # estimación del par (libre)
    outputs=[theta0],           # queremos θ = π/4
    output_indices=[0],         # esa salida queda fijada
    input_indices=[])           # τ es incógnita
x0, u0 = op.states, op.inputs
print("x0 =", np.round(x0, 4))
print("u0 =", np.round(u0, 4), "N·m")

# --- 3. Linealización y análisis ---------------------------------------------
lin = ct.linearize(pendulo, x0, u0)
print("A =\n", np.round(lin.A, 3))
print("B =\n", np.round(lin.B, 3))
print("C =", lin.C, "  D =", lin.D)
G = ct.ss2tf(lin)
print("G(s) =", G)
print("polos =", np.round(ct.poles(lin), 3))
wn, zeta, _ = ct.damp(lin, doprint=False)
print(f"wn = {wn[0]:.3f} rad/s   zeta = {zeta[0]:.3f}")

# --- 4. Validación: modelo no lineal frente a lineal -------------------------
T = np.linspace(0, 20, 2001)
fig, axs = plt.subplots(1, 2, figsize=(11, 3.8))
for ax, dtau in zip(axs, [0.1, 1.0]):                        # escalones de par
    U = (u0[0] + dtau) * np.ones_like(T)
    nl = ct.input_output_response(pendulo, T, U, X0=x0)       # no lineal
    li = ct.forced_response(lin, T, U - u0[0], X0=[0, 0])     # lineal (desviaciones)
    theta_nl = nl.outputs                 # ángulo absoluto
    theta_li = theta0 + li.outputs        # desviación + punto de trimado
    ax.plot(T, np.degrees(theta_nl), lw=2.5, label="no lineal")
    ax.plot(T, np.degrees(theta_li), "--", lw=2.5, label="lineal")
    ax.set_title(f"Escalón de par δτ = {dtau} N·m")
    ax.set_xlabel("tiempo (s)"); ax.set_ylabel("θ (grados)"); ax.grid(True); ax.legend()
plt.tight_layout()
plt.savefig("pendulo_validacion.png", dpi=150)
plt.show()
