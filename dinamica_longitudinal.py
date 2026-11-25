# -*- coding: utf-8 -*-
"""
MODELO DINÁMICO DE LA DINÁMICA LONGITUDINAL DE UN AVIÓN DE ALA FIJA
(versión con la librería python-control:  pip install control)
==================================================================

Forma del modelo:   dx = f(x, u)

Vector de estado (6 componentes):
    x[0] = x      -> posición horizontal                     [m]
    x[1] = z      -> posición vertical (POSITIVA HACIA ABAJO) [m]
                     (convención aeronáutica: altitud h = -z)
    x[2] = V      -> velocidad aerodinámica (módulo)          [m/s]
    x[3] = alpha  -> ángulo de ataque                         [rad]
    x[4] = theta  -> ángulo de asiento (cabeceo)              [rad]
    x[5] = q      -> velocidad angular de cabeceo             [rad/s]

Vector de entrada (2 componentes):
    u[0] = Acc    -> palanca de gases, de 0 (ralentí) a 1 (máximo)  [-]
    u[1] = Delev  -> deflexión del timón de profundidad             [rad]
                     (positiva hacia abajo -> momento de picado)

Contenido del fichero:
    f(x, u)            -> modelo dinámico (función de Python normal)
    crear_avion()      -> el mismo modelo como sistema no lineal de python-control
    trimado(avion,V,h) -> punto de equilibrio en vuelo horizontal

Hipótesis simplificadoras:
    - Tierra plana y sin viento.
    - Avión rígido de masa constante.
    - Empuje alineado con el eje longitudinal del avión.
    - Coeficientes aerodinámicos lineales (polar parabólica para la resistencia).

Parámetros: aproximadamente los de una Cessna 172 (avioneta ligera).
"""

import numpy as np
import control as ct     # librería python-control


# ---------------------------------------------------------------------------
# PARÁMETROS DEL AVIÓN (Cessna 172, valores aproximados)
# ---------------------------------------------------------------------------

# --- Constantes físicas ---
g = 9.81          # aceleración de la gravedad [m/s^2]
rho0 = 1.225      # densidad del aire a nivel del mar [kg/m^3]

# --- Masa, inercia y geometría ---
m = 1043.0        # masa del avión [kg]
Iyy = 1825.0      # momento de inercia en cabeceo [kg*m^2]
S = 16.2          # superficie alar [m^2]
c = 1.49          # cuerda media aerodinámica [m]

# --- Motor ---
T_max = 2200.0    # empuje máximo (aprox. constante a baja velocidad) [N]

# --- Coeficiente de sustentación: CL = CL0 + CLa*alpha + CLq*(q*c/2V) + CLde*Delev ---
CL0 = 0.31        # sustentación con alpha = 0
CLa = 5.143       # pendiente de sustentación [1/rad]
CLq = 3.9         # efecto de la velocidad de cabeceo
CLde = 0.43       # efecto del timón de profundidad [1/rad]

# --- Coeficiente de resistencia (polar parabólica): CD = CD0 + k*CL^2 ---
CD0 = 0.031       # resistencia parásita
k = 0.054         # factor de resistencia inducida

# --- Coeficiente de momento de cabeceo: Cm = Cm0 + Cma*alpha + Cmq*(q*c/2V) + Cmde*Delev ---
Cm0 = -0.015      # momento con alpha = 0
Cma = -0.89       # estabilidad estática (negativo = estable) [1/rad]
Cmq = -12.4       # amortiguamiento en cabeceo
Cmde = -1.28      # eficacia del timón de profundidad [1/rad]


# ---------------------------------------------------------------------------
# FUNCIÓN DEL MODELO: dx = f(x, u)
# ---------------------------------------------------------------------------

def f(x, u):
    """Devuelve la derivada temporal del estado, dx/dt, dados el estado x y la entrada u."""

    # 1) Desempaquetar el vector de estado
    xpos  = x[0]   # posición horizontal [m]  (no influye en la dinámica)
    z     = x[1]   # posición vertical, positiva hacia abajo [m]
    V     = x[2]   # velocidad [m/s]
    alpha = x[3]   # ángulo de ataque [rad]
    theta = x[4]   # ángulo de asiento [rad]
    q     = x[5]   # velocidad de cabeceo [rad/s]

    # 2) Desempaquetar el vector de entrada
    Acc   = u[0]   # palanca de gases [0..1]
    Delev = u[1]   # timón de profundidad [rad]

    # 3) Ángulo de trayectoria: gamma = theta - alpha
    #    (ángulo entre la velocidad y el horizonte; positivo = subiendo)
    gamma = theta - alpha

    # 4) Densidad del aire: modelo exponencial sencillo con la altitud h = -z
    h = -z
    rho = rho0 * np.exp(-h / 8500.0)

    # 5) Presión dinámica: q_bar = 1/2 * rho * V^2
    q_bar = 0.5 * rho * V**2

    # 6) Velocidad de cabeceo adimensional (aparece en CL y Cm)
    q_hat = q * c / (2.0 * V)

    # 7) Coeficientes aerodinámicos
    CL = CL0 + CLa * alpha + CLq * q_hat + CLde * Delev   # sustentación
    CD = CD0 + k * CL**2                                  # resistencia
    Cm = Cm0 + Cma * alpha + Cmq * q_hat + Cmde * Delev   # momento de cabeceo

    # 8) Fuerzas y momento aerodinámicos (dimensionales)
    L = q_bar * S * CL          # sustentación [N]
    D = q_bar * S * CD          # resistencia [N]
    M = q_bar * S * c * Cm      # momento de cabeceo [N*m]

    # 9) Empuje del motor (proporcional a la palanca de gases)
    T = T_max * Acc             # [N]

    # 10) ECUACIONES DEL MOVIMIENTO
    # --- Cinemática de la posición ---
    dxpos = V * np.cos(gamma)           # avance horizontal
    dz    = -V * np.sin(gamma)          # z hacia abajo: si sube (gamma>0), z disminuye

    # --- Fuerzas en la dirección de la velocidad (ecuación de V) ---
    #     m*dV/dt = T*cos(alpha) - D - m*g*sin(gamma)
    dV = (T * np.cos(alpha) - D - m * g * np.sin(gamma)) / m

    # --- Fuerzas perpendiculares a la velocidad (ecuación de gamma) ---
    #     m*V*dgamma/dt = T*sin(alpha) + L - m*g*cos(gamma)
    dgamma = (T * np.sin(alpha) + L - m * g * np.cos(gamma)) / (m * V)

    # --- Ángulo de ataque: como alpha = theta - gamma  ->  dalpha = q - dgamma ---
    dalpha = q - dgamma

    # --- Cinemática del cabeceo ---
    dtheta = q

    # --- Momento de cabeceo: Iyy*dq/dt = M ---
    dq = M / Iyy

    # 11) Empaquetar y devolver la derivada del estado
    dx = np.array([dxpos, dz, dV, dalpha, dtheta, dq])
    return dx


# ---------------------------------------------------------------------------
# EL AVIÓN COMO SISTEMA NO LINEAL DE PYTHON-CONTROL
# ---------------------------------------------------------------------------
# python-control describe un sistema no lineal con dos funciones:
#   - updfcn(t, x, u, params) -> dx/dt   (ecuación de estado)
#   - outfcn(t, x, u, params) -> y       (salidas, lo que "medimos")
# Además, cada entrada, salida y estado tiene un NOMBRE. Los nombres sirven
# para conectar sistemas entre sí automáticamente con ct.interconnect().

def _ecuacion_estado(t, x, u, params):
    # python-control pide la forma (t, x, u, params); llamamos a nuestra f(x, u)
    return f(x, u)


def _salidas(t, x, u, params):
    # Salidas del avión: lo que los sensores pueden medir
    z, V, alpha, theta, q = x[1], x[2], x[3], x[4], x[5]
    h = -z                                  # altitud
    h_dot = V * np.sin(theta - alpha)       # velocidad vertical = V*sin(gamma)
    return np.array([x[0], h, V, alpha, theta, q, h_dot])


def crear_avion():
    """Devuelve el modelo del avión como sistema no lineal (ct.NonlinearIOSystem)."""
    avion = ct.nlsys(
        _ecuacion_estado, _salidas,
        name='avion',
        states=['x', 'z', 'V', 'alpha', 'theta', 'q'],        # 6 estados
        inputs=['Acc', 'Delev'],                              # 2 entradas
        outputs=['xpos', 'h', 'V', 'alpha', 'theta', 'q', 'h_dot'])  # 7 salidas
    return avion


# ---------------------------------------------------------------------------
# TRIMADO: vuelo horizontal, rectilíneo y a velocidad constante
# ---------------------------------------------------------------------------

def trimado(avion, V, h):
    """
    Calcula el punto de equilibrio (trimado) para vuelo horizontal
    a velocidad V [m/s] y altitud h [m], usando ct.find_operating_point().

    Le decimos a python-control:
      - qué estados están FIJOS (ix):    x, z (altitud), V y q = 0
      - qué derivadas deben ser CERO (idx): dV, dalpha, dq
      - qué salidas tienen un valor IMPUESTO (iy, y0): h_dot = 0 (no sube ni baja)
    Y él busca el resto (4 incógnitas): alpha, theta, Acc y Delev.
    Como h_dot = V*sin(gamma) = 0 obliga a gamma = 0, el resultado cumple theta = alpha.
    (dx/dt = V no es cero: el avión avanza; dtheta = q ya es cero porque q = 0)

    Devuelve:
        x_trim -> vector de estado de equilibrio (6 componentes)
        u_trim -> vector de entrada de equilibrio (2 componentes)
    """
    x_guess = [0.0, -h, V, 0.0, 0.0, 0.0]   # estimación inicial del estado
    u_guess = [0.5, 0.0]                    # estimación inicial de la entrada
    y_deseada = np.zeros(7)                 # valores deseados de las salidas
    y_deseada[6] = 0.0                      # salida 6 = h_dot = 0

    punto = ct.find_operating_point(
        avion, x_guess, u_guess, y_deseada,
        ix=[0,1, 2, 5],          # estados fijos: x, z, V, q
        idx=[2, 3, 5],            # derivadas nulas: dV, dalpha, dq
        iy=[6])                   # salida impuesta: h_dot = 0
    # 4 incógnitas (alpha, theta, Acc, Delev)  <->  4 ecuaciones (dV, dalpha, dq, h_dot)

    x_trim = np.array(punto.states)
    u_trim = np.array(punto.inputs)

    # Comprobaciones sencillas
    if np.max(np.abs(f(x_trim, u_trim)[1:])) > 1e-6:
        print("AVISO: el trimado no ha convergido bien")
    if u_trim[0] < 0.0 or u_trim[0] > 1.0:
        print("AVISO: la palanca de gases necesaria (%.2f) está fuera de [0, 1]" % u_trim[0])
    if abs(u_trim[1]) > np.radians(25):
        print("AVISO: el timón necesario supera los 25 grados")

    return x_trim, u_trim


# ---------------------------------------------------------------------------
# EJEMPLO DE USO (solo se ejecuta si se lanza este fichero directamente)
# ---------------------------------------------------------------------------
if __name__ == "__main__":
    avion = crear_avion()
    print(avion)

    x_trim, u_trim = trimado(avion, 60.0, 2000.0)
    print("\nTrimado a 60 m/s y 2000 m:")
    print("  alpha = %.2f grados" % np.degrees(x_trim[3]))
    print("  theta = %.2f grados" % np.degrees(x_trim[4]))
    print("  Acc   = %.3f" % u_trim[0])
    print("  Delev = %.2f grados" % np.degrees(u_trim[1]))
    print("dx/dt en el trimado:", f(x_trim, u_trim))
