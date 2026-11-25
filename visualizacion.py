# -*- coding: utf-8 -*-
"""
FUNCIONES DE VISUALIZACIÓN PARA LA DINÁMICA LONGITUDINAL
========================================================

Funciones auxiliares de gráficas y animación usadas en el cuaderno
simulacion_longitudinal.ipynb. No contienen nada del modelo ni del control:
solo sirven para dibujar los resultados de las simulaciones.

Contenido del fichero:
    senal(resp, nombre)      -> extrae una salida de una respuesta de python-control por su nombre
    dibujar(resp, titulo...) -> dibuja h, V, theta, alpha, q y el timón (pedido y real)
    rotar(puntos, angulo)    -> gira un conjunto de puntos 2D
    animar_actitud(...)      -> animación de un avión esquemático que sube/baja y cabecea
    mostrar(anim)            -> muestra una animación dentro de un cuaderno Jupyter
"""

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation   # para crear animaciones

plt.rcParams['animation.embed_limit'] = 60       # tamaño máximo de la animación [MB]


# ---------------------------------------------------------------------------
# GRÁFICAS DE LAS SIMULACIONES CON CONTROL
# ---------------------------------------------------------------------------

def senal(resp, nombre):
    """Devuelve la salida llamada 'nombre' de una respuesta de python-control."""
    i = resp.output_labels.index(nombre)
    return resp.outputs[i]


def dibujar(resp, titulo, theta_ref=None, h_ref=None):
    """Dibuja h, V, theta, alpha, q y el timón (pedido y real)."""
    t = resp.time
    fig, ejes = plt.subplots(3, 2, figsize=(12, 9), sharex=True)
    ejes = ejes.flatten()

    ejes[0].plot(t, senal(resp, 'h'), 'b', label='h')
    if h_ref is not None:
        ejes[0].plot(t, h_ref, 'r--', label='referencia'); ejes[0].legend()
    ejes[0].set_ylabel('h [m]')

    ejes[1].plot(t, senal(resp, 'V'), 'b')
    ejes[1].set_ylabel('V [m/s]')

    ejes[2].plot(t, np.degrees(senal(resp, 'theta')), 'b', label='theta')
    if theta_ref is not None:
        ejes[2].plot(t, np.degrees(theta_ref), 'r--', label='referencia'); ejes[2].legend()
    ejes[2].set_ylabel('theta [grados]')

    ejes[3].plot(t, np.degrees(senal(resp, 'alpha')), 'b')
    ejes[3].set_ylabel('alpha [grados]')

    ejes[4].plot(t, np.degrees(senal(resp, 'q')), 'b')
    ejes[4].set_ylabel('q [grados/s]')
    ejes[4].set_xlabel('tiempo [s]')

    ejes[5].plot(t, np.degrees(senal(resp, 'de_cmd')), 'r', label='pedido (cmd)')
    ejes[5].plot(t, np.degrees(senal(resp, 'Delev')), 'b', label='real (actuador)')
    ejes[5].set_ylabel('Delev [grados]'); ejes[5].set_xlabel('tiempo [s]')
    ejes[5].legend()

    for e in ejes:
        e.grid(True)
    fig.suptitle(titulo)
    plt.tight_layout()
    plt.show()


# ---------------------------------------------------------------------------
# ANIMACIÓN DE LA ACTITUD DEL AVIÓN
# ---------------------------------------------------------------------------

# Silueta del avión vista de lado, en ejes cuerpo: morro hacia +x, longitud ~1
SILUETA = np.array([
    [ 0.50,  0.00],   # morro
    [ 0.42,  0.05],
    [ 0.25,  0.08],   # parabrisas
    [ 0.05,  0.08],   # techo de la cabina
    [-0.32,  0.04],
    [-0.42,  0.22],   # deriva (timón de dirección)
    [-0.52,  0.22],
    [-0.52,  0.01],
    [-0.30, -0.04],
    [ 0.40, -0.05],   # vientre
])
ALA   = np.array([[0.15, 0.03], [-0.05, 0.03]])    # ala (vista de canto: una línea)
ESTAB = np.array([[-0.36, 0.03], [-0.56, 0.03]])   # estabilizador horizontal


def rotar(puntos, angulo):
    """Gira un conjunto de puntos (N x 2) un ángulo [rad] alrededor del origen."""
    c, s = np.cos(angulo), np.sin(angulo)
    R = np.array([[c, -s],
                  [s,  c]])
    return puntos @ R.T


def animar_actitud(t, h, theta, alpha, q, titulo='', h_ref=None,
                   amplificacion=1.0, n_fotogramas=150, intervalo=60):
    """Anima un avión esquemático que sube/baja con h y gira con theta.

    t, h, theta, alpha, q : vectores de la simulación (ángulos en rad, q en rad/s)
    h_ref          : vector con la altitud de referencia (opcional)
    amplificacion  : factor para exagerar los ángulos en el dibujo (1 = real)
    n_fotogramas   : número de imágenes de la animación
    intervalo      : tiempo entre imágenes al reproducir [ms]
    """
    # --- Instantes que vamos a dibujar (no hace falta dibujar todos) ---
    idx = np.linspace(0, len(t) - 1, n_fotogramas).astype(int)

    # --- Escala vertical: la variación de altitud se lleva a [-0.6, 0.6] del dibujo ---
    h0 = h[0]
    rango = max(np.max(np.abs(h - h0)), 1.0)
    if h_ref is not None:
        rango = max(rango, np.max(np.abs(h_ref - h0)))
    escala = rango / 0.6                  # metros por unidad de dibujo
    y = (h - h0) / escala                 # posición vertical del avión en el dibujo

    gamma = theta - alpha                 # ángulo de trayectoria

    # --- Figura con dos paneles ---
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(12, 4.8),
                                   gridspec_kw={'width_ratios': [1.15, 1]})

    # Panel izquierdo: la "ventana" donde se ve el avión
    ax1.set_xlim(-1.75, 1.3); ax1.set_ylim(-1.0, 1.0)   # hueco a la izquierda para el texto
    ax1.set_aspect('equal')                                   # para que los ángulos se vean bien
    ax1.set_facecolor('#EAF2FB')                              # cielo
    ax1.set_xticks([])
    marcas = np.array([-0.6, -0.3, 0, 0.3, 0.6])
    ax1.set_yticks(marcas)
    ax1.set_yticklabels(['%.0f' % (h0 + m * escala) for m in marcas])
    ax1.set_ylabel('altitud [m]')
    ax1.axhline(0, color='gray', linestyle=':', linewidth=1)  # altitud inicial
    if h_ref is not None:
        ax1.axhline((h_ref[-1] - h0) / escala, color='r', linestyle='--',
                    linewidth=1, label='h de referencia')
        ax1.legend(loc='lower right', fontsize=9)
    titulo_izq = 'Actitud'
    if amplificacion != 1:
        titulo_izq += ' (ángulos x%.0f)' % amplificacion
    ax1.set_title(titulo_izq)

    # Elementos que se moverán (se crean vacíos y se actualizan en cada fotograma)
    cuerpo = plt.Polygon(SILUETA, closed=True, color='#14213D')
    ax1.add_patch(cuerpo)
    linea_ala,   = ax1.plot([], [], color='#14213D', linewidth=5, solid_capstyle='round')
    linea_estab, = ax1.plot([], [], color='#14213D', linewidth=3, solid_capstyle='round')
    linea_eje,   = ax1.plot([], [], color='#14213D', linewidth=1, linestyle='--')  # eje del avión
    linea_vel,   = ax1.plot([], [], color='#D9782D', linewidth=2.5)               # vector velocidad
    punta_vel,   = ax1.plot([], [], color='#D9782D', marker='o', markersize=6)
    arco_q,      = ax1.plot([], [], color='#C0392B', linewidth=3)                 # indicador de q
    punta_q,     = ax1.plot([], [], color='#C0392B', marker='o', markersize=7)
    texto = ax1.text(-1.7, 0.95, '', va='top', fontsize=9, family='monospace',
                     bbox=dict(facecolor='white', alpha=0.85, edgecolor='none'))

    # Panel derecho: h y theta frente al tiempo
    ax2.plot(t, h, 'b', label='h')
    if h_ref is not None:
        ax2.plot(t, h_ref, 'b:', linewidth=1)
    ax2.set_xlabel('tiempo [s]'); ax2.set_ylabel('h [m]', color='b'); ax2.grid(True)
    ax2b = ax2.twinx()                                   # segundo eje vertical para theta
    ax2b.plot(t, np.degrees(theta), 'g', label='theta')
    ax2b.set_ylabel('theta [grados]', color='g')
    cursor = ax2.axvline(t[0], color='k', linewidth=1)   # marca el instante actual
    ax2.set_title('Evolución temporal')
    fig.suptitle(titulo)
    plt.tight_layout()

    def actualizar(k):
        """Dibuja el fotograma k."""
        i = idx[k]
        ang = amplificacion * theta[i]          # ángulo de giro del dibujo
        centro = np.array([0.0, y[i]])          # el avión sube y baja con h

        # Avión girado y desplazado
        cuerpo.set_xy(rotar(SILUETA, ang) + centro)
        ala = rotar(ALA, ang) + centro
        linea_ala.set_data(ala[:, 0], ala[:, 1])
        est = rotar(ESTAB, ang) + centro
        linea_estab.set_data(est[:, 0], est[:, 1])
        eje = rotar(np.array([[0.5, 0], [1.1, 0]]), ang) + centro
        linea_eje.set_data(eje[:, 0], eje[:, 1])

        # Vector velocidad, inclinado gamma
        g = amplificacion * gamma[i]
        fin = centro + 0.95 * np.array([np.cos(g), np.sin(g)])
        linea_vel.set_data([centro[0], fin[0]], [centro[1], fin[1]])
        punta_vel.set_data([fin[0]], [fin[1]])

        # Arco de q: empieza en el morro y su longitud es proporcional a q
        q_grados = np.degrees(q[i])
        apertura = np.radians(np.clip(6 * q_grados, -150, 150))  # 6 grados de arco por grado/s
        angulos = ang + np.linspace(0, apertura, 30)
        arco_q.set_data(0.62 * np.cos(angulos), centro[1] + 0.62 * np.sin(angulos))
        punta_q.set_data([0.62 * np.cos(angulos[-1])], [centro[1] + 0.62 * np.sin(angulos[-1])])

        # Valores numéricos reales
        texto.set_text('t     = %6.1f s\nh     = %6.1f m\ntheta = %6.2f grados\nq     = %6.2f grados/s'
                       % (t[i], h[i], np.degrees(theta[i]), q_grados))

        cursor.set_xdata([t[i], t[i]])
        return cuerpo, linea_ala, linea_estab, linea_eje, linea_vel, punta_vel, arco_q, punta_q, texto, cursor

    anim = FuncAnimation(fig, actualizar, frames=n_fotogramas, interval=intervalo, blit=True)
    plt.close(fig)               # evita que se muestre además una imagen estática
    return anim


def mostrar(anim):
    """Muestra la animación en el cuaderno.
    Si está instalado ffmpeg, como vídeo (ligero); si no, como secuencia de imágenes
    con botones de reproducción (más pesada, pero no necesita nada más)."""
    from IPython.display import HTML   # solo hace falta dentro de Jupyter
    try:
        return HTML(anim.to_html5_video())
    except Exception:
        return HTML(anim.to_jshtml())
