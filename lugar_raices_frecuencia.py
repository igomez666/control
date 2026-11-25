"""Lugar de las raíces, Bode, Nyquist y márgenes con python-control (0.10).

Ejemplo: L(s) = K / (s (s+1) (s+2)) con realimentación unitaria.
"""
import numpy as np
import matplotlib.pyplot as plt
import control as ct

s = ct.tf('s')
G = 1 / (s * (s + 1) * (s + 2))

# --- 1. Lugar de las raíces --------------------------------------------------
plt.figure()
ct.root_locus_plot(G)                 # polos del bucle cerrado al variar K de 0 a ∞
plt.title("Lugar de las raíces de 1/(s(s+1)(s+2))")

# ganancia que da ζ = 0.5 en los polos dominantes (búsqueda sencilla)
for K in np.arange(0.3, 6, 1e-4):
    p = ct.poles(ct.feedback(K * G, 1))
    c = p[np.abs(p.imag) > 1e-9]
    if len(c) and abs(-c[0].real / abs(c[0]) - 0.5) < 1e-4:
        print(f"ζ = 0,5 con K = {K:.2f}; polos = {np.round(p, 3)}")
        break

# --- 2. Diagrama de Bode y márgenes (K = 2) -----------------------------------
L = 2 * G
plt.figure()
ct.bode_plot(L, display_margins=True)

gm, pm, w180, wc = ct.margin(L)
print(f"Margen de ganancia = {gm:.2f} ({20*np.log10(gm):.1f} dB) en ω = {w180:.3f} rad/s")
print(f"Margen de fase     = {pm:.1f}° en ωc = {wc:.3f} rad/s")
print(f"Retardo máximo tolerable = {np.radians(pm) / wc:.2f} s")

# --- 3. Diagrama de Nyquist y criterio ----------------------------------------
for K in [2, 10]:
    plt.figure()
    resp = ct.nyquist_response(K * G)
    resp.plot()
    P = sum(np.real(ct.poles(K * G)) > 0)     # polos de L en el semiplano derecho
    N = resp.count                            # rodeos al −1 (sentido horario)
    print(f"K = {K:2d}: N = {N}, P = {P}  ->  Z = N + P = {N + P} polos inestables en bucle cerrado")

plt.show()
