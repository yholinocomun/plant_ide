#!/usr/bin/env python3
"""
============================================================
FASE 4 - Ensamblar el ESPACIO DE ESTADOS  (modelo WIP)
============================================================
Péndulo invertido sobre DOS RUEDAS (Wheeled Inverted Pendulum).

NO es el cart-pole clásico: aquí la "masa del carro" incluye la
inercia ROTACIONAL de las ruedas (J_w/r²), y la entrada es el PAR
del motor τ, no una fuerza F sobre un riel.

Estado:   X = [x, ẋ, θ, θ̇]ᵀ
Entrada:  u = par τ [N·m]   (τ = K_tau · PWM)
Salida:   y = [x, θ]   (x del encoder, θ del MPU)

Edita el dict PARAMS con TUS valores medidos (Fases 0,1,2,3)
y ejecuta:
    python3 ensamblar_modelo.py
============================================================
"""

import numpy as np

# ══════════════════════════════════════════════════════════════════
#  PARÁMETROS — reemplaza con TUS mediciones reales
# ══════════════════════════════════════════════════════════════════
PARAMS = {
    # --- FASE 0: mediciones físicas ---
    "M":   0.45,    # masa del cuerpo/péndulo [kg]        (balanza)
    "l":   0.09,    # distancia eje ruedas → CM cuerpo [m] (equilibrio en filo)
    "m_w": 0.05,    # masa de UNA rueda [kg]              (balanza)
    "r":   0.034,   # radio de rueda [m]                  (calibrador)
    "n":   2,       # número de ruedas

    # --- FASE 2: oscilación libre ---
    "I_p": 0.0196,  # inercia del cuerpo respecto al eje [kg·m²]  (T → I_p)

    # --- FASE 3: inercia de rueda ---
    # disco macizo: J_w = 0.5·m_w·r² ;  aro: J_w = m_w·r²
    "J_w": None,    # si None se calcula como disco

    # --- FASE 1: motor ---
    "K":   0.055,   # ganancia ω_ss/PWM [rad/s/PWM]
    "tau": 0.30,    # constante de tiempo [s]
    "u_dead": 40,   # zona muerta [PWM]
    "V_nom":  12.0, # voltaje nominal [V]
    "w_nom":  13.93,# velocidad nominal de rueda [rad/s] (133 RPM)
    "reduccion": 74.8,

    "g":   9.81,
}


# ══════════════════════════════════════════════════════════════════
def construir_modelo(p):
    M, l, r, n = p["M"], p["l"], p["r"], p["n"]
    I_p, m_w, g = p["I_p"], p["m_w"], p["g"]

    # Inercia de rueda (disco si no se da)
    J_w = p["J_w"] if p["J_w"] is not None else 0.5 * m_w * r**2

    # Masa equivalente de ruedas incluyendo su rotación
    M_w_eq = n * (m_w + J_w / r**2)

    # Denominador común
    den = (M + M_w_eq) * (I_p + M * l**2) - (M * l)**2

    # ---- Matriz A ----
    a23 = -(M * l)**2 * g / den
    a43 =  (M + M_w_eq) * M * g * l / den

    A = np.array([
        [0, 1,   0,   0],
        [0, 0,   a23, 0],
        [0, 0,   0,   1],
        [0, 0,   a43, 0],
    ])

    # ---- Vector B (entrada = par τ [N·m]) ----
    b2 =  (I_p + M * l**2 + M * l) / (den * r)
    b4 = -(M + M_w_eq + M * l)     / (den * r)
    B_tau = np.array([[0], [b2], [0], [b4]])

    # ---- Salidas ----
    C = np.array([
        [1, 0, 0, 0],   # x (encoder)
        [0, 0, 1, 0],   # θ (MPU)
    ])
    D = np.zeros((2, 1))

    return A, B_tau, C, D, J_w, M_w_eq, den


def estimar_K_tau(p):
    """Constante de par del motor: PWM -> par τ [N·m/PWM]."""
    # Constante de motor aproximada (back-EMF):
    # Kt ≈ V_nom / (w_nom · reduccion)   [V·s/rad ≈ N·m/A en SI]
    Kt = p["V_nom"] / (p["w_nom"] * p["reduccion"])
    # Par por unidad de PWM (aprox): a 255 PWM -> par de arranque
    # Aproximación práctica: K_tau ≈ Kt · reduccion / 255 · (factor)
    K_tau = Kt * p["reduccion"] / 255.0
    return K_tau


def analizar(A, B, C, D, p):
    print("\n" + "═"*60)
    print("  ESPACIO DE ESTADOS — Péndulo sobre dos ruedas (WIP)")
    print("  X = [x, ẋ, θ, θ̇]ᵀ      u = par τ [N·m]")
    print("═"*60)

    def pm(nombre, Mtx):
        print(f"\n{nombre} =")
        for row in np.atleast_2d(Mtx):
            print("  " + "  ".join(f"{v:+11.5f}" for v in row))

    pm("A", A)
    pm("B (entrada par τ)", B)

    # Convertir B a entrada PWM
    K_tau = estimar_K_tau(p)
    B_pwm = B * K_tau
    print(f"\n  K_tau ≈ {K_tau:.6f} N·m/PWM")
    pm("B (entrada PWM)", B_pwm)
    pm("C", C)
    pm("D", D)

    # Valores propios
    eig = np.linalg.eigvals(A)
    print("\nValores propios de A:")
    for e in eig:
        estado = "INESTABLE" if e.real > 1e-9 else ("estable" if e.real < -1e-9 else "integrador")
        print(f"  λ = {e:+.5f}   [{estado}]")

    polo_inest = max(e.real for e in eig)
    if polo_inest > 0:
        print(f"\n  Polo inestable: +{polo_inest:.4f} rad/s")
        print(f"  τ_caída = {1.0/polo_inest:.4f} s  -> el robot cae en ~{1.0/polo_inest:.2f}s")

    # Controlabilidad
    n_st = A.shape[0]
    Co = B_pwm.copy()
    for i in range(1, n_st):
        Co = np.hstack([Co, np.linalg.matrix_power(A, i) @ B_pwm])
    rCo = np.linalg.matrix_rank(Co)
    print(f"\nControlabilidad: {rCo}/{n_st}  -> "
          f"{'CONTROLABLE ✓' if rCo == n_st else 'NO controlable ✗'}")

    # Observabilidad
    Ob = C.copy()
    for i in range(1, n_st):
        Ob = np.vstack([Ob, C @ np.linalg.matrix_power(A, i)])
    rOb = np.linalg.matrix_rank(Ob)
    print(f"Observabilidad : {rOb}/{n_st}  -> "
          f"{'OBSERVABLE ✓' if rOb == n_st else 'NO observable ✗'}")

    print("\n  >>> Listo para diseñar LQR / pole-placement con A, B_pwm <<<")
    return B_pwm


# ══════════════════════════════════════════════════════════════════
if __name__ == "__main__":
    A, B_tau, C, D, J_w, M_w_eq, den = construir_modelo(PARAMS)
    print(f"\n[Intermedios]  J_w={J_w:.6f} kg·m²   M_w_eq={M_w_eq:.4f} kg   den={den:.6e}")
    analizar(A, B_tau, C, D, PARAMS)
