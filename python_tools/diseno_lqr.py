#!/usr/bin/env python3
"""
============================================================
FASE 5 - Diseño del controlador LQR
============================================================
Toma el modelo en espacio de estados (de ensamblar_modelo.py)
y calcula las ganancias óptimas K = [k1, k2, k3, k4] que
equilibran el robot, luego simula la respuesta.

Ley de control:   u = -K · X = -(k1·x + k2·ẋ + k3·θ + k4·θ̇)

Uso:
  python3 diseno_lqr.py
  python3 diseno_lqr.py --q_theta 200 --q_x 10   # ajustar pesos
============================================================
"""

import argparse
import numpy as np

from ensamblar_modelo import construir_modelo, estimar_K_tau, PARAMS

try:
    from scipy.linalg import solve_continuous_are
    SCIPY_OK = True
except ImportError:
    SCIPY_OK = False


def lqr(A, B, Q, R):
    """Resuelve el LQR continuo: K = R^-1 B^T P, con P de la ecuación de Riccati."""
    P = solve_continuous_are(A, B, Q, R)
    K = np.linalg.inv(R) @ B.T @ P
    return K, P


def simular(A, B, K, x0, dt=0.001, t_final=3.0):
    """Simula el sistema en lazo cerrado con u = -K·x (Euler)."""
    n  = int(t_final / dt)
    t  = np.linspace(0, t_final, n)
    X  = np.zeros((n, 4))
    U  = np.zeros(n)
    X[0] = x0
    Acl = A - B @ K
    for i in range(1, n):
        u = float(-K @ X[i-1])
        U[i-1] = u
        X[i] = X[i-1] + dt * (A @ X[i-1] + (B.flatten() * u))
    return t, X, U


def main():
    ap = argparse.ArgumentParser(description="Diseño LQR del balancín")
    ap.add_argument("--q_x",      type=float, default=1.0,   help="peso posición x")
    ap.add_argument("--q_xdot",   type=float, default=1.0,   help="peso velocidad ẋ")
    ap.add_argument("--q_theta",  type=float, default=100.0, help="peso ángulo θ")
    ap.add_argument("--q_thetadot", type=float, default=10.0, help="peso θ̇")
    ap.add_argument("--R",        type=float, default=1.0,   help="peso del esfuerzo de control")
    ap.add_argument("--theta0",   type=float, default=5.0,   help="ángulo inicial sim [grados]")
    ap.add_argument("--no-sim",   action="store_true",       help="no graficar simulación")
    args = ap.parse_args()

    if not SCIPY_OK:
        print("[ERROR] scipy requerido:  pip install scipy")
        return

    # --- Modelo (entrada en PWM) ---
    A, B_tau, C, D, J_w, M_w_eq, den = construir_modelo(PARAMS)
    K_tau = estimar_K_tau(PARAMS)
    B = B_tau * K_tau     # entrada en PWM

    # --- Matrices de peso ---
    Q = np.diag([args.q_x, args.q_xdot, args.q_theta, args.q_thetadot])
    R = np.array([[args.R]])

    print("═"*60)
    print("  DISEÑO LQR — Robot balancín")
    print("═"*60)
    print(f"\n  Q = diag({np.diag(Q)})")
    print(f"  R = {R.flatten()}")

    # --- Calcular ganancias ---
    K, P = lqr(A, B, Q, R)
    print("\n  GANANCIAS LQR  (u = -K·X):")
    print(f"    k1 (x)   = {K[0,0]:+.4f}")
    print(f"    k2 (ẋ)   = {K[0,1]:+.4f}")
    print(f"    k3 (θ)   = {K[0,2]:+.4f}")
    print(f"    k4 (θ̇)   = {K[0,3]:+.4f}")

    # --- Estabilidad en lazo cerrado ---
    Acl = A - B @ K
    eig = np.linalg.eigvals(Acl)
    print("\n  Polos en lazo cerrado:")
    estable = True
    for e in eig:
        ok = e.real < 0
        estable &= ok
        print(f"    λ = {e:+.4f}   [{'estable' if ok else 'INESTABLE'}]")
    print(f"\n  Sistema {'ESTABLE ✓ — el robot se equilibra' if estable else 'INESTABLE ✗ — ajusta Q/R'}")

    # --- Código listo para el ESP32 ---
    print("\n" + "─"*60)
    print("  Pega esto en tu firmware de control:")
    print("─"*60)
    print(f"  // theta en RADIANES, x en METROS")
    print(f"  const float k1 = {K[0,0]:.4f};  // x")
    print(f"  const float k2 = {K[0,1]:.4f};  // x_dot")
    print(f"  const float k3 = {K[0,2]:.4f};  // theta")
    print(f"  const float k4 = {K[0,3]:.4f};  // theta_dot")
    print(f"  // u_pwm = -(k1*x + k2*x_dot + k3*theta + k4*theta_dot)")

    # --- Simulación ---
    if not args.no_sim:
        x0 = np.array([0, 0, np.deg2rad(args.theta0), 0])
        t, X, U = simular(A, B, K, x0)
        print(f"\n  [Simulación] arranque desde θ={args.theta0}°")
        print(f"    θ máx     = {np.rad2deg(np.abs(X[:,2]).max()):.2f}°")
        print(f"    PWM máx   = {np.abs(U).max():.1f}")
        print(f"    x deriva  = {X[-1,0]*100:.2f} cm")

        try:
            import matplotlib
            matplotlib.use("TkAgg")
            import matplotlib.pyplot as plt
            from pathlib import Path
            from datetime import datetime
            fig, ax = plt.subplots(3, 1, figsize=(10, 8), sharex=True)
            ax[0].plot(t, np.rad2deg(X[:,2]), color="tab:blue")
            ax[0].axhline(0, color="k", lw=0.5); ax[0].set_ylabel("θ [°]")
            ax[0].set_title(f"Simulación LQR — arranque {args.theta0}°"); ax[0].grid(alpha=0.3)
            ax[1].plot(t, X[:,0]*100, color="tab:green")
            ax[1].set_ylabel("x [cm]"); ax[1].grid(alpha=0.3)
            ax[2].plot(t, U, color="tab:red")
            ax[2].set_ylabel("u [PWM]"); ax[2].set_xlabel("t [s]"); ax[2].grid(alpha=0.3)
            plt.tight_layout()
            PLOTS = Path(__file__).parent.parent / "plots"
            PLOTS.mkdir(exist_ok=True)
            ruta = PLOTS / f"lqr_sim_{datetime.now():%Y%m%d_%H%M%S}.png"
            plt.savefig(ruta, dpi=150)
            print(f"    [Gráfica] {ruta}")
            plt.show()
        except Exception as e:
            print(f"    [sin gráfica: {e}]")


if __name__ == "__main__":
    main()
