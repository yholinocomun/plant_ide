"""
LQG para el robot balancin  =  LQR (realimentacion de estado) + Kalman (observador).
Convencion del curso (control_procesos/LQG_Avanzado_Continuo): lqr + lqe + Kr.
Simula el lazo cerrado con observador (como correria en el Arduino).
"""
import numpy as np
from scipy.linalg import solve_continuous_are, solve_discrete_are
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import sys; import os; sys.path.insert(0, os.path.dirname(__file__))
from planta import plant

A, B, C, D = plant()
n = 4
R2D = 180/np.pi

# ---------- 1) LQR (realimentacion de estado) ----------
# Pesos sobre [x, x_dot, theta, theta_dot]
Q = np.diag([1.0, 1.0, 300.0, 10.0])
Rr = np.array([[0.0002]])
P  = solve_continuous_are(A, B, Q, Rr)
K  = np.linalg.inv(Rr) @ B.T @ P          # u = -K x
print("K (LQR) =", np.round(K.flatten(), 3))
print(f"  -> Kang_p = {-K[0,2]/R2D:6.2f} PWM/deg   Kang_d = {-K[0,3]/R2D:5.2f} PWM/(deg/s)")
print(f"  -> Kpos_p = {-K[0,0]:6.2f} PWM/m       Kpos_d = {-K[0,1]:5.2f} PWM/(m/s)")

# ---------- 2) Observador de Kalman (discreto, dt=10ms) ----------
dt = 0.01
Ad = np.eye(n) + A*dt + 0.5*(A@A)*dt**2     # discretizacion
Bd = (np.eye(n)*dt + 0.5*A*dt**2) @ B
Qn = np.diag([1e-4, 1e-3, 1e-4, 1e-3])       # ruido de proceso
Rn = np.diag([2e-4, 3e-4])                    # ruido de medida (x, theta)
Pk = solve_discrete_are(Ad.T, C.T, Qn, Rn)
L  = Pk @ C.T @ np.linalg.inv(C @ Pk @ C.T + Rn)   # ganancia Kalman
print("L (Kalman) shape", L.shape, " polos obs:",
      np.round(np.abs(np.linalg.eigvals(Ad - L@C@Ad)), 3))

# ---------- 3) Simulacion lazo cerrado CON observador ----------
T = 3.0; N = int(T/dt)
x  = np.array([0.0, 0.0, 5/R2D, 0.0])   # inclinacion inicial 5 deg
xh = np.zeros(4)                        # estado estimado
X = np.zeros((N,4)); U=np.zeros(N); XH=np.zeros((N,4))
for k in range(N):
    u = float((-K @ xh)[0])                  # control sobre el ESTIMADO (LQG)
    u = np.clip(u, -255, 255)
    X[k]=x; XH[k]=xh; U[k]=u
    # planta real (discreta)
    x = Ad@x + (Bd.flatten()*u)
    y = C@x + np.array([np.random.randn()*0.014, np.random.randn()*0.017])
    # observador: predice y corrige
    xh = Ad@xh + Bd.flatten()*u
    xh = xh + L@(y - C@xh)

t=np.arange(N)*dt
estable = abs(X[-1,2]*R2D)<1.0 and abs(X[-1,0])<0.3 and np.max(np.abs(X[:,2]))<0.6
print("\nESTABLE:", estable, " theta_final=%.3f deg"%(X[-1,2]*R2D),
      " |u|max=%.0f PWM"%np.max(np.abs(U)))

fig,ax=plt.subplots(3,1,figsize=(9,7),sharex=True)
ax[0].plot(t,X[:,2]*R2D,label="theta real"); ax[0].plot(t,XH[:,2]*R2D,'--',label="theta est")
ax[0].set_ylabel("theta [deg]"); ax[0].legend(); ax[0].grid(alpha=.3); ax[0].set_title("LQG - balancin")
ax[1].plot(t,X[:,0],label="x real"); ax[1].plot(t,XH[:,0],'--',label="x est")
ax[1].set_ylabel("x [m]"); ax[1].legend(); ax[1].grid(alpha=.3)
ax[2].plot(t,U); ax[2].set_ylabel("u [PWM]"); ax[2].set_xlabel("t [s]"); ax[2].grid(alpha=.3)
plt.tight_layout(); plt.savefig(os.path.join(os.path.dirname(__file__),"..","data","sim_lqg.png"),dpi=100)
print("grafica -> balancin/data/sim_lqg.png")
