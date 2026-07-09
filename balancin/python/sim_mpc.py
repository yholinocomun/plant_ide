"""
#2  MPC / LQR PREDICTIVO para el balancin.
Convencion del curso (control_procesos/MPC_funcion): c2d + horizonte N + costo cuadratico.
Version verificable: LQR de horizonte finito por recursion de Riccati hacia atras
(= "LQR predictivo"); la ganancia del 1er paso K_mpc es la que se implementa.
El QP con restricciones (|u|<=255) se resuelve por saturacion (ver nota MATLAB).
"""
import numpy as np
from scipy.linalg import solve_discrete_are
import matplotlib; matplotlib.use("Agg"); import matplotlib.pyplot as plt
import os,sys; sys.path.insert(0,os.path.dirname(__file__)); from planta import plant

A,B,C,D = plant(); n=4; R2D=180/np.pi; dt=0.01
# --- discretizar (ZOH aprox) ---
Ad = np.eye(n)+A*dt+0.5*(A@A)*dt**2
Bd = (np.eye(n)*dt+0.5*A*dt**2)@B
# --- pesos MPC ---
Q  = np.diag([1.,1.,300.,10.]);  Rw = np.array([[0.0002]]);  Np = 60
# --- Riccati hacia atras en horizonte Np (LQR predictivo) ---
P = Q.copy()
for _ in range(Np):
    K = np.linalg.inv(Rw + Bd.T@P@Bd) @ (Bd.T@P@Ad)      # ganancia paso a paso
    P = Q + Ad.T@P@(Ad - Bd@K)
Kmpc = K.flatten()                                        # u = -Kmpc x  (1er paso)
print("Kmpc =", np.round(Kmpc,3))
print(f"  Kang_p={Kmpc[2]/R2D:.2f} PWM/deg  Kang_d={Kmpc[3]/R2D:.2f}  Kpos_p={Kmpc[0]:.2f}  Kpos_d={Kmpc[1]:.2f}")

# --- simulacion lazo cerrado con saturacion (restriccion de entrada) ---
N=int(3/dt); x=np.array([0,0,5/R2D,0.]); X=np.zeros((N,4)); U=np.zeros(N)
for k in range(N):
    u=np.clip(float(-Kmpc@x),-255,255); X[k]=x; U[k]=u
    x=Ad@x+Bd.flatten()*u
t=np.arange(N)*dt
print("ESTABLE:", abs(X[-1,2]*R2D)<1 and np.max(np.abs(X[:,2]))<0.6,
      " theta_f=%.3f deg |u|max=%.0f"%(X[-1,2]*R2D,np.max(np.abs(U))))
fig,ax=plt.subplots(2,1,figsize=(9,5),sharex=True)
ax[0].plot(t,X[:,2]*R2D); ax[0].set_ylabel("theta[deg]"); ax[0].grid(alpha=.3); ax[0].set_title("MPC / LQR predictivo")
ax[1].plot(t,U); ax[1].set_ylabel("u[PWM]"); ax[1].set_xlabel("t[s]"); ax[1].grid(alpha=.3)
plt.tight_layout(); plt.savefig(os.path.join(os.path.dirname(__file__),"..","data","sim_mpc.png"),dpi=100)
