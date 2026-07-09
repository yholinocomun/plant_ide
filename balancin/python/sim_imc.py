"""
#4  CONTROL IMC para el balancin.
Tu IMC (control_procesos/IMC): F=(gamma*s+1)/(lambda*s+1)^n, Q=inv(Gm)*F, requiere
PLANTA ESTABLE. El balancin es INESTABLE -> primero se ESTABILIZA con un lazo
interno (LQR de angulo); el mapa resultante  w->x  ya es estable y sobre ESE se
aplica IMC para seguir POSICION. Zeros de fase no minima se factorizan (allpass).
"""
import numpy as np, control as ct
import matplotlib; matplotlib.use("Agg"); import matplotlib.pyplot as plt
import os,sys; sys.path.insert(0,os.path.dirname(__file__)); from planta import plant
from scipy.linalg import solve_continuous_are
A,B,C,D=plant()

# --- 1) estabilizar (lazo interno LQR) ---
Q=np.diag([1,1,300,10.]); R=np.array([[0.0002]])
P=solve_continuous_are(A,B,Q,R); K=np.linalg.inv(R)@B.T@P
Acl=A-B@K                                  # estable
# planta estabilizada  w->x  (w = comando extra sumado a u; salida x)
Cx=np.array([[1,0,0,0]])
Pss=ct.ss(Acl,B,Cx,0); Pt=ct.tf(Pss); Pt=ct.minreal(Pt,verbose=False)
print("Planta estabilizada P(s) w->x  polos:",np.round(ct.poles(Pt),2))

# --- 2) IMC sobre P (estable). Factorizar zeros RHP (fase no minima) ---
z=ct.zeros(Pt); rhp=z[np.real(z)>1e-6]
print("zeros RHP (NMP):", np.round(rhp,3))
s=ct.tf('s')
allpass=ct.tf([1],[1])
Pmin=Pt
for zz in rhp:
    if abs(np.imag(zz))<1e-6:
        zr=float(np.real(zz))
        allpass=allpass*ct.tf([-1,zr],[1,zr]); Pmin=Pmin*ct.tf([1,zr],[-1,zr])  # quitar RHP
Pmin=ct.minreal(Pmin,verbose=False)

# --- 3) filtro IMC y controlador ---
lam=0.35; n=4                                    # n >= grado relativo
F=ct.tf([1],np.poly([-1/lam]*n))                 # 1/(lam s+1)^n  (aprox)
F=ct.tf([1],[lam,1])**n
Qc=ct.minreal(F/Pmin,verbose=False)              # Q = inv(Pmin)*F
Cimc=ct.minreal(Qc/(1-Qc*Pt),verbose=False)      # C = Q/(1-Q P)

# --- 4) lazo cerrado: seguir escalon de posicion 0->0.1 m ---
T=ct.minreal(ct.feedback(Cimc*Pt,1),verbose=False)
print("lazo cerrado polos (real max):", round(max(np.real(ct.poles(T))),3),
      "-> ESTABLE" if max(np.real(ct.poles(T)))<0 else "-> INESTABLE")
tt=np.linspace(0,4,800); _,y=ct.step_response(0.1*T,tt)
try:
    info=ct.step_info(T); print("ts=%.2fs  overshoot=%.1f%%"%(info['SettlingTime'],info['Overshoot']))
except Exception as e: print("step_info:",e)
plt.figure(figsize=(9,4)); plt.plot(tt,y); plt.axhline(0.1,ls='--',c='k',alpha=.5)
plt.title(f"IMC (lambda={lam}) - seguimiento de posicion"); plt.xlabel("t[s]"); plt.ylabel("x[m]"); plt.grid(alpha=.3)
plt.savefig(os.path.join(os.path.dirname(__file__),"..","data","sim_imc.png"),dpi=100)
# controlador discreto para implementacion
try:
    Cd=ct.tf(ct.c2d(ct.minreal(Cimc,verbose=False),0.01,'tustin'))
    print("C_IMC discreto: num=",np.round(np.array(Cd.num[0][0]),5)," den=",np.round(np.array(Cd.den[0][0]),5))
except Exception as e: print("discretizacion IMC:",e)
