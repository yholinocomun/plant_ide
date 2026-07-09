"""
#5  PID FRACCIONARIO (FOPID)  C(s)=Kp + Ki/s^lam + Kd*s^mu   para el balancin.
No existia en tu repo -> construido con aproximacion de OUSTALOUP de s^alpha.
Se aplica al subsistema de ANGULO (SISO inestable) G=b4/(s^2-104.68).
Para el Arduino, cada s^alpha se discretiza en biquads (Tustin).
"""
import numpy as np, control as ct
import matplotlib; matplotlib.use("Agg"); import matplotlib.pyplot as plt
import os,sys; sys.path.insert(0,os.path.dirname(__file__)); from planta import plant
A,B,C,D=plant(); a43=104.6806; b4=float(B[3,0])
G=ct.tf([b4],[1,0,-a43])

def oustaloup(alpha, wb=0.01, wh=100.0, N=4):
    """Aproximacion racional de s^alpha en [wb,wh] (orden 2N+1)."""
    k=np.arange(-N,N+1)
    wz = wb*(wh/wb)**((k+N+0.5*(1-alpha))/(2*N+1))   # zeros
    wp = wb*(wh/wb)**((k+N+0.5*(1+alpha))/(2*N+1))   # polos
    Kg = wh**alpha
    num=np.poly(-wz); den=np.poly(-wp)
    return ct.tf(Kg*num, den)

def fopid(Kp,Ki,Kd,lam,mu):
    s_lam = oustaloup(-lam)      # 1/s^lam
    s_mu  = oustaloup(mu)        # s^mu
    return Kp + Ki*s_lam + Kd*s_mu

# --- buscar ganancias que estabilicen (escala en RAD, tipo LQR) ---
best=None; Ki=50.0
for Kd in [300,450,650]:
  for Kp in [2500,4000,5500]:
    for mu in [1.0,1.1,1.2]:
        C=fopid(-Kp,-Ki,-Kd,0.9,mu)      # signo neg: b4<0
        T=ct.feedback(C*G,1)
        pr=max(np.real(ct.poles(T)))
        if pr<0 and (best is None or pr<best[0]): best=(pr,Kp,Ki,Kd,mu,C,T)
if best is None: print("NO se hallo FOPID estable"); sys.exit()
pr,Kp,Ki,Kd,mu,C,T=best
print(f"FOPID estable: Kp={Kp} Ki={Ki} Kd={Kd} lam=0.9 mu={mu}  polo_real_max={pr:.2f}")

tt=np.linspace(0,3,900)
# respuesta a condicion inicial de angulo (perturbacion) via impulso escalado
_,y=ct.step_response(ct.feedback(G, C), tt)   # rechazo de perturbacion de entrada
plt.figure(figsize=(9,4)); plt.plot(tt,y*5); plt.grid(alpha=.3)
plt.title(f"FOPID (mu={mu}): rechazo de perturbacion en angulo"); plt.xlabel("t[s]"); plt.ylabel("theta [deg]")
plt.savefig(os.path.join(os.path.dirname(__file__),"..","data","sim_fopid.png"),dpi=100)
# discretizar los operadores fraccionarios para Arduino
for tag,al in [("s^-0.9",-0.9),("s^%.2f"%mu,mu)]:
    Fd=ct.tf(ct.c2d(oustaloup(al),0.01,'tustin'))
    print(f"{tag} discreto: num[:3]=",np.round(np.array(Fd.num[0][0])[:3],4),"...")
print("Ganancias FOPID (implementacion): Kp=%d Ki=%g Kd=%d lambda=0.9 mu=%.2f"%(Kp,Ki,Kd,mu))
