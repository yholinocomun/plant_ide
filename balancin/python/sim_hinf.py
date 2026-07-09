"""
#3  CONTROL H-INFINITO (sensibilidad mixta) para el balancin.
Convencion del curso (PC3/Control_H_infinito): ponderadores W1,W2,W3 + hinfsyn/mixsyn.
Se disena sobre el subsistema de ANGULO (SISO, inestable):
   theta''(t) = 104.68 theta + b4 u  ->  G(s) = b4/(s^2 - 104.68)
El controlador K(s) resulta dinamico -> se discretiza (Tustin) para el Arduino.
"""
import numpy as np, control as ct
import matplotlib; matplotlib.use("Agg"); import matplotlib.pyplot as plt
import os,sys; sys.path.insert(0,os.path.dirname(__file__)); from planta import plant
A,B,C,D=plant(); R2D=180/np.pi

a43=104.6806; b4=float(B[3,0])            # subsistema angulo
s=ct.tf('s'); G=ct.tf([b4],[1,0,-a43])    # planta SISO inestable
print("G(s)=",G)

# --- ponderadores de sensibilidad mixta (tu estilo makeweight/mixsyn) ---
def makeweight(dc,wc,hf):          # peso 1er orden: |W(0)|=dc, |W(inf)|=hf
    s=ct.tf('s'); return (hf*s + wc*dc)/(s + wc)
W1 = makeweight(50, 6, 0.05)       # S: desempeno
W2 = ct.tf([1e-3],[1])             # KS: esfuerzo de control
W3 = makeweight(0.05, 40, 2)       # T: robustez
K,CL,info = ct.mixsyn(G, W1, W2, W3); g_=float(info[0])
print("gamma =", round(g_,3), " (bueno si ~1)")

# --- lazo cerrado, respuesta a condicion inicial (via step de perturbacion) ---
L=ct.series(K,G); T=ct.feedback(L,1)
print("polos lazo cerrado (parte real max):", round(max(np.real(ct.poles(T))),3),
      "-> ESTABLE" if max(np.real(ct.poles(T)))<0 else "-> INESTABLE")

# respuesta: seguir un escalon pequeno de referencia de angulo (0 -> se rechaza dist)
tt=np.linspace(0,3,600)
_,y=ct.step_response(T, tt)
# discretizar el controlador para Arduino (Tustin, dt=10ms)
Kd=ct.tf(ct.c2d(K,0.01,'tustin')); num=np.array(Kd.num[0][0]); den=np.array(Kd.den[0][0])
print("Controlador discreto K(z): num=",np.round(num,5)," den=",np.round(den,5))
plt.figure(figsize=(9,4)); plt.plot(tt,y); plt.grid(alpha=.3)
plt.title(f"H-inf: respuesta lazo cerrado (gamma={g_:.2f})"); plt.xlabel("t[s]"); plt.ylabel("theta/ref")
plt.savefig(os.path.join(os.path.dirname(__file__),"..","data","sim_hinf.png"),dpi=100)
