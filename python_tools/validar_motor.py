import numpy as np
from scipy.optimize import curve_fit

PPR = 1945.0   # 13 CPR x 2 x 74.8
def load(f):
    a = np.loadtxt(f, delimiter=",", skiprows=1)
    t = a[:,0]/1000.0; pwm=a[:,1]
    enc = (a[:,2]+a[:,3])/2.0          # promedio ambas ruedas [cuentas]
    ang = enc/PPR*2*np.pi              # angulo de rueda [rad]
    return t,pwm,ang

def omega_seg(t,ang,mask):
    """velocidad de regimen = pendiente de ang(t) en la ventana (ajuste lineal)."""
    tt,aa = t[mask],ang[mask]
    if len(tt)<5: return np.nan
    A=np.vstack([tt,np.ones_like(tt)]).T
    sl,_=np.linalg.lstsq(A,aa,rcond=None)[0]
    return sl

print("="*64); print("  EXP B — GANANCIA K (escalera)"); print("="*64)
t,pwm,ang = load("data/motor_B_20260628_202400.csv")
# segmentar por cambios de PWM
idx = np.where(np.diff(pwm)!=0)[0]+1
bounds = np.concatenate([[0],idx,[len(pwm)]])
rows=[]
for i in range(len(bounds)-1):
    s,e = bounds[i],bounds[i+1]
    p = pwm[s]
    if p==0: continue
    # ultimo 50% del segmento = regimen permanente
    m = np.zeros(len(t),bool); m[s+(e-s)//2:e]=True
    w = omega_seg(t,ang,m)
    rows.append((p,w,w/p))
    print(f"  PWM={int(p):+4d}   w_ss={w:+7.3f} rad/s   K={w/p:.5f}")
rows=np.array(rows)
# regresion w = K*pwm (con zona muerta): ajustar solo magnitudes
P=rows[:,0]; W=rows[:,1]
K_fit = np.sum(P*W)/np.sum(P*P)     # K por minimos cuadrados (sin offset)
Kf = np.mean(W[P>0]/P[P>0]); Kb=np.mean(W[P<0]/P[P<0])
print(f"\n  K (regresion global)  = {K_fit:.5f} rad/s/PWM")
print(f"  K adelante={Kf:.5f}  K reversa={Kb:.5f}  simetria f/b={Kf/Kb:.3f}")

print("\n"+"="*64); print("  EXP C — CONSTANTE DE TIEMPO tau (escalon 0->120)"); print("="*64)
t,pwm,ang = load("data/motor_C_20260628_202442.csv")
t0 = t[np.argmax(pwm>0)]
m = pwm>0
tt = t[m]-t0
# velocidad instantanea por diferencias suavizadas
w_inst = np.gradient(ang[m],t[m])
def step(x,wss,tau): return wss*(1-np.exp(-x/tau))
p0=[w_inst[-5:].mean(),0.06]
popt,_=curve_fit(step,tt,w_inst,p0=p0,maxfev=10000)
wss_C,tau_C = popt
K_C = wss_C/120.0
print(f"  w_ss={wss_C:.3f} rad/s   tau={tau_C*1000:.1f} ms   K_C={K_C:.5f} rad/s/PWM")

print("\n"+"="*64); print("  EXP A — ZONA MUERTA (rampa 0->120)"); print("="*64)
t,pwm,ang = load("data/motor_A_20260628_202239.csv")
w = np.gradient(ang,t)
# primer PWM con velocidad sostenida > umbral
thr=0.3
mov = np.where(np.abs(w)>thr)[0]
pwm_dead = pwm[mov[0]] if len(mov) else np.nan
# mas robusto: primer nivel cuya |w| media >0.3
for p in sorted(set(pwm)):
    if p<=0: continue
    mm = pwm==p
    if np.abs(np.gradient(ang[mm],t[mm])).mean()>thr:
        pwm_dead=p; break
print(f"  Zona muerta u_dead ~ {int(pwm_dead)} PWM")

print("\n"+"="*64); print("  VALIDACION CRUZADA  (B -> predecir C)"); print("="*64)
# Con K de B y tau de C, predecir el escalon C y medir residuo
t,pwm,ang = load("data/motor_C_20260628_202442.csv")
m=pwm>0; tt=t[m]-t[np.argmax(pwm>0)]; w_meas=np.gradient(ang[m],t[m])
w_pred = step(tt, K_fit*120.0, tau_C)
res = w_meas-w_pred
rmse=np.sqrt(np.mean(res**2)); rng=w_meas.max()-w_meas.min()
print(f"  Prediccion con K(de B)={K_fit:.5f} y tau(de C)={tau_C*1000:.0f}ms")
print(f"  RMSE={rmse:.3f} rad/s   ({100*rmse/rng:.1f}% del rango)  -> {'VALIDO ✓' if rmse/rng<0.1 else 'revisar'}")

print("\n"+"="*64); print("  TEORICO vs EXPERIMENTAL (motor)"); print("="*64)
w_noload=13.93   # 133 RPM nominal a 12V
K_teo = w_noload/255.0
print(f"  K teorico (vacio nominal 133RPM/255PWM) = {K_teo:.5f} rad/s/PWM")
print(f"  K experimental (datos)                  = {K_fit:.5f} rad/s/PWM")
print(f"  error = {100*abs(K_fit-K_teo)/K_teo:.1f}%")

np.save("/tmp/claude-0/-home-user-plant-ide/bd300bbb-62f5-56e5-9252-4a0f562f89c7/scratchpad/motor_params.npy",
        {"K":K_fit,"tau":tau_C,"u_dead":pwm_dead})
