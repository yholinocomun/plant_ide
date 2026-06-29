import numpy as np
from scipy.optimize import curve_fit
M,l,g=0.710,0.10,9.81
def load(f):
    a=np.loadtxt(f,delimiter=",",skiprows=1); return a[:,0]/1000.0,a[:,2]
def damped(t,A,zw,wd,ph,c): return A*np.exp(-zw*t)*np.cos(wd*t+ph)+c
def analiza(fn):
    t,ang=load(fn); t=t-t[0]
    eq=np.mean(ang[-200:])                # equilibrio (cola)
    amp0=np.max(np.abs(ang[:200]-eq))
    p0=[amp0,0.2,2*np.pi/0.79,0,eq]
    popt,_=curve_fit(damped,t,ang,p0=p0,maxfev=200000)
    A,zw,wd,ph,c=popt
    wn=np.sqrt(wd**2+zw**2); zeta=zw/wn; T=2*np.pi/wd
    I_p=M*g*l*(T/(2*np.pi))**2; I_cm=I_p-M*l**2
    resid=ang-damped(t,*popt); rmse=np.sqrt(np.mean(resid**2))
    print(f"\n--- {fn.split('/')[-1]} ---")
    print(f"  equilibrio={c:.1f}°  amplitud inicial={abs(A):.1f}°  {'(GRANDE: no-lineal!)' if abs(A)>15 else '(pequeña: lineal OK)'}")
    print(f"  T={T:.4f}s  w_n={wn:.3f} rad/s  zeta={zeta:.4f}")
    print(f"  I_p={I_p:.5f}  I_cm={I_cm:.5f}  (ajuste RMSE={rmse:.2f}°)")
    return T,wn,zeta,I_p,I_cm,abs(A)
print("="*64);print("  PENDULO — AJUSTE DE SENOIDE AMORTIGUADA");print("="*64)
R=[analiza(f"data/pendulo_{x}.csv") for x in ["20260628_212859","20260628_213317"]]
R=np.array(R)
print("\n"+"="*64)
print(f"  T medio={R[:,0].mean():.4f}±{R[:,0].std():.4f}s   w_n={R[:,1].mean():.3f} rad/s")
print(f"  I_p medio={R[:,3].mean():.5f}  I_cm={R[:,4].mean():.5f} kg·m²")
print(f"  amplitudes={R[:,5].round(1)}  ->  ", 
      "OSCILACION GRANDE, el periodo medido es mayor que el lineal" if R[:,5].mean()>15 else "lineal")
# Correccion no-lineal: T_lineal = T_medido / (1 + theta0^2/16)  (1er orden)
th0=np.radians(R[:,5].mean())
fac=1+th0**2/16
print(f"\n  Correccion por amplitud (1er orden): T_lin = T/{fac:.3f}")
T_lin=R[:,0].mean()/fac; Ip_lin=M*g*l*(T_lin/(2*np.pi))**2
print(f"  T_lineal={T_lin:.4f}s  ->  I_p(lineal)={Ip_lin:.5f}  I_cm={Ip_lin-M*l**2:.5f}")
np.save("/tmp/claude-0/-home-user-plant-ide/bd300bbb-62f5-56e5-9252-4a0f562f89c7/scratchpad/pend_fit.npy",
        {"T":R[:,0].mean(),"I_p":R[:,3].mean(),"I_p_lin":Ip_lin})
