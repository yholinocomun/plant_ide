import numpy as np, matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from scipy.optimize import curve_fit
PPR=1945.0
def load(f):
    a=np.loadtxt(f,delimiter=",",skiprows=1);return a[:,0]/1000,a[:,1],(a[:,2]+a[:,3])/2/PPR*2*np.pi

fig,ax=plt.subplots(1,3,figsize=(16,4.5))

# --- 1) Motor B: w_ss vs PWM ---
t,pwm,ang=load("data/motor_B_20260628_202400.csv")
idx=np.where(np.diff(pwm)!=0)[0]+1; bnd=np.concatenate([[0],idx,[len(pwm)]])
Ps,Ws=[],[]
for i in range(len(bnd)-1):
    s,e=bnd[i],bnd[i+1]; p=pwm[s]
    if p==0:continue
    m=slice(s+(e-s)//2,e)
    sl=np.polyfit(t[m],ang[m],1)[0]; Ps.append(p);Ws.append(sl)
Ps,Ws=np.array(Ps),np.array(Ws)
K=np.sum(Ps*Ws)/np.sum(Ps*Ps)
ax[0].plot(Ps,Ws,'o',ms=9,label='datos')
xx=np.linspace(-200,200,10);ax[0].plot(xx,K*xx,'r-',label=f'ajuste K={K:.4f}')
ax[0].set_title("Motor EXP-B: ganancia (lineal y simetrico)");ax[0].set_xlabel("PWM");ax[0].set_ylabel("w_ss [rad/s]")
ax[0].grid(alpha=.3);ax[0].legend();ax[0].axhline(0,c='k',lw=.5);ax[0].axvline(0,c='k',lw=.5)

# --- 2) Motor C: escalon + ajuste + validacion cruzada ---
t,pwm,ang=load("data/motor_C_20260628_202442.csv")
m=pwm>0;t0=t[np.argmax(pwm>0)];tt=t[m]-t0;w=np.gradient(ang[m],t[m])
step=lambda x,wss,tau:wss*(1-np.exp(-x/tau))
(wss,tau),_=curve_fit(step,tt,w,p0=[6,.06])
ax[1].plot(tt,w,'.',ms=3,alpha=.4,label='datos C')
ax[1].plot(tt,step(tt,wss,tau),'g-',lw=2,label=f'ajuste tau={tau*1000:.0f}ms')
ax[1].plot(tt,step(tt,K*120,tau),'r--',lw=2,label='prediccion con K de EXP-B')
ax[1].set_title("Motor EXP-C: escalon + validacion cruzada");ax[1].set_xlabel("t [s]");ax[1].set_ylabel("w [rad/s]")
ax[1].grid(alpha=.3);ax[1].legend()

# --- 3) Pendulo: ajuste senoide amortiguada ---
a=np.loadtxt("data/pendulo_20260628_213317.csv",delimiter=",",skiprows=1)
tp=a[:,0]/1000-a[0,0]/1000;ang=a[:,2]
damped=lambda t,A,zw,wd,ph,c:A*np.exp(-zw*t)*np.cos(wd*t+ph)+c
popt,_=curve_fit(damped,tp,ang,p0=[1.2,.2,2*np.pi/.81,0,176.5],maxfev=200000)
T=2*np.pi/popt[2]
ax[2].plot(tp,ang,'.',ms=3,alpha=.4,label='datos')
ax[2].plot(tp,damped(tp,*popt),'m-',lw=1.5,label=f'ajuste T={T:.3f}s')
ax[2].set_title("Pendulo: oscilacion libre + ajuste");ax[2].set_xlabel("t [s]");ax[2].set_ylabel("angulo [deg]")
ax[2].grid(alpha=.3);ax[2].legend();ax[2].set_xlim(0,6)

plt.tight_layout()
plt.savefig("analisis/validacion_planta.png",dpi=110)
print("OK -> analisis/validacion_planta.png")
