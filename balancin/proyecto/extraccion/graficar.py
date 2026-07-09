"""
Figura ESTANDAR de un controlador a partir de su CSV.  Mismo formato para los 6.
Uso:  python3 graficar.py resultados/lqr_XXXX.csv
Genera <csv>.png (3 paneles: theta, x, u) + caja de metricas.
"""
import sys,os,json
import numpy as np, matplotlib; matplotlib.use("Agg"); import matplotlib.pyplot as plt
sys.path.insert(0,os.path.dirname(__file__)); from estilo import aplicar; from metricas import calcular
sys.path.insert(0,os.path.join(os.path.dirname(__file__),"..","config")); import controladores as C
aplicar()

csvf=sys.argv[1]; slug=os.path.basename(csvf).split("_")[0]
info=C.CTRL.get(slug,{"nombre":slug,"color":"#333"})
d=np.loadtxt(csvf,delimiter=",",skiprows=1); t=(d[:,0]-d[0,0])/1000
met=calcular(d); col=info["color"]
fig,ax=plt.subplots(3,1,sharex=True)
fig.suptitle(f"{info['nombre']} — respuesta experimental (hardware)",fontweight="bold")
ax[0].plot(t,d[:,1],color=col); ax[0].axhline(0,ls='--',c='k',alpha=.4); ax[0].set_ylabel("θ [deg]")
ax[1].plot(t,d[:,3],color=col); ax[1].set_ylabel("x [m]")
ax[2].plot(t,d[:,5],color=col); ax[2].set_ylabel("u [PWM]"); ax[2].set_xlabel("t [s]")
ax[2].axhline(255,ls=':',c='r',alpha=.5); ax[2].axhline(-255,ls=':',c='r',alpha=.5)
txt=f"θRMS={met['theta_RMS_deg']}°  θstd={met['theta_std_deg']}°  |θ|max={met['theta_max_abs_deg']}°\n"\
    f"|u|max={met['u_max_abs_pwm']}  sat={met['saturacion_pct']}%  derivaX={met['deriva_x_m']}m"
ax[0].text(0.99,0.02,txt,transform=ax[0].transAxes,ha="right",va="bottom",fontsize=8,
           bbox=dict(boxstyle="round",fc="white",alpha=.8))
out=csvf.replace(".csv",".png"); plt.savefig(out); print("figura ->",out)
