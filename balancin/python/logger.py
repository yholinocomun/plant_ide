"""
Extrae datos del robot por serial y los guarda en CSV + grafica.
Sirve para CUALQUIER controlador (LQG/IMC/Hinf/MPC/FOPID): solo lee la
telemetria CSV que imprime el firmware:  t_ms,theta,theta_d,x,pwm
Uso:  python3 logger.py --port /dev/ttyUSB0 --seg 20 --out data/run.csv
"""
import argparse, time, csv, sys
import numpy as np
try:
    import serial
except ImportError:
    sys.exit("pip install pyserial")

ap=argparse.ArgumentParser()
ap.add_argument("--port", default="/dev/ttyUSB0")
ap.add_argument("--baud", type=int, default=115200)
ap.add_argument("--seg",  type=float, default=20.0)
ap.add_argument("--out",  default="data/run.csv")
a=ap.parse_args()

ser=serial.Serial(a.port,a.baud,timeout=1); time.sleep(2)
ser.reset_input_buffer()
print(f"Grabando {a.seg}s de {a.port} ...  (activa el control en el robot)")
filas=[]; t0=time.time()
while time.time()-t0 < a.seg:
    ln=ser.readline().decode("utf-8","ignore").strip()
    if not ln or ln.startswith("#") or ln.startswith(">>") or "," not in ln: continue
    p=ln.split(",")
    if len(p)<5: continue
    try: filas.append([float(v) for v in p[:5]])
    except ValueError: continue
ser.close()

d=np.array(filas)
with open(a.out,"w",newline="") as f:
    w=csv.writer(f); w.writerow(["t_ms","theta","theta_d","x","pwm"]); w.writerows(filas)
print(f"{len(d)} muestras -> {a.out}")

# --- metricas de validacion ---
if len(d)>10:
    t=(d[:,0]-d[0,0])/1000; th=d[:,1]
    print(f"theta: std={th.std():.2f} deg  |max|={np.abs(th).max():.2f} deg  media={th.mean():.2f}")
    print(f"pwm:   |max|={np.abs(d[:,4]).max():.0f}  (satura={'SI' if np.abs(d[:,4]).max()>=255 else 'no'})")
    try:
        import matplotlib; matplotlib.use("Agg"); import matplotlib.pyplot as plt
        fig,ax=plt.subplots(3,1,figsize=(9,7),sharex=True)
        ax[0].plot(t,th); ax[0].set_ylabel("theta[deg]"); ax[0].grid(alpha=.3)
        ax[1].plot(t,d[:,3]); ax[1].set_ylabel("x[m]"); ax[1].grid(alpha=.3)
        ax[2].plot(t,d[:,4]); ax[2].set_ylabel("pwm"); ax[2].set_xlabel("t[s]"); ax[2].grid(alpha=.3)
        plt.tight_layout(); plt.savefig(a.out.replace(".csv",".png"),dpi=100)
        print("grafica ->", a.out.replace(".csv",".png"))
    except Exception as e: print("sin grafica:",e)
