"""
Extraccion de data experimental UNIFORME para cualquier controlador.
El firmware imprime (tecla 't') el CSV estandar:
  t_ms,theta_deg,theta_dot_dps,x_m,x_dot_ms,u_pwm,setpoint_deg,modo
Uso:  python3 extraer_datos.py --controlador lqr --port /dev/ttyUSB0 --seg 25
Guarda resultados/<controlador>_<fecha>.csv  y su .json de metricas.
"""
import argparse,time,csv,json,sys,os,datetime
import numpy as np
sys.path.insert(0,os.path.join(os.path.dirname(__file__),"..","config"))
import controladores as C
from metricas import calcular
try: import serial
except ImportError: sys.exit("pip install pyserial")

ap=argparse.ArgumentParser()
ap.add_argument("--controlador",required=True,choices=list(C.CTRL))
ap.add_argument("--port",default="/dev/ttyUSB0"); ap.add_argument("--baud",type=int,default=115200)
ap.add_argument("--seg",type=float,default=25.0)
a=ap.parse_args()
info=C.CTRL[a.controlador]
outdir=os.path.join(os.path.dirname(__file__),"..","resultados"); os.makedirs(outdir,exist_ok=True)
stamp=datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
base=os.path.join(outdir,f"{a.controlador}_{stamp}")

ser=serial.Serial(a.port,a.baud,timeout=1); time.sleep(2); ser.reset_input_buffer()
print(f"[{info['nombre']}] grabando {a.seg}s de {a.port}. Activa el control (space) y la telemetria (t).")
filas=[]; t0=time.time()
while time.time()-t0<a.seg:
    ln=ser.readline().decode("utf-8","ignore").strip()
    if not ln or ln.startswith("#") or ln.startswith(">>") or "," not in ln: continue
    p=ln.split(",")
    if len(p)<len(C.COLS): continue
    try: filas.append([float(v) for v in p[:len(C.COLS)]])
    except ValueError: continue
ser.close()
d=np.array(filas)
with open(base+".csv","w",newline="") as f:
    w=csv.writer(f); w.writerow(C.COLS); w.writerows(filas)
met=calcular(d) if len(d)>10 else {}
met.update({"controlador":a.controlador,"nombre":info["nombre"],"ganancias":info["ganancias"],"fecha":stamp})
json.dump(met,open(base+".json","w"),indent=2,ensure_ascii=False)
print(f"{len(d)} muestras -> {base}.csv")
print(json.dumps(met,indent=2,ensure_ascii=False))
