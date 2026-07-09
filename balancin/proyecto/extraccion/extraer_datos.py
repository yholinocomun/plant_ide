"""
Extraccion de data experimental UNIFORME para cualquier controlador.
Hace TODO por Python (no necesitas el Monitor Serial de Arduino):
  conecta -> espera calibracion -> (Enter) trim 'z' -> (Enter) activar space
  -> inicia telemetria 't' -> graba N segundos -> detiene.
Firmware imprime el CSV estandar:
  t_ms,theta_deg,theta_dot_dps,x_m,x_dot_ms,u_pwm,setpoint_deg,modo
Uso:  python3 extraer_datos.py --controlador lqr --port /dev/ttyUSB0 --seg 25
      (--auto  salta las pausas de Enter; --no-activar  si ya esta activado)
"""
import argparse,time,csv,json,sys,os,datetime
import numpy as np
sys.path.insert(0,os.path.join(os.path.dirname(__file__),"..","config")); import controladores as C
sys.path.insert(0,os.path.dirname(__file__)); from metricas import calcular
try: import serial
except ImportError: sys.exit("Falta pyserial:  pip install pyserial")

ap=argparse.ArgumentParser()
ap.add_argument("--controlador",required=True,choices=list(C.CTRL))
ap.add_argument("--port",default="/dev/ttyUSB0"); ap.add_argument("--baud",type=int,default=115200)
ap.add_argument("--seg",type=float,default=25.0)
ap.add_argument("--auto",action="store_true",help="sin pausas de Enter")
ap.add_argument("--no-activar",action="store_true",help="no envia z/space (ya activo)")
a=ap.parse_args()
info=C.CTRL[a.controlador]
outdir=os.path.join(os.path.dirname(__file__),"..","resultados"); os.makedirs(outdir,exist_ok=True)
stamp=datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
base=os.path.join(outdir,f"{a.controlador}_{stamp}")

ser=serial.Serial(a.port,a.baud,timeout=0.2)
def leer_un_rato(seg,mostrar=True):
    t=time.time()
    while time.time()-t<seg:
        ln=ser.readline().decode("utf-8","ignore").strip()
        if ln and mostrar: print("  ",ln)
def enviar(k): ser.write(k.encode())

print(f"\n=== EXTRACCION [{info['nombre']}]  puerto {a.port} ===")
time.sleep(2)                                   # arranque de la placa
print(">> Esperando arranque/calibracion del robot (mensajes del firmware):")
leer_un_rato(3)                                 # muestra 'Calib...' y 'Listo'
ser.reset_input_buffer()

if not a.no_activar:
    if not a.auto: input("\n[1] Pon el robot VERTICAL y presiona ENTER para fijar el trim (z)...")
    enviar('z'); time.sleep(0.3); leer_un_rato(0.5)
    if not a.auto: input("[2] SUJETALO en equilibrio y presiona ENTER para ACTIVAR motores (space)...")
    enviar(' '); time.sleep(0.3); leer_un_rato(0.5)

print(f"\n[3] Iniciando TELEMETRIA y grabando {a.seg}s... (deja que controle)")
enviar('t'); time.sleep(0.2); ser.reset_input_buffer()
filas=[]; t0=time.time()
while time.time()-t0<a.seg:
    ln=ser.readline().decode("utf-8","ignore").strip()
    if not ln or ln.startswith("#") or ln.startswith(">>") or "," not in ln: continue
    p=ln.split(",")
    if len(p)<len(C.COLS): continue
    try: filas.append([float(v) for v in p[:len(C.COLS)]])
    except ValueError: continue
enviar('t')                                     # detener telemetria
if not a.no_activar: enviar(' ')                # desactivar motores
ser.close()

d=np.array(filas)
with open(base+".csv","w",newline="") as f:
    w=csv.writer(f); w.writerow(C.COLS); w.writerows(filas)
met=calcular(d) if len(d)>10 else {"error":"pocas muestras"}
met.update({"controlador":a.controlador,"nombre":info["nombre"],"ganancias":info["ganancias"],
            "tipo":"HW","fecha":stamp})
json.dump(met,open(base+".json","w"),indent=2,ensure_ascii=False)
print(f"\n== {len(d)} muestras guardadas -> {base}.csv ==")
print(json.dumps(met,indent=2,ensure_ascii=False))
print(f"\nGrafica:  python3 extraccion/graficar.py {base}.csv")
