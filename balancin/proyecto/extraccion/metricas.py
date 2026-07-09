"""Metricas UNIFORMES de desempeno para todos los controladores."""
import numpy as np
def calcular(d):
    t=(d[:,0]-d[0,0])/1000.0; th=d[:,1]; u=d[:,5]
    # ventana en regimen (ultimo 60%)
    m=t>0.4*t[-1]
    return {
      "duracion_s": round(float(t[-1]),2),
      "theta_RMS_deg": round(float(np.sqrt(np.mean(th**2))),3),
      "theta_std_deg": round(float(np.std(th[m])),3),
      "theta_max_abs_deg": round(float(np.max(np.abs(th))),3),
      "theta_medio_deg": round(float(np.mean(th[m])),3),
      "u_max_abs_pwm": round(float(np.max(np.abs(u))),1),
      "u_RMS_pwm": round(float(np.sqrt(np.mean(u**2))),1),
      "saturacion_pct": round(float(100*np.mean(np.abs(u)>=254)),1),
      "deriva_x_m": round(float(d[-1,3]-d[0,3]),3),
      "muestras": int(len(d)),
    }
