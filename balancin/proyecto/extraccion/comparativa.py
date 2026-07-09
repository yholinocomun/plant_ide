"""Tabla comparativa SIM vs HARDWARE (lee resultados/*_SIM.json y los *.json de HW)."""
import sys,os,glob,json
import numpy as np, matplotlib; matplotlib.use("Agg"); import matplotlib.pyplot as plt
sys.path.insert(0,os.path.dirname(__file__)); from estilo import aplicar
sys.path.insert(0,os.path.join(os.path.dirname(__file__),"..","config")); import controladores as C
aplicar()
res=os.path.join(os.path.dirname(__file__),"..","resultados")
def load(slug,tipo):
    if tipo=="SIM":
        p=os.path.join(res,f"{slug}_SIM.json"); return json.load(open(p)) if os.path.exists(p) else None
    js=sorted(glob.glob(os.path.join(res,f"{slug}_2*.json")))   # HW: <slug>_<fecha>.json
    return json.load(open(js[-1])) if js else None
hdr="| Controlador | θRMS sim [°] | θRMS HW [°] | |u|max sim | |u|max HW | estable sim | sat HW % |"
sep="|---|---|---|---|---|---|---|"; lines=[hdr,sep]; print(hdr); print(sep)
for slug in C.CTRL:
    s=load(slug,"SIM"); h=load(slug,"HW"); nom=C.CTRL[slug]["nombre"]
    def g(d,k): return d.get(k,"—") if d else "—"
    row=f"| {nom} | {g(s,'theta_RMS_deg')} | {g(h,'theta_RMS_deg')} | {g(s,'u_max_abs_pwm')} | "\
        f"{g(h,'u_max_abs_pwm')} | {('sí' if s and s.get('estable_sim') else 'no') if s else '—'} | {g(h,'saturacion_pct')} |"
    print(row); lines.append(row)
open(os.path.join(res,"tabla_comparativa.md"),"w").write("# Comparativa sim vs hardware\n\n"+"\n".join(lines)+"\n")
# figura comparativa de theta (HW si existe, si no SIM) - se rellena al tener data
print("\n-> resultados/tabla_comparativa.md")
