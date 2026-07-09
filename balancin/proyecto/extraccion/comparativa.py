"""Tabla + figura comparativa de todos los controladores (lee resultados/*.json/.csv)."""
import sys,os,glob,json
import numpy as np, matplotlib; matplotlib.use("Agg"); import matplotlib.pyplot as plt
sys.path.insert(0,os.path.dirname(__file__)); from estilo import aplicar
sys.path.insert(0,os.path.join(os.path.dirname(__file__),"..","config")); import controladores as C
aplicar()
res=os.path.join(os.path.dirname(__file__),"..","resultados")
jsons=sorted(glob.glob(os.path.join(res,"*.json")))
# tabla markdown
filas=[]
for j in jsons:
    m=json.load(open(j)); filas.append(m)
hdr="| Controlador | θRMS [°] | θstd [°] | |θ|max [°] | |u|max | sat % | derivaX [m] |"
sep="|---|---|---|---|---|---|---|"
print(hdr); print(sep)
lines=[hdr,sep]
for m in filas:
    row=f"| {m.get('nombre','')} | {m.get('theta_RMS_deg','')} | {m.get('theta_std_deg','')} | "\
        f"{m.get('theta_max_abs_deg','')} | {m.get('u_max_abs_pwm','')} | {m.get('saturacion_pct','')} | {m.get('deriva_x_m','')} |"
    print(row); lines.append(row)
open(os.path.join(res,"tabla_comparativa.md"),"w").write("\n".join(lines))
# figura: theta de todos superpuesto
plt.figure(figsize=(10,5))
for j in jsons:
    slug=os.path.basename(j).split("_")[0]; info=C.CTRL.get(slug,{})
    csvf=j.replace(".json",".csv")
    if not os.path.exists(csvf): continue
    d=np.loadtxt(csvf,delimiter=",",skiprows=1); t=(d[:,0]-d[0,0])/1000
    plt.plot(t,d[:,1],label=info.get("nombre",slug),color=info.get("color"))
plt.axhline(0,ls='--',c='k',alpha=.4); plt.xlabel("t [s]"); plt.ylabel("θ [deg]")
plt.title("Comparativa de controladores — ángulo (hardware)"); plt.legend()
plt.savefig(os.path.join(res,"comparativa_theta.png")); print("-> comparativa_theta.png + tabla_comparativa.md")
