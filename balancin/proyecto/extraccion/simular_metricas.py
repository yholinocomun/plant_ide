"""Simula los 6 controladores (ganancias de HW) y guarda metricas _SIM.json,
para llenar la tabla comparativa junto con la data experimental."""
import numpy as np, json, os, sys
sys.path.insert(0,os.path.join(os.path.dirname(__file__),"..","..","python")); from planta import plant
sys.path.insert(0,os.path.join(os.path.dirname(__file__),"..","config")); import controladores as C
sys.path.insert(0,os.path.dirname(__file__)); from metricas import calcular
A,B,Cc,D=plant(); n=4; dt=0.01; R2D=180/np.pi
Ad=np.eye(n)+A*dt+0.5*(A@A)*dt**2; Bd=(np.eye(n)*dt+0.5*A*dt**2)@B.flatten()
res=os.path.join(os.path.dirname(__file__),"..","resultados"); os.makedirs(res,exist_ok=True)
N=int(4/dt)

def sim(ctrl_fn):
    x=np.array([0,0,5/R2D,0.]); TH=[]; U=[]; st={}; estable=True
    for k in range(N):
        u=float(np.clip(ctrl_fn(x,st),-255,255))
        TH.append(x[2]*R2D); U.append(u)
        x=Ad@x+Bd*u
        if abs(x[2]*R2D)>60:            # cayo (fuera de rango real)
            estable=False; break
    m=len(TH); d=np.zeros((m,6)); d[:,0]=np.arange(m)*1000*dt; d[:,1]=TH; d[:,3]=x[0]; d[:,5]=U
    return d, estable

# --- leyes de control con ganancias de HW ---
g=C.CTRL
def f_lqr(x,st):
    G=g["lqr"]["ganancias"]; return G["Kang_p"]*(x[2]*R2D)+G["Kang_d"]*(x[3]*R2D)
def f_lqg(x,st):
    K=np.array(g["lqg"]["ganancias"]["K"]); return -(K[2]*x[2]+K[3]*x[3])
def f_mpc(x,st):
    K=np.array(g["mpc"]["ganancias"]["Kmpc"]); return -(K[2]*x[2]+K[3]*x[3])
def f_imc(x,st):
    G=g["imc"]["ganancias"]; uref=G["K_ANG"]*(x[2]*R2D)+G["K_GYRO"]*(x[3]*R2D)
    b=dt/(G["LAMBDA"]+dt); st["u"]=st.get("u",0)+b*(uref-st.get("u",0)); return G["GAIN"]*st["u"]
def f_fopid(x,st):
    G=g["fopid"]["ganancias"]; lam,mu=G["lambda"],G["mu"]
    e=x[2]*R2D; buf=st.setdefault("e",[]); buf.insert(0,e); buf[:]=buf[:64]
    if "cI" not in st:
        L=64; cI=[1.0];cD=[1.0]
        for j in range(1,L): cI.append(cI[-1]*(1-(-lam+1)/j)); cD.append(cD[-1]*(1-(mu+1)/j))
        st["cI"]=np.array(cI); st["cD"]=np.array(cD)
    cI,cD=st["cI"][:len(buf)],st["cD"][:len(buf)]; bb=np.array(buf)
    I=(dt**lam)*np.sum(cI*bb); Dv=(dt**-mu)*np.sum(cD*bb)
    I=np.clip(I,-200,200); return G["Kp"]*e+G["Ki"]*I+G["Kd"]*Dv
def f_hinf(x,st):
    num=np.array([-9644.251425,14377.850121,4271.651618,-14402.416908,5348.033021])
    den=np.array([1.0,-1.024125,-0.727214,1.035496,-0.261415]); HG=g["hinf"]["ganancias"]["HGAIN"]
    eb=st.setdefault("eb",[0.]*5); ub=st.setdefault("ub",[0.]*5)
    e=-x[2]; eb.insert(0,e); eb.pop(); 
    u=(np.dot(num,eb)-np.dot(den[1:],ub[:4]))/den[0]; ub.insert(0,u); ub.pop()
    return HG*u

for slug,fn in [("lqr",f_lqr),("lqg",f_lqg),("mpc",f_mpc),("imc",f_imc),("fopid",f_fopid),("hinf",f_hinf)]:
    d,estable=sim(fn); m=calcular(d); m.update({"controlador":slug,"nombre":g[slug]["nombre"],"tipo":"SIM","estable_sim":bool(estable)})
    json.dump(m,open(os.path.join(res,f"{slug}_SIM.json"),"w"),indent=2,ensure_ascii=False)
    print(f"{g[slug]['nombre']:22} SIM: estable={estable}  thetaRMS={m['theta_RMS_deg']:.2f}  |u|max={m['u_max_abs_pwm']:.0f}")
