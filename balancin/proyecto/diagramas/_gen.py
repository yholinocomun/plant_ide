# Genera diagramas de bloques (.drawio) del control de ANGULO para cada controlador.
# Estilo decente: cajas blancas, borde negro, sin colores llamativos.
import html, os
OUT=os.path.dirname(__file__)
BOX='rounded=0;whiteSpace=wrap;html=1;fillColor=#ffffff;strokeColor=#000000;fontColor=#000000;fontSize=12;'
PLANT='rounded=0;whiteSpace=wrap;html=1;fillColor=#f5f5f5;strokeColor=#000000;fontColor=#000000;fontSize=12;fontStyle=1;'
SUM='ellipse;whiteSpace=wrap;html=1;fillColor=#ffffff;strokeColor=#000000;fontSize=14;'
EDGE='edgeStyle=orthogonalEdgeStyle;rounded=0;html=1;strokeColor=#000000;endArrow=block;endFill=1;fontSize=10;'
TXT='text;html=1;align=center;fontSize=11;fontColor=#000000;'

def box(i,x,y,w,h,val,style=BOX):
    return (f'<mxCell id="{i}" value="{html.escape(val)}" style="{style}" vertex="1" parent="1">'
            f'<mxGeometry x="{x}" y="{y}" width="{w}" height="{h}" as="geometry"/></mxCell>')
def edge(i,s,t,val=""):
    return (f'<mxCell id="{i}" value="{html.escape(val)}" style="{EDGE}" edge="1" parent="1" source="{s}" target="{t}">'
            f'<mxGeometry relative="1" as="geometry"/></mxCell>')
def label(i,x,y,val):
    return (f'<mxCell id="{i}" value="{html.escape(val)}" style="{TXT}" vertex="1" parent="1">'
            f'<mxGeometry x="{x}" y="{y}" width="90" height="20" as="geometry"/></mxCell>')

def wrap(name,cells):
    body='\n'.join(cells)
    return (f'<mxfile host="app.diagrams.net">\n<diagram name="{name}" id="{name}">\n'
            f'<mxGraphModel dx="900" dy="500" grid="1" gridSize="10" guides="1" '
            f'tooltips="1" connect="1" arrows="1" page="1" pageWidth="900" pageHeight="380" math="0">\n'
            f'<root><mxCell id="0"/><mxCell id="1" parent="0"/>\n{body}\n</root></mxGraphModel></diagram></mxfile>')

def guardar(name,cells):
    open(os.path.join(OUT,name),'w').write(wrap(name.replace('.drawio',''),cells))
    print("->",name)

# ---------- lazo simple:  ref -> (Σ) -> [Ctrl] -> u -> [Planta] -> theta -> [sensor] -> feedback ----------
def lazo_simple(fname, ctrl_label):
    c=[]
    c+=[box('ref',20,150,70,40,'θ_ref = 0')]
    c+=[box('sum',140,150,40,40,'Σ',SUM)]
    c+=[label('lm',120,120,'+')]
    c+=[label('lf',150,185,'−')]
    c+=[box('ctrl',230,140,180,60,ctrl_label)]
    c+=[box('plant',470,140,190,60,'Planta\\nMotor + Péndulo',PLANT)]
    c+=[box('sens',470,270,190,40,'Sensor  MPU6050 ( θ )')]
    c+=[label('out',700,150,'θ')]
    c+=[edge('e1','ref','sum')]
    c+=[edge('e2','sum','ctrl','u→')]
    c+=[edge('e3','ctrl','plant','u [PWM]')]
    c+=[edge('e4','plant','sens','θ')]
    c+=[edge('e5','sens','sum')]
    guardar(fname,c)

lazo_simple('bloque_lqr.drawio',
    'Controlador LQR\\nu = -(K_p\\u00b7θ + K_d\\u00b7θ̇)\\nK_p=59.5, K_d=1.7')
lazo_simple('bloque_imc.drawio',
    'IMC\\nu_ref=K_ang\\u00b7θ + K_gyro\\u00b7θ̇\\nFiltro Q: β=Δt/(λ+Δt)')
lazo_simple('bloque_hinf.drawio',
    'Controlador H∞  K(z)\\n(sensibilidad mixta)\\nmixsyn(G,W1,W2,W3)')

# ---------- LQG: planta -> y -> [Kalman] -> x_est -> [-K] -> u ----------
def lqg():
    c=[]
    c+=[box('ref',20,150,70,40,'θ_ref = 0')]
    c+=[box('sum',140,150,40,40,'Σ',SUM)]
    c+=[label('lf',150,185,'−')]
    c+=[box('gain',230,140,150,60,'Ganancia\\nu = -K\\u00b7x̂')]
    c+=[box('plant',440,140,180,60,'Planta\\nMotor + Péndulo',PLANT)]
    c+=[box('kal',360,270,200,60,'Observador de Kalman\\nx̂ = Ax̂+Bu+L(y-Cx̂)')]
    c+=[label('out',660,150,'θ')]
    c+=[edge('e1','ref','sum')]
    c+=[edge('e2','sum','gain','u→')]
    c+=[edge('e3','gain','plant','u [PWM]')]
    c+=[edge('e4','plant','kal','y=[x,θ]')]
    c+=[edge('e5','kal','sum','x̂')]
    guardar('bloque_lqg.drawio',c)
lqg()

# ---------- Cascada: PI externo -> P interno (con w=thetadot+alpha*theta) ----------
def cascada():
    c=[]
    c+=[box('ref',10,150,60,40,'θ_ref=0')]
    c+=[box('s1',110,150,40,40,'Σ',SUM)]; c+=[label('l1',120,185,'−')]
    c+=[box('ext',180,140,150,60,'Lazo externo PI\\nv=k_c(e+α∫e)')]
    c+=[box('s2',360,150,40,40,'Σ',SUM)]; c+=[label('l2',370,185,'−')]
    c+=[box('int',420,140,150,60,'Lazo interno P\\nu=k_2(v-w)')]
    c+=[box('plant',600,140,170,60,'Planta\\nMotor + Péndulo',PLANT)]
    c+=[box('w',420,270,150,50,'w = θ̇ + α·θ')]
    c+=[box('sens',180,270,150,40,'Sensor  ( θ )')]
    c+=[label('out',790,150,'θ')]
    c+=[edge('e1','ref','s1')]
    c+=[edge('e2','s1','ext','e')]
    c+=[edge('e3','ext','s2','v')]
    c+=[edge('e4','s2','int')]
    c+=[edge('e5','int','plant','u [PWM]')]
    c+=[edge('e6','plant','w','θ,θ̇')]
    c+=[edge('e7','w','s2')]
    c+=[edge('e8','plant','sens')]
    c+=[edge('e9','sens','s1')]
    guardar('bloque_cascada.drawio',c)
cascada()
print("Listo.")
