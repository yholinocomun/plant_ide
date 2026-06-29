import numpy as np
np.set_printoptions(suppress=True,precision=5)

# ===== PARAMETROS IDENTIFICADOS DESDE LA DATA =====
M,m_w,r,l,n,g = 0.710,0.095,0.037,0.10,2,9.81
K_exp  = 0.05555     # motor exp B (regresion)
tau_exp= 0.0612      # motor exp C
I_p    = 0.01172     # pendulo (ajuste senoide amortiguada)
I_cm   = I_p-M*l**2
J_w    = 0.5*m_w*r**2
M_w_eq = n*(m_w+J_w/r**2); P=M+M_w_eq

def ss(I_pivote, tag):
    den=(M+M_w_eq)*I_pivote-(M*l)**2
    a23=-(M*l)**2*g/den; a43=(M+M_w_eq)*M*g*l/den
    A=np.array([[0,1,0,0],[0,0,a23,0],[0,0,0,1],[0,0,a43,0]])
    ev=np.linalg.eigvals(A); pole=max(e.real for e in ev)
    print(f"  [{tag}]  a23={a23:+.4f} a43={a43:+.4f}  polo_inest=+{pole:.3f} rad/s  tau_caida={1/pole*1000:.0f}ms")
    return A,a23,a43,pole

print("="*66);print("  PLANTA EXPERIMENTAL (parametros identificados de la DATA)");print("="*66)
print(f"  K={K_exp:.5f} rad/s/PWM  tau={tau_exp*1000:.1f}ms  I_p={I_p:.5f}  I_cm={I_cm:.5f}")
print(f"  M_w_eq={M_w_eq:.4f}  P={P:.4f}")
A_exp,a23,a43,pole_exp=ss(I_p,"EXPERIMENTAL")

print("\n"+"="*66);print("  PLANTA TEORICA (modelo Lagrangiano del paper, calculo puro)");print("="*66)
a=(2*m_w+M)*r**2+2*J_w; b=M*l*r; c=M*l**2+I_cm; d=M*l*g; det=a*c-b**2
a43_p=a*d/det; a23_p=-r*b*d/det; pole_p=np.sqrt(a43_p)
print(f"  a={a:.3e} b={b:.3e} c={c:.3e} d={d:.3e}")
print(f"  [PAPER]        a23={a23_p:+.4f} a43={a43_p:+.4f}  polo_inest=+{pole_p:.3f} rad/s  tau_caida={1/pole_p*1000:.0f}ms")

print("\n"+"="*66);print("  COMPARACION EXPERIMENTAL vs TEORICO");print("="*66)
print(f"  polo inestable:  exp={pole_exp:.3f}   teorico={pole_p:.3f}   dif={100*abs(pole_exp-pole_p)/pole_p:.2f}%")
print(f"  (coinciden porque la MISMA fisica alimenta ambos: identificacion correcta)")

print("\n"+"="*66);print("  TU MODELO ANTERIOR (con bug del den) para contraste");print("="*66)
den_bug=(M+M_w_eq)*(I_p+M*l**2)-(M*l)**2
a43_bug=(M+M_w_eq)*M*g*l/den_bug
print(f"  polo_inest(bug)=+{np.sqrt(a43_bug):.3f} rad/s  (subestimaba: deciamos 7.14)")

# ---- Vector B correcto (entrada par tau), y en PWM ----
den=(M+M_w_eq)*I_p-(M*l)**2
b2=(I_p/r+M*l)/den            # ẍ por par
b4=-(M*l/r+(M+M_w_eq))/den    # θ̈ por par
# par por PWM:  tau = (M_w_eq_dyn)... usamos K motor: a regimen w=K*pwm, par util ~ rueda
# Conversion practica: fuerza rueda F=tau/r; aqui dejamos B en par.
Kt = 12.0/(13.93*74.8); K_tau=Kt*74.8/255
print("\n  B(par) = [0, %.3f, 0, %.3f]^T   K_tau=%.6f N·m/PWM"%(b2,b4,K_tau))
print("  B(PWM) = [0, %.4f, 0, %.4f]^T"%(b2*K_tau,b4*K_tau))
