# Cálculo / obtención de ganancias (uniforme por controlador)

Para cada controlador: (A) diseño teórico en simulación, (B) valor llevado a HW.
La planta de diseño usa el **actuador calibrado** (B×0.16) para que las ganancias
del diseño caigan cerca de las que funcionan en hardware.

## 1) LQR pre-gain
`K=lqr(A,B,Q,R)`, `Q=diag([1,1,300,10])`, `R=2e-4`. Se pasa a unidades de firmware:
`Kang_p = K₃·(π/180)` [PWM/°], `Kang_d = K₄·(π/180)`. HW: 59.5 / 1.7.

## 2) LQG
Mismo `K` del LQR + observador `L=lqe(A,Gw,C,Qw,Vv)` (o `kalman`). Se discretiza
(Ad,Bd) e incrusta L. HW: `K=[−70.7,−197,−1985,−284]`, `u=−K·x̂`.

## 3) PID Fraccionario
Paso 1: `Gc=pidtune(Gp,'PID')` → `Kp,Ki,Kd` de arranque. Paso 2: órdenes
fraccionarios `λ,μ` con aproximación de **Oustaloup** (`s^α`, banda [wb,wh], N).
HW re-sintonizado: `Kp=45, Ki=12 (λ=0.95), Kd=2.5 (μ=0.15)`.

## 4) LQR predictivo (MPC)
`Kmpc` = ganancia del **1er paso** de la recursión de Riccati de horizonte finito
(N=60): iterar `K=(R+Bdᵀ P Bd)⁻¹(Bdᵀ P Ad); P=Q+Adᵀ P(Ad−Bd K)`. → `u=−Kmpc·x`.

## 5) IMC
Filtro `Q` de 1er orden: `β=dt/(λ+dt)`, `λ=0.010 s`. Ley: PD (`K_ANG,K_GYRO`)
suavizado por `Q`: `u_imc(k)=u_imc(k−1)+β(u_ref−u_imc(k−1))`. HW: 43.5 / 3.10.

## 6) H∞
`P=augw(Gp,W1,W2,W3)` con pesos (W1 desempeño, W2 esfuerzo, W3 robustez);
`[Sc,CL,γ]=hinfsyn(P,1,1)`. Controlador `Sc` → discretizar `c2d(...,Ts,'tustin')`
→ `(Ad,Bd,Cd,Dd)` incrustados. HW: orden 6, `HGAIN=0.10` global.

## Nota sobre ajuste manual (sim → HW)
La planta real es difícil de modelar con exactitud (fricción, holguras, batería
arriba). Por eso todas las ganancias se **afinaron a mano** partiendo del valor de
simulación: se sube P hasta casi sostener, luego D hasta quitar oscilación, y se
ajusta el filtro/λ/escala global. Las ganancias finales de HW son las de las tablas.
