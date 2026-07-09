# Análisis de los 6 controladores (simulación + hardware)

Planta común (identificada y validada): péndulo invertido sobre 2 ruedas,
`X=[x, ẋ, θ, θ̇]`, polo inestable **+10.23 rad/s**, entrada PWM.
```
A=[0 1 0 0; 0 0 -7.4697 0; 0 0 0 1; 0 0 104.6806 0]   B=[0;0.1979;0;-1.4869]
```
Nota transversal: el actuador real es **~6× más débil** que la placa → todas las
ganancias de hardware son mayores que las del diseño nominal (por eso `Kang_p≈59.5`).

| # | Controlador | Ley de control (HW) | Ganancias sintonizadas | ¿Controla? | Diseño (sim) |
|---|---|---|---|---|---|
| 1 | **LQR pre-gain** | `u=Kang_p·θ+Kang_d·θ̇ (+pos)` | 59.5 / 1.7 / 30.36 / 61.09 | ✅ bien | `lqr(A,B,Q,R)` |
| 2 | **LQG** | `u=−K·x̂` (Kalman) | K=[−70.7,−197,−1985,−284] | ✅ | `lqr`+`lqe` |
| 3 | **PID Fraccionario** | `u=Kp·e+Ki·I^λ+Kd·D^μ` | 45 / 12 / 2.5, λ=0.95, μ=0.15 | ✅ algo | `pidtune`→FOPID (Oustaloup) |
| 4 | **LQR predictivo (MPC)** | *(no había código)* | — | ⚠️ falta | Riccati horizonte N |
| 5 | **IMC** | PD + filtro Q 1er orden | 43.5 / 3.10, λ=0.010, gain=0.75 | ✅ | `Q=inv(Gm)F`, F=1/(λs+1)³ |
| 6 | **H∞** | `xc(k+1)=Ad xc+Bd e; u=Cd xc+Dd e` | orden 6, HGAIN=0.10 | ✅ | `augw`+`hinfsyn` (controla x) |

## Observaciones por controlador
- **LQR**: referencia que funciona. Complementario 0.98/0.02, zona muerta comp.,
  seguridad no trabante. Es el **patrón** para estandarizar los demás.
- **LQG**: implementa el observador de Kalman (A,B,K,L incrustadas). Ojo: usa
  `pwm=abs(u)` + dirección por signo → correcto pero conviene unificar con `constrain(u,-255,255)`.
- **FOPID**: en MATLAB parten de `pidtune` (PID entero) y lo pasan a fraccionario
  con Oustaloup (μ,λ≈0.98). En **hardware** re-sintonizaron a μ=0.15 (derivada casi
  entera-baja) — discrepancia sim/HW normal (planta real más débil + ruido).
- **MPC**: **vacío** en el documento. Propuesta: usar el LQR predictivo (Riccati de
  horizonte finito) ya construido en `balancin/` → ganancia `Kmpc`.
- **IMC**: no es el IMC clásico (planta inestable) sino un **PD con filtro Q**
  (`beta=dt/(λ+dt)`), que es la forma práctica e implementable. Correcto para HW.
- **H∞**: sintetizado sobre la salida **x** (posición) con `hinfsyn`; controlador
  dinámico de orden 6 corriendo como ecuación de estados en el ESP32. El archivo
  además trae una variante de realimentación de estado (Kx,Kxd,Kθ,Kθ̇).

## Estandarización requerida (para el microcontrolador)
Todos deben compartir, tomando el **LQR como plantilla**:
1. Mismos pines/MPU/encoders/PWM (core 3.x `ledcAttach`).
2. Filtro complementario 0.98/0.02 y `theta_dot` = giroscopio.
3. Seguridad no trabante (`ANGULO_CAIDA`), zona muerta (`U_DEAD`), banda muerta.
4. **Mismas teclas base** (space,z,i,g,o,f) + **`t` de telemetría uniforme**.
5. **Mismo bloque de telemetría** (ver `firmware/BLOQUE_TELEMETRIA.md`).
