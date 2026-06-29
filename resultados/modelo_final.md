# Modelo final en espacio de estados (WIP)

Robot péndulo invertido sobre dos ruedas — todos los parámetros REALES.

## Parámetros

| Símbolo | Valor | Fuente |
|---------|-------|--------|
| M | 0.710 kg | Fase 0 |
| m_w | 0.095 kg | Fase 0 |
| r | 0.037 m | Fase 0 |
| l | 0.10 m | Fase 0 |
| I_p | 0.01164 kg·m² | Fase 2 |
| J_w | 0.000065 kg·m² | calculado (disco) |
| K | 0.0527 rad/s/PWM | Fase 1 |
| τ | 0.060 s | Fase 1 |
| u_dead | 30 PWM | Fase 1 |

## Espacio de estados

Estado: X = [x, ẋ, θ, θ̇]ᵀ      Salida: y = [x, θ]

```
A = [ 0    1        0         0 ]
    [ 0    0     -7.4697      0 ]
    [ 0    0        0         1 ]
    [ 0    0    104.6806      0 ]

B (entrada par τ [N·m]) = [0, 58.570, 0, -440.142]ᵀ
B (entrada PWM)         = [0, 0.1979, 0, -1.4869]ᵀ

C = [ 1 0 0 0 ]
    [ 0 0 1 0 ]
```

> Corregido: el modelo anterior daba +7.14 rad/s por doble conteo de M·l²
> en `den` (I_p ya es respecto al pivote). Ver `validacion_planta.md`.

## Análisis
- Polo inestable: +10.23 rad/s → τ_caída = 0.098 s
- Controlable: 4/4 ✓
- Observable: 4/4 ✓
- τ_motor (0.060s) << τ_caída (0.098s) → cascada PID o LQR viables
- Verificado experimental (data) vs teórico (Lagrangiano del paper): 0% de
  diferencia en el polo; motor teórico vs experimental: 1.7% en K.

## Listo para diseñar el controlador (LQR / pole-placement).
