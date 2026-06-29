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
    [ 0    0     -3.6348      0 ]
    [ 0    0        0         1 ]
    [ 0    0     50.9381      0 ]

B (entrada par τ [N·m]) = [0, 178.27, 0, -2117.62]ᵀ
B (entrada PWM)         = [0, 0.6022, 0, -7.1538]ᵀ

C = [ 1 0 0 0 ]
    [ 0 0 1 0 ]
```

## Análisis
- Polo inestable: +7.14 rad/s → τ_caída = 0.14 s
- Controlable: 4/4 ✓
- Observable: 4/4 ✓
- τ_motor (0.060s) << τ_caída (0.14s) → cascada PID o LQR viables

## Listo para diseñar el controlador (LQR / pole-placement).
