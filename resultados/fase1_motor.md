# Resultados Fase 1 — Identificación del motor

Fecha de captura: 2026-06-28
Montaje: ruedas en el aire (sin carga del suelo)

## Modelo del motor

```
G(s) = K / (τ·s + 1) = 0.0527 / (0.060·s + 1)
```

| Parámetro | Valor | Unidad |
|-----------|-------|--------|
| K (ganancia) | 0.0527 | rad/s/PWM |
| τ (constante de tiempo) | 0.060 | s |
| u_dead (zona muerta) | 30 | PWM |

## Sub-experimento A — Zona muerta
- u_dead adelante: 30 PWM
- u_dead reversa: ~30 PWM (por simetría)

## Sub-experimento B — Ganancia K (escalera)

| PWM | ω_ss [rad/s] | K [rad/s/PWM] |
|-----|--------------|---------------|
| -160 | -9.271 | 0.05795 |
| -120 | -6.690 | 0.05575 |
| -80  | -4.103 | 0.05128 |
| 80   | 3.815  | 0.04769 |
| 120  | 6.328  | 0.05274 |
| 160  | 8.845  | 0.05528 |
| 200  | 11.381 | 0.05690 |

- K promedio: 0.05394 rad/s/PWM
- Asimetría f/b: 0.967 (excelente, ideal=1.0)

## Sub-experimento C — Constante de tiempo τ
- Escalón PWM = 120
- ω_ss = 6.319 rad/s
- τ (63.2%) = 0.0600 s
- τ (ajuste exp) = 0.0597 s
- K (ajuste exp) = 0.05256 rad/s/PWM

## Conclusión
- Motor lineal y casi simétrico → datos confiables
- τ pequeño (0.060s) → el motor es rápido → separación de escalas
  con el péndulo se cumple → cascada PID o LQR ambos viables
