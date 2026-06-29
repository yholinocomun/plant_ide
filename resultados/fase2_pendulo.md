# Resultados Fase 2 — Oscilación libre del péndulo

Fecha: 2026-06-28
Archivo: data/pendulo_20260628_213317.csv

## Nota sobre el análisis
La primera detección de periodo falló (ruido del giroscopio → cruces
falsos → T=0.46s, I_cm negativo = imposible). Re-analizado con filtro
pasa-bajos + histéresis + FFT → resultado válido.

## Resultado válido

| Parámetro | Valor |
|-----------|-------|
| Periodo T | 0.812 s (±0.020) |
| ω_n | 7.74 rad/s |
| I_p | 0.01164 kg·m² |
| I_cm | 0.00454 kg·m² (positivo ✓) |
| ζ | 0.036 (poco amortiguado) |
| Ciclos | 9 (limpios) |

Validación física: T=0.812s > T_min=0.634s ✓ → dato confiable.
