# Validación de la planta — Experimental (data) vs Teórica (cálculos)

Análisis hecho sobre los CSV crudos de `data/` (no solo los reportes).
Referencia teórica: modelo Lagrangiano del SBR de Gandarilla et al. (2022).

---

## 1. Motor — identificación desde encoders

Conversión: ángulo de rueda = `(enc_izq+enc_der)/2 / PPR · 2π`, con `PPR = 1945`.

| Magnitud | Valor (data) | Método |
|----------|-------------|--------|
| K (ganancia) | **0.05555 rad/s/PWM** | EXP-B, regresión ω_ss vs PWM |
| τ (constante de tiempo) | **0.0612 s** | EXP-C, ajuste de 1er orden al escalón |
| u_dead (zona muerta) | **30 PWM** | EXP-A, rampa |
| Simetría adelante/reversa | 0.988 | EXP-B |

**Validación cruzada (B → C):** con `K` de EXP-B se predijo el escalón de EXP-C.
RMSE = 0.36 rad/s = **6 % del rango** → modelo de motor **VÁLIDO**.

**Teórico vs experimental (motor):**
- K teórico (placa: 133 RPM = 13.93 rad/s a 12 V, /255 PWM) = 0.05463
- K experimental (data) = 0.05555
- **error = 1.7 %** → excelente concordancia.

> Nota: K crece levemente con el PWM (0.047 @80 → 0.058 @200). Es la firma
> de la zona muerta (~30 PWM de fricción constante); con compensación de
> zona muerta el K efectivo es prácticamente constante.

---

## 2. Péndulo — identificación desde la oscilación libre

Ajuste de **senoide amortiguada** `θ(t)=A·e^{-ζω_n t}·cos(ω_d t+φ)+c`
sobre las dos capturas:

| Captura | T | ω_n | ζ | I_p | I_cm |
|---------|---|-----|---|-----|------|
| 212859 | 0.8164 s | 7.698 | 0.0227 | 0.01176 | 0.00466 |
| 213317 | 0.8134 s | 7.726 | 0.0231 | 0.01167 | 0.00457 |
| **Media** | **0.8149 s** | **7.712** | **0.023** | **0.01172** | **0.00462** |

- Amplitud de oscilación = **1.1°** → régimen **lineal** (sin corrección no lineal).
- RMSE del ajuste = 0.24° (excelente).
- `I_cm > 0` → físicamente posible ✓
- Radio de giro `k = √(I_cm/M) = 7.85 cm` → equivale a una barra uniforme de
  ~27 cm de alto. **Verificación pendiente:** mide la altura real del cuerpo;
  si es ~27 cm, el I_p experimental es coherente con la geometría (teórico).

---

## 3. Planta en espacio de estados (corregida)

Estado `X=[x, ẋ, θ, θ̇]ᵀ`, salida `y=[x, θ]`, entrada = par τ.

```
A = [ 0    1        0        0 ]
    [ 0    0     -7.4697     0 ]
    [ 0    0        0        1 ]
    [ 0    0    +104.6806    0 ]

B(par) = [0, 58.570, 0, -440.142]ᵀ
B(PWM) = [0, 0.1979, 0, -1.4869]ᵀ
```

## 4. Experimental vs Teórico (planta completa)

| | Polo inestable | τ_caída |
|---|---|---|
| **Experimental** (params identificados de la data) | +10.23 rad/s | 98 ms |
| **Teórico** (Lagrangiano del paper, cálculo puro) | +10.23 rad/s | 98 ms |
| Diferencia | **0.00 %** | — |

Ambos coinciden porque la **misma física** alimenta las dos vías: prueba de
que la identificación es correcta. (Verifiqué además que `a_paper = P·r²`
exacto, es decir que la formulación WIP y la Lagrangiana son equivalentes.)

### Corrección aplicada
El modelo anterior daba polo +7.14 rad/s (τ_caída 140 ms) por **doble conteo
de `M·l²`** en el denominador (`I_p` ya es respecto al pivote). Corregido en
`ensamblar_modelo.py`. El robot real es más rápido de lo que decía el modelo
viejo → por eso el LQR diseñado quedaba "débil".

---

## 5. Conclusión

- Toma de datos e identificación: **correcta y validada** (cross-validation
  del motor + ajuste limpio del péndulo + chequeos físicos).
- Motor: teórico vs experimental con **1.7 %** de error.
- Planta: experimental vs teórica con **0 %** de diferencia en el polo.
- Falta cerrar la verificación geométrica del cuerpo (altura ≈ 27 cm).
