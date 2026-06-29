# Plant IDE — Péndulo Invertido sobre dos ruedas (Robot Balancín)

Identificación de la planta por fases (grey-box) y modelo en espacio de estados.
Autor: Yholinño Comun Perez · ESP32-S3 + JGA25-370B (74.8:1) + GM25-13CPR + MPU6050

> **Empieza por [`GUIA_TOMA_DATOS.md`](GUIA_TOMA_DATOS.md)** — el paso a paso completo.

## Estructura

```
plant_ide/
├── GUIA_TOMA_DATOS.md                 ← guía paso a paso (LEER PRIMERO)
├── firmware/
│   ├── exp1_motor/exp1_motor.ino      ← Fase 1: identificar el motor
│   └── exp2_pendulo/exp2_pendulo.ino  ← Fase 2: oscilación del péndulo
├── python_tools/
│   ├── verificar_setup.py             ← comprobación previa (correr primero)
│   ├── analizar_motor.py              ← Fase 1: K, τ, zona muerta
│   ├── analizar_pendulo.py            ← Fase 2: I_p
│   ├── ensamblar_modelo.py           ← Fase 4: A, B, C, D
│   └── requirements.txt
├── data/                              ← CSVs capturados
└── plots/                            ← gráficas generadas
```

## Modelo en Espacio de Estados (WIP — péndulo sobre ruedas)

```
Estado:   X = [x,  ẋ,  θ,  θ̇]ᵀ
Entrada:  u = par del motor τ [N·m]   (τ = K_tau · PWM)
Salida:   y = [x, θ]   (x del encoder, θ del MPU)

ẋ = A·X + B·u
y = C·X

La "masa del carro" incluye la inercia rotacional de las ruedas (J_w/r²).
NO es el cart-pole clásico.
```

## Parámetros medidos

| Símbolo | Valor | Origen |
|---------|-------|--------|
| M | 0.710 kg | balanza (cuerpo sin ruedas) |
| m_w | 0.095 kg | balanza (una rueda) |
| r | 0.037 m | regla |
| l | 0.10 m | equilibrio en filo |
| PPR | 1945 | 13 CPR × 2 × 74.8 |
| K, τ, u_dead | Fase 1 | experimento motor |
| I_p | Fase 2 | experimento péndulo |
| J_w | ½·m_w·r² | calculado |

## Setup rápido (Ubuntu)

```bash
pip install -r python_tools/requirements.txt
sudo usermod -aG dialout $USER     # cerrar sesión y volver a entrar
```

## Flujo de trabajo

```
0. verificar_setup.py                     ← comprobar puerto + encoder
1. exp1_motor.ino + analizar_motor.py     ← K, τ, u_dead (capturas A/B/C)
2. exp2_pendulo.ino + analizar_pendulo.py ← I_p
3. (fórmula, automático)                  ← J_w
4. ensamblar_modelo.py                    ← A, B, C, D
```

Ver comandos exactos en `GUIA_TOMA_DATOS.md`.
