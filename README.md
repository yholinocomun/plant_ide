# Plant IDE — Péndulo Invertido (Robot Balancín)

Workspace para identificación de la planta y diseño de control en espacio de estados.

## Estructura

```
plant_ide/
├── firmware/
│   └── esp32_data_collector/
│       └── esp32_data_collector.ino   ← Firmware de identificación para ESP32-S3
├── python_tools/
│   ├── collect_data.py                ← Script principal de adquisición y análisis
│   └── requirements.txt
├── data/                              ← CSVs capturados
└── plots/                             ← Gráficas generadas
```

---

## Modelo en Espacio de Estados

```
x = [x,  x_dot,  theta,  theta_dot]^T
u = fuerza de entrada [N]  (escala PWM → N estimada desde datos)

ẋ = A·x + B·u
y = C·x + D·u

y = [x, theta]
```

---

## Setup rápido (Ubuntu)

```bash
cd plant_ide/python_tools
pip install -r requirements.txt
```

---

## Flujo de trabajo

### 1. Flashear firmware

Abre `firmware/esp32_data_collector/esp32_data_collector.ino` en Arduino IDE
y flashéalo al ESP32-S3.

Ajusta en el `.ino` antes de flashear:
- `PULSOS_POR_VUELTA` → resolución de tu encoder (cuadratura × PPR)
- `RADIO_RUEDA_M`     → radio real de tu rueda en metros

### 2. Ver puertos disponibles

```bash
python3 python_tools/collect_data.py --puertos
```

### 3. Capturar datos con señal PRBS (recomendado para identificación)

```bash
python3 python_tools/collect_data.py \
    --port /dev/ttyUSB0 \
    --modo prbs \
    --duracion 30
```

### 4. Capturar datos con escalón (para estimar K_pwm)

```bash
python3 python_tools/collect_data.py \
    --port /dev/ttyUSB0 \
    --modo escalon \
    --duracion 20
```

### 5. PWM manual fijo

```bash
python3 python_tools/collect_data.py \
    --port /dev/ttyUSB0 \
    --modo manual \
    --pwm 100 \
    --duracion 10
```

### 6. Analizar un CSV ya capturado

```bash
python3 python_tools/collect_data.py --analizar data/experimento_20240101_120000.csv
```

### 7. Gráfica en vivo durante captura

```bash
python3 python_tools/collect_data.py \
    --port /dev/ttyUSB0 \
    --modo prbs \
    --duracion 30 \
    --plot
```

---

## Comandos Serial (monitor serie directo)

| Comando | Efecto |
|---------|--------|
| `S`     | Start (inicia envío de datos y señal) |
| `T`     | Stop |
| `P`     | Modo PRBS automático |
| `E`     | Modo escalón alternado |
| `M100`  | PWM manual = +100 |
| `M-80`  | PWM manual = -80 |

---

## Formato CSV de salida

```
t_ms, angulo_deg, vel_angular_dps, pos_x_m, vel_x_ms, pwm
```

| Columna           | Variable estado | Unidad |
|-------------------|-----------------|--------|
| `angulo_deg`      | θ               | grados |
| `vel_angular_dps` | θ̇              | °/s    |
| `pos_x_m`         | x               | metros |
| `vel_x_ms`        | ẋ               | m/s    |
| `pwm`             | u (entrada)     | PWM    |

---

## Parámetros físicos a ajustar

Edita el dict `PARAMS` en `collect_data.py`:

```python
PARAMS = {
    "M":  0.5,   # masa del carro [kg]
    "m":  0.2,   # masa del péndulo [kg]
    "l":  0.15,  # longitud al CG del péndulo [m]
    "I":  0.006, # momento de inercia del péndulo [kg·m²]
    "g":  9.81,
    "b":  0.1,   # fricción viscosa del carro [N·s/m]
    "dt": 0.010, # tiempo de muestreo [s]
}
```

---

## Salida del análisis

El script imprime y guarda en `data/analisis_TIMESTAMP.txt`:

- Matrices A, B, C, D del modelo linealizado
- K_pwm estimado desde datos reales (regresión lineal)
- Valores propios de A (verifica polo inestable del péndulo)
- Rango de controlabilidad y observabilidad
