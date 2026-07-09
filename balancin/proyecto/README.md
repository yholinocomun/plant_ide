# Proyecto Final — 6 controladores estandarizados + extracción de data

Análisis, estandarización para **ESP32-S3**, extracción de data experimental y
comparación de 6 controladores del balancín. Todo **modular y uniforme**:
un código de control, un extractor y un graficador por controlador, con el mismo
formato y estilo.

## Estructura (modular)
```
proyecto/
├── config/controladores.py     ← ganancias/metadatos de los 6 (fuente única)
├── firmware/                    ← .ino estandarizados + BLOQUE_TELEMETRIA.md
├── extraccion/
│   ├── extraer_datos.py  .m     ← serial -> CSV uniforme (por controlador)
│   ├── graficar.py       .m     ← figura ESTANDAR (misma para los 6)
│   ├── comparativa.py           ← tabla + figura de TODOS juntos
│   ├── metricas.py       .m     ← métricas uniformes (RMS, sat, deriva...)
│   └── estilo.py                ← estilo único de figuras (Python)
├── resultados/                  ← CSV + JSON + PNG por corrida
└── docs/  analisis.md · calculo_ganancias.md
```

## Flujo de trabajo (por controlador)
```bash
# 1) Flashear el .ino (con el bloque de telemetría). Activar control y pulsar 't'.
# 2) Extraer data:
python3 extraccion/extraer_datos.py --controlador lqr --port /dev/ttyUSB0 --seg 25
# 3) Graficar (figura estándar):
python3 extraccion/graficar.py resultados/lqr_<fecha>.csv
# 4) Cuando tengas los 6, comparar:
python3 extraccion/comparativa.py        # -> tabla_comparativa.md + comparativa_theta.png
```
MATLAB: `extraer_datos("lqr","/dev/ttyUSB0",25)` · `graficar("resultados/lqr_...csv")`

## Formato de telemetría UNIFORME (todos los firmwares)
```
t_ms, theta_deg, theta_dot_dps, x_m, x_dot_ms, u_pwm, setpoint_deg, modo
```
Ver `firmware/BLOQUE_TELEMETRIA.md` (bloque a pegar + justificación).
