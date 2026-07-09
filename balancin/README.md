# Balancín — 5 controladores avanzados

Diseño, **simulación** (MATLAB + Python) e **implementación** (Arduino IDE) de
5 controladores para el robot péndulo invertido sobre dos ruedas, sobre la
**planta identificada y corregida** (polo inestable +10.23 rad/s) con el
**actuador calibrado** (motor real ≈6× más débil que la placa).

Base de referencia: firmware **LQR** que funciona en hardware (`Kang_p≈59.5`).
Convenciones tomadas de `control_procesos` (LQG con `lqr`+`lqe`, IMC con
`F=(γs+1)/(λs+1)^n`, H∞ con `augw/mixsyn`, MPC por QP).

## Estructura
```
balancin/
├── python/    planta.py, sim_lqg/mpc/hinf/imc/fopid.py, logger.py
├── matlab/    planta.m, sim_lqg/mpc/hinf/imc/fopid.m, logger.m
├── arduino/   balance_LQG.ino, balance_MPC.ino, balance_dinamico.ino
└── data/      graficas de simulacion + coeficientes_discretos.txt
```

## Los 5 controladores (todos verificados: estabilizan en simulación)
| # | Controlador | Sim | Implementación Arduino |
|---|-------------|-----|------------------------|
| 1 | **LQG** | `sim_lqg` | `balance_LQG.ino` (Kalman + u=−K·x̂) |
| 2 | **MPC / LQR predictivo** | `sim_mpc` | `balance_MPC.ino` (ganancia predictiva) |
| 3 | **H∞** (sensibilidad mixta) | `sim_hinf` | `balance_dinamico.ino` (IIR K(z), cargado) |
| 4 | **IMC** (con estabilización interna) | `sim_imc` | outer IIR sobre lazo LQG |
| 5 | **PID fraccionario** (Oustaloup) | `sim_fopid` | `balance_dinamico.ino` (cambiar NUM/DEN) |

## Notas técnicas
- La planta es **inestable y MIMO**; IMC/H∞ (que en el curso se usan para
  plantas estables SISO) se adaptan: H∞ sobre el subsistema de ángulo; IMC
  sobre el lazo **ya estabilizado** (cero de fase no mínima en +6.97 factorizado).
- LQG/MPC → ganancia de estado (van directo al Arduino).
- H∞/IMC/FOPID → controlador **dinámico** → se discretiza (Tustin, 10 ms) y se
  corre como **ecuación en diferencias** en `balance_dinamico.ino`.
- FOPID orden 9 (Oustaloup N=4): para hardware conviene bajar a N=2 (biquads).

## Uso
- Python:  `python3 python/sim_lqg.py`  (idem mpc/hinf/imc/fopid)
- MATLAB:  `>> sim_lqg`  (necesita Control System / Robust Control Toolbox)
- Datos:   `python3 python/logger.py --port /dev/ttyUSB0 --seg 20`  ó  `logger("/dev/ttyUSB0",20)`
