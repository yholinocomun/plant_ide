# Comparativa sim vs hardware

| Controlador | θRMS sim [°] | θRMS HW [°] | |u|max sim | |u|max HW | estable sim | sat HW % |
|---|---|---|---|---|---|---|
| LQR (pre-gain) | 0.522 | 0.234 | 255.0 | 83.0 | sí | 0.0 |
| LQG (Kalman) | 0.788 | 0.728 | 173.2 | 26.0 | sí | 0.0 |
| PID Fraccionario | 21.236 | 0.356 | 255.0 | 53.0 | no | 0.0 |
| LQR predictivo (MPC) | 0.835 | — | 123.4 | — | sí | — |
| IMC (filtro Q) | 0.649 | 0.326 | 117.8 | 43.0 | sí | 0.0 |
| H-infinito | 23.123 | 0.297 | 255.0 | 61.0 | no | 0.0 |
| Cascada (colocacion polos) | 0.869 | 0.189 | 81.9 | 32.0 | sí | 0.0 |
