# Comparativa sim vs hardware

| Controlador | θRMS sim [°] | θRMS HW [°] | |u|max sim | |u|max HW | estable sim | sat HW % |
|---|---|---|---|---|---|---|
| LQR (pre-gain) | 0.522 | — | 255.0 | — | sí | — |
| LQG (Kalman) | 0.788 | — | 173.2 | — | sí | — |
| PID Fraccionario | 21.236 | — | 255.0 | — | no | — |
| LQR predictivo (MPC) | 0.835 | — | 123.4 | — | sí | — |
| IMC (filtro Q) | 0.649 | — | 117.8 | — | sí | — |
| H-infinito | 23.123 | — | 255.0 | — | no | — |
| Cascada (colocacion polos) | 0.869 | — | 81.9 | — | sí | — |
