# Bloque de telemetría UNIFORME (pegar en cada firmware)

Para que **un solo** `extraer_datos.py` sirva a los 6 controladores, cada
firmware debe imprimir EXACTALMENTE el mismo CSV al pulsar la tecla `t`.

## 1) Variables globales (junto a las demás)
```cpp
bool  telem=false; unsigned long t0_telem=0; int telem_div=0;
const int TELEM_CADA=2;              // 100Hz/2 = 50 Hz de muestreo
```
## 2) En el `switch` de teclas serial
```cpp
case 't': telem=!telem;
  if(telem){ t0_telem=millis();
    Serial.print("# CONTROLADOR="); Serial.print("lqr");   // <-- cambia el slug por firmware
    Serial.println(" dt=0.01");
    Serial.println("t_ms,theta_deg,theta_dot_dps,x_m,x_dot_ms,u_pwm,setpoint_deg,modo"); }
  else Serial.println("# fin"); break;
```
## 3) Al final del `loop()` (después de calcular pwm)
```cpp
if(telem && ++telem_div>=TELEM_CADA){ telem_div=0;
  Serial.print(millis()-t0_telem); Serial.print(',');
  Serial.print(theta_deg,2);   Serial.print(',');   // ángulo respecto a setpoint [deg]
  Serial.print(theta_dot,1);   Serial.print(',');   // vel. angular [deg/s]
  Serial.print(x,4);           Serial.print(',');   // posición [m]
  Serial.print(x_dot,3);       Serial.print(',');   // vel. lineal [m/s]
  Serial.print(pwm);           Serial.print(',');   // control aplicado [PWM]
  Serial.print(setpoint,2);    Serial.print(',');   // punto de operación
  Serial.println(usePos?1:0);                       // modo (0=solo ángulo,1=+posición)
}
```

## ¿Por qué esta modificación? (justificación)
- **Extraer TODA la data relevante** en un formato único → una sola herramienta
  de extracción/graficado sirve para los 6 (modular y uniforme = ahorra tiempo).
- **50 Hz** (no 100): a 115200 baudios, 100 Hz saturaría el puerto; 50 Hz captura
  bien la dinámica (oscilación ~1-2 Hz) sin perder muestras.
- Incluye **θ, θ̇, x, ẋ, u, setpoint, modo**: permite calcular todas las métricas
  (RMS, saturación, deriva) y comparar con la simulación **sin re-flashear**.
- La cabecera `#` identifica el controlador → trazabilidad en el CSV.
