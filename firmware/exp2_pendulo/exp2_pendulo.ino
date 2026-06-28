/***
  ============================================================
  FASE 2 — OSCILACION LIBRE DEL PENDULO  (inercia I_p)
  Péndulo invertido sobre dos ruedas - ESP32-S3-N16R8
  ============================================================

  OBJETIVO:
    Medir el periodo natural T del cuerpo oscilando como pendulo
    fisico de eje fijo, para obtener:
      I_p = M*g*l*(T/2*PI)^2

  MONTAJE:
    Eje de ruedas FIJO en una prensa/soporte (horizontal).
    El cuerpo cuelga y oscila libre en el plano vertical.
    MOTORES APAGADOS. Ruedas quitadas o libres.
    Todo el peso real montado (bateria, ESP32, drivers).

  PROCEDIMIENTO:
    1. Envia 'S' para empezar a capturar.
    2. Desplaza el cuerpo 5-8 grados desde el reposo y SUELTA (sin empujar).
    3. Deja oscilar varios segundos (10-20 ciclos).
    4. Envia 'T' para parar.
    5. Procesa offline: extrae T de los cruces por cero de gy.

  IMPORTANTE:
    Se usa el GIROSCOPIO (gy), NO el acelerometro, porque el MPU
    esta lejos del eje y el accel se contamina con r*theta_ddot.
    gy (velocidad angular) es identica en todo el cuerpo rigido.

  SALIDA CSV:
    t_ms,gy_dps,ang_acc_deg
============================================================
***/

#include <Arduino.h>
#include <Wire.h>
#include <math.h>

const int SDA_PIN = 21;
const int SCL_PIN = 47;
const int MPU_ADDR = 0x68;

const float Ts = 0.010;
const unsigned long Ts_us = 10000;

unsigned long t_anterior_us = 0;
unsigned long t_inicio_ms   = 0;
bool capturando = false;

void initMPU();
void leerSensor(float &ang_acc_deg, float &gy_dps);

// ============================================================
void setup() {
  Serial.begin(115200);
  Wire.begin(SDA_PIN, SCL_PIN);
  initMPU();
  t_anterior_us = micros();

  Serial.println("# FASE 2 - OSCILACION LIBRE DEL PENDULO");
  Serial.println("# Comandos: S=start  T=stop");
  Serial.println("# Suelta el cuerpo desde 5-8 grados sin empujar");
  Serial.println("t_ms,gy_dps,ang_acc_deg");
}

// ============================================================
void loop() {
  while (Serial.available() > 0) {
    char c = (char)Serial.read();
    if (c == 'S') { capturando = true; t_inicio_ms = millis(); Serial.println("# >>> CAPTURANDO"); }
    else if (c == 'T') { capturando = false; Serial.println("# STOP"); }
  }

  if (micros() - t_anterior_us >= Ts_us) {
    t_anterior_us += Ts_us;

    float gy = 0.0, ang_acc = 0.0;
    leerSensor(ang_acc, gy);

    if (capturando) {
      Serial.print(millis() - t_inicio_ms); Serial.print(",");
      Serial.print(gy, 4);                  Serial.print(",");
      Serial.println(ang_acc, 4);
    }
  }
}

// ============================================================
void initMPU() {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6B);
  Wire.write(0);
  Wire.endTransmission(true);
  delay(100);
}

void leerSensor(float &angulo_acelerometro, float &gy_dps) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, 14, true);

  int16_t ax = Wire.read() << 8 | Wire.read();
  int16_t ay = Wire.read() << 8 | Wire.read();
  int16_t az = Wire.read() << 8 | Wire.read();
  /* temp */    Wire.read(); Wire.read();
  /* gx  */    Wire.read(); Wire.read();
  int16_t gy = Wire.read() << 8 | Wire.read();
  /* gz  */    Wire.read(); Wire.read();

  // Solo referencia del cero en reposo (NO usar en dinamica)
  angulo_acelerometro = atan2f((float)ax, (float)az) * 180.0f / M_PI;
  // Variable real para extraer el periodo
  gy_dps = (float)gy / 131.0f;
}
