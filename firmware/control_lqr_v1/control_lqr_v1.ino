/***
  ============================================================
  CONTROL LQR v1 — Ganancias del DISEÑO (para validar sim vs HW)
  Robot balancín sobre dos ruedas · ESP32-S3-N16R8
  SIN librerías externas
  ============================================================

  Objetivo de este firmware:
    Usar EXACTAMENTE las ganancias que dio diseno_lqr.py con los
    pesos por defecto, para comparar el comportamiento real del
    hardware contra la simulación. NO busca el mejor balanceo;
    busca verificar que el modelo y la realidad coinciden.

  Estado:  X = [x, ẋ, θ, θ̇]   (x en metros, θ en radianes)
  Ley:     u_pwm = -(k1·x + k2·ẋ + k3·θ + k4·θ̇)

  Ganancias del diseño (diseno_lqr.py, Q=diag(1,1,100,10), R=1):
    k1=-1.0000  k2=-6.5460  k3=-20.7926  k4=-4.5256

  Los motores arrancan ACTIVOS (no necesitas enviar 'S').
    'T' -> apaga    'S' -> reactiva

  Telemetría CSV (para comparar con la simulación):
    t_ms,theta_deg,theta_dot_dps,x_m,u_pwm
============================================================
***/

#include <Arduino.h>
#include <Wire.h>
#include <math.h>

// ---------------- PINES MOTOR ----------------
const int enableAPin = 13, motorAPin1 = 12, motorAPin2 = 14;   // Motor A (izq)
const int enableBPin = 9,  motorBPin1 = 10, motorBPin2 = 11;   // Motor B (der)

// ---------------- ENCODERS ----------------
const int encoderPinIzqA = 4, encoderPinIzqB = 5;
const int encoderPinDerA = 6, encoderPinDerB = 7;

// ---------------- I2C MPU6050 ----------------
const int SDA_PIN = 21, SCL_PIN = 47;
const int MPU_ADDR = 0x68;

// ---------------- PWM ----------------
const int freq = 30000, resolution = 8;
const int pwmChannelA = 0, pwmChannelB = 1;

// ---------------- PARÁMETROS FÍSICOS ----------------
const float PPR = 1945.0;
const float R_RUEDA = 0.037;
const float M_POR_PULSO = (2.0 * M_PI * R_RUEDA) / PPR;

// ---------------- TIEMPO ----------------
const float dt = 0.010;
const unsigned long DT_US = 10000;

// ---------------- GANANCIAS LQR (DISEÑO ORIGINAL) ----------------
const float k1 = -1.0000;   // x
const float k2 = -6.5460;   // x_dot
const float k3 = -20.7926;  // theta
const float k4 = -4.5256;   // theta_dot

// Si el control empuja hacia donde cae, invierte el signo:
const float INVERTIR_CONTROL = 1.0;

// ---------------- CALIBRACIÓN / SEGURIDAD ----------------
float SETPOINT_THETA   = 0.0;        // [rad] ángulo de equilibrio real
const float ANGULO_CAIDA = 0.6;      // ~34° -> apaga por seguridad
const int   U_DEAD = 30;             // zona muerta medida [PWM]
const int   PWM_MAX = 255;
const bool  USAR_ZONA_MUERTA = true; // compensación de zona muerta

// ---------------- ESTADO ----------------
volatile long encIzq = 0, encDer = 0;
long enc_prev = 0;
float theta = 0.0, theta_dot = 0.0;
float x = 0.0, x_dot = 0.0;
unsigned long t_prev_us = 0;
unsigned long t0_ms = 0;
bool control_on = true;              // arranca ACTIVO

// ---------------- PROTOTIPOS ----------------
void initMPU();
void leerMPU(float &ang_acc_rad, float &gyro_rps);
void moverMotores(int potencia);
int  compensarZonaMuerta(int u);
void IRAM_ATTR encIzqISR();
void IRAM_ATTR encDerISR();

// ============================================================
void setup() {
  Serial.begin(115200);

  pinMode(motorAPin1, OUTPUT); pinMode(motorAPin2, OUTPUT);
  pinMode(motorBPin1, OUTPUT); pinMode(motorBPin2, OUTPUT);
  ledcAttachChannel(enableAPin, freq, resolution, pwmChannelA);
  ledcAttachChannel(enableBPin, freq, resolution, pwmChannelB);
  moverMotores(0);

  pinMode(encoderPinIzqA, INPUT_PULLUP); pinMode(encoderPinIzqB, INPUT_PULLUP);
  pinMode(encoderPinDerA, INPUT_PULLUP); pinMode(encoderPinDerB, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(encoderPinIzqA), encIzqISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoderPinDerA), encDerISR, CHANGE);

  Wire.begin(SDA_PIN, SCL_PIN);
  initMPU();

  t_prev_us = micros();
  t0_ms = millis();
  Serial.println("# CONTROL LQR v1 (ganancias del diseño). Motores ACTIVOS. 'T'=stop");
  Serial.println("t_ms,theta_deg,theta_dot_dps,x_m,u_pwm");
}

// ============================================================
void loop() {
  while (Serial.available() > 0) {
    char c = (char)Serial.read();
    if (c == 'S') {
      control_on = true;
      encIzq = 0; encDer = 0; enc_prev = 0; x = 0; x_dot = 0;
      Serial.println("# CONTROL ON");
    } else if (c == 'T') {
      control_on = false; moverMotores(0);
      Serial.println("# CONTROL OFF");
    }
  }

  if (micros() - t_prev_us >= DT_US) {
    t_prev_us += DT_US;

    // 1) Ángulo (filtro complementario)
    float ang_acc, gyro;
    leerMPU(ang_acc, gyro);
    theta     = 0.98f * (theta + gyro * dt) + 0.02f * ang_acc;
    theta_dot = gyro;

    // 2) Posición lineal (encoders)
    noInterrupts();
    long ei = encIzq, ed = encDer;
    interrupts();
    long enc = (ei + ed) / 2;
    long denc = enc - enc_prev;
    enc_prev = enc;
    float dx = denc * M_POR_PULSO;
    x += dx;
    x_dot = dx / dt;

    // 3) Ley de control LQR
    float th = theta - SETPOINT_THETA;
    float u = -(k1 * x + k2 * x_dot + k3 * th + k4 * theta_dot);
    u *= INVERTIR_CONTROL;

    int u_pwm = constrain((int)u, -PWM_MAX, PWM_MAX);
    if (USAR_ZONA_MUERTA) u_pwm = compensarZonaMuerta(u_pwm);

    // 4) Seguridad NO trabante: si pasa el ángulo de caída, apaga la
    //    salida PERO no desactiva el control; se re-activa solo al
    //    volver a poner el robot derecho (ideal para pruebas de banco).
    bool fuera_de_rango = (fabs(th) > ANGULO_CAIDA);

    // 5) Aplicar
    if (control_on && !fuera_de_rango) {
      moverMotores(u_pwm);
    } else {
      moverMotores(0);
      u_pwm = 0;
    }

    // 6) Telemetría CSV (cada 20 ms para no saturar)
    static int cnt = 0;
    if (++cnt >= 2) {
      cnt = 0;
      Serial.print(millis() - t0_ms);            Serial.print(",");
      Serial.print(th * 180.0 / M_PI, 2);        Serial.print(",");
      Serial.print(theta_dot * 180.0 / M_PI, 2); Serial.print(",");
      Serial.print(x, 4);                        Serial.print(",");
      Serial.println(u_pwm);
    }
  }
}

// ============================================================
int compensarZonaMuerta(int u) {
  if (u > 0)      return constrain(u + U_DEAD, 0,  PWM_MAX);
  else if (u < 0) return constrain(u - U_DEAD, -PWM_MAX, 0);
  return 0;
}

void initMPU() {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6B); Wire.write(0);
  Wire.endTransmission(true);
  delay(100);
}

void leerMPU(float &ang_acc_rad, float &gyro_rps) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, 14, true);
  int16_t ax = Wire.read() << 8 | Wire.read();
  int16_t ay = Wire.read() << 8 | Wire.read();
  int16_t az = Wire.read() << 8 | Wire.read();
  Wire.read(); Wire.read();   // temp
  Wire.read(); Wire.read();   // gx
  int16_t gy = Wire.read() << 8 | Wire.read();
  Wire.read(); Wire.read();   // gz
  ang_acc_rad = atan2f((float)ax, (float)az);
  gyro_rps    = ((float)gy / 131.0f) * (float)M_PI / 180.0f;
}

void moverMotores(int potencia) {
  potencia = constrain(potencia, -PWM_MAX, PWM_MAX);
  int pwm  = abs(potencia);
  if (potencia > 0) {
    digitalWrite(motorAPin1, LOW);  digitalWrite(motorAPin2, HIGH);
    digitalWrite(motorBPin1, LOW);  digitalWrite(motorBPin2, HIGH);
  } else if (potencia < 0) {
    digitalWrite(motorAPin1, HIGH); digitalWrite(motorAPin2, LOW);
    digitalWrite(motorBPin1, HIGH); digitalWrite(motorBPin2, LOW);
  } else {
    digitalWrite(motorAPin1, LOW);  digitalWrite(motorAPin2, LOW);
    digitalWrite(motorBPin1, LOW);  digitalWrite(motorBPin2, LOW);
  }
  ledcWrite(enableAPin, pwm);
  ledcWrite(enableBPin, pwm);
}

void IRAM_ATTR encIzqISR() {
  if (digitalRead(encoderPinIzqA) == digitalRead(encoderPinIzqB)) encIzq++;
  else                                                            encIzq--;
}
void IRAM_ATTR encDerISR() {
  if (digitalRead(encoderPinDerA) == digitalRead(encoderPinDerB)) encDer++;
  else                                                            encDer--;
}
