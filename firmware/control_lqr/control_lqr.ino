/***
  ============================================================
  CONTROL LQR — Robot balancín sobre dos ruedas
  ESP32-S3-N16R8  ·  SIN librerías externas
  ============================================================

  Estado:  X = [x, ẋ, θ, θ̇]   (x en metros, θ en radianes)
  Ley:     u_pwm = -(k1·x + k2·ẋ + k3·θ + k4·θ̇)

  Ganancias LQR (de diseno_lqr.py):
    k1=-1.00  k2=-6.55  k3=-20.79  k4=-4.53

  Sensores:
    MPU6050 (I2C directo) -> θ, θ̇  (filtro complementario)
    Encoders (interrupción) -> x, ẋ

  Comandos serial (115200):
    'S' -> habilita control (motores activos)
    'T' -> detiene (motores apagados)
    En reposo manda el robot DERECHO antes de enviar 'S'.

  IMPORTANTE (ajustes que probablemente debas tocar):
    - SETPOINT_THETA: el ángulo de equilibrio real (calibración física)
    - Si el robot acelera hacia donde cae (en vez de corregir),
      invierte el signo global de 'u' (ver INVERTIR_CONTROL)
    - U_DEAD: zona muerta medida = 30
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
const float PPR = 1945.0;            // pulsos por vuelta de rueda
const float R_RUEDA = 0.037;         // radio de rueda [m]
const float M_POR_PULSO = (2.0 * M_PI * R_RUEDA) / PPR;   // metros por pulso

// ---------------- TIEMPO ----------------
const float dt = 0.010;              // 10 ms (100 Hz)
const unsigned long DT_US = 10000;

// ---------------- GANANCIAS LQR ----------------
// (theta en rad, x en m)  u = -(k1*x + k2*xd + k3*th + k4*thd)
const float k1 = -1.0000;   // x
const float k2 = -6.5460;   // x_dot
const float k3 = -20.7926;  // theta
const float k4 = -4.5256;   // theta_dot

// Si el robot empeora la caída en vez de corregir, pon -1.0 aquí
const float INVERTIR_CONTROL = 1.0;

// ---------------- CALIBRACIÓN / SEGURIDAD ----------------
float SETPOINT_THETA   = 0.0;        // ángulo de equilibrio [rad] (ajustar)
const float ANGULO_CAIDA = 0.6;      // ~34° -> apaga por seguridad [rad]
const int   U_DEAD = 30;             // zona muerta medida [PWM]
const int   PWM_MAX = 255;

// ---------------- ESTADO ----------------
volatile long encIzq = 0, encDer = 0;
long enc_prev = 0;

float theta = 0.0, theta_dot = 0.0;  // [rad], [rad/s]
float x = 0.0, x_dot = 0.0;          // [m], [m/s]
float theta_prev = 0.0;

unsigned long t_prev_us = 0;
bool control_on = false;

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
  Serial.println("# CONTROL LQR listo. Pon el robot DERECHO y envia 'S'.");
}

// ============================================================
void loop() {
  // --- comandos ---
  while (Serial.available() > 0) {
    char c = (char)Serial.read();
    if (c == 'S') {
      control_on = true;
      // reset de estado al activar
      encIzq = 0; encDer = 0; enc_prev = 0;
      x = 0; x_dot = 0;
      Serial.println("# CONTROL ON");
    } else if (c == 'T') {
      control_on = false;
      moverMotores(0);
      Serial.println("# CONTROL OFF");
    }
  }

  // --- lazo a 100 Hz ---
  if (micros() - t_prev_us >= DT_US) {
    t_prev_us += DT_US;

    // 1) Ángulo del MPU (filtro complementario)
    float ang_acc, gyro;
    leerMPU(ang_acc, gyro);
    theta     = 0.98f * (theta + gyro * dt) + 0.02f * ang_acc;
    theta_dot = gyro;                       // velocidad angular directa del giro
    (void)theta_prev;

    // 2) Posición lineal de los encoders
    noInterrupts();
    long ei = encIzq, ed = encDer;
    interrupts();
    long enc = (ei + ed) / 2;               // promedio de ambas ruedas
    long denc = enc - enc_prev;
    enc_prev = enc;
    float dx = denc * M_POR_PULSO;
    x     += dx;
    x_dot  = dx / dt;

    // 3) Error de ángulo respecto al setpoint
    float th = theta - SETPOINT_THETA;

    // 4) Ley de control LQR
    float u = -(k1 * x + k2 * x_dot + k3 * th + k4 * theta_dot);
    u *= INVERTIR_CONTROL;

    int u_pwm = (int)u;
    u_pwm = constrain(u_pwm, -PWM_MAX, PWM_MAX);
    u_pwm = compensarZonaMuerta(u_pwm);

    // 5) Seguridad por caída
    if (fabs(th) > ANGULO_CAIDA) {
      control_on = false;
      moverMotores(0);
      Serial.println("# CAIDA -> control OFF");
    }

    // 6) Aplicar
    if (control_on) moverMotores(u_pwm);
    else            moverMotores(0);

    // 7) Telemetría (cada 100 ms)
    static int cnt = 0;
    if (++cnt >= 10) {
      cnt = 0;
      Serial.print("th_deg:");  Serial.print(th * 180.0 / M_PI, 2);
      Serial.print("\tx:");     Serial.print(x, 3);
      Serial.print("\tu:");     Serial.println(u_pwm);
    }
  }
}

// ============================================================
// Compensa la zona muerta: si u != 0, súmale U_DEAD en su sentido
int compensarZonaMuerta(int u) {
  if (u > 0)      return constrain(u + U_DEAD, 0,  PWM_MAX);
  else if (u < 0) return constrain(u - U_DEAD, -PWM_MAX, 0);
  return 0;
}

// ============================================================
void initMPU() {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6B); Wire.write(0);     // despertar
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
  /* temp */    Wire.read(); Wire.read();
  /* gx  */    Wire.read(); Wire.read();
  int16_t gy = Wire.read() << 8 | Wire.read();
  /* gz  */    Wire.read(); Wire.read();

  // Ángulo del acelerómetro [rad] (eje X-Z, igual que en la identificación)
  ang_acc_rad = atan2f((float)ax, (float)az);
  // Velocidad angular [rad/s]: gy / 131 (°/s) -> rad/s
  gyro_rps = ((float)gy / 131.0f) * (float)M_PI / 180.0f;
}

// ============================================================
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

// ============================================================
void IRAM_ATTR encIzqISR() {
  if (digitalRead(encoderPinIzqA) == digitalRead(encoderPinIzqB)) encIzq++;
  else                                                            encIzq--;
}
void IRAM_ATTR encDerISR() {
  if (digitalRead(encoderPinDerA) == digitalRead(encoderPinDerB)) encDer++;
  else                                                            encDer--;
}
