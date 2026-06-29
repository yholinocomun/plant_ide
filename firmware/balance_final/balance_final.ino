/***
  ============================================================
  BALANCE FINAL — Robot péndulo invertido sobre dos ruedas
  ESP32-S3-N16R8 · SIN librerías externas
  ============================================================

  Mejoras clave respecto a versiones anteriores:
    1) CALIBRACIÓN del bias del giroscopio al arrancar (2 s quieto).
       Sin esto el ángulo deriva y el robot nunca se equilibra.
    2) SINTONIZACIÓN EN VIVO por serial (no necesitas reflashear):
         p / P  -> baja / sube  Kp  (ganancia del ángulo)
         d / D  -> baja / sube  Kd  (ganancia de la velocidad)
         w / x  -> sube / baja  el setpoint (punto de equilibrio)
         i      -> invierte el sentido del control
         space  -> activa / desactiva los motores
         z      -> pone el setpoint en el ángulo actual (auto-trim)
         f      -> imprime las ganancias actuales
    3) Compensación de ZONA MUERTA (U_DEAD = 30).
    4) Seguridad no trabante (se re-activa al enderezar).

  Control (PD sobre el ángulo, equivalente al LQR de ángulo):
    u = Kp*(theta - setpoint) + Kd*theta_dot     [+ compensación]

  PROCEDIMIENTO:
    1) Enciende con el robot QUIETO y vertical -> calibra el giro 2 s.
    2) Sujétalo, abre monitor serial (115200, "Sin ajuste de línea").
    3) Pon el robot en su punto de equilibrio y pulsa 'z' (auto-trim).
    4) Pulsa 'space' para activar.
    5) Ajusta con p/P y d/D hasta que se sostenga.
============================================================
***/

#include <Arduino.h>
#include <Wire.h>
#include <math.h>

// ---------------- PINES MOTOR ----------------
const int enableAPin = 13, motorAPin1 = 12, motorAPin2 = 14;
const int enableBPin = 9,  motorBPin1 = 10, motorBPin2 = 11;

// ---------------- ENCODERS ----------------
const int encoderPinIzqA = 4, encoderPinIzqB = 5;
const int encoderPinDerA = 6, encoderPinDerB = 7;

// ---------------- I2C MPU6050 ----------------
const int SDA_PIN = 21, SCL_PIN = 47;
const int MPU_ADDR = 0x68;

// ---------------- PWM ----------------
const int freq = 30000, resolution = 8;
const int pwmChannelA = 0, pwmChannelB = 1;

// ---------------- TIEMPO ----------------
const float dt = 0.010;
const unsigned long DT_US = 10000;

// ---------------- GANANCIAS INICIALES (ajustables en vivo) ----------------
// Trabajamos en GRADOS para que los números sean intuitivos al sintonizar.
//   u = Kp*error_grados + Kd*vel_grados_por_seg
float Kp = 12.0;    // empuje por grado de inclinación
float Kd = 0.6;     // amortiguamiento por velocidad angular
float setpoint = 0.0;        // [grados] punto de equilibrio
float inv = 1.0;             // sentido del control (+1 / -1)

// ---------------- SEGURIDAD / ZONA MUERTA ----------------
const float ANGULO_CAIDA = 35.0;   // grados
const int   U_DEAD = 30;           // zona muerta medida [PWM]
const int   PWM_MAX = 255;

// ---------------- ESTADO ----------------
volatile long encIzq = 0, encDer = 0;
float theta = 0.0;           // ángulo filtrado [grados]
float gyro_bias = 0.0;       // bias del giroscopio [°/s]
unsigned long t_prev_us = 0, t0_ms = 0;
bool motores_on = false;     // arranca DESACTIVADO (seguridad)

// ---------------- PROTOTIPOS ----------------
void initMPU();
void calibrarGiroscopio();
void leerMPU(float &ang_acc_deg, float &gyro_dps);
void moverMotores(int potencia);
int  compensarZonaMuerta(int u);
void procesarSerial();
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
  Wire.setClock(400000);     // I2C rápido para no meter jitter
  initMPU();

  Serial.println("# === BALANCE FINAL ===");
  Serial.println("# Manten el robot QUIETO y vertical: calibrando giroscopio...");
  calibrarGiroscopio();

  // Inicializa el ángulo con el acelerómetro
  float ang_acc, g;
  leerMPU(ang_acc, g);
  theta = ang_acc;

  Serial.println("# Listo. Comandos:");
  Serial.println("#  space=on/off  z=auto-trim  i=invertir");
  Serial.println("#  p/P=Kp-/+   d/D=Kd-/+   w/x=setpoint+/-   f=ver ganancias");
  Serial.println("# Pon el robot en equilibrio, pulsa 'z', luego 'space'.");

  t_prev_us = micros();
  t0_ms = millis();
}

// ============================================================
void loop() {
  procesarSerial();

  if (micros() - t_prev_us >= DT_US) {
    t_prev_us += DT_US;

    // --- Sensor ---
    float ang_acc, gyro_dps;
    leerMPU(ang_acc, gyro_dps);
    gyro_dps -= gyro_bias;                 // quita el bias calibrado

    // Filtro complementario (ángulo en grados)
    theta = 0.98f * (theta + gyro_dps * dt) + 0.02f * ang_acc;

    // --- Control PD sobre el ángulo ---
    float error = theta - setpoint;
    float u = inv * (Kp * error + Kd * gyro_dps);

    int u_pwm = constrain((int)u, -PWM_MAX, PWM_MAX);
    if (u_pwm != 0) u_pwm = compensarZonaMuerta(u_pwm);

    // --- Seguridad no trabante ---
    bool caido = (fabs(error) > ANGULO_CAIDA);

    if (motores_on && !caido) moverMotores(u_pwm);
    else                      { moverMotores(0); u_pwm = 0; }

    // --- Telemetría ---
    static int cnt = 0;
    if (++cnt >= 5) {     // cada 50 ms
      cnt = 0;
      Serial.print("th:");  Serial.print(theta, 2);
      Serial.print(" err:"); Serial.print(error, 2);
      Serial.print(" u:");  Serial.print(u_pwm);
      Serial.print(" Kp:"); Serial.print(Kp, 1);
      Serial.print(" Kd:"); Serial.print(Kd, 2);
      Serial.print(" sp:"); Serial.print(setpoint, 2);
      Serial.print(motores_on ? " [ON]" : " [OFF]");
      Serial.println();
    }
  }
}

// ============================================================
void procesarSerial() {
  while (Serial.available() > 0) {
    char c = (char)Serial.read();
    switch (c) {
      case ' ': motores_on = !motores_on;
                Serial.println(motores_on ? "# MOTORES ON" : "# MOTORES OFF");
                break;
      case 'z': setpoint = theta;
                Serial.print("# auto-trim: setpoint = "); Serial.println(setpoint, 2);
                break;
      case 'i': inv = -inv; Serial.print("# inv = "); Serial.println(inv); break;
      case 'P': Kp += 1.0;  Serial.print("# Kp="); Serial.println(Kp,1); break;
      case 'p': Kp -= 1.0;  if (Kp<0) Kp=0; Serial.print("# Kp="); Serial.println(Kp,1); break;
      case 'D': Kd += 0.1;  Serial.print("# Kd="); Serial.println(Kd,2); break;
      case 'd': Kd -= 0.1;  if (Kd<0) Kd=0; Serial.print("# Kd="); Serial.println(Kd,2); break;
      case 'w': setpoint += 0.2; Serial.print("# sp="); Serial.println(setpoint,2); break;
      case 'x': setpoint -= 0.2; Serial.print("# sp="); Serial.println(setpoint,2); break;
      case 'f': Serial.print("# Kp="); Serial.print(Kp,1);
                Serial.print(" Kd="); Serial.print(Kd,2);
                Serial.print(" sp="); Serial.print(setpoint,2);
                Serial.print(" inv="); Serial.println(inv); break;
      default: break;
    }
  }
}

// ============================================================
void calibrarGiroscopio() {
  // Promedia 400 muestras (≈2 s) con el robot quieto
  const int N = 400;
  float suma = 0.0;
  for (int i = 0; i < N; i++) {
    float ang_acc, g;
    leerMPU(ang_acc, g);
    suma += g;
    delay(5);
  }
  gyro_bias = suma / N;
  Serial.print("# gyro_bias = "); Serial.print(gyro_bias, 4); Serial.println(" deg/s");
}

void initMPU() {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6B); Wire.write(0);
  Wire.endTransmission(true);
  delay(100);
}

void leerMPU(float &ang_acc_deg, float &gyro_dps) {
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
  ang_acc_deg = atan2f((float)ax, (float)az) * 180.0f / (float)M_PI;
  gyro_dps    = (float)gy / 131.0f;
}

// ============================================================
int compensarZonaMuerta(int u) {
  if (u > 0)      return constrain(u + U_DEAD, 0,  PWM_MAX);
  else if (u < 0) return constrain(u - U_DEAD, -PWM_MAX, 0);
  return 0;
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
