/***
  Robot Balancín - Modo Identificación de Planta
  ESP32-S3-N16R8

  Envía por Serial (115200 baud) en formato CSV:
    t_ms,angulo,vel_angular,pos_x,vel_x,pwm_output

  Comandos recibidos:
    'S' -> Start (activa envío de datos + señal de excitación)
    'T' -> Stop
    'M<valor>' -> Setear PWM manual (-255 a 255)
    'P' -> Señal PRBS automática
    'E' -> Señal escalón automática
***/

#include <Arduino.h>
#include <Wire.h>
#include <math.h>

// --- PINES MOTOR A (Izquierdo) ---
const int enableAPin = 13;
const int motorAPin1 = 12;
const int motorAPin2 = 14;

// --- PINES MOTOR B (Derecho) ---
const int enableBPin  = 9;
const int motorBPin1  = 10;
const int motorBPin2  = 11;

// --- I2C MPU6050 ---
const int SDA_PIN = 21;
const int SCL_PIN = 47;
const int MPU_ADDR = 0x68;

// --- ENCODERS ---
const int encoderPinIzqA = 4;
const int encoderPinIzqB = 5;
const int encoderPinDerA = 6;
const int encoderPinDerB = 7;

// --- PWM ---
const int freq       = 30000;
const int resolution = 8;
const int pwmChannelA = 0;
const int pwmChannelB = 1;

// --- PARÁMETROS FÍSICOS ---
// Resolución encoder: pulsos por vuelta (cuadratura x4)
// Ajustar según tu encoder real
const float PULSOS_POR_VUELTA = 1320.0;
// Radio de la rueda en metros (ajustar)
const float RADIO_RUEDA_M = 0.034;
// Metros por pulso
const float METROS_POR_PULSO = (2.0 * M_PI * RADIO_RUEDA_M) / PULSOS_POR_VUELTA;

// --- TIEMPO DE MUESTREO ---
const float dt = 0.010;             // 10 ms
const unsigned long DT_US = 10000;

// --- VARIABLES ESTADO ---
volatile long contadorEncoderIzq = 0;
volatile long contadorEncoderDer = 0;

float angulo_actual   = 0.0;  // theta [grados]
float vel_angular     = 0.0;  // theta_dot [grados/s]
float pos_x           = 0.0;  // posición [m]
float vel_x           = 0.0;  // velocidad lineal [m/s]

float angulo_previo   = 0.0;
long  encoder_prev_izq = 0;
long  encoder_prev_der = 0;

int   pwm_output = 0;

// --- MODOS DE OPERACIÓN ---
enum Modo { DETENIDO, MANUAL, PRBS, ESCALON };
Modo modo_actual = DETENIDO;

// --- PRBS (Pseudo-Random Binary Sequence) ---
uint32_t prbs_reg    = 0xACE1u;
int      prbs_pwm    = 80;       // amplitud PWM para PRBS
unsigned long prbs_timer = 0;
const unsigned long PRBS_PERIODO_MS = 50; // cambio cada 50 ms

// --- ESCALÓN ---
int   escalon_pwm    = 100;
unsigned long escalon_timer = 0;
const unsigned long ESCALON_DURACION_MS = 2000; // 2 s adelante, 2 s atrás
bool  escalon_fase = false;

// --- SEGURIDAD ---
const float ANGULO_CAIDA = 40.0;

unsigned long t_anterior_us = 0;
unsigned long t_inicio_ms   = 0;
bool iniciado = false;

// --- PROTOTIPOS ---
void initMPU();
void leerSensor(float &ang_acc, float &vel_gyr);
void moverMotores(int potencia);
void IRAM_ATTR encoderIzqISR();
void IRAM_ATTR encoderDerISR();
int  prbs_step();
void procesarComando(char cmd);

// ======================================================================== //
void setup() {
  Serial.begin(115200);

  // Motores
  pinMode(motorAPin1, OUTPUT); pinMode(motorAPin2, OUTPUT);
  pinMode(motorBPin1, OUTPUT); pinMode(motorBPin2, OUTPUT);
  ledcAttachChannel(enableAPin, freq, resolution, pwmChannelA);
  ledcAttachChannel(enableBPin, freq, resolution, pwmChannelB);
  moverMotores(0);

  // Encoders
  pinMode(encoderPinIzqA, INPUT_PULLUP);
  pinMode(encoderPinIzqB, INPUT_PULLUP);
  pinMode(encoderPinDerA, INPUT_PULLUP);
  pinMode(encoderPinDerB, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(encoderPinIzqA), encoderIzqISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoderPinDerA), encoderDerISR, CHANGE);

  // IMU
  Wire.begin(SDA_PIN, SCL_PIN);
  initMPU();

  t_anterior_us = micros();

  // Cabecera CSV para el script de Python
  Serial.println("t_ms,angulo_deg,vel_angular_dps,pos_x_m,vel_x_ms,pwm");
}

// ======================================================================== //
void loop() {
  // Leer comandos Serial
  while (Serial.available() > 0) {
    char c = (char)Serial.read();
    if (c == 'M') {
      int val = Serial.parseInt();
      pwm_output = constrain(val, -255, 255);
      modo_actual = MANUAL;
    } else {
      procesarComando(c);
    }
    while (Serial.available() && Serial.peek() == '\n') Serial.read();
  }

  // Lazo de control/adquisición a 100 Hz
  if (micros() - t_anterior_us >= DT_US) {
    t_anterior_us += DT_US;

    // --- LEER IMU ---
    float ang_acc = 0.0, vel_gyr = 0.0;
    leerSensor(ang_acc, vel_gyr);

    // Filtro complementario
    angulo_actual = 0.98f * (angulo_actual + vel_gyr * dt) + 0.02f * ang_acc;
    vel_angular   = (angulo_actual - angulo_previo) / dt;
    angulo_previo = angulo_actual;

    // --- LEER ENCODERS ---
    noInterrupts();
    long enc_izq = contadorEncoderIzq;
    long enc_der = contadorEncoderDer;
    interrupts();

    long delta_izq = enc_izq - encoder_prev_izq;
    long delta_der = enc_der - encoder_prev_der;
    encoder_prev_izq = enc_izq;
    encoder_prev_der = enc_der;

    // Promedio de ambas ruedas para posición lineal
    float delta_m = ((float)(delta_izq + delta_der) / 2.0f) * METROS_POR_PULSO;
    pos_x  += delta_m;
    vel_x   = delta_m / dt;

    // --- SEÑAL DE EXCITACIÓN ---
    if (iniciado) {
      unsigned long ahora_ms = millis();

      if (modo_actual == PRBS) {
        if (ahora_ms - prbs_timer >= PRBS_PERIODO_MS) {
          prbs_timer = ahora_ms;
          pwm_output = prbs_step() ? prbs_pwm : -prbs_pwm;
        }
      } else if (modo_actual == ESCALON) {
        if (ahora_ms - escalon_timer >= ESCALON_DURACION_MS) {
          escalon_timer = ahora_ms;
          escalon_fase  = !escalon_fase;
          pwm_output    = escalon_fase ? escalon_pwm : -escalon_pwm;
        }
      }
    }

    // --- SEGURIDAD ---
    if (abs(angulo_actual) > ANGULO_CAIDA) {
      moverMotores(0);
      pwm_output = 0;
    } else if (iniciado) {
      moverMotores(pwm_output);
    }

    // --- ENVIAR DATOS ---
    if (iniciado) {
      unsigned long t_ms = millis() - t_inicio_ms;
      Serial.print(t_ms);        Serial.print(",");
      Serial.print(angulo_actual, 4); Serial.print(",");
      Serial.print(vel_angular, 4);   Serial.print(",");
      Serial.print(pos_x, 6);        Serial.print(",");
      Serial.print(vel_x, 6);        Serial.print(",");
      Serial.println(pwm_output);
    }
  }
}

// ======================================================================== //
void procesarComando(char cmd) {
  switch (cmd) {
    case 'S':
      iniciado      = true;
      t_inicio_ms   = millis();
      prbs_timer    = millis();
      escalon_timer = millis();
      pos_x = 0.0;
      contadorEncoderIzq = 0;
      contadorEncoderDer = 0;
      Serial.println("# START");
      break;
    case 'T':
      iniciado    = false;
      modo_actual = DETENIDO;
      moverMotores(0);
      pwm_output  = 0;
      Serial.println("# STOP");
      break;
    case 'P':
      modo_actual = PRBS;
      Serial.println("# MODO PRBS");
      break;
    case 'E':
      modo_actual    = ESCALON;
      escalon_fase   = true;
      pwm_output     = escalon_pwm;
      Serial.println("# MODO ESCALON");
      break;
    default:
      break;
  }
}

// ======================================================================== //
// PRBS de 16 bits (registro de desplazamiento con retroalimentación)
int prbs_step() {
  uint32_t bit = ((prbs_reg >> 0) ^ (prbs_reg >> 2) ^ (prbs_reg >> 3) ^ (prbs_reg >> 5)) & 1;
  prbs_reg = (prbs_reg >> 1) | (bit << 15);
  return (int)(prbs_reg & 1);
}

// ======================================================================== //
void initMPU() {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6B);
  Wire.write(0);
  Wire.endTransmission(true);
  delay(100);
}

void leerSensor(float &angulo_acelerometro, float &velocidad_giroscopio) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, 14, true);

  int16_t ax   = Wire.read() << 8 | Wire.read();
  int16_t ay   = Wire.read() << 8 | Wire.read();
  int16_t az   = Wire.read() << 8 | Wire.read();
  /* temp */     Wire.read(); Wire.read();
  /* gx  */     Wire.read(); Wire.read();
  int16_t gy   = Wire.read() << 8 | Wire.read();
  /* gz  */     Wire.read(); Wire.read();

  angulo_acelerometro  = atan2f((float)ax, (float)az) * 180.0f / M_PI;
  velocidad_giroscopio = (float)gy / 131.0f;
}

// ======================================================================== //
void moverMotores(int potencia) {
  potencia = constrain(potencia, -255, 255);
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

// ======================================================================== //
void IRAM_ATTR encoderIzqISR() {
  if (digitalRead(encoderPinIzqA) == digitalRead(encoderPinIzqB))
    contadorEncoderIzq++;
  else
    contadorEncoderIzq--;
}

void IRAM_ATTR encoderDerISR() {
  if (digitalRead(encoderPinDerA) == digitalRead(encoderPinDerB))
    contadorEncoderDer++;
  else
    contadorEncoderDer--;
}
