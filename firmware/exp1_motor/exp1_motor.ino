/***
  ============================================================
  FASE 1 — IDENTIFICACIÓN DEL MOTOR  (JGA25-370B + GM25-13CPR)
  Péndulo invertido sobre dos ruedas - ESP32-S3-N16R8
  ============================================================

  OBJETIVO:
    Identificar G(s) = omega_rueda(s)/PWM(s) = K/(tau*s + 1)
    + zona muerta (u_dead) + asimetria adelante/atras

  MONTAJE:
    Robot apoyado en el suelo (ruedas rodando) o rueda al aire.
    Cuerpo sujeto con la mano. MOTORES son la entrada, ENCODER la salida.
    MPU APAGADO (no se usa, para no meter jitter en el Ts).

  COMANDOS SERIAL (115200 baud):
    'A'  -> Sub-experimento A: Rampa lenta (zona muerta)
    'B'  -> Sub-experimento B: Escalera multinivel (ganancia K)
    'C'  -> Sub-experimento C: Escalon aislado PWM=120 (constante tau)
    'M<v>' -> PWM manual (ej. M120, M-80) para pruebas
    'T'  -> Stop (PWM=0)

  SALIDA CSV:
    t_ms,pwm,enc_izq,enc_der

  PROCESA OFFLINE en Python (analizar_motor.py):
    omega = (delta_pulsos / Ts) * (2*PI / PPR)
============================================================
***/

#include <Arduino.h>

// --- PINES MOTOR A (Izquierdo) ---
const int enableAPin = 13;
const int motorAPin1 = 12;
const int motorAPin2 = 14;

// --- PINES MOTOR B (Derecho) ---
const int enableBPin = 9;
const int motorBPin1 = 10;
const int motorBPin2 = 11;

// --- ENCODERS ---
const int encoderPinIzqA = 4;
const int encoderPinIzqB = 5;
const int encoderPinDerA = 6;
const int encoderPinDerB = 7;

// --- PWM ---
const int freq = 30000;
const int resolution = 8;
const int pwmChannelA = 0;
const int pwmChannelB = 1;

// --- PARAMETROS DEL ENCODER (tu hardware) ---
// 13 CPR * 2 (CHANGE 1 canal) * 74.8 reductor = 1945 PPR
const float PPR = 1945.0;
const float Ts  = 0.010;          // 10 ms
const unsigned long Ts_us = 10000;

// --- VARIABLES ENCODER ---
volatile long contadorEncoderIzq = 0;
volatile long contadorEncoderDer = 0;

// --- ESTADO DE EXPERIMENTO ---
enum Modo { DETENIDO, RAMPA, ESCALERA, ESCALON, MANUAL };
Modo modo = DETENIDO;

int  pwm_actual = 0;
unsigned long t_anterior_us = 0;
unsigned long t_inicio_ms   = 0;
bool capturando = false;

// --- RAMPA (Sub-experimento A) ---
int  rampa_pwm = 0;
unsigned long rampa_timer = 0;
const unsigned long RAMPA_PASO_MS = 1500;  // sube cada 1.5 s
const int RAMPA_INCREMENTO = 5;            // de 5 en 5
const int RAMPA_MAX = 120;

// --- ESCALERA (Sub-experimento B) ---
const int  escalera_niveles[] = {80, 120, 160, 200, 0, -80, -120, -160, 0};
const int  N_ESCALERA = 9;
int        escalera_idx = 0;
unsigned long escalera_timer = 0;
const unsigned long ESCALERA_DUR_MS = 3000; // 3 s por nivel

// --- ESCALON (Sub-experimento C) ---
const int  ESCALON_PWM = 120;
unsigned long escalon_timer = 0;
const unsigned long ESCALON_DUR_MS = 2000;  // 2 s

// --- PROTOTIPOS ---
void moverMotores(int potencia);
void IRAM_ATTR encoderIzqISR();
void IRAM_ATTR encoderDerISR();

// ============================================================
void setup() {
  Serial.begin(115200);

  pinMode(motorAPin1, OUTPUT); pinMode(motorAPin2, OUTPUT);
  pinMode(motorBPin1, OUTPUT); pinMode(motorBPin2, OUTPUT);
  ledcAttachChannel(enableAPin, freq, resolution, pwmChannelA);
  ledcAttachChannel(enableBPin, freq, resolution, pwmChannelB);
  moverMotores(0);

  pinMode(encoderPinIzqA, INPUT_PULLUP);
  pinMode(encoderPinIzqB, INPUT_PULLUP);
  pinMode(encoderPinDerA, INPUT_PULLUP);
  pinMode(encoderPinDerB, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(encoderPinIzqA), encoderIzqISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoderPinDerA), encoderDerISR, CHANGE);

  t_anterior_us = micros();

  Serial.println("# FASE 1 - IDENTIFICACION DEL MOTOR");
  Serial.println("# Comandos: A=rampa  B=escalera  C=escalon  M<v>=manual  T=stop");
  Serial.println("t_ms,pwm,enc_izq,enc_der");
}

// ============================================================
void loop() {
  // --- Leer comandos ---
  while (Serial.available() > 0) {
    char c = (char)Serial.read();
    if (c == 'M') {
      pwm_actual  = constrain((int)Serial.parseInt(), -255, 255);
      modo        = MANUAL;
      capturando  = true;
      t_inicio_ms = millis();
    } else if (c == 'A') {
      modo = RAMPA; rampa_pwm = 0; rampa_timer = millis();
      capturando = true; t_inicio_ms = millis();
      contadorEncoderIzq = 0; contadorEncoderDer = 0;
      Serial.println("# >>> RAMPA (zona muerta)");
    } else if (c == 'B') {
      modo = ESCALERA; escalera_idx = 0; escalera_timer = millis();
      pwm_actual = escalera_niveles[0];
      capturando = true; t_inicio_ms = millis();
      contadorEncoderIzq = 0; contadorEncoderDer = 0;
      Serial.println("# >>> ESCALERA (ganancia K)");
    } else if (c == 'C') {
      modo = ESCALON; escalon_timer = millis();
      pwm_actual = 0;
      capturando = true; t_inicio_ms = millis();
      contadorEncoderIzq = 0; contadorEncoderDer = 0;
      Serial.println("# >>> ESCALON PWM=120 (constante tau)");
    } else if (c == 'T') {
      modo = DETENIDO; pwm_actual = 0; capturando = false;
      moverMotores(0);
      Serial.println("# STOP");
    }
  }

  // --- Lazo a 100 Hz ---
  if (micros() - t_anterior_us >= Ts_us) {
    t_anterior_us += Ts_us;
    unsigned long ahora = millis();

    // Generar la entrada segun el modo
    switch (modo) {
      case RAMPA:
        if (ahora - rampa_timer >= RAMPA_PASO_MS) {
          rampa_timer = ahora;
          rampa_pwm  += RAMPA_INCREMENTO;
          if (rampa_pwm > RAMPA_MAX) { modo = DETENIDO; rampa_pwm = 0; capturando = false; Serial.println("# FIN RAMPA"); }
        }
        pwm_actual = rampa_pwm;
        break;

      case ESCALERA:
        if (ahora - escalera_timer >= ESCALERA_DUR_MS) {
          escalera_timer = ahora;
          escalera_idx++;
          if (escalera_idx >= N_ESCALERA) { modo = DETENIDO; pwm_actual = 0; capturando = false; Serial.println("# FIN ESCALERA"); }
          else pwm_actual = escalera_niveles[escalera_idx];
        }
        break;

      case ESCALON:
        if (ahora - escalon_timer < 200)            pwm_actual = 0;             // 0.2s en reposo
        else if (ahora - escalon_timer < 200 + ESCALON_DUR_MS) pwm_actual = ESCALON_PWM;
        else { modo = DETENIDO; pwm_actual = 0; capturando = false; Serial.println("# FIN ESCALON"); }
        break;

      default: break;
    }

    moverMotores(pwm_actual);

    // Volcar datos
    if (capturando) {
      noInterrupts();
      long ei = contadorEncoderIzq;
      long ed = contadorEncoderDer;
      interrupts();

      Serial.print(ahora - t_inicio_ms); Serial.print(",");
      Serial.print(pwm_actual);          Serial.print(",");
      Serial.print(ei);                  Serial.print(",");
      Serial.println(ed);
    }
  }
}

// ============================================================
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

// ============================================================
void IRAM_ATTR encoderIzqISR() {
  if (digitalRead(encoderPinIzqA) == digitalRead(encoderPinIzqB)) contadorEncoderIzq++;
  else                                                            contadorEncoderIzq--;
}
void IRAM_ATTR encoderDerISR() {
  if (digitalRead(encoderPinDerA) == digitalRead(encoderPinDerB)) contadorEncoderDer++;
  else                                                            contadorEncoderDer--;
}
