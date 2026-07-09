/*  BALANCE CASCADA (colocacion de polos) - ESP32-S3 core 3.x - ESTANDAR + TELEMETRIA
    Control en cascada del angulo, CORREGIDO para NO cancelar el polo inestable:
      senal interna  w = theta_dot + alpha*theta   (= salida del subsistema P2)
      lazo externo PI (cancela el polo ESTABLE -alpha):  v = kc*(e1 + alpha*I1)
      lazo interno proporcional (estabiliza P2):         u = k2*(v - w)
    Constantes de tiempo del diseno:  T22=0.05 (inner -20),  T11=0.50 (outer -2).
    CASGAIN escala la salida (ajuste fino en HW).  Teclas: space p/P r i g f t */
#include <Arduino.h>
#include <Wire.h>
// ====== CALIBRACION FIJA (robot ya calibrado en su cero; SIN espera de 2s) ======
const float GYRO_BIAS_FIJO   = 0.461259f;   // bias fijo del gyro Y [deg/s]
const float ANGULO_CERO_FIJO = 0.162628f;   // lectura del accel en el cero fisico [deg]
const float SETPOINT_FIJO    = 0.0f;        // trim de equilibrio relativo al cero [deg]
#include <math.h>
const int enableAPin=13,motorAPin1=12,motorAPin2=14, enableBPin=9,motorBPin1=10,motorBPin2=11;
const int encIzqA=4,encIzqB=5,encDerA=6,encDerB=7; const float PPR=1945.0,R_RUEDA=0.037;
const int SDA_PIN=21,SCL_PIN=47,MPU_ADDR=0x68,freq=30000,resolution=8;
const float dt=0.010,R2D=57.29578,ANG_CAIDA=35.0; const unsigned long DT_US=10000;
const int U_DEAD=30,PWM_MIN=3,PWM_MAX=255;
// --- parametros de la cascada (diseno) ---
const float ALPHA=10.2314f, K2=-20.33f, KC=1.19f;   // interno, inner-P, outer-PI
const float I_CLAMP=2.0f;                            // anti-windup de la integral
float CASGAIN=4.0f;                                  // escala de salida (ajustable p/P)
float inv=1.0,gyroSign=-1.0; bool control_on=false;
volatile long encIzq=0,encDer=0; float ang=0,x_prev=0,I1=0;
bool telem=false; unsigned long t0t=0; int tdiv=0; const int TELEM_CADA=2;
void IRAM_ATTR isrIzq(){ if(digitalRead(encIzqB))encIzq++; else encIzq--; }
void IRAM_ATTR isrDer(){ if(digitalRead(encDerB))encDer++; else encDer--; }
void initMPU(){Wire.beginTransmission(MPU_ADDR);Wire.write(0x6B);Wire.write(0);Wire.endTransmission(true);}
void leer(float&a,float&g){Wire.beginTransmission(MPU_ADDR);Wire.write(0x3B);Wire.endTransmission(false);
 Wire.requestFrom(MPU_ADDR,14,true);int16_t ax=Wire.read()<<8|Wire.read(),ay=Wire.read()<<8|Wire.read(),az=Wire.read()<<8|Wire.read();
 Wire.read();Wire.read();Wire.read();Wire.read();int16_t gy=Wire.read()<<8|Wire.read();Wire.read();Wire.read();
 a=atan2f((float)ax,(float)az)*R2D;g=(float)gy/131.0f;}
void setMot(int en,int i1,int i2,int pwm){int m=abs(pwm);if(m<PWM_MIN){ledcWrite(en,0);return;}bool f=pwm>=0;m+=U_DEAD;if(m>PWM_MAX)m=PWM_MAX;digitalWrite(i1,f?HIGH:LOW);digitalWrite(i2,f?LOW:HIGH);ledcWrite(en,m);}
void parar(){ledcWrite(enableAPin,0);ledcWrite(enableBPin,0);}
void telemHdr(const char*s){Serial.print("# CONTROLADOR=");Serial.print(s);Serial.println(" dt=0.01");
 Serial.println("t_ms,theta_deg,theta_dot_dps,x_m,x_dot_ms,u_pwm,setpoint_deg,modo");}
void setup(){Serial.begin(115200);delay(400);
 pinMode(motorAPin1,OUTPUT);pinMode(motorAPin2,OUTPUT);pinMode(motorBPin1,OUTPUT);pinMode(motorBPin2,OUTPUT);
 ledcAttach(enableAPin,freq,resolution);ledcAttach(enableBPin,freq,resolution);
 pinMode(encIzqA,INPUT_PULLUP);pinMode(encIzqB,INPUT_PULLUP);pinMode(encDerA,INPUT_PULLUP);pinMode(encDerB,INPUT_PULLUP);
 attachInterrupt(digitalPinToInterrupt(encIzqA),isrIzq,RISING);attachInterrupt(digitalPinToInterrupt(encDerA),isrDer,RISING);
 Wire.begin(SDA_PIN,SCL_PIN);Wire.setClock(400000);initMPU();delay(100);{float a0,g0;leer(a0,g0);ang=a0;}
 Serial.println("CASCADA listo (cero fijo). space p/P r i g f t=telemetria");}
unsigned long t_ant=0;
void loop(){
 if(Serial.available()){char c=Serial.read();
  if(c==' '){control_on=!control_on;if(!control_on){parar();I1=0;}Serial.println(control_on?">>ON":">>OFF");}
  else if(c=='p'){CASGAIN-=0.5f;if(CASGAIN<0)CASGAIN=0;} else if(c=='P')CASGAIN+=0.5f;
  else if(c=='r'){I1=0;} else if(c=='i')inv=-inv; else if(c=='g')gyroSign=-gyroSign;
  else if(c=='t'){telem=!telem; if(telem){t0t=millis();telemHdr("cascada");} else Serial.println("# fin");}
  else if(c=='f'){Serial.print("CASGAIN=");Serial.print(CASGAIN,2);Serial.print(" K2=");Serial.print(K2,2);Serial.print(" KC=");Serial.println(KC,2);}
  else if(c=='z'){Serial.println("# cero y setpoint FIJOS en el codigo");} }
 unsigned long now=micros(); if(now-t_ant<DT_US)return; t_ant=now;
 float aa,gr;leer(aa,gr);float gy=gyroSign*(gr-GYRO_BIAS_FIJO); ang=0.98f*(ang+gy*dt)+0.02f*aa;
 float theta_deg=(ang-ANGULO_CERO_FIJO)-SETPOINT_FIJO, theta_dot=gy;
 float th=theta_deg/R2D, thd=theta_dot/R2D;                    // [rad]
 long enc=(encIzq+encDer)/2; float x=(enc/PPR)*2*M_PI*R_RUEDA, x_dot=(x-x_prev)/dt; x_prev=x;
 int pwm=0;
 if(fabs(theta_deg)<=ANG_CAIDA && control_on){
   float e1=-th;                                                // referencia de angulo = 0
   I1+=e1*dt; if(I1>I_CLAMP)I1=I_CLAMP; if(I1<-I_CLAMP)I1=-I_CLAMP;
   float v=KC*(e1+ALPHA*I1);                                    // lazo externo PI
   float w=thd+ALPHA*th;                                        // senal interna (P2)
   float u=inv*CASGAIN*(K2*(v-w));                              // lazo interno
   pwm=(int)constrain(u,-PWM_MAX,PWM_MAX);
   setMot(enableAPin,motorAPin1,motorAPin2,pwm); setMot(enableBPin,motorBPin1,motorBPin2,pwm);
 } else {parar(); I1=0;}
 if(telem && ++tdiv>=TELEM_CADA){tdiv=0;
   Serial.print(millis()-t0t);Serial.print(',');Serial.print(theta_deg,2);Serial.print(',');
   Serial.print(theta_dot,1);Serial.print(',');Serial.print(x,4);Serial.print(',');
   Serial.print(x_dot,3);Serial.print(',');Serial.print(pwm);Serial.print(',');
   Serial.print(SETPOINT_FIJO,2);Serial.print(',');Serial.println(0);}
}
