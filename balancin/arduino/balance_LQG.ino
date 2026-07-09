/*  BALANCE LQG - ESP32-S3 core 3.x - SIN librerias externas
    LQG = observador de Kalman (4 estados) + realimentacion u=-K*x_est
    Base: firmware LQR de referencia (mismos pines/MPU/encoders/teclas).
    Matrices Ad,Bd,C,L,K disenadas en balancin/python/sim_lqg.py
    Teclas: space=on/off  z=trim  o=posicion  i=inv control  g=inv giro  f=estado
*/
#include <Arduino.h>
#include <Wire.h>
#include <math.h>
// ---- PINES ----
const int enableAPin=13,motorAPin1=12,motorAPin2=14, enableBPin=9,motorBPin1=10,motorBPin2=11;
const int encIzqA=4,encIzqB=5, encDerA=6,encDerB=7;
const float PPR=1945.0, R_RUEDA=0.037;
const int SDA_PIN=21,SCL_PIN=47,MPU_ADDR=0x68, freq=30000,resolution=8;
const float dt=0.010; const unsigned long DT_US=10000;
const float ANG_CAIDA=35.0; const int U_DEAD=30,PWM_MIN=3,PWM_MAX=255;
const float R2D=57.29578;

// ---- GANANCIA LQR (u = -K*x),  x=[x,xd,theta(rad),thetad] ----
float K[4] = { -70.71, -196.97, -1985.22, -284.52 };
// ---- MODELO DISCRETO (dt=10ms) ----
const float Ad[4][4]={{1,0.01,-0.000373,0},{0,1,-0.074697,-0.000373},
                      {0,0,1.005234,0.01},{0,0,1.046806,1.005234}};
const float Bd[4]={0.000002,0.000316,-0.000012,-0.002376};
// ---- GANANCIA KALMAN L (4x2), medida y=[x,theta] ----
const float Lk[4][2]={{0.515267,-0.000429},{1.558156,-0.065012},
                      {-0.000643,0.463548},{-0.02667,2.358773}};

float inv=1.0, gyroSign=-1.0, setpoint=0.0; bool usePos=false, control_on=false;
volatile long encIzq=0,encDer=0; float gyroBias=0, ang_comp=0;
float xh[4]={0,0,0,0};              // estado estimado (observador)

void IRAM_ATTR isrIzq(){ if(digitalRead(encIzqB))encIzq++; else encIzq--; }
void IRAM_ATTR isrDer(){ if(digitalRead(encDerB))encDer++; else encDer--; }
void initMPU(){ Wire.beginTransmission(MPU_ADDR);Wire.write(0x6B);Wire.write(0);Wire.endTransmission(true);}
void leerSensor(float&ang,float&gy){
  Wire.beginTransmission(MPU_ADDR);Wire.write(0x3B);Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR,14,true);
  int16_t ax=Wire.read()<<8|Wire.read(),ay=Wire.read()<<8|Wire.read(),az=Wire.read()<<8|Wire.read();
  Wire.read();Wire.read();Wire.read();Wire.read(); int16_t g=Wire.read()<<8|Wire.read();
  Wire.read();Wire.read();
  ang=atan2f((float)ax,(float)az)*R2D; gy=(float)g/131.0f;
}
void calibrar(){ Serial.println("Calib giro 2s QUIETO"); float s=0; for(int i=0;i<200;i++){float a,g;leerSensor(a,g);s+=g;delay(10);} gyroBias=s/200; float a,g;leerSensor(a,g); ang_comp=a; }
void setMotor(int en,int i1,int i2,int pwm){ int m=abs(pwm); if(m<PWM_MIN){ledcWrite(en,0);return;} bool f=pwm>=0; m+=U_DEAD; if(m>PWM_MAX)m=PWM_MAX; digitalWrite(i1,f?HIGH:LOW);digitalWrite(i2,f?LOW:HIGH); ledcWrite(en,m);}
void parar(){ ledcWrite(enableAPin,0); ledcWrite(enableBPin,0);}

void setup(){
  Serial.begin(115200); delay(400);
  pinMode(motorAPin1,OUTPUT);pinMode(motorAPin2,OUTPUT);pinMode(motorBPin1,OUTPUT);pinMode(motorBPin2,OUTPUT);
  ledcAttach(enableAPin,freq,resolution); ledcAttach(enableBPin,freq,resolution);
  pinMode(encIzqA,INPUT_PULLUP);pinMode(encIzqB,INPUT_PULLUP);pinMode(encDerA,INPUT_PULLUP);pinMode(encDerB,INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(encIzqA),isrIzq,RISING); attachInterrupt(digitalPinToInterrupt(encDerA),isrDer,RISING);
  Wire.begin(SDA_PIN,SCL_PIN); Wire.setClock(400000); initMPU(); delay(100); calibrar();
  Serial.println("LQG listo. space=on z=trim o=pos i=inv g=giro f=estado");
}

unsigned long t_ant=0;
void loop(){
  if(Serial.available()){ char c=Serial.read();
    if(c==' '){control_on=!control_on; if(!control_on)parar(); Serial.println(control_on?">>ON":">>OFF");}
    else if(c=='z'){setpoint=ang_comp; Serial.print("setpoint=");Serial.println(setpoint,2);}
    else if(c=='o'){usePos=!usePos; Serial.println(usePos?">>POS ON":">>POS OFF");}
    else if(c=='i'){inv=-inv;} else if(c=='g'){gyroSign=-gyroSign;}
    else if(c=='f'){Serial.print("K=[");for(int i=0;i<4;i++){Serial.print(K[i]);Serial.print(i<3?",":"]");}Serial.print(" set=");Serial.println(setpoint,2);}
  }
  unsigned long now=micros(); if(now-t_ant<DT_US)return; t_ant=now;

  // --- medida ---
  float ang_acc,gy_raw; leerSensor(ang_acc,gy_raw);
  float gy=gyroSign*(gy_raw-gyroBias);
  ang_comp=0.98f*(ang_comp+gy*dt)+0.02f*ang_acc;      // complementario (ref para trim/seguridad)
  long enc=(encIzq+encDer)/2;
  float xm=(enc/PPR)*2*M_PI*R_RUEDA;                   // x medido [m]
  float th=(ang_comp-setpoint)/R2D;                    // theta medido [rad]
  float y0=xm, y1=th;

  // --- OBSERVADOR KALMAN: predecir con u previo, luego corregir ---
  static float u_prev=0;
  float xp[4];
  for(int i=0;i<4;i++){ xp[i]=Bd[i]*u_prev; for(int j=0;j<4;j++) xp[i]+=Ad[i][j]*xh[j]; }
  float e0=y0-xp[0], e1=y1-xp[2];                      // innovacion (C toma x1 y x3)
  for(int i=0;i<4;i++) xh[i]=xp[i]+Lk[i][0]*e0+Lk[i][1]*e1;

  // --- seguridad ---
  if(fabs(th*R2D)>ANG_CAIDA){ parar(); u_prev=0; return; }
  if(!control_on){ parar(); u_prev=0; return; }

  // --- CONTROL LQG: u=-K*x_est (posicion opcional) ---
  float u = -(K[2]*xh[2]+K[3]*xh[3]);                  // angulo (siempre)
  if(usePos) u += -(K[0]*xh[0]+K[1]*xh[1]);            // posicion
  u*=inv; u_prev=u;
  int pwm=(int)constrain(u,-PWM_MAX,PWM_MAX);
  setMotor(enableAPin,motorAPin1,motorAPin2,pwm);
  setMotor(enableBPin,motorBPin1,motorBPin2,pwm);
}
