/*  BALANCE MPC / LQR-PREDICTIVO - ESP32-S3 core 3.x - ESTANDAR + TELEMETRIA
    u = -Kmpc*x (estados medidos; Kmpc = Riccati horizonte finito). Teclas: space z(fijo) o i g f t */
#include <Arduino.h>
#include <Wire.h>
#include <math.h>
// ====== CALIBRACION FIJA (robot ya calibrado en su cero; SIN espera de 2s) ======
const float GYRO_BIAS_FIJO   = 0.461259f;   // bias fijo del gyro Y [deg/s]
const float ANGULO_CERO_FIJO = 0.162628f;   // lectura del accel en el cero fisico [deg]
const float SETPOINT_FIJO    = 0.0f;        // trim de equilibrio relativo al cero [deg]
const int enableAPin=13,motorAPin1=12,motorAPin2=14, enableBPin=9,motorBPin1=10,motorBPin2=11;
const int encIzqA=4,encIzqB=5,encDerA=6,encDerB=7; const float PPR=1945.0,R_RUEDA=0.037;
const int SDA_PIN=21,SCL_PIN=47,MPU_ADDR=0x68,freq=30000,resolution=8;
const float dt=0.010,R2D=57.29578,ANG_CAIDA=35.0; const unsigned long DT_US=10000;
const int U_DEAD=30,PWM_MIN=3,PWM_MAX=255;
float Kmpc[4]={ -1.395,-8.131,-1414.19,-197.57 };
float inv=1.0,gyroSign=-1.0; bool usePos=false,control_on=false;
volatile long encIzq=0,encDer=0; float ang=0,x_prev=0;
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
 Serial.println("MPC listo. space z(fijo) o i g f t=telemetria");}
unsigned long t_ant=0;
void loop(){
 if(Serial.available()){char c=Serial.read();
  if(c==' '){control_on=!control_on;if(!control_on)parar();Serial.println(control_on?">>ON":">>OFF");}
  else if(c=='z'){Serial.println("# cero y SETPOINT_FIJO FIJOS en el codigo");}
  else if(c=='o')usePos=!usePos; else if(c=='i')inv=-inv; else if(c=='g')gyroSign=-gyroSign;
  else if(c=='t'){telem=!telem; if(telem){t0t=millis();telemHdr("mpc");} else Serial.println("# fin");}
  else if(c=='f'){Serial.print("MPC set=");Serial.println(SETPOINT_FIJO,2);} }
 unsigned long now=micros(); if(now-t_ant<DT_US)return; t_ant=now;
 float aa,gr;leer(aa,gr);float gy=gyroSign*(gr-GYRO_BIAS_FIJO); ang=0.98f*(ang+gy*dt)+0.02f*aa;
 float theta_deg=(ang-ANGULO_CERO_FIJO)-SETPOINT_FIJO, theta_dot=gy, th=theta_deg/R2D, thd=gy/R2D;
 long enc=(encIzq+encDer)/2; float x=(enc/PPR)*2*M_PI*R_RUEDA, x_dot=(x-x_prev)/dt; x_prev=x;
 int pwm=0;
 if(fabs(theta_deg)<=ANG_CAIDA && control_on){
   float u=-(Kmpc[2]*th+Kmpc[3]*thd); if(usePos)u+=-(Kmpc[0]*x+Kmpc[1]*x_dot);
   u*=inv; pwm=(int)constrain(u,-PWM_MAX,PWM_MAX);
   setMot(enableAPin,motorAPin1,motorAPin2,pwm); setMot(enableBPin,motorBPin1,motorBPin2,pwm);
 } else parar();
 if(telem && ++tdiv>=TELEM_CADA){tdiv=0;
   Serial.print(millis()-t0t);Serial.print(',');Serial.print(theta_deg,2);Serial.print(',');
   Serial.print(theta_dot,1);Serial.print(',');Serial.print(x,4);Serial.print(',');
   Serial.print(x_dot,3);Serial.print(',');Serial.print(pwm);Serial.print(',');
   Serial.print(SETPOINT_FIJO,2);Serial.print(',');Serial.println(usePos?1:0);}
}
