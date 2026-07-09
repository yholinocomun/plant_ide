/*  BALANCE - CONTROLADOR DINAMICO DISCRETO (IIR)  - ESP32-S3 core 3.x
    Ejecuta  u[k] = (SUM num[i]*e[k-i] - SUM den[j]*u[k-j]) / den[0]
    con e[k] = (setpoint - theta) en RADIANES.  Sirve para H-INF y FOPID:
    solo cambia NUM/DEN (de balancin/data/coeficientes_discretos.txt).
    >>> Cargado por defecto con el controlador H-INFINITO (4o orden). <<<
    FOPID (orden 9) es numericamente delicado en float: si lo usas, reduce
    el orden de Oustaloup a N=2 y regenera los coeficientes.
    Teclas: space=on/off  z=trim  i=inv  g=giro */
#include <Arduino.h>
#include <Wire.h>
#include <math.h>
const int enableAPin=13,motorAPin1=12,motorAPin2=14, enableBPin=9,motorBPin1=10,motorBPin2=11;
const int SDA_PIN=21,SCL_PIN=47,MPU_ADDR=0x68,freq=30000,resolution=8;
const float dt=0.010,R2D=57.29578,ANG_CAIDA=35.0; const unsigned long DT_US=10000;
const int U_DEAD=30,PWM_MIN=3,PWM_MAX=255;

// ===== CONTROLADOR DISCRETO K(z)  (H-INFINITO, Tustin dt=10ms) =====
#define NB 5     // numero de coeficientes (orden+1)
const float num[NB]={ -9644.251425, 14377.850121, 4271.651618, -14402.416908, 5348.033021 };
const float den[NB]={ 1.0, -1.024125, -0.727214, 1.035496, -0.261415 };
float ebuf[NB]={0}, ubuf[NB]={0};   // historicos de error y salida

float inv=1.0,gyroSign=-1.0,setpoint=0.0; bool control_on=false;
float gyroBias=0,ang=0;
void initMPU(){Wire.beginTransmission(MPU_ADDR);Wire.write(0x6B);Wire.write(0);Wire.endTransmission(true);}
void leer(float&a,float&g){Wire.beginTransmission(MPU_ADDR);Wire.write(0x3B);Wire.endTransmission(false);
 Wire.requestFrom(MPU_ADDR,14,true); int16_t ax=Wire.read()<<8|Wire.read(),ay=Wire.read()<<8|Wire.read(),az=Wire.read()<<8|Wire.read();
 Wire.read();Wire.read();Wire.read();Wire.read(); int16_t gy=Wire.read()<<8|Wire.read(); Wire.read();Wire.read();
 a=atan2f((float)ax,(float)az)*R2D; g=(float)gy/131.0f;}
void calib(){Serial.println("Calib giro 2s QUIETO");float s=0;for(int i=0;i<200;i++){float a2,g;leer(a2,g);s+=g;delay(10);}gyroBias=s/200;float a2,g;leer(a2,g);ang=a2;}
void setMot(int en,int i1,int i2,int pwm){int m=abs(pwm);if(m<PWM_MIN){ledcWrite(en,0);return;}bool f=pwm>=0;m+=U_DEAD;if(m>PWM_MAX)m=PWM_MAX;digitalWrite(i1,f?HIGH:LOW);digitalWrite(i2,f?LOW:HIGH);ledcWrite(en,m);}
void parar(){ledcWrite(enableAPin,0);ledcWrite(enableBPin,0);}
void setup(){Serial.begin(115200);delay(400);
 pinMode(motorAPin1,OUTPUT);pinMode(motorAPin2,OUTPUT);pinMode(motorBPin1,OUTPUT);pinMode(motorBPin2,OUTPUT);
 ledcAttach(enableAPin,freq,resolution);ledcAttach(enableBPin,freq,resolution);
 Wire.begin(SDA_PIN,SCL_PIN);Wire.setClock(400000);initMPU();delay(100);calib();
 Serial.println("Dinamico(H-inf) listo. space z i g");}
unsigned long t_ant=0;
void loop(){
 if(Serial.available()){char c=Serial.read();
  if(c==' '){control_on=!control_on;if(!control_on){parar();for(int i=0;i<NB;i++){ebuf[i]=0;ubuf[i]=0;}}Serial.println(control_on?">>ON":">>OFF");}
  else if(c=='z'){setpoint=ang;Serial.print("set=");Serial.println(setpoint,2);}
  else if(c=='i'){inv=-inv;} else if(c=='g'){gyroSign=-gyroSign;} }
 unsigned long now=micros(); if(now-t_ant<DT_US)return; t_ant=now;
 float aa,gr; leer(aa,gr); float gy=gyroSign*(gr-gyroBias);
 ang=0.98f*(ang+gy*dt)+0.02f*aa;
 float theta=(ang-setpoint)/R2D;                 // rad
 if(fabs(theta*R2D)>ANG_CAIDA){parar();return;} if(!control_on){parar();return;}
 float e=setpoint/R2D - theta;                    // error (ref angulo ~0)
 // desplazar historicos
 for(int i=NB-1;i>0;i--){ebuf[i]=ebuf[i-1];ubuf[i]=ubuf[i-1];}
 ebuf[0]=e;
 // ecuacion en diferencias
 float u=0; for(int i=0;i<NB;i++) u+=num[i]*ebuf[i];
 for(int j=1;j<NB;j++) u-=den[j]*ubuf[j];
 u/=den[0]; ubuf[0]=u;
 u*=inv; int pwm=(int)constrain(u,-PWM_MAX,PWM_MAX);
 setMot(enableAPin,motorAPin1,motorAPin2,pwm); setMot(enableBPin,motorBPin1,motorBPin2,pwm);
}
