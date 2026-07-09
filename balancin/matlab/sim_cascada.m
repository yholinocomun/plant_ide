clear; clc; close all;
%% ============================================================
%  CONTROL EN CASCADA - VERSION CORREGIDA (colocacion de polos)
%  El metodo por CANCELACION de polos cancela el polo INESTABLE (+alpha)
%  con un cero del controlador -> lazo INTERNAMENTE INESTABLE (modo oculto +).
%  Aqui se conserva la estructura de cascada y las mismas constantes de
%  tiempo (T22,T11), pero se ESTABILIZA de verdad el polo (no se cancela).
%% ============================================================
A=[0 1 0 0; 0 0 -7.4697 0; 0 0 0 1; 0 0 104.6806 0];
B=[0;0.1979;0;-1.4869]; C=[0 0 1 0]; D=0;
a=104.6806; b=-1.4869; alpha=sqrt(a);
Gp=minreal(tf(ss(A,B,C,D)));           % planta angulo (inestable)
fprintf('Polos de la planta: %.3f  %.3f  (uno inestable)\n', pole(Gp));

%% Diseno cascada corregido
% Senal interna: w = theta_dot + alpha*theta  ( = P2*u, medible con gyro+angulo )
% Lazo interno (proporcional, estabiliza P2=b/(s-alpha)):  u = k2*(v - w)
%   coloca el polo interno en -1/T22 = -20
T22=0.05; k2=(alpha+1/T22)/b;          % = -20.33
% Lazo externo (PI, cancela el polo ESTABLE -alpha y coloca ~ -1/T11):
%   v = kc*(e1 + alpha*I1)
T11=0.50; kc=1.19;                     % ajustado para polo dominante ~ -2
fprintf('k2(inner)=%.3f  kc(outer)=%.3f  alpha=%.4f\n', k2, kc, alpha);

%% Simulacion discreta (igual que el firmware)
dt=0.01; N=round(4/dt); R2D=180/pi;
sysd=c2d(ss(A,B,C,0),dt,'zoh'); Ad=sysd.A; Bd=sysd.B;
x=[0;0;5/R2D;0]; I1=0; TH=zeros(1,N); U=zeros(1,N); CASGAIN=1.0;
for k=1:N
   th=x(3); thd=x(4); e1=-th; I1=max(min(I1+e1*dt,2),-2);
   v=kc*(e1+alpha*I1); w=thd+alpha*th; u=CASGAIN*k2*(v-w);
   u=max(min(u,255),-255); TH(k)=th*R2D; U(k)=u; x=Ad*x+Bd*u;
end
t=(0:N-1)*dt;
figure; subplot(2,1,1); plot(t,TH,'LineWidth',1.6); grid on; ylabel('\theta [deg]');
title('Cascada corregida (colocacion de polos) - estable'); yline(0,'--k');
subplot(2,1,2); plot(t,U,'LineWidth',1.6); grid on; ylabel('u [PWM]'); xlabel('t [s]');
fprintf('theta final = %.3f deg  |u|max = %.0f\n', TH(end), max(abs(U)));
