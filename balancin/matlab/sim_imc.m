clc; clear; close all;               % #4 IMC (con estabilizacion interna)
[A,B,C,D]=planta();
Q=diag([1 1 300 10]); R=0.0002; K=lqr(A,B,Q,R); Acl=A-B*K;  % lazo interno
P=minreal(tf(ss(Acl,B,[1 0 0 0],0)));   % planta estabilizada w->x (estable)
z=zero(P); rhp=z(real(z)>1e-6);         % ceros de fase no minima
s=tf('s'); Pmin=P;
for zz=rhp.', if isreal(zz), Pmin=Pmin*(s+zz)/(-s+zz); end, end
lam=0.35; n=4; F=1/(lam*s+1)^n;         % filtro IMC (tu convencion)
Q_=minreal(F/Pmin); Cimc=minreal(Q_/(1-Q_*P));
T=minreal(feedback(Cimc*P,1));
figure; step(0.1*T); title('IMC seguimiento de posicion'); grid on;
Cd=c2d(Cimc,0.01,'tustin'); [num,den]=tfdata(Cd,'v');
