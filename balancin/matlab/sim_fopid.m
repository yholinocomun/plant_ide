clc; clear; close all;               % #5 PID FRACCIONARIO (Oustaloup)
[A,B,C,D]=planta(); a43=104.6806; b4=B(4);
s=tf('s'); G=b4/(s^2-a43);
oust=@(al) local_oustaloup(al,0.01,100,4);
Kp=2500; Ki=50; Kd=650; lam=0.9; mu=1.0;
C = -Kp + (-Ki)*oust(-lam) + (-Kd)*oust(mu);   % C(s)=Kp+Ki/s^lam+Kd s^mu (signo b4<0)
T=feedback(C*G,1);
fprintf('polo real max=%.3f\n',max(real(pole(T))));
figure; step(feedback(G,C)); title('FOPID rechazo perturbacion'); grid on;
function W=local_oustaloup(al,wb,wh,N)
  k=-N:N; wz=wb*(wh/wb).^((k+N+0.5*(1-al))/(2*N+1)); wp=wb*(wh/wb).^((k+N+0.5*(1+al))/(2*N+1));
  W=zpk(-wz,-wp,wh^al);
end
