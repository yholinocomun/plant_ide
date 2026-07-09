clc; clear; close all;               % #2 MPC / LQR PREDICTIVO
[A,B,C,D]=planta(); R2D=180/pi; dt=0.01;
sysd=c2d(ss(A,B,C,D),dt,'zoh'); Ad=sysd.A; Bd=sysd.B;
Q=diag([1 1 300 10]); Rw=0.0002; Np=60;
% --- LQR de horizonte finito (Riccati hacia atras) ---
P=Q;
for i=1:Np
  K=(Rw+Bd'*P*Bd)\(Bd'*P*Ad); P=Q+Ad'*P*(Ad-Bd*K);
end
Kmpc=K; fprintf('Kmpc=[%.2f %.2f %.2f %.2f]\n',Kmpc);
% --- (opcional) QP con restricciones estilo tu MPC_funcion:
%   quadprog con Umax/Umin. Aqui usamos saturacion de la ganancia. ---
x=[0;0;5/R2D;0]; N=round(3/dt); X=zeros(4,N); U=zeros(1,N);
for k=1:N
  u=max(min(-Kmpc*x,255),-255); X(:,k)=x; U(k)=u; x=Ad*x+Bd*u;
end
t=(0:N-1)*dt;
subplot(2,1,1);plot(t,X(3,:)*R2D);ylabel('\theta[deg]');grid on;title('MPC/LQR predictivo');
subplot(2,1,2);plot(t,U);ylabel('u[PWM]');xlabel('t[s]');grid on;
