clc; clear; close all;
[A,B,C,D] = planta();  R2D = 180/pi;  dt = 0.01;

%% 1) LQR (realimentacion de estado) - pesos sobre [x xd th thd]
Q = diag([1 1 300 10]);   Rr = 0.0002;
K = lqr(A,B,Q,Rr);                        % u = -K x
fprintf('K = [%.1f %.1f %.1f %.1f]\n',K);
fprintf('Kang_p=%.2f PWM/deg  Kang_d=%.2f  Kpos_p=%.2f  Kpos_d=%.2f\n',...
        K(3)/R2D, K(4)/R2D, K(1), K(2));

%% 2) Observador de Kalman (discreto)  -- convencion lqe del curso
sysd = c2d(ss(A,B,C,D), dt, 'zoh');  Ad=sysd.A; Bd=sysd.B;
Qn = diag([1e-4 1e-3 1e-4 1e-3]);  Rn = diag([2e-4 3e-4]);
[~,L,~] = kalman(ss(Ad,[Bd eye(4)],C,0,dt), Qn, Rn);  % ganancia observador

%% 3) Lazo cerrado con observador (LQG)  -- simulacion
T=3; N=round(T/dt); x=[0;0;5/R2D;0]; xh=zeros(4,1);
X=zeros(4,N); U=zeros(1,N);
for k=1:N
    u = -K*xh;  u = max(min(u,255),-255);
    X(:,k)=x; U(k)=u;
    x  = Ad*x + Bd*u;
    y  = C*x + [0.014*randn; 0.017*randn];
    xh = Ad*xh + Bd*u;  xh = xh + L*(y - C*xh);
end
t=(0:N-1)*dt;
figure; subplot(3,1,1); plot(t,X(3,:)*R2D); ylabel('\theta [deg]'); grid on; title('LQG balancin');
subplot(3,1,2); plot(t,X(1,:)); ylabel('x [m]'); grid on;
subplot(3,1,3); plot(t,U); ylabel('u [PWM]'); xlabel('t [s]'); grid on;
