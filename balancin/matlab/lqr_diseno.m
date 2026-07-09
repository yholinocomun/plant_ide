clc; clear; close all;
%% ============================================================
%  DISENO LQR DEL BALANCIN  (Q y R por REGLA DE BRYSON)
%  Verifica el procedimiento del documento LaTeX (informe_lqr.tex)
%% ============================================================
R2D = 180/pi;

%% 1) Modelo en espacio de estados (planta identificada, actuador calibrado)
A = [0 1        0   0;
     0 0  -7.4697   0;
     0 0        0   1;
     0 0 104.6806   0];
B = [0; 0.1979; 0; -1.4869]*(0.00054/0.00338);   % B REAL (PWM)
C = [1 0 0 0; 0 0 1 0];  D = zeros(2,1);

fprintf('Polo inestable de la planta: %.3f rad/s\n', max(real(eig(A))));
Co = ctrb(A,B);
if rank(Co)==4, disp('Controlabilidad: 4/4 -> CONTROLABLE');
else,           disp('Controlabilidad: NO controlable'); end

%% 2) REGLA DE BRYSON:  Q_ii = 1/max_i^2 ,  R = 1/u_max^2
% Maximos admisibles (fisicos) de cada estado y del control:
x_max     = 0.10;        % [m]      desviacion maxima de posicion
xd_max    = 0.5;         % [m/s]    velocidad lineal maxima
th_max    = 10/R2D;      % [rad]    inclinacion maxima (10 deg)
thd_max   = 90/R2D;      % [rad/s]  velocidad angular maxima (90 deg/s)
u_max     = 200;         % [PWM]    esfuerzo de control maximo (<255)

Q = diag([1/x_max^2, 1/xd_max^2, 1/th_max^2, 1/thd_max^2]);
R = 1/u_max^2;
disp('Q ='); disp(Q); fprintf('R = %.3e\n', R);

%% 3) Solucion LQR:  K = R^-1 B^T P ,  P de la ecuacion de Riccati (ARE)
K = lqr(A,B,Q,R);
fprintf('\nK = [%.1f  %.1f  %.1f  %.1f]\n', K);

%% 4) Lazo cerrado y desempeno
Acl = A - B*K;
pol = eig(Acl);
disp('Polos de lazo cerrado:'); disp(pol);
[wn,zeta] = damp(Acl); [wn_min,im]=min(wn);
fprintf('Polos dominantes: wn=%.2f rad/s  zeta=%.3f\n', wn_min, zeta(im));

% Respuesta a condicion inicial theta0 = 5 deg  (recuperacion de inclinacion)
sys_th = ss(Acl, zeros(4,1), [0 0 1 0], 0);
x0 = [0;0;5/R2D;0];
t = 0:0.002:4;
[y,t] = initial(sys_th, x0, t);
th = y*R2D;
overshoot = max(0,-min(th))/abs(th(1))*100;
idx = find(abs(th) > 0.02*abs(th(1)), 1, 'last'); ts = t(idx);
fprintf('Recuperacion desde 5deg:  overshoot=%.1f%%  t_settling(2%%)=%.2fs\n', overshoot, ts);

%% 5) Mapeo a ganancias del firmware (theta en GRADOS)
Kang_p = -K(3)*pi/180;   Kang_d = -K(4)*pi/180;
Kpos_p = -K(1);          Kpos_d = -K(2);
fprintf('\n== GANANCIAS PARA EL FIRMWARE ==\n');
fprintf('Kang_p=%.2f PWM/deg   Kang_d=%.3f PWM/(deg/s)\n', Kang_p, Kang_d);
fprintf('Kpos_p=%.2f           Kpos_d=%.2f\n', Kpos_p, Kpos_d);
fprintf('(HW sintonizado: Kang_p=59.5, Kang_d=1.7  -> coincide en orden)\n');

%% 6) Graficas
figure('Color','w','Position',[100 100 760 560]);
subplot(2,1,1); plot(t,th,'LineWidth',1.7); grid on; yline(0,'--k');
ylabel('\theta [deg]'); title('LQR - recuperacion desde \theta_0=5° (condicion inicial)');
text(0.98,0.9,sprintf('OS=%.1f%%   t_s=%.2fs   \\zeta=%.2f',overshoot,ts,min(zeta)),...
     'Units','normalized','HorizontalAlignment','right');
% control u = -K x  durante la recuperacion
sysu = ss(Acl, zeros(4,1), -K, 0); [u,~]=initial(sysu,x0,t);
subplot(2,1,2); plot(t,u,'LineWidth',1.7); grid on;
ylabel('u [PWM]'); xlabel('t [s]'); title('Senal de control');
saveas(gcf,'lqr_diseno.png');

%% 7) Comparacion de dos Q (estilo LQR_BASICO del curso)
Q2 = diag([1/x_max^2, 1/xd_max^2, 1/(5/R2D)^2, 1/thd_max^2]);  % mas peso en theta
K2 = lqr(A,B,Q2,R);
figure('Color','w'); 
initial(ss(A-B*K, zeros(4,1),[0 0 1 0],0), x0, t); hold on;
initial(ss(A-B*K2,zeros(4,1),[0 0 1 0],0), x0, t);
grid on; legend('Q base','Q con mas peso en \theta'); title('Efecto de Q en \theta');
disp('Listo. Figuras: lqr_diseno.png');
