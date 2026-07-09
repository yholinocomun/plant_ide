function [A,B,C,D] = planta()
% Robot balancin - espacio de estados CORREGIDO + actuador calibrado.
% X=[x, x_dot, theta(rad), theta_dot],  u=PWM,  y=[x, theta]
A = [0 1        0   0;
     0 0  -7.4697   0;
     0 0        0   1;
     0 0 104.6806   0];
B_nom = [0; 0.1979; 0; -1.4869];        % PWM con K_tau nominal
ESCALA_ACT = 0.00054/0.00338;           % motor real ~6x mas debil
B = B_nom*ESCALA_ACT;                    % B REAL (usar para disenar)
C = [1 0 0 0; 0 0 1 0];
D = zeros(2,1);
end
