clc; clear; close all;               % #3 H-INFINITO (sensibilidad mixta)
[A,B,C,D]=planta(); a43=104.6806; b4=B(4);
s=tf('s'); G=b4/(s^2-a43);           % subsistema angulo (inestable)
W1=makeweight(50,6,0.05);            % S: desempeno
W2=1e-3;                             % KS: esfuerzo
W3=makeweight(0.05,40,2);            % T: robustez
[K,CL,gamma]=mixsyn(G,W1,W2,W3);
fprintf('gamma=%.2f\n',gamma);
T=feedback(G*K,1); figure; step(T); title('H-inf lazo cerrado'); grid on;
Kd=c2d(K,0.01,'tustin'); [num,den]=tfdata(Kd,'v');
disp('K(z) num='); disp(num); disp('K(z) den='); disp(den);
