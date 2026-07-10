clc; clear; close all;               % #3 H-INFINITO (sensibilidad mixta)
% Diseno H-infinito para el subsistema angular inestable del balancin.
% Usa exactamente los ponderadores solicitados y entrega K(z) para firmware.

[A,B,C,D]=planta(); %#ok<ASGLU>
a43=104.6806;
b4=B(4);
Ts=0.01;
HGAIN=0.10;                         % misma escala global usada en hardware

s=tf('s');
G=b4/(s^2-a43);                     % subsistema angulo: theta/u (inestable)

% Ponderadores de sensibilidad mixta
W1=makeweight(200,60,0.05);         % S: desempeno / rechazo error
W2=1e-3;                            % KS: esfuerzo de control
W3=makeweight(0.20,40,2);           % T: robustez / ruido alta frecuencia

% Sintesis H-infinito por sensibilidad mixta
[K,CL,gamma]=mixsyn(G,W1,W2,W3); %#ok<ASGLU>
fprintf('gamma=%.4f\n',gamma);

% Lazo cerrado de referencia y sensibilidad
T=feedback(G*K,1);
S=feedback(1,G*K);
KS=feedback(K,G);

figure('Color','w','Position',[80 80 900 620]);
subplot(2,2,1); step(T); grid on; title('H-inf: T=GK/(1+GK)'); ylabel('\theta/ref');
subplot(2,2,2); bodemag(S,T); grid on; legend('S','T'); title('Sensibilidad y robustez');
subplot(2,2,3); bodemag(KS); grid on; title('Esfuerzo KS');
subplot(2,2,4); pzmap(T); grid on; title('Polos/cero lazo cerrado');
sgtitle(sprintf('H-infinito sensibilidad mixta, gamma=%.3f',gamma));

% Discretizacion para firmware / simulacion
Kd=c2d(K,Ts,'tustin');
[num,den]=tfdata(Kd,'v');
num=real(num); den=real(den);
num=num/den(1); den=den/den(1);

fprintf('\nK(z) Tustin Ts=%.3f s\n',Ts);
fprintf('num = ['); fprintf(' %.9g',num); fprintf(' ];\n');
fprintf('den = ['); fprintf(' %.9g',den); fprintf(' ];\n');
fprintf('HGAIN = %.3f\n',HGAIN);

% Prueba discreta equivalente desde theta0=5 deg sobre el mismo modelo lineal
R2D=180/pi; Tf=4; N=round(Tf/Ts);
sysd=c2d(ss(A,B,eye(4),zeros(4,1)),Ts,'zoh');
Ad=sysd.A; Bd=sysd.B;
x=[0;0;5/R2D;0]; th=zeros(1,N); ulog=zeros(1,N);
nb=numel(num); na=numel(den); ehist=zeros(1,nb); uhist=zeros(1,na);
for k=1:N
    e=-x(3);                         % setpoint theta=0 rad
    ehist=[e ehist(1:end-1)];
    uc=(num*ehist' - den(2:end)*uhist(1:end-1)')/den(1);
    uhist=[uc uhist(1:end-1)];
    u=max(min(HGAIN*uc,255),-255);
    th(k)=x(3)*R2D; ulog(k)=u;
    x=Ad*x+Bd*u;
    if abs(x(3)*R2D)>60
        th=th(1:k); ulog=ulog(1:k); break;
    end
end
t=(0:numel(th)-1)*Ts;
figure('Color','w','Position',[120 120 760 520]);
subplot(2,1,1); plot(t,th,'LineWidth',1.6); grid on; yline(0,'--k');
ylabel('\theta [deg]'); title('H-inf discreto aplicado al modelo completo');
subplot(2,1,2); plot(t,ulog,'LineWidth',1.6); grid on; ylabel('u [PWM]'); xlabel('t [s]');

% Guardar coeficientes para copiar a Arduino si se vuelve a sintetizar K
out=fullfile(fileparts(mfilename('fullpath')),'..','data','coeficientes_hinf_mixsyn.txt');
fid=fopen(out,'w');
fprintf(fid,'// H-INF mixsyn, Ts=%.3f, HGAIN=%.3f, gamma=%.6f\n',Ts,HGAIN,gamma);
fprintf(fid,'const float num[%d]={',nb); fprintf(fid,' %.9gf,',num); fprintf(fid,' };\n');
fprintf(fid,'const float den[%d]={',na); fprintf(fid,' %.9gf,',den); fprintf(fid,' };\n');
fclose(fid);
fprintf('Coeficientes guardados en: %s\n',out);
