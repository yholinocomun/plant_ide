function comparar(slug)
% COMPARAR  Simula un controlador, guarda su imagen y compara con hardware.
%   comparar('lqr')   % tambien: lqg, cascada, fopid, imc, hinf
%
% - Simula el lazo cerrado (recuperacion desde theta0=5 deg) -> guarda <slug>_sim.png
% - Carga el CSV real mas reciente de ../resultados/<slug>_*.csv
% - Grafica y guarda la comparativa sim vs hardware -> <slug>_comparativa.png
% - Imprime metricas (theta_RMS, |theta|max, |u|max) de ambos.

if nargin<1, slug='lqr'; end
here   = fileparts(mfilename('fullpath'));
resdir = fullfile(here,'..','resultados');
outdir = here;

%% 1) Simulacion
[t_s, th_s, u_s] = sim_ctrl(slug);
% ---- guardar imagen de la simulacion ----
f1=figure('Color','w','Position',[80 80 640 460]);
subplot(2,1,1); plot(t_s,th_s,'LineWidth',1.6); grid on; ylabel('\theta [deg]');
title(sprintf('%s - Simulacion (recuperacion desde 5°)',upper(slug))); yline(0,'--k');
subplot(2,1,2); plot(t_s,u_s,'LineWidth',1.6); grid on; ylabel('u [PWM]'); xlabel('t [s]');
saveas(f1,fullfile(outdir,[slug '_sim.png']));

%% 2) Cargar data real (hardware)
d = dir(fullfile(resdir,[slug '_2*.csv']));
if isempty(d)
   warning('No hay CSV de hardware para "%s" en %s',slug,resdir);
   th_h=[]; t_h=[]; u_h=[];
else
   [~,ix]=max([d.datenum]); csv=fullfile(resdir,d(ix).name);
   T=readtable(csv); D=T{:,:};
   t_h=(D(:,1)-D(1,1))/1000; th_h=D(:,2); u_h=D(:,6);
   fprintf('Hardware: %s (%d muestras)\n',d(ix).name,numel(t_h));
end

%% 3) Comparativa sim vs hardware
f2=figure('Color','w','Position',[100 100 900 560]);
subplot(2,2,1); plot(t_s,th_s,'b','LineWidth',1.5); grid on; yline(0,'--k');
   title('SIMULACION'); ylabel('\theta [deg]');
subplot(2,2,2);
   if ~isempty(th_h), plot(t_h,th_h,'r','LineWidth',1.0); end
   grid on; yline(0,'--k'); title('HARDWARE'); ylabel('\theta [deg]');
subplot(2,2,3); plot(t_s,u_s,'b','LineWidth',1.5); grid on;
   ylabel('u [PWM]'); xlabel('t [s]');
subplot(2,2,4);
   if ~isempty(u_h), plot(t_h,u_h,'r','LineWidth',1.0); end
   grid on; ylabel('u [PWM]'); xlabel('t [s]');
sgtitle(sprintf('%s : Simulacion vs Hardware',upper(slug)));
saveas(f2,fullfile(outdir,[slug '_comparativa.png']));

%% 4) Metricas
rms=@(v) sqrt(mean(v.^2));
fprintf('\n== %s ==\n',upper(slug));
fprintf('  SIM : thetaRMS=%.3f deg  |theta|max=%.2f  |u|max=%.0f\n',...
        rms(th_s),max(abs(th_s)),max(abs(u_s)));
if ~isempty(th_h)
  fprintf('  HW  : thetaRMS=%.3f deg  |theta|max=%.2f  |u|max=%.0f\n',...
        rms(th_h),max(abs(th_h)),max(abs(u_h)));
end
fprintf('  Imagenes: %s_sim.png , %s_comparativa.png\n',slug,slug);
end

% =====================================================================
function [t,th_deg,u_out]=sim_ctrl(slug)
% Simula el lazo cerrado discreto (dt=10ms) desde theta0=5 deg.
R2D=180/pi; dt=0.01; Tf=4; N=round(Tf/dt);
A=[0 1 0 0; 0 0 -7.4697 0; 0 0 0 1; 0 0 104.6806 0];
B=[0;0.1979;0;-1.4869]*(0.00054/0.00338);      % B REAL
sysd=c2d(ss(A,B,eye(4),zeros(4,1)),dt,'zoh'); Ad=sysd.A; Bd=sysd.B;

% --- coeficientes de controladores dinamicos ---
alpha=10.2314; k2=-20.33; kc=1.19; CASGAIN=4;             % cascada
Klqg=[-70.71 -196.97 -1985.22 -284.52];                  % lqg
num=[-9644.251425 14377.850121 4271.651618 -14402.416908 5348.033021];
den=[1 -1.024125 -0.727214 1.035496 -0.261415];          % hinf
% GL para FOPID (lambda=0.95, mu=0.15)
L=64; lam=0.95; mu=0.15; cI=zeros(1,L); cD=zeros(1,L); cI(1)=1; cD(1)=1;
for j=2:L, cI(j)=cI(j-1)*(1-(-lam+1)/(j-1)); cD(j)=cD(j-1)*(1-(mu+1)/(j-1)); end
hlam=dt^lam; hmu=dt^-mu; ebuf=zeros(1,L);
% estado interno
I1=0; u_imc=0; ehist=zeros(1,5); uhist=zeros(1,5);

x=[0;0;5/R2D;0]; th_deg=zeros(1,N); u_out=zeros(1,N);
for kk=1:N
   th=x(3); thd=x(4);
   switch slug
     case 'lqr'
        u = 59.5*(th*R2D) + 1.7*(thd*R2D);
     case 'lqg'
        u = -(Klqg(3)*th + Klqg(4)*thd);
     case 'cascada'
        e1=-th; I1=max(min(I1+e1*dt,2),-2);
        v=kc*(e1+alpha*I1); w=thd+alpha*th; u=CASGAIN*k2*(v-w);
     case 'imc'
        uref=43.5*(th*R2D)+3.10*(thd*R2D); beta=dt/(0.010+dt);
        u_imc=u_imc+beta*(uref-u_imc); u=0.75*u_imc;
     case 'fopid'
        e=th*R2D; ebuf=[e ebuf(1:end-1)];
        I=hlam*sum(cI.*ebuf); Dv=hmu*sum(cD.*ebuf); I=max(min(I,200),-200);
        u=45*e+12*I+2.5*Dv;
     case 'hinf'
        e=-th; ehist=[e ehist(1:end-1)];
        u=(num*ehist' - den(2:end)*uhist(1:end-1)')/den(1);
        uhist=[u uhist(1:end-1)]; u=0.10*u;
     otherwise, error('controlador desconocido: %s',slug);
   end
   u=max(min(u,255),-255);
   th_deg(kk)=th*R2D; u_out(kk)=u;
   x=Ad*x+Bd*u;
   if abs(x(3)*R2D)>60, th_deg=th_deg(1:kk); u_out=u_out(1:kk); break; end  % cayo
end
t=(0:numel(th_deg)-1)*dt;
end
