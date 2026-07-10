function comparar(slug)
% COMPARAR  Simula un controlador y lo compara con el hardware en UNA figura
%           con 2 graficas superpuestas (sim + hardware) y metricas visibles.
%   comparar('lqr')     % tambien: lqg, cascada, imc, hinf   (fopid: ya hecho aparte)
%
% Genera <slug>_comparativa.png (theta y u, sim vs HW en la misma grafica) e
% imprime en consola: overshoot, tiempo de establecimiento, thetaRMS, |u|max
% de simulacion y de hardware, listo para pegar en una tabla.

if nargin<1, slug='lqr'; end
here   = fileparts(mfilename('fullpath'));
resdir = fullfile(here,'..','resultados');
outdir = here;
rms = @(v) sqrt(mean(v.^2));

%% 1) Simulacion (recuperacion desde theta0 = 5 deg)
[t_s, th_s, u_s] = sim_ctrl(slug);
OS_s = max(0,-min(th_s))/abs(th_s(1))*100;             % sobrepaso [%]
is = find(abs(th_s) > 0.02*abs(th_s(1)), 1, 'last');   % settling 2%
ts_s = is*0.01;
RMS_s = rms(th_s); Umax_s = max(abs(u_s));

%% 2) Data real (hardware) mas reciente
d = dir(fullfile(resdir,[slug '_2*.csv']));
th_h=[]; t_h=[]; u_h=[]; RMS_h=NaN; Thmax_h=NaN; Umax_h=NaN; ts_h=NaN;
if ~isempty(d)
   [~,ix]=max([d.datenum]);
   T=readtable(fullfile(resdir,d(ix).name)); D=T{:,:};
   t_h=(D(:,1)-D(1,1))/1000; th_h=D(:,2); u_h=D(:,6);
   RMS_h=rms(th_h); Thmax_h=max(abs(th_h)); Umax_h=max(abs(u_h));
   tail=rms(th_h(t_h>0.6*t_h(end))); band=max(2*tail,0.5);   % settling HW
   ih=find(abs(th_h)>band,1,'last'); if ~isempty(ih), ts_h=t_h(ih); end
end

%% 3) Figura: 2 graficas superpuestas + recuadro de metricas
nombres = containers.Map({'lqr','lqg','cascada','fopid','imc','hinf'}, ...
   {'LQR','LQG','Cascada','FOPID','IMC','H_{\infty}'});
f=figure('Color','w','Position',[100 100 720 540]);

ax1=subplot(2,1,1); hold on; grid on;
plot(t_s, th_s, 'b-', 'LineWidth',1.9);
if ~isempty(th_h), plot(t_h, th_h, 'r-', 'LineWidth',1.0); end
yline(0,'--k','HandleVisibility','off');
ylabel('\theta [deg]','FontSize',11);
title([nombres(slug) ' : Simulacion vs Hardware'],'FontSize',12,'Interpreter','tex');
legend('Simulacion','Hardware','Location','northeast','Interpreter','tex');
% recuadro con parametros de desempeno (sim y hardware)
txt = { sprintf('SIM:  OS=%.1f%%   t_s=%.2fs   \\theta_{RMS}=%.2f\\circ   |u|_{max}=%.0f',OS_s,ts_s,RMS_s,Umax_s), ...
        sprintf('HW:   \\theta_{RMS}=%.2f\\circ   |\\theta|_{max}=%.2f\\circ   |u|_{max}=%.0f   t_s=%.1fs',RMS_h,Thmax_h,Umax_h,ts_h) };
yl=ylim; xl=xlim;
text(xl(1)+0.02*range(xl), yl(2)-0.05*range(yl), txt, 'VerticalAlignment','top',...
     'FontSize',8.5,'BackgroundColor','w','EdgeColor',[.4 .4 .4],'Margin',3);

ax2=subplot(2,1,2); hold on; grid on;
plot(t_s, u_s, 'b-', 'LineWidth',1.9);
if ~isempty(u_h), plot(t_h, u_h, 'r-', 'LineWidth',1.0); end
ylabel('u [PWM]','FontSize',11); xlabel('t [s]','FontSize',11);
legend('Simulacion','Hardware','Location','northeast','Interpreter','tex');
linkaxes([ax1 ax2],'x');
saveas(f, fullfile(outdir,[slug '_comparativa.png']));

%% 4) Impresion en consola (para armar la tabla)
fprintf('\n===== %s =====\n', upper(slug));
fprintf('SIM : OS=%.1f%%  ts=%.2fs  thetaRMS=%.3f deg  |u|max=%.0f\n', OS_s,ts_s,RMS_s,Umax_s);
if ~isempty(th_h)
  fprintf('HW  : thetaRMS=%.3f deg  |theta|max=%.2f deg  |u|max=%.0f  ts=%.1fs  (%s)\n',...
          RMS_h,Thmax_h,Umax_h,ts_h,d(ix).name);
else
  fprintf('HW  : (sin CSV en resultados/)\n');
end
fprintf('Imagen: %s_comparativa.png\n', slug);
end

% =====================================================================
function [t,th_deg,u_out]=sim_ctrl(slug)
% Lazo cerrado discreto (dt=10ms) desde theta0=5 deg.
R2D=180/pi; dt=0.01; Tf=4; N=round(Tf/dt);
A=[0 1 0 0; 0 0 -7.4697 0; 0 0 0 1; 0 0 104.6806 0];
B=[0;0.1979;0;-1.4869]*(0.00054/0.00338);          % B REAL
sysd=c2d(ss(A,B,eye(4),zeros(4,1)),dt,'zoh'); Ad=sysd.A; Bd=sysd.B;

alpha=10.2314; k2=-20.33; kc=1.19; CASGAIN=4;                 % cascada
Klqg=[-70.71 -196.97 -1985.22 -284.52];                       % lqg
% H-INFINITO con pesos NUEVOS  W1=makeweight(200,60,0.05), W2=1e-3, W3=makeweight(0.20,40,2)
% (K(z) por mixsyn+c2d tustin; escala 1.0 -> ESTABILIZA)
num=[-38006.829174 52497.395668 19204.787074 -52729.296467 18570.141301];
den=[1 -0.113033 -0.865016 0.281061 0.033044]; HINF_SCALE=1.0;
% FOPID (no se modifica; incluido por completitud)
L=64; lam=0.95; mu=0.15; cI=zeros(1,L); cD=zeros(1,L); cI(1)=1; cD(1)=1;
for j=2:L, cI(j)=cI(j-1)*(1-(-lam+1)/(j-1)); cD(j)=cD(j-1)*(1-(mu+1)/(j-1)); end
hlam=dt^lam; hmu=dt^-mu; ebuf=zeros(1,L);
I1=0; u_imc=0; ehist=zeros(1,5); uhist=zeros(1,5);

x=[0;0;5/R2D;0]; th_deg=zeros(1,N); u_out=zeros(1,N);
for kk=1:N
   th=x(3); thd=x(4);
   switch slug
     case 'lqr',     u = 59.5*(th*R2D) + 1.7*(thd*R2D);
     case 'lqg',     u = -(Klqg(3)*th + Klqg(4)*thd);
     case 'cascada'
        e1=-th; I1=max(min(I1+e1*dt,2),-2);
        u=CASGAIN*k2*(kc*(e1+alpha*I1)-(thd+alpha*th));
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
        uhist=[u uhist(1:end-1)]; u=HINF_SCALE*u;
     otherwise, error('controlador desconocido: %s',slug);
   end
   u=max(min(u,255),-255);
   th_deg(kk)=th*R2D; u_out(kk)=u;
   x=Ad*x+Bd*u;
   if abs(x(3)*R2D)>60, th_deg=th_deg(1:kk); u_out=u_out(1:kk); break; end
end
t=(0:numel(th_deg)-1)*dt;
end
