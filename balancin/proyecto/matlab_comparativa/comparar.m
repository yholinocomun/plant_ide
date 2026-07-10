function comparar(slug)
% COMPARAR  Comparativa MATLAB automatizada simulacion vs hardware.
%   comparar('lqr')     % tambien: lqg, cascada, fopid, imc, hinf
%   comparar('todos')   % figura IEEE con TODOS: 2 graficas + metricas visibles
%
% Para un controlador individual guarda <slug>_sim.png y <slug>_comparativa.png.
% Para 'todos' guarda en ../resultados:
%   - comparativa_ieee_matlab.png      (una imagen, dos graficas, tablas visibles)
%   - tabla_comparativa_matlab.csv     (metricas SIM/HW)

if nargin<1, slug='hinf'; end
here   = fileparts(mfilename('fullpath'));
resdir = fullfile(here,'..','resultados');
outdir = here;
ctrls  = {'lqr','lqg','cascada','fopid','imc','hinf'};

if strcmpi(slug,'todos') || strcmpi(slug,'all')
    comparar_todos_en_una_figura(ctrls,resdir);
    return;
end

%% 1) Simulacion individual
[t_s, th_s, u_s, estable_s] = sim_ctrl(lower(slug));
f1=figure('Color','w','Position',[80 80 640 460]);
subplot(2,1,1); plot(t_s,th_s,'LineWidth',1.6); grid on; ylabel('\theta [deg]');
title(sprintf('%s - Simulacion desde 5 deg',upper(slug))); yline(0,'--k');
subplot(2,1,2); plot(t_s,u_s,'LineWidth',1.6); grid on; ylabel('u [PWM]'); xlabel('t [s]');
saveas(f1,fullfile(outdir,[lower(slug) '_sim.png']));

%% 2) Cargar data real (hardware)
[t_h, th_h, u_h, csvname] = cargar_hw(lower(slug),resdir);
if isempty(th_h)
   warning('No hay CSV de hardware para "%s" en %s',slug,resdir);
else
   fprintf('Hardware: %s (%d muestras)\n',csvname,numel(t_h));
end

%% 3) Comparativa individual sim vs hardware
f2=figure('Color','w','Position',[100 100 920 560]);
subplot(2,1,1); hold on; plot(t_s,th_s,'b','LineWidth',1.5);
if ~isempty(th_h), plot(t_h,th_h,'r','LineWidth',1.0); end
grid on; yline(0,'--k'); ylabel('\theta [deg]'); legend('Sim','Hardware','Location','best');
title(sprintf('%s : angulo simulacion vs hardware',upper(slug)));
subplot(2,1,2); hold on; plot(t_s,u_s,'b','LineWidth',1.5);
if ~isempty(u_h), plot(t_h,u_h,'r','LineWidth',1.0); end
grid on; ylabel('u [PWM]'); xlabel('t [s]'); legend('Sim','Hardware','Location','best');
sgtitle(sprintf('%s : Simulacion vs Hardware',upper(slug)));
saveas(f2,fullfile(outdir,[lower(slug) '_comparativa.png']));

%% 4) Metricas individuales
ms=metricas(th_s,u_s,estable_s); mh=metricas(th_h,u_h,[]);
fprintf('\n== %s ==\n',upper(slug));
fprintf('  SIM : thetaRMS=%.3f deg  |theta|max=%.2f  |u|max=%.0f  estable=%d\n',...
        ms.thetaRMS,ms.thetaMax,ms.uMax,ms.estable);
if ~isempty(th_h)
  fprintf('  HW  : thetaRMS=%.3f deg  |theta|max=%.2f  |u|max=%.0f\n',...
        mh.thetaRMS,mh.thetaMax,mh.uMax);
end
fprintf('  Imagenes: %s_sim.png , %s_comparativa.png\n',lower(slug),lower(slug));
end

% =====================================================================
function comparar_todos_en_una_figura(ctrls,resdir)
% Una sola imagen para IEEE: exactamente dos graficas (SIM y HW) con todos
% los controladores superpuestos, mas metricas de desempeno visibles.
colors = colores_ctrl();
labels = nombres_ctrl();
M = struct([]);

fig=figure('Color','w','Position',[40 40 1450 860]);
ax1=subplot('Position',[0.07 0.57 0.58 0.34]); hold(ax1,'on'); grid(ax1,'on');
ax2=subplot('Position',[0.07 0.13 0.58 0.34]); hold(ax2,'on'); grid(ax2,'on');

for i=1:numel(ctrls)
    slug=ctrls{i}; c=colors.(slug);
    [t_s,th_s,u_s,estable_s]=sim_ctrl(slug);
    [t_h,th_h,u_h,csvname]=cargar_hw(slug,resdir);
    plot(ax1,t_s,th_s,'LineWidth',1.45,'Color',c,'DisplayName',labels.(slug));
    if ~isempty(th_h)
        plot(ax2,t_h,th_h,'LineWidth',1.25,'Color',c,'DisplayName',labels.(slug));
    end
    ms=metricas(th_s,u_s,estable_s); mh=metricas(th_h,u_h,[]);
    M(i).slug=slug; M(i).nombre=labels.(slug); M(i).sim=ms; M(i).hw=mh; M(i).csv=csvname; %#ok<AGROW>
end

yline(ax1,0,'--k','HandleVisibility','off'); yline(ax2,0,'--k','HandleVisibility','off');
xlabel(ax1,'t [s]'); ylabel(ax1,'\theta [deg]'); title(ax1,'Simulacion: todos los controladores superpuestos');
xlabel(ax2,'t [s]'); ylabel(ax2,'\theta [deg]'); title(ax2,'Hardware: datos experimentales superpuestos');
legend(ax1,'Location','eastoutside','FontSize',8); legend(ax2,'Location','eastoutside','FontSize',8);

% Paneles de metricas como texto monoespaciado: visibles sin crear mas graficas.
simtxt = sprintf('METRICAS SIMULACION\n%-12s %8s %8s %8s %8s\n','Ctrl','RMS','MaxTh','MaxU','Est');
hwtxt  = sprintf('METRICAS HARDWARE\n%-12s %8s %8s %8s %8s\n','Ctrl','RMS','MaxTh','MaxU','Sat%');
for i=1:numel(M)
    simtxt = [simtxt sprintf('%-12s %8.3f %8.2f %8.0f %8d\n',M(i).slug,M(i).sim.thetaRMS,M(i).sim.thetaMax,M(i).sim.uMax,M(i).sim.estable)]; %#ok<AGROW>
    if isnan(M(i).hw.thetaRMS)
        hwtxt = [hwtxt sprintf('%-12s %8s %8s %8s %8s\n',M(i).slug,'--','--','--','--')]; %#ok<AGROW>
    else
        hwtxt = [hwtxt sprintf('%-12s %8.3f %8.2f %8.0f %8.1f\n',M(i).slug,M(i).hw.thetaRMS,M(i).hw.thetaMax,M(i).hw.uMax,M(i).hw.satPct)]; %#ok<AGROW>
    end
end
annotation(fig,'textbox',[0.70 0.55 0.28 0.36],'String',simtxt,'FontName','Consolas','FontSize',9,...
    'EdgeColor',[0.25 0.25 0.25],'BackgroundColor',[1 1 1],'FitBoxToText','off');
annotation(fig,'textbox',[0.70 0.12 0.28 0.36],'String',hwtxt,'FontName','Consolas','FontSize',9,...
    'EdgeColor',[0.25 0.25 0.25],'BackgroundColor',[1 1 1],'FitBoxToText','off');
sgtitle('Comparativa automatizada SIM vs Hardware - figura para IEEE');

if ~exist(resdir,'dir'), mkdir(resdir); end
saveas(fig,fullfile(resdir,'comparativa_ieee_matlab.png'));
try
    exportgraphics(fig,fullfile(resdir,'comparativa_ieee_matlab.pdf'),'ContentType','vector');
catch
end
escribir_tabla_csv(M,fullfile(resdir,'tabla_comparativa_matlab.csv'));
fprintf('Guardado: %s\n',fullfile(resdir,'comparativa_ieee_matlab.png'));
fprintf('Guardado: %s\n',fullfile(resdir,'tabla_comparativa_matlab.csv'));
end

% =====================================================================
function [t,th_deg,u_out,estable]=sim_ctrl(slug)
% Simula el lazo cerrado discreto (dt=10ms) desde theta0=5 deg.
R2D=180/pi; dt=0.01; Tf=4; N=round(Tf/dt); estable=true;
A=[0 1 0 0; 0 0 -7.4697 0; 0 0 0 1; 0 0 104.6806 0];
B=[0;0.1979;0;-1.4869]*(0.00054/0.00338);      % B REAL usado en hardware
sysd=c2d(ss(A,B,eye(4),zeros(4,1)),dt,'zoh'); Ad=sysd.A; Bd=sysd.B;

% --- coeficientes/controladores ---
alpha=10.2314; k2=-20.33; kc=1.19; CASGAIN=4;             % cascada
Klqg=[-70.71 -196.97 -1985.22 -284.52];                  % lqg
[num,den]=coef_hinf_mixsyn(A,B,dt);                         % H-inf mixsyn Ts=0.01
L=64; lam=0.95; mu=0.15; cI=zeros(1,L); cD=zeros(1,L); cI(1)=1; cD(1)=1;
for j=2:L, cI(j)=cI(j-1)*(1-(-lam+1)/(j-1)); cD(j)=cD(j-1)*(1-(mu+1)/(j-1)); end
hlam=dt^lam; hmu=dt^-mu; ebuf=zeros(1,L);
I1=0; u_imc=0; ehist=zeros(1,5); uhist=zeros(1,5);

x=[0;0;5/R2D;0]; th_deg=zeros(1,N); u_out=zeros(1,N);
for kk=1:N
   th=x(3); thd=x(4);
   switch lower(slug)
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
        uc=(num*ehist' - den(2:end)*uhist(1:end-1)')/den(1);
        uhist=[uc uhist(1:end-1)]; u=0.10*uc;
     otherwise
        error('controlador desconocido: %s',slug);
   end
   u=max(min(u,255),-255);
   th_deg(kk)=th*R2D; u_out(kk)=u;
   x=Ad*x+Bd*u;
   if abs(x(3)*R2D)>60
       estable=false; th_deg=th_deg(1:kk); u_out=u_out(1:kk); break;
   end
end
t=(0:numel(th_deg)-1)*dt;
end

% =====================================================================
function [num,den]=coef_hinf_mixsyn(A,B,dt)
% Sintetiza el controlador H-infinito con los ponderadores pedidos.
% Si MATLAB no tiene Robust Control Toolbox, usa los coeficientes ya validados
% para que la comparativa siga siendo reproducible.
persistent NUM DEN
if ~isempty(NUM)
    num=NUM; den=DEN; return;
end
try
    a43=104.6806; b4=B(4);
    s=tf('s');
    G=b4/(s^2-a43);                         % subsistema angulo inestable
    W1=makeweight(200,60,0.05);             % S: desempeno
    W2=1e-3;                                % KS: esfuerzo
    W3=makeweight(0.20,40,2);               % T: robustez
    [K,~,gamma]=mixsyn(G,W1,W2,W3);         %#ok<ASGLU>
    Kd=c2d(K,dt,'tustin');
    [num,den]=tfdata(Kd,'v');
    num=real(num); den=real(den);
    num=num/den(1); den=den/den(1);
    fprintf('H-inf mixsyn recalculado: gamma=%.3f\n',gamma);
catch ME
    warning('No se pudo recalcular H-inf con mixsyn (%s). Uso coeficientes guardados.',ME.message);
    num=[-9644.251425 14377.850121 4271.651618 -14402.416908 5348.033021];
    den=[1 -1.024125 -0.727214 1.035496 -0.261415];
end
NUM=num; DEN=den;
end

% =====================================================================
function [t_h,th_h,u_h,csvname]=cargar_hw(slug,resdir)
d = dir(fullfile(resdir,[slug '_2*.csv']));
t_h=[]; th_h=[]; u_h=[]; csvname='';
if isempty(d), return; end
[~,ix]=max([d.datenum]); csv=fullfile(resdir,d(ix).name); csvname=d(ix).name;
T=readtable(csv); D=T{:,:};
t_h=(D(:,1)-D(1,1))/1000; th_h=D(:,2); u_h=D(:,6);
end

% =====================================================================
function m=metricas(th,u,estable)
if isempty(th)
    m.thetaRMS=nan; m.thetaMax=nan; m.uMax=nan; m.satPct=nan; m.estable=nan; return;
end
m.thetaRMS=sqrt(mean(th.^2));
m.thetaMax=max(abs(th));
m.uMax=max(abs(u));
m.satPct=100*mean(abs(u)>=254);
if isempty(estable), m.estable=~any(abs(th)>35); else, m.estable=estable; end
end

% =====================================================================
function escribir_tabla_csv(M,pathout)
fid=fopen(pathout,'w');
fprintf(fid,'controlador,thetaRMS_sim,thetaMax_sim,uMax_sim,estable_sim,thetaRMS_hw,thetaMax_hw,uMax_hw,satPct_hw,csv_hw\n');
for i=1:numel(M)
    fprintf(fid,'%s,%.6g,%.6g,%.6g,%d,%.6g,%.6g,%.6g,%.6g,%s\n',...
        M(i).slug,M(i).sim.thetaRMS,M(i).sim.thetaMax,M(i).sim.uMax,M(i).sim.estable,...
        M(i).hw.thetaRMS,M(i).hw.thetaMax,M(i).hw.uMax,M(i).hw.satPct,M(i).csv);
end
fclose(fid);
end

% =====================================================================
function c=colores_ctrl()
c.lqr     =[0.1216 0.4667 0.7059];
c.lqg     =[1.0000 0.4980 0.0549];
c.cascada =[0.0902 0.7451 0.8118];
c.fopid   =[0.1725 0.6275 0.1725];
c.imc     =[0.5804 0.4039 0.7412];
c.hinf    =[0.5490 0.3373 0.2941];
end

% =====================================================================
function n=nombres_ctrl()
n.lqr='LQR'; n.lqg='LQG'; n.cascada='Cascada'; n.fopid='FOPID'; n.imc='IMC'; n.hinf='H-infinito';
end
