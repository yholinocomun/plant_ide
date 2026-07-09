function graficar(csvfile)
% Figura ESTANDAR (MATLAB) de un controlador.  Mismo formato para los 6.
T=readtable(csvfile); D=T{:,:}; t=(D(:,1)-D(1,1))/1000; m=metricas(D);
[~,name,~]=fileparts(csvfile);
figure('Position',[100 100 800 640],'Color','w');
subplot(3,1,1); plot(t,D(:,2),'LineWidth',1.4); grid on; ylabel('\theta [deg]');
title(strrep(name,'_','\_'),'FontWeight','bold'); yline(0,'--k');
text(0.99,0.05,sprintf('\\thetaRMS=%.2f°  |u|max=%.0f  sat=%.1f%%',...
     m.theta_RMS_deg,m.u_max_abs_pwm,m.saturacion_pct),'Units','normalized','HorizontalAlignment','right');
subplot(3,1,2); plot(t,D(:,4),'LineWidth',1.4); grid on; ylabel('x [m]');
subplot(3,1,3); plot(t,D(:,6),'LineWidth',1.4); grid on; ylabel('u [PWM]'); xlabel('t [s]');
yline(255,':r'); yline(-255,':r');
saveas(gcf, strrep(csvfile,'.csv','.png'));
end
