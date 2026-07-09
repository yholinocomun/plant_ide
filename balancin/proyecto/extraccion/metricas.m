function m=metricas(D)
t=(D(:,1)-D(1,1))/1000; th=D(:,2); u=D(:,6); mask=t>0.4*t(end);
m.duracion_s=round(t(end),2);
m.theta_RMS_deg=round(sqrt(mean(th.^2)),3);
m.theta_std_deg=round(std(th(mask)),3);
m.theta_max_abs_deg=round(max(abs(th)),3);
m.u_max_abs_pwm=round(max(abs(u)),1);
m.saturacion_pct=round(100*mean(abs(u)>=254),1);
m.deriva_x_m=round(D(end,4)-D(1,4),3);
end
