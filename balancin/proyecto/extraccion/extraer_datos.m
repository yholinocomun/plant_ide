function extraer_datos(controlador, port, seg)
% Extraccion UNIFORME de data por serial.  Firmware imprime (tecla 't'):
%  t_ms,theta_deg,theta_dot_dps,x_m,x_dot_ms,u_pwm,setpoint_deg,modo
% Uso:  extraer_datos("lqr","/dev/ttyUSB0",25)
if nargin<2, port="/dev/ttyUSB0"; end
if nargin<3, seg=25; end
s=serialport(port,115200); configureTerminator(s,"LF"); flush(s);
fprintf("[%s] grabando %g s ... activa control (space) y telemetria (t)\n",controlador,seg);
D=[]; t0=tic;
while toc(t0)<seg
  ln=strtrim(readline(s));
  if startsWith(ln,"#")||startsWith(ln,">>")||~contains(ln,","), continue; end
  v=str2double(split(ln,","));
  if numel(v)>=8 && all(~isnan(v(1:8))), D=[D; v(1:8)']; end
end
clear s;
stamp=string(datetime("now","Format","yyyyMMdd_HHmmss"));
base=fullfile("..","resultados",controlador+"_"+stamp);
T=array2table(D,'VariableNames',{'t_ms','theta_deg','theta_dot_dps','x_m','x_dot_ms','u_pwm','setpoint_deg','modo'});
writetable(T,base+".csv");
m=metricas(D); m.controlador=controlador;
fprintf("%d muestras -> %s.csv\n",size(D,1),base);
disp(m);
end
