function logger(port, seg, out)
% Extrae telemetria del robot por serial y guarda CSV + grafica.
% Firmware imprime:  t_ms,theta,theta_d,x,pwm
% Uso:  logger("/dev/ttyUSB0", 20, "data/run.csv")
if nargin<1, port="/dev/ttyUSB0"; end
if nargin<2, seg=20; end
if nargin<3, out="data/run.csv"; end
s=serialport(port,115200); configureTerminator(s,"LF"); flush(s);
fprintf("Grabando %g s ...\n",seg);
D=[]; t0=tic;
while toc(t0)<seg
    ln=strtrim(readline(s));
    if startsWith(ln,"#")||startsWith(ln,">>")||~contains(ln,","), continue; end
    v=str2double(split(ln,","));
    if numel(v)>=5 && all(~isnan(v(1:5))), D=[D; v(1:5)']; end
end
clear s;
T=array2table(D,'VariableNames',{'t_ms','theta','theta_d','x','pwm'});
writetable(T,out); fprintf("%d muestras -> %s\n",size(D,1),out);
t=(D(:,1)-D(1,1))/1000;
fprintf("theta std=%.2f  |max|=%.2f  | pwm |max|=%.0f\n",std(D(:,2)),max(abs(D(:,2))),max(abs(D(:,5))));
figure; subplot(3,1,1);plot(t,D(:,2));ylabel('\theta[deg]');grid on;
subplot(3,1,2);plot(t,D(:,4));ylabel('x[m]');grid on;
subplot(3,1,3);plot(t,D(:,5));ylabel('pwm');xlabel('t[s]');grid on;
end
