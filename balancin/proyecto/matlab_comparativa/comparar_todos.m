% Genera la comparativa sim-vs-hardware de los controladores (excepto FOPID,
% que ya esta hecho). Imprime metricas en consola y guarda <slug>_comparativa.png
ctrls = {'lqr','lqg','cascada','imc','hinf'};
for i=1:numel(ctrls), comparar(ctrls{i}); end
disp('Listo. Copia lo impreso en consola para armar la tabla.');
