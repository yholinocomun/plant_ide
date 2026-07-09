% Corre la comparativa sim-vs-hardware para los 6 controladores.
ctrls = {'lqr','lqg','cascada','fopid','imc','hinf'};
for i=1:numel(ctrls)
    comparar(ctrls{i});
end
disp('Listo: <slug>_sim.png y <slug>_comparativa.png para cada controlador.');
