# Comandos completos en Windows para ejecutar la comparativa MATLAB

> Estos comandos NO suben nada a `main`. Trabajan en una rama local y ejecutan MATLAB
> desde tu carpeta `C:\A_CURSOS20261\CONTROL\plant_ide`.

## 1) Preparar tu copia local

```powershell
cd C:\A_CURSOS20261\CONTROL\plant_ide
git checkout main
git pull origin main
git checkout -B mi_comparativa_matlab
```

## 2) Verificar que están los archivos MATLAB

```powershell
Test-Path .\balancin\matlab\sim_hinf.m
Test-Path .\balancin\proyecto\matlab_comparativa\comparar.m
Test-Path .\balancin\proyecto\matlab_comparativa\comparar_todos.m
```

Los tres deben responder `True`.

## 3) Ejecutar la comparativa de todos los controladores desde PowerShell

Si `matlab` está en el PATH:

```powershell
matlab -batch "cd('C:\A_CURSOS20261\CONTROL\plant_ide\balancin\proyecto\matlab_comparativa'); comparar('todos');"
```

Si `matlab` NO está en el PATH, usa la ruta completa, ajustando la versión si corresponde:

```powershell
& "C:\Program Files\MATLAB\R2024b\bin\matlab.exe" -batch "cd('C:\A_CURSOS20261\CONTROL\plant_ide\balancin\proyecto\matlab_comparativa'); comparar('todos');"
```

## 4) Ejecutar solo el diseño H-infinito

```powershell
matlab -batch "cd('C:\A_CURSOS20261\CONTROL\plant_ide\balancin\matlab'); sim_hinf;"
```

## 5) Archivos de salida esperados

```powershell
dir .\balancin\proyecto\resultados\comparativa_ieee_matlab.*
dir .\balancin\proyecto\resultados\tabla_comparativa_matlab.csv
```

La figura principal para el paper es:

```text
balancin\proyecto\resultados\comparativa_ieee_matlab.png
```
