# Guía paso a paso — Toma de datos para identificar la planta

> Todo se hace desde la terminal de Ubuntu con Python.
> Tú solo flasheas el firmware con Arduino IDE, lo **cierras**, y corres un comando.

---

## Tus parámetros medidos (Fase 0)

| Símbolo | Valor | Qué es | Origen |
|---------|-------|--------|--------|
| M | 0.710 kg | masa del cuerpo (sin ruedas) | balanza |
| m_w | 0.095 kg | masa de una rueda | balanza |
| r | 0.037 m | radio de rueda | regla |
| l | 0.10 m | eje → centro de masa | equilibrio en filo |
| PPR | 1945 | pulsos por vuelta | 13 CPR × 2 × 74.8 |

Pendientes de medir en los experimentos:
- **K, τ, u_dead** → Fase 1 (motor)
- **I_p** → Fase 2 (péndulo)
- **J_w** → Fase 3 (calculado: ½·m_w·r², automático en el script)

---

## Concepto clave: el encoder solo cuenta pulsos

El encoder **NO mide posición ni velocidad**. Solo genera pulsos al girar, y el ESP32 los cuenta.

```
POSICIÓN  = pulsos convertidos a radianes (regla de tres)
            θ = pulsos × (2π / 1945)
            x = θ × r   (posición lineal del robot)

VELOCIDAD = cuántos pulsos nuevos llegan por unidad de tiempo
            ω = Δpulsos / Ts × (2π / 1945)
```

Por eso el firmware manda **pulsos crudos** y Python calcula posición y velocidad después.

---

## ANTES DE EMPEZAR — checklist

| # | Qué revisar | Cómo |
|---|-------------|------|
| 1 | Librerías Python | `pip install -r python_tools/requirements.txt` |
| 2 | Permiso del puerto | `sudo usermod -aG dialout $USER` → cerrar sesión |
| 3 | Arduino IDE **cerrado** | El puerto es de un solo programa a la vez |
| 4 | ESP32 conectado | `python3 python_tools/verificar_setup.py --puertos` |

### Verificación automática (corre esto primero)

```bash
# Con exp1_motor.ino flasheado y Arduino IDE cerrado:
python3 python_tools/verificar_setup.py --port /dev/ttyACM0
```
Esto comprueba librerías, puerto, respuesta del ESP32 y **te pide girar
una rueda a mano para confirmar que el encoder cuenta**.

---

## FASE 1 — Identificar el MOTOR

**Montaje:** robot apoyado en el suelo (ruedas rodando) o rueda al aire,
cuerpo sujeto con la mano. El MPU no se usa.

**Firmware:** flashea `firmware/exp1_motor/exp1_motor.ino`, cierra Arduino IDE.

```bash
# 1A) Zona muerta (rampa lenta de PWM)
python3 python_tools/analizar_motor.py --port /dev/ttyACM0 --exp A --duracion 40

# 1B) Ganancia K (escalera de PWM, ambos sentidos)
python3 python_tools/analizar_motor.py --port /dev/ttyACM0 --exp B --duracion 30

# 1C) Constante de tiempo tau (un escalón aislado)
python3 python_tools/analizar_motor.py --port /dev/ttyACM0 --exp C --duracion 5
```

Cada comando: abre el puerto, manda la señal, captura, guarda el CSV en
`data/`, imprime los resultados y muestra la gráfica. **No tocas nada más.**

**Resultado:** `K`, `τ`, `u_dead`.

---

## FASE 2 — Oscilación libre del PÉNDULO

**Montaje:** eje de ruedas **fijo en una prensa**, cuerpo colgando,
**motores apagados**, ruedas quitadas o libres. Todo el peso real montado.

**Firmware:** flashea `firmware/exp2_pendulo/exp2_pendulo.ino`, cierra Arduino IDE.

```bash
python3 python_tools/analizar_pendulo.py --port /dev/ttyACM0 --duracion 15 --M 0.710 --l 0.10
```

Cuando el script diga **">>> Suelta el cuerpo AHORA <<<"**, desplaza el
cuerpo 5–8° y suéltalo sin empujar. Déjalo oscilar.

> `--M` = masa del cuerpo [kg], `--l` = distancia eje→centro de masa [m].
> Mídelos antes (balanza + equilibrio en un filo).

**Resultado:** periodo `T` → inercia `I_p`.

---

## FASE 3 — Inercia de la rueda (J_w)

**No necesita firmware ni toma de datos.** `J_w` se calcula con una fórmula
a partir de la masa y el radio de la rueda (que ya mediste en Fase 0):

```
Rueda tipo DISCO macizo:  J_w = ½ · m_w · r²
Rueda tipo ARO:           J_w = m_w · r²
```

Con tus valores (disco): `J_w = 0.5 × 0.095 × 0.037² ≈ 0.000065 kg·m²`

Esto ya está automatizado dentro de `ensamblar_modelo.py` (con `"J_w": None`
lo calcula como disco). No tienes que hacer nada manual.

---

## FASE 4 — Ensamblar el espacio de estados

Edita el dict `PARAMS` en `python_tools/ensamblar_modelo.py` con TODOS los
valores que mediste (Fases 0, 1, 2, 3) y corre:

```bash
python3 python_tools/ensamblar_modelo.py
```

Imprime las matrices A, B, C, D, los valores propios (polo inestable),
y verifica controlabilidad y observabilidad.

---

## Orden completo resumido

```
0. verificar_setup.py                    ← comprobar que todo funciona
1. exp1_motor.ino  + analizar_motor.py   ← K, τ, u_dead   (3 capturas A/B/C)
2. exp2_pendulo.ino + analizar_pendulo.py← I_p            (1 captura)
3. (CAD/regla)                           ← J_w, r
4. ensamblar_modelo.py                   ← A, B, C, D
```

---

## Si algo falla

| Síntoma | Causa probable | Solución |
|---------|----------------|----------|
| `Permission denied` | Falta grupo dialout | `sudo usermod -aG dialout $USER` + relogin |
| `Device busy` | Arduino IDE abierto | Cierra Arduino IDE completo |
| No llegan datos | Firmware espera comando | Normal — el script manda el comando solo |
| Encoder no cuenta | Cableado A/B | Revisa pines del encoder |
| Pocas oscilaciones | Soltaste tarde/empujaste | Repite, suelta limpio desde 5-8° |
