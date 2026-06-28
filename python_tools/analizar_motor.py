#!/usr/bin/env python3
"""
============================================================
FASE 1 - Análisis de identificación del MOTOR
============================================================
Procesa el CSV del firmware exp1_motor y extrae:
  - Zona muerta (u_dead) de la rampa
  - Ganancia K de la escalera
  - Constante de tiempo tau del escalón

Modelo objetivo:  G(s) = K / (tau*s + 1)

Uso:
  # Capturar en vivo desde el ESP32:
  python3 analizar_motor.py --port /dev/ttyACM0 --exp B --duracion 30

  # Analizar un CSV ya guardado:
  python3 analizar_motor.py --analizar data/motor_escalera.csv --tipo escalera
============================================================
"""

import argparse
import csv
import time
import sys
from pathlib import Path
from datetime import datetime

import numpy as np

try:
    import serial
    import serial.tools.list_ports
    SERIAL_OK = True
except ImportError:
    SERIAL_OK = False

try:
    import matplotlib
    matplotlib.use("TkAgg")
    import matplotlib.pyplot as plt
    PLOT_OK = True
except ImportError:
    PLOT_OK = False

try:
    from scipy.optimize import curve_fit
    SCIPY_OK = True
except ImportError:
    SCIPY_OK = False

# ── Parámetros del hardware ───────────────────────────────────────
PPR = 1945.0       # pulsos por revolución de rueda
Ts  = 0.010        # tiempo de muestreo [s]

DATA_DIR  = Path(__file__).parent.parent / "data"
PLOTS_DIR = Path(__file__).parent.parent / "plots"
DATA_DIR.mkdir(exist_ok=True)
PLOTS_DIR.mkdir(exist_ok=True)


# ══════════════════════════════════════════════════════════════════
def pulsos_a_omega(enc):
    """Convierte conteo de encoder a velocidad angular [rad/s]."""
    d = np.diff(enc, prepend=enc[0])
    return d / Ts * (2.0 * np.pi / PPR)


def cargar_csv(ruta):
    arr = np.loadtxt(ruta, delimiter=",", skiprows=1, comments="#")
    return {
        "t_s":     arr[:, 0] / 1000.0,
        "pwm":     arr[:, 1],
        "enc_izq": arr[:, 2],
        "enc_der": arr[:, 3],
    }


# ══════════════════════════════════════════════════════════════════
def analizar_rampa(d):
    """Zona muerta: primer PWM que produce movimiento sostenido."""
    print("\n" + "═"*55)
    print("  SUB-EXP A — ZONA MUERTA (rampa)")
    print("═"*55)
    w = pulsos_a_omega(d["enc_izq"])
    pwm = d["pwm"]

    u_dead_fwd = None
    u_dead_bwd = None
    for nivel in np.unique(pwm):
        mask = pwm == nivel
        w_nivel = np.abs(w[mask]).mean()
        if nivel > 0 and w_nivel > 0.3 and u_dead_fwd is None:
            u_dead_fwd = nivel
        if nivel < 0 and w_nivel > 0.3 and u_dead_bwd is None:
            u_dead_bwd = abs(nivel)

    print(f"  u_dead adelante : {u_dead_fwd} PWM")
    print(f"  u_dead reversa  : {u_dead_bwd} PWM")
    print(f"  Zona muerta     : [-{u_dead_bwd}, +{u_dead_fwd}]")
    return u_dead_fwd, u_dead_bwd


def analizar_escalera(d):
    """Ganancia K por nivel y linealidad."""
    print("\n" + "═"*55)
    print("  SUB-EXP B — GANANCIA K (escalera)")
    print("═"*55)
    w = pulsos_a_omega(d["enc_izq"])
    pwm = d["pwm"]

    print(f"  {'PWM':>6} {'w_ss[rad/s]':>14} {'K[rad/s/PWM]':>16}")
    print("  " + "-"*40)
    niveles, Ks = [], []
    for nivel in np.unique(pwm):
        if nivel == 0:
            continue
        mask = pwm == nivel
        idx = np.where(mask)[0]
        if len(idx) < 30:
            continue
        # promedio del último tercio (régimen permanente)
        w_ss = w[idx[-len(idx)//3:]].mean()
        K = w_ss / nivel
        niveles.append(nivel); Ks.append(K)
        print(f"  {int(nivel):>6} {w_ss:>14.3f} {K:>16.5f}")

    if Ks:
        K_prom = np.mean(np.abs(Ks))
        print("  " + "-"*40)
        print(f"  K promedio      : {K_prom:.5f} rad/s/PWM")
        # asimetría
        fwd = [k for n, k in zip(niveles, Ks) if n > 0]
        bwd = [k for n, k in zip(niveles, Ks) if n < 0]
        if fwd and bwd:
            asim = np.mean(np.abs(fwd)) / np.mean(np.abs(bwd))
            print(f"  Asimetria f/b   : {asim:.3f}  (ideal=1.0)")
        return K_prom
    return None


def analizar_escalon(d):
    """Constante de tiempo tau por 63.2% y por ajuste exponencial."""
    print("\n" + "═"*55)
    print("  SUB-EXP C — CONSTANTE DE TIEMPO tau (escalón)")
    print("═"*55)
    w   = pulsos_a_omega(d["enc_izq"])
    pwm = d["pwm"]
    t   = d["t_s"]

    # localizar el escalón (donde pwm pasa a no-cero)
    idx_step = np.argmax(pwm != 0)
    t0   = t[idx_step]
    tt   = t[idx_step:] - t0
    ww   = w[idx_step:]
    nivel = pwm[idx_step]

    w_ss = ww[-len(ww)//4:].mean()
    K = w_ss / nivel

    # Método 63.2%
    w_63 = 0.632 * w_ss
    cruce = np.where(ww >= w_63)[0]
    tau_63 = tt[cruce[0]] if len(cruce) else float("nan")

    print(f"  PWM escalón     : {int(nivel)}")
    print(f"  w_ss            : {w_ss:.3f} rad/s")
    print(f"  K               : {K:.5f} rad/s/PWM")
    print(f"  tau (63.2%)     : {tau_63:.4f} s")

    # Método ajuste exponencial (robusto)
    tau_fit = tau_63
    if SCIPY_OK:
        def modelo(t, K_, tau_):
            return K_ * nivel * (1.0 - np.exp(-t / tau_))
        try:
            popt, _ = curve_fit(modelo, tt, ww, p0=[K, tau_63], maxfev=5000)
            K_fit, tau_fit = popt
            print(f"  tau (ajuste exp): {tau_fit:.4f} s   <- usar este")
            print(f"  K   (ajuste exp): {K_fit:.5f} rad/s/PWM")
        except Exception as e:
            print(f"  [ajuste exp falló: {e}]")

    print(f"\n  MODELO:  G(s) = {K:.5f} / ({tau_fit:.4f}*s + 1)")
    return K, tau_fit


# ══════════════════════════════════════════════════════════════════
def graficar(d, titulo=""):
    if not PLOT_OK:
        return
    w = pulsos_a_omega(d["enc_izq"])
    fig, ax = plt.subplots(2, 1, figsize=(11, 7), sharex=True)
    ax[0].plot(d["t_s"], w, color="tab:blue")
    ax[0].set_ylabel("ω rueda [rad/s]"); ax[0].grid(alpha=0.3)
    ax[0].set_title(f"Identificación motor\n{titulo}")
    ax[1].step(d["t_s"], d["pwm"], where="post", color="tab:red")
    ax[1].set_ylabel("PWM"); ax[1].set_xlabel("t [s]"); ax[1].grid(alpha=0.3)
    plt.tight_layout()
    ruta = PLOTS_DIR / f"motor_{datetime.now():%Y%m%d_%H%M%S}.png"
    plt.savefig(ruta, dpi=150)
    print(f"\n[Gráfica] {ruta}")
    plt.show()


# ══════════════════════════════════════════════════════════════════
def capturar(port, baud, exp, duracion):
    if not SERIAL_OK:
        print("[ERROR] pyserial no instalado"); sys.exit(1)
    ser = serial.Serial(port, baud, timeout=1.0)
    time.sleep(2.0)
    ser.reset_input_buffer()
    print(f"[OK] {port} @ {baud}")

    cmd = {"A": "A", "B": "B", "C": "C"}.get(exp.upper(), "B")
    ser.write((cmd + "\n").encode())
    print(f"[Enviado '{cmd}'] capturando {duracion}s...")

    rows = []
    t_end = time.time() + duracion
    while time.time() < t_end:
        linea = ser.readline().decode("utf-8", "ignore").strip()
        if not linea or linea.startswith("#") or linea.startswith("t_ms"):
            if linea.startswith("#"):
                print("  " + linea)
            continue
        p = linea.split(",")
        if len(p) == 4:
            try:
                rows.append([float(x) for x in p])
            except ValueError:
                pass
    ser.write(b"T\n")
    ser.close()

    if not rows:
        print("[ERROR] sin datos"); sys.exit(1)
    arr = np.array(rows)
    ruta = DATA_DIR / f"motor_{exp}_{datetime.now():%Y%m%d_%H%M%S}.csv"
    with open(ruta, "w", newline="") as f:
        wcsv = csv.writer(f)
        wcsv.writerow(["t_ms", "pwm", "enc_izq", "enc_der"])
        wcsv.writerows(arr)
    print(f"[Guardado] {ruta}  ({len(rows)} muestras)")
    return str(ruta)


# ══════════════════════════════════════════════════════════════════
def main():
    ap = argparse.ArgumentParser(description="Análisis identificación motor")
    ap.add_argument("--port")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--exp", choices=["A", "B", "C"], default="B",
                    help="A=rampa(zona muerta) B=escalera(K) C=escalon(tau)")
    ap.add_argument("--duracion", type=float, default=30)
    ap.add_argument("--analizar", help="CSV existente a analizar")
    ap.add_argument("--tipo", choices=["rampa", "escalera", "escalon"],
                    help="tipo de análisis para --analizar")
    ap.add_argument("--puertos", action="store_true")
    args = ap.parse_args()

    if args.puertos:
        for p in serial.tools.list_ports.comports():
            print(f"  {p.device:18s} {p.description}")
        return

    if args.analizar:
        d = cargar_csv(args.analizar)
        tipo = args.tipo or {"A": "rampa", "B": "escalera", "C": "escalon"}[args.exp]
        if tipo == "rampa":    analizar_rampa(d)
        elif tipo == "escalera": analizar_escalera(d)
        elif tipo == "escalon":  analizar_escalon(d)
        graficar(d, args.analizar)
        return

    if not args.port:
        print("[ERROR] usa --port o --analizar  (--puertos para listar)")
        return

    ruta = capturar(args.port, args.baud, args.exp, args.duracion)
    d = cargar_csv(ruta)
    if args.exp == "A":   analizar_rampa(d)
    elif args.exp == "B": analizar_escalera(d)
    elif args.exp == "C": analizar_escalon(d)
    graficar(d, ruta)


if __name__ == "__main__":
    main()
