#!/usr/bin/env python3
"""
============================================================
FASE 2 - Análisis de oscilación libre del PÉNDULO
============================================================
Procesa el CSV del firmware exp2_pendulo y extrae:
  - Periodo natural T (de los cruces por cero de gy)
  - Frecuencia natural ω_n = 2π/T
  - Inercia I_p = M*g*l*(T/2π)²
  - Amortiguamiento ζ (decremento logarítmico, bonus)

Uso:
  python3 analizar_pendulo.py --port /dev/ttyACM0 --duracion 15 --M 0.45 --l 0.09
  python3 analizar_pendulo.py --analizar data/pendulo.csv --M 0.45 --l 0.09
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

Ts = 0.010
g  = 9.81
DATA_DIR  = Path(__file__).parent.parent / "data"
PLOTS_DIR = Path(__file__).parent.parent / "plots"
DATA_DIR.mkdir(exist_ok=True)
PLOTS_DIR.mkdir(exist_ok=True)


# ══════════════════════════════════════════════════════════════════
def cargar_csv(ruta):
    arr = np.loadtxt(ruta, delimiter=",", skiprows=1, comments="#")
    return {
        "t_s":      arr[:, 0] / 1000.0,
        "gy_dps":   arr[:, 1],
        "ang_deg":  arr[:, 2],
    }


def _filtrar(gy, fs):
    """Filtro pasa-bajos para eliminar el ruido del giroscopio."""
    try:
        from scipy.signal import butter, filtfilt
        # corte a 5 Hz (las oscilaciones del péndulo son < 3 Hz)
        fc = 5.0
        b, a = butter(2, fc / (fs / 2.0), btype="low")
        return filtfilt(b, a, gy)
    except Exception:
        # respaldo: media móvil
        k = 7
        return np.convolve(gy, np.ones(k) / k, mode="same")


def periodo_por_fft(t, gy):
    """Periodo dominante por FFT (robusto al ruido)."""
    gy = gy - np.mean(gy)
    n  = len(gy)
    dt = np.mean(np.diff(t))
    fft = np.abs(np.fft.rfft(gy * np.hanning(n)))
    freqs = np.fft.rfftfreq(n, dt)
    # ignorar DC y frecuencias muy bajas (< 0.3 Hz)
    valido = freqs > 0.3
    if not np.any(valido):
        return None
    f_dom = freqs[valido][np.argmax(fft[valido])]
    return 1.0 / f_dom if f_dom > 0 else None


def medir_periodo(t, gy):
    """
    Periodo robusto: filtra el ruido, detecta cruces por cero ascendentes
    con HISTÉRESIS (exige que la señal supere un umbral entre cruces),
    y verifica contra la FFT. Evita los cruces falsos por ruido.
    """
    gy = gy - np.mean(gy)
    fs = 1.0 / np.mean(np.diff(t))
    gyf = _filtrar(gy, fs)

    # umbral de histéresis: 20% de la amplitud típica
    amp = np.percentile(np.abs(gyf), 90)
    umbral = 0.2 * amp

    # cruces por cero ascendentes con histéresis:
    # solo cuenta si la señal bajó de -umbral antes de subir sobre +umbral
    cruces = []
    armado = False
    for i in range(1, len(gyf)):
        if gyf[i] < -umbral:
            armado = True
        if armado and gyf[i-1] < 0 and gyf[i] >= 0:
            cruces.append(i)
            armado = False
    cruces = np.array(cruces)

    if len(cruces) < 2:
        return None, None, cruces

    t_cruces = t[cruces]
    periodos = np.diff(t_cruces)
    # descartar periodos atípicos (outliers por ruido residual)
    med = np.median(periodos)
    buenos = periodos[(periodos > 0.5 * med) & (periodos < 1.5 * med)]
    T = np.mean(buenos) if len(buenos) else np.mean(periodos)

    # verificación cruzada con FFT
    T_fft = periodo_por_fft(t, gy)
    if T_fft is not None and abs(T - T_fft) / T_fft > 0.25:
        print(f"  [AVISO] Cruces dan T={T:.3f}s pero FFT da T={T_fft:.3f}s.")
        print(f"          Usando FFT (más robusto al ruido).")
        T = T_fft

    return T, periodos, cruces


def medir_amortiguamiento(t, gy, cruces):
    """Decremento logarítmico sobre la envolvente de picos."""
    gy = gy - np.mean(gy)
    # amplitud de cada medio-ciclo: máximo entre cruces consecutivos
    picos = []
    for i in range(len(cruces) - 1):
        seg = np.abs(gy[cruces[i]:cruces[i+1]])
        if len(seg):
            picos.append(seg.max())
    picos = np.array(picos)
    if len(picos) < 3:
        return None, None
    # decremento logarítmico promedio
    n = len(picos) - 1
    A0, An = picos[0], picos[-1]
    if An <= 0 or A0 <= 0:
        return None, None
    delta = (1.0 / n) * np.log(A0 / An)
    zeta  = delta / np.sqrt(4 * np.pi**2 + delta**2)
    return delta, zeta


# ══════════════════════════════════════════════════════════════════
def analizar(d, M, l):
    print("\n" + "═"*55)
    print("  FASE 2 — OSCILACIÓN LIBRE DEL PÉNDULO")
    print("═"*55)
    t  = d["t_s"]
    gy = d["gy_dps"]

    T, periodos, cruces = medir_periodo(t, gy)
    if T is None:
        print("  [ERROR] no se detectaron suficientes oscilaciones.")
        print("          Asegúrate de soltar el cuerpo y capturar varios ciclos.")
        return None

    n_ciclos = len(periodos)
    omega_n  = 2.0 * np.pi / T
    print(f"  Ciclos detectados : {n_ciclos}")
    print(f"  Periodo T         : {T:.4f} s  (±{np.std(periodos):.4f})")
    print(f"  Frec. natural ω_n : {omega_n:.4f} rad/s")
    print(f"  Polo inestable    : ±{omega_n:.4f} rad/s  (versión base fija)")
    print(f"  τ_caída           : {1.0/omega_n:.4f} s")

    # --- VALIDACIÓN FÍSICA ---
    if l:
        T_min = 2 * np.pi * np.sqrt(l / g)   # periodo mínimo físico (masa puntual)
        if T < T_min:
            print("\n  " + "!"*50)
            print(f"  [RESULTADO IMPOSIBLE] T={T:.3f}s < T_min={T_min:.3f}s")
            print(f"  Un péndulo con l={l}m NO puede oscilar más rápido que {T_min:.3f}s.")
            print(f"  Causas probables:")
            print(f"    - El detector contó cruces falsos por ruido (revisa la gráfica)")
            print(f"    - El pivote tiene fricción del reductor (¿usaste el eje del motor?)")
            print(f"    - l real distinto al medido")
            print(f"  NO uses este I_p. Repite con pivote libre y suelta limpio.")
            print("  " + "!"*50)

    # Amortiguamiento
    delta, zeta = medir_amortiguamiento(t, gy, cruces)
    if zeta is not None:
        print(f"  ζ (decr. log.)    : {zeta:.4f}  "
              f"[OJO: válido sólo si fricción viscosa]")
        omega_n_corr = omega_n / np.sqrt(1 - zeta**2) if zeta < 1 else omega_n
        print(f"  ω_n corregida     : {omega_n_corr:.4f} rad/s")

    # Inercia I_p
    if M and l:
        I_p = M * g * l * (T / (2 * np.pi))**2
        print("\n  " + "-"*45)
        print(f"  Con M={M} kg, l={l} m:")
        print(f"  I_p = M·g·l·(T/2π)² = {I_p:.6f} kg·m²")
        # verificación por ejes paralelos
        I_cm = I_p - M * l**2
        print(f"  I_cm (ejes paralelos) = {I_cm:.6f} kg·m²")
        print(f"  (compara I_cm con tu estimación de CAD)")
    else:
        I_p = None
        print("\n  [Para I_p pasa --M (masa cuerpo) y --l (eje→CM)]")

    # Guardar reporte
    rep = DATA_DIR / f"pendulo_reporte_{datetime.now():%Y%m%d_%H%M%S}.txt"
    with open(rep, "w") as f:
        f.write("FASE 2 - OSCILACION LIBRE DEL PENDULO\n")
        f.write(f"Generado: {datetime.now()}\n\n")
        f.write(f"T = {T:.6f} s\nomega_n = {omega_n:.6f} rad/s\n")
        if I_p:
            f.write(f"M = {M} kg\nl = {l} m\nI_p = {I_p:.6f} kg*m^2\n")
    print(f"\n[Reporte] {rep}")
    return I_p


def graficar(d, cruces=None, titulo=""):
    if not PLOT_OK:
        return
    t = d["t_s"]; gy = d["gy_dps"] - np.mean(d["gy_dps"])
    fig, ax = plt.subplots(2, 1, figsize=(11, 7), sharex=True)
    ax[0].plot(t, gy, color="tab:blue", label="gy (sin bias)")
    if cruces is not None and len(cruces):
        ax[0].plot(t[cruces], gy[cruces], "r.", label="cruces por 0")
    ax[0].axhline(0, color="k", lw=0.5); ax[0].set_ylabel("θ̇ [°/s]")
    ax[0].legend(fontsize=8); ax[0].grid(alpha=0.3)
    ax[0].set_title(f"Oscilación libre del péndulo\n{titulo}")
    ax[1].plot(t, d["ang_deg"], color="tab:orange")
    ax[1].set_ylabel("ángulo accel [°]"); ax[1].set_xlabel("t [s]")
    ax[1].grid(alpha=0.3)
    plt.tight_layout()
    ruta = PLOTS_DIR / f"pendulo_{datetime.now():%Y%m%d_%H%M%S}.png"
    plt.savefig(ruta, dpi=150)
    print(f"[Gráfica] {ruta}")
    plt.show()


# ══════════════════════════════════════════════════════════════════
def capturar(port, baud, duracion):
    if not SERIAL_OK:
        print("[ERROR] pyserial no instalado"); sys.exit(1)
    ser = serial.Serial(port, baud, timeout=1.0)
    time.sleep(2.0)
    ser.reset_input_buffer()
    print(f"[OK] {port}")
    print(">>> Suelta el cuerpo desde 5-8° AHORA <<<")
    ser.write(b"S\n")
    rows = []
    t_end = time.time() + duracion
    while time.time() < t_end:
        linea = ser.readline().decode("utf-8", "ignore").strip()
        if not linea or linea.startswith("#") or linea.startswith("t_ms"):
            continue
        p = linea.split(",")
        if len(p) == 3:
            try:
                rows.append([float(x) for x in p])
            except ValueError:
                pass
    ser.write(b"T\n")
    ser.close()
    if not rows:
        print("[ERROR] sin datos"); sys.exit(1)
    arr = np.array(rows)
    ruta = DATA_DIR / f"pendulo_{datetime.now():%Y%m%d_%H%M%S}.csv"
    with open(ruta, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["t_ms", "gy_dps", "ang_acc_deg"])
        w.writerows(arr)
    print(f"[Guardado] {ruta}  ({len(rows)} muestras)")
    return str(ruta)


# ══════════════════════════════════════════════════════════════════
def main():
    ap = argparse.ArgumentParser(description="Análisis oscilación péndulo")
    ap.add_argument("--port")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--duracion", type=float, default=15)
    ap.add_argument("--analizar")
    ap.add_argument("--M", type=float, help="masa del cuerpo [kg]")
    ap.add_argument("--l", type=float, help="distancia eje→CM [m]")
    ap.add_argument("--puertos", action="store_true")
    args = ap.parse_args()

    if args.puertos:
        for p in serial.tools.list_ports.comports():
            print(f"  {p.device:18s} {p.description}")
        return

    if args.analizar:
        d = cargar_csv(args.analizar)
        _, _, cruces = medir_periodo(d["t_s"], d["gy_dps"])
        analizar(d, args.M, args.l)
        graficar(d, cruces, args.analizar)
        return

    if not args.port:
        print("[ERROR] usa --port o --analizar")
        return

    ruta = capturar(args.port, args.baud, args.duracion)
    d = cargar_csv(ruta)
    _, _, cruces = medir_periodo(d["t_s"], d["gy_dps"])
    analizar(d, args.M, args.l)
    graficar(d, cruces, ruta)


if __name__ == "__main__":
    main()
