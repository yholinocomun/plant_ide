#!/usr/bin/env python3
"""
Herramienta de adquisición de datos para identificación de planta
del Péndulo Invertido (Robot Balancín ESP32-S3).

Estado: x = [x, x_dot, theta, theta_dot]^T
Entrada: u = PWM (-255 a 255)

Uso:
    python3 collect_data.py --port /dev/ttyUSB0 --modo prbs --duracion 30
    python3 collect_data.py --port /dev/ttyUSB0 --modo escalon --duracion 20
    python3 collect_data.py --port /dev/ttyUSB0 --modo manual --pwm 100 --duracion 10

Después de capturar, analiza con:
    python3 collect_data.py --analizar data/experimento_XXXXXX.csv
"""

import argparse
import csv
import os
import sys
import time
import signal
import threading
from datetime import datetime
from pathlib import Path

import numpy as np

# ── Importaciones opcionales ──────────────────────────────────────────────────
try:
    import serial
    import serial.tools.list_ports
    SERIAL_OK = True
except ImportError:
    SERIAL_OK = False
    print("[AVISO] pyserial no instalado. Instala con:  pip install pyserial")

try:
    import matplotlib
    matplotlib.use("TkAgg")
    import matplotlib.pyplot as plt
    import matplotlib.animation as animation
    PLOT_OK = True
except ImportError:
    PLOT_OK = False
    print("[AVISO] matplotlib no disponible. Los gráficos estarán desactivados.")

try:
    from scipy import signal as sp_signal
    from scipy.linalg import lstsq
    SCIPY_OK = True
except ImportError:
    SCIPY_OK = False
    print("[AVISO] scipy no instalado. El análisis de sistema estará limitado.")

# ─────────────────────────────────────────────────────────────────────────────
DATA_DIR  = Path(__file__).parent.parent / "data"
PLOTS_DIR = Path(__file__).parent.parent / "plots"
DATA_DIR.mkdir(exist_ok=True)
PLOTS_DIR.mkdir(exist_ok=True)

# ─────────────────────────────────────────────────────────────────────────────
# Constantes físicas del robot (ajustar a tu robot real)
# ─────────────────────────────────────────────────────────────────────────────
PARAMS = {
    "M":   0.5,     # masa del carro [kg]
    "m":   0.2,     # masa del péndulo [kg]
    "l":   0.15,    # longitud al CG del péndulo [m]
    "I":   0.006,   # momento de inercia del péndulo [kg·m²]
    "g":   9.81,    # gravedad [m/s²]
    "b":   0.1,     # fricción del carro [N·s/m]
    "dt":  0.010,   # tiempo de muestreo [s]
}


# ═══════════════════════════════════════════════════════════════════════════ #
#  COLECTOR DE DATOS                                                          #
# ═══════════════════════════════════════════════════════════════════════════ #

class DataCollector:
    """Lee datos del ESP32 vía Serial y los guarda en CSV."""

    HEADER = ["t_ms", "angulo_deg", "vel_angular_dps", "pos_x_m", "vel_x_ms", "pwm"]

    def __init__(self, port: str, baud: int = 115200):
        if not SERIAL_OK:
            raise RuntimeError("pyserial requerido. pip install pyserial")
        self.port   = port
        self.baud   = baud
        self.ser    = None
        self.rows   = []          # filas capturadas
        self._stop  = threading.Event()
        self._lock  = threading.Lock()

    # ── Conexión ──────────────────────────────────────────────────────────────
    def conectar(self):
        self.ser = serial.Serial(self.port, self.baud, timeout=1.0)
        time.sleep(2.0)           # esperar reset del ESP32
        self.ser.reset_input_buffer()
        print(f"[OK] Conectado a {self.port} @ {self.baud} baud")

    def desconectar(self):
        if self.ser and self.ser.is_open:
            self._enviar("T")     # detener el robot
            time.sleep(0.1)
            self.ser.close()

    # ── Comandos ──────────────────────────────────────────────────────────────
    def _enviar(self, cmd: str):
        if self.ser and self.ser.is_open:
            self.ser.write((cmd + "\n").encode())

    def iniciar(self):
        self._enviar("S")

    def detener(self):
        self._enviar("T")

    def modo_prbs(self):
        self._enviar("P")

    def modo_escalon(self):
        self._enviar("E")

    def modo_manual(self, pwm: int):
        self._enviar(f"M{int(pwm)}")

    # ── Lectura ───────────────────────────────────────────────────────────────
    def leer_loop(self):
        """Hilo lector: parsea líneas CSV del ESP32."""
        while not self._stop.is_set():
            try:
                linea = self.ser.readline().decode("utf-8", errors="ignore").strip()
            except Exception:
                continue

            if not linea or linea.startswith("#") or linea.startswith("t_ms"):
                if linea.startswith("#"):
                    print(f"  ESP32: {linea}")
                continue

            partes = linea.split(",")
            if len(partes) != 6:
                continue

            try:
                fila = [float(p) for p in partes]
            except ValueError:
                continue

            with self._lock:
                self.rows.append(fila)

            # Eco en terminal cada 0.5 s (cada 50 muestras a 100 Hz)
            if len(self.rows) % 50 == 0:
                t   = fila[0] / 1000.0
                ang = fila[1]
                px  = fila[3]
                pwm = fila[5]
                print(f"  t={t:6.2f}s  θ={ang:+7.3f}°  x={px:+.4f}m  u={int(pwm):+4d}")

    # ── Captura completa ──────────────────────────────────────────────────────
    def capturar(self, duracion_s: float, modo: str, pwm_manual: int = 80):
        self.rows = []
        self._stop.clear()

        hilo = threading.Thread(target=self.leer_loop, daemon=True)
        hilo.start()

        print(f"\n[Iniciando captura | modo={modo} | duración={duracion_s}s]")
        self.iniciar()
        time.sleep(0.1)

        if modo == "prbs":
            self.modo_prbs()
        elif modo == "escalon":
            self.modo_escalon()
        elif modo == "manual":
            self.modo_manual(pwm_manual)
        else:
            print("[AVISO] Modo desconocido, usando PRBS")
            self.modo_prbs()

        try:
            time.sleep(duracion_s)
        except KeyboardInterrupt:
            print("\n[Interrumpido por usuario]")

        self.detener()
        self._stop.set()
        hilo.join(timeout=2.0)

        print(f"[Captura finalizada] {len(self.rows)} muestras recibidas")
        return self.get_dataframe()

    # ── Exportar ──────────────────────────────────────────────────────────────
    def get_dataframe(self):
        """Devuelve dict de arrays numpy con los datos."""
        if not self.rows:
            return None
        arr = np.array(self.rows)
        return {
            "t_ms":           arr[:, 0],
            "t_s":            arr[:, 0] / 1000.0,
            "theta_deg":      arr[:, 1],
            "theta_rad":      np.deg2rad(arr[:, 1]),
            "theta_dot_dps":  arr[:, 2],
            "theta_dot_rps":  np.deg2rad(arr[:, 2]),
            "pos_x":          arr[:, 3],
            "vel_x":          arr[:, 4],
            "pwm":            arr[:, 5],
        }

    def guardar_csv(self, datos: dict, nombre: str = None) -> Path:
        if datos is None:
            print("[ERROR] Sin datos para guardar")
            return None
        if nombre is None:
            ts    = datetime.now().strftime("%Y%m%d_%H%M%S")
            nombre = f"experimento_{ts}.csv"
        ruta = DATA_DIR / nombre
        with open(ruta, "w", newline="") as f:
            writer = csv.writer(f)
            writer.writerow(self.HEADER)
            n = len(datos["t_ms"])
            for i in range(n):
                writer.writerow([
                    datos["t_ms"][i],
                    datos["theta_deg"][i],
                    datos["theta_dot_dps"][i],
                    datos["pos_x"][i],
                    datos["vel_x"][i],
                    datos["pwm"][i],
                ])
        print(f"[Guardado] {ruta}")
        return ruta


# ═══════════════════════════════════════════════════════════════════════════ #
#  ANÁLISIS Y MODELO EN ESPACIO DE ESTADOS                                   #
# ═══════════════════════════════════════════════════════════════════════════ #

def cargar_csv(ruta: str) -> dict:
    arr = np.loadtxt(ruta, delimiter=",", skiprows=1)
    return {
        "t_ms":           arr[:, 0],
        "t_s":            arr[:, 0] / 1000.0,
        "theta_deg":      arr[:, 1],
        "theta_rad":      np.deg2rad(arr[:, 1]),
        "theta_dot_dps":  arr[:, 2],
        "theta_dot_rps":  np.deg2rad(arr[:, 2]),
        "pos_x":          arr[:, 3],
        "vel_x":          arr[:, 4],
        "pwm":            arr[:, 5],
    }


def modelo_linealizado_analitico(p: dict):
    """
    Modelo lineal del péndulo invertido sobre carro.

    Estado:  x  = [x, x_dot, theta, theta_dot]^T
    Entrada: u  = fuerza [N]  (necesitas escalar PWM -> N)
    Salida:  y  = [x, theta]

    dot(x) = A x + B u
    y      = C x + D u
    """
    M, m, l, I, g, b = p["M"], p["m"], p["l"], p["I"], p["g"], p["b"]
    den = I * (M + m) + M * m * l**2

    A = np.array([
        [0,   1,                     0,              0],
        [0,  -b*(I + m*l**2)/den,    m**2*g*l**2/den, 0],
        [0,   0,                     0,              1],
        [0,  -m*l*b/den,             m*g*l*(M+m)/den, 0],
    ])

    B = np.array([
        [0],
        [(I + m*l**2) / den],
        [0],
        [m * l / den],
    ])

    C = np.array([
        [1, 0, 0, 0],   # posición x
        [0, 0, 1, 0],   # ángulo theta
    ])

    D = np.zeros((2, 1))
    return A, B, C, D


def imprimir_modelo(A, B, C, D):
    print("\n" + "═"*60)
    print("  MODELO EN ESPACIO DE ESTADOS  (linealizado en vertical)")
    print("  x = [x,  x_dot,  theta,  theta_dot]^T")
    print("  u = fuerza de entrada [N]")
    print("  y = [x, theta]")
    print("═"*60)
    print("\nMatriz A:")
    for row in A:
        print("  " + "  ".join(f"{v:+10.5f}" for v in row))
    print("\nMatriz B:")
    for row in B:
        print("  " + "  ".join(f"{v:+10.5f}" for v in row))
    print("\nMatriz C:")
    for row in C:
        print("  " + "  ".join(f"{v:+10.5f}" for v in row))
    print("\nMatriz D:")
    for row in D:
        print("  " + "  ".join(f"{v:+10.5f}" for v in row))
    print()


def estimar_ganancia_pwm_fuerza(datos: dict, p: dict):
    """
    Estima K_pwm [N/PWM] con regresión lineal simple usando
    los datos de velocidad angular en respuesta a entradas escalón.
    """
    if not SCIPY_OK:
        print("[AVISO] scipy no disponible, saltando estimación de K_pwm")
        return 1.0

    u   = datos["pwm"]
    tdd = np.gradient(datos["theta_dot_rps"], datos["t_s"])  # theta_ddot

    M, m, l, I, g = p["M"], p["m"], p["l"], p["I"], p["g"]
    den = I * (M + m) + M * m * l**2

    # theta_ddot ≈ (m*g*l*(M+m)/den)*theta + (m*l/den)*F
    # F = K_pwm * pwm
    # Regresión: tdd = a*theta + b*u  -> estimamos b = m*l/den * K_pwm
    theta = datos["theta_rad"]
    mask  = np.abs(theta) < np.deg2rad(15)   # solo ángulos pequeños

    X = np.column_stack([theta[mask], u[mask]])
    y = tdd[mask]
    coeffs, _, _, _ = lstsq(X, y)
    b_est  = coeffs[1]
    K_pwm  = b_est * den / (m * l)
    print(f"\n[Estimación] K_pwm ≈ {K_pwm:.4f} N/PWM  (b_est={b_est:.6f})")
    return K_pwm


def analizar(ruta_csv: str):
    print(f"\n[Analizando] {ruta_csv}")
    datos = cargar_csv(ruta_csv)
    n     = len(datos["t_s"])
    print(f"  Muestras: {n}")
    print(f"  Duración: {datos['t_s'][-1]:.2f} s")
    print(f"  θ  rango: [{datos['theta_deg'].min():.2f}, {datos['theta_deg'].max():.2f}]°")
    print(f"  x  rango: [{datos['pos_x'].min():.4f}, {datos['pos_x'].max():.4f}] m")
    print(f"  u  rango: [{datos['pwm'].min():.0f}, {datos['pwm'].max():.0f}] PWM")

    # Modelo analítico
    A, B, C, D = modelo_linealizado_analitico(PARAMS)
    imprimir_modelo(A, B, C, D)

    # Estimar ganancia PWM->Fuerza desde datos reales
    K_pwm = estimar_ganancia_pwm_fuerza(datos, PARAMS)
    print(f"  Escala B_real = B_analitico * K_pwm = B * {K_pwm:.4f}")
    B_real = B * K_pwm
    print("\nMatriz B escalada (entrada en PWM):")
    for row in B_real:
        print("  " + "  ".join(f"{v:+10.5f}" for v in row))

    # Valores propios (deben tener un polo inestable para el péndulo)
    eigs = np.linalg.eigvals(A)
    print("\nValores propios de A:")
    for e in eigs:
        estado = "INESTABLE" if e.real > 0 else "estable"
        print(f"  λ = {e:.5f}  [{estado}]")

    # Controlabilidad
    n_states = A.shape[0]
    Co = B_real.copy()
    for i in range(1, n_states):
        Co = np.hstack([Co, np.linalg.matrix_power(A, i) @ B_real])
    rank_Co = np.linalg.matrix_rank(Co)
    print(f"\nRango matriz de controlabilidad: {rank_Co}/{n_states}  "
          f"-> {'CONTROLABLE' if rank_Co == n_states else 'NO controlable'}")

    # Observabilidad
    Ob = C.copy()
    for i in range(1, n_states):
        Ob = np.vstack([Ob, C @ np.linalg.matrix_power(A, i)])
    rank_Ob = np.linalg.matrix_rank(Ob)
    print(f"Rango matriz de observabilidad:  {rank_Ob}/{n_states}  "
          f"-> {'OBSERVABLE' if rank_Ob == n_states else 'NO observable'}")

    # Guardar resumen
    ts   = datetime.now().strftime("%Y%m%d_%H%M%S")
    ruta_reporte = DATA_DIR / f"analisis_{ts}.txt"
    with open(ruta_reporte, "w") as f:
        f.write("MODELO EN ESPACIO DE ESTADOS - PENDULO INVERTIDO\n")
        f.write(f"Generado: {datetime.now()}\n")
        f.write(f"Dataset:  {ruta_csv}\n\n")
        f.write(f"A =\n{A}\n\nB_analitico =\n{B}\n\n")
        f.write(f"K_pwm = {K_pwm:.6f} N/PWM\n")
        f.write(f"B_real (entrada PWM) =\n{B_real}\n\n")
        f.write(f"C =\n{C}\n\nD =\n{D}\n\n")
        f.write(f"Valores propios: {eigs}\n")
        f.write(f"Controlabilidad: {rank_Co}/{n_states}\n")
        f.write(f"Observabilidad:  {rank_Ob}/{n_states}\n")
    print(f"\n[Reporte guardado] {ruta_reporte}")

    graficar(datos, ruta_csv)
    return A, B_real, C, D


# ═══════════════════════════════════════════════════════════════════════════ #
#  GRÁFICAS                                                                  #
# ═══════════════════════════════════════════════════════════════════════════ #

def graficar(datos: dict, titulo: str = ""):
    if not PLOT_OK:
        print("[AVISO] matplotlib no disponible, sin gráficas")
        return

    fig, axes = plt.subplots(4, 1, figsize=(12, 10), sharex=True)
    t = datos["t_s"]

    axes[0].plot(t, datos["theta_deg"], color="tab:blue")
    axes[0].axhline(0, color="k", linewidth=0.5, linestyle="--")
    axes[0].set_ylabel("θ [°]")
    axes[0].set_title(f"Estado del Péndulo Invertido\n{titulo}")
    axes[0].grid(True, alpha=0.3)

    axes[1].plot(t, datos["theta_dot_dps"], color="tab:orange")
    axes[1].set_ylabel("θ̇ [°/s]")
    axes[1].grid(True, alpha=0.3)

    axes[2].plot(t, datos["pos_x"], color="tab:green",  label="x [m]")
    axes[2].plot(t, datos["vel_x"], color="tab:red",    label="ẋ [m/s]", alpha=0.7)
    axes[2].set_ylabel("Posición / Velocidad")
    axes[2].legend(loc="upper right", fontsize=8)
    axes[2].grid(True, alpha=0.3)

    axes[3].step(t, datos["pwm"], color="tab:purple", where="post")
    axes[3].set_ylabel("u [PWM]")
    axes[3].set_xlabel("Tiempo [s]")
    axes[3].grid(True, alpha=0.3)

    plt.tight_layout()
    ts   = datetime.now().strftime("%Y%m%d_%H%M%S")
    ruta = PLOTS_DIR / f"plot_{ts}.png"
    plt.savefig(ruta, dpi=150)
    print(f"[Gráfica guardada] {ruta}")
    plt.show()


def graficar_en_vivo(collector: "DataCollector"):
    """Gráfica animada en tiempo real durante la captura."""
    if not PLOT_OK:
        return

    fig, (ax1, ax2, ax3) = plt.subplots(3, 1, figsize=(10, 7), sharex=False)
    fig.suptitle("Adquisición en Tiempo Real")

    def actualizar(_):
        with collector._lock:
            rows = list(collector.rows)
        if len(rows) < 2:
            return
        arr  = np.array(rows)
        t    = arr[:, 0] / 1000.0
        ang  = arr[:, 1]
        px   = arr[:, 3]
        pwm  = arr[:, 5]

        ax1.cla(); ax1.plot(t, ang,  color="tab:blue");   ax1.set_ylabel("θ [°]");    ax1.grid(True, alpha=0.3)
        ax2.cla(); ax2.plot(t, px,   color="tab:green");  ax2.set_ylabel("x [m]");    ax2.grid(True, alpha=0.3)
        ax3.cla(); ax3.step(t, pwm,  color="tab:purple", where="post"); ax3.set_ylabel("u [PWM]"); ax3.set_xlabel("t [s]"); ax3.grid(True, alpha=0.3)

    ani = animation.FuncAnimation(fig, actualizar, interval=500, cache_frame_data=False)
    plt.tight_layout()
    plt.show()


# ═══════════════════════════════════════════════════════════════════════════ #
#  UTILIDADES DE PUERTO                                                       #
# ═══════════════════════════════════════════════════════════════════════════ #

def listar_puertos():
    if not SERIAL_OK:
        print("pyserial no disponible")
        return
    puertos = list(serial.tools.list_ports.comports())
    if not puertos:
        print("No se encontraron puertos seriales.")
        return
    print("Puertos disponibles:")
    for p in puertos:
        print(f"  {p.device:20s}  {p.description}")


# ═══════════════════════════════════════════════════════════════════════════ #
#  PUNTO DE ENTRADA                                                           #
# ═══════════════════════════════════════════════════════════════════════════ #

def main():
    parser = argparse.ArgumentParser(
        description="Adquisición de datos del Péndulo Invertido (ESP32-S3)",
        formatter_class=argparse.RawTextHelpFormatter,
    )
    parser.add_argument("--port",     default=None,   help="Puerto serial, ej. /dev/ttyUSB0")
    parser.add_argument("--baud",     default=115200, type=int)
    parser.add_argument("--modo",     default="prbs",
                        choices=["prbs", "escalon", "manual"],
                        help="Señal de excitación:\n"
                             "  prbs    - Secuencia binaria pseudo-aleatoria\n"
                             "  escalon - Escalón alternado (recomendado para K_pwm)\n"
                             "  manual  - PWM fijo (usar con --pwm)")
    parser.add_argument("--pwm",      default=80,     type=int, help="PWM para modo manual (-255 a 255)")
    parser.add_argument("--duracion", default=20,     type=float, help="Duración de captura en segundos")
    parser.add_argument("--salida",   default=None,   help="Nombre del archivo CSV de salida")
    parser.add_argument("--analizar", default=None,   help="Analizar un CSV existente (no captura)")
    parser.add_argument("--puertos",  action="store_true", help="Listar puertos seriales disponibles")
    parser.add_argument("--plot",     action="store_true", help="Mostrar gráfica en vivo durante captura")

    args = parser.parse_args()

    # ── Solo listar puertos ──────────────────────────────────────────────────
    if args.puertos:
        listar_puertos()
        return

    # ── Solo analizar ────────────────────────────────────────────────────────
    if args.analizar:
        if not os.path.exists(args.analizar):
            print(f"[ERROR] Archivo no encontrado: {args.analizar}")
            sys.exit(1)
        analizar(args.analizar)
        return

    # ── Captura + análisis ───────────────────────────────────────────────────
    if not args.port:
        print("[ERROR] Debes especificar un puerto con --port")
        print("        Usa --puertos para ver los disponibles")
        listar_puertos()
        sys.exit(1)

    collector = DataCollector(args.port, args.baud)

    # Manejar Ctrl+C limpiamente
    def _sigint(sig, frame):
        print("\n[Ctrl+C recibido, deteniendo...]")
        collector.detener()
        sys.exit(0)
    signal.signal(signal.SIGINT, _sigint)

    collector.conectar()

    # Gráfica en vivo en hilo separado
    if args.plot and PLOT_OK:
        hilo_plot = threading.Thread(
            target=graficar_en_vivo, args=(collector,), daemon=True
        )
        hilo_plot.start()

    datos = collector.capturar(
        duracion_s=args.duracion,
        modo=args.modo,
        pwm_manual=args.pwm,
    )
    collector.desconectar()

    if datos is None:
        print("[ERROR] No se recibieron datos. Verifica conexión y puerto.")
        sys.exit(1)

    ruta_csv = collector.guardar_csv(datos, args.salida)
    analizar(str(ruta_csv))


if __name__ == "__main__":
    main()
