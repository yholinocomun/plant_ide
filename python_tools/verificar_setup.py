#!/usr/bin/env python3
"""
============================================================
VERIFICACIÓN PREVIA — corre esto ANTES de las fases
============================================================
Comprueba que todo está listo para tomar data:
  1. Librerías Python instaladas
  2. Puerto serial accesible (Arduino IDE cerrado)
  3. El ESP32 responde y envía datos
  4. El encoder cuenta cuando giras la rueda a mano
  5. (opcional) El MPU responde

Uso:
  python3 verificar_setup.py --port /dev/ttyACM0
  python3 verificar_setup.py --puertos        # solo listar puertos
============================================================
"""

import argparse
import sys
import time


def check_librerias():
    print("\n[1] LIBRERÍAS PYTHON")
    ok = True
    for nombre in ["serial", "numpy", "matplotlib", "scipy"]:
        try:
            __import__(nombre)
            print(f"    ✓ {nombre}")
        except ImportError:
            crit = nombre in ("serial", "numpy")
            print(f"    {'✗' if crit else '⚠'} {nombre} "
                  f"{'(OBLIGATORIA)' if crit else '(opcional)'}")
            if crit:
                ok = False
    if not ok:
        print("\n    Instala con:  pip install -r python_tools/requirements.txt")
    return ok


def check_puerto(port):
    print(f"\n[2] PUERTO SERIAL  ({port})")
    import serial
    try:
        ser = serial.Serial(port, 115200, timeout=1.0)
        print(f"    ✓ Puerto abierto correctamente")
        return ser
    except serial.SerialException as e:
        msg = str(e)
        print(f"    ✗ No se pudo abrir: {msg}")
        if "Permission" in msg or "denied" in msg:
            print("    → Falta permiso. Ejecuta:")
            print("        sudo usermod -aG dialout $USER")
            print("      luego CIERRA SESIÓN y vuelve a entrar.")
        elif "busy" in msg or "Device or resource" in msg:
            print("    → Puerto ocupado. CIERRA Arduino IDE por completo")
            print("      (no solo el monitor serial, toda la aplicación).")
        else:
            print("    → ¿Está conectado el ESP32? Revisa con --puertos")
        return None


def check_esp32(ser):
    print("\n[3] RESPUESTA DEL ESP32")
    time.sleep(2.0)               # esperar reset
    ser.reset_input_buffer()
    print("    Escuchando 3 segundos...")
    lineas = []
    t_end = time.time() + 3.0
    while time.time() < t_end:
        l = ser.readline().decode("utf-8", "ignore").strip()
        if l:
            lineas.append(l)
    if not lineas:
        print("    ⚠ No llegó nada. Normal si el firmware espera un comando")
        print("      (manda 'A'/'B'/'C' o 'S' para que empiece a enviar).")
        return lineas
    print(f"    ✓ Llegaron {len(lineas)} líneas. Muestra:")
    for l in lineas[:4]:
        print(f"        {l}")
    return lineas


def check_encoder(ser):
    """Solo aplica al firmware exp1_motor (CSV de 4 columnas)."""
    print("\n[4] PRUEBA DE ENCODER  (firmware exp1_motor)")
    print("    >>> GIRA UNA RUEDA A MANO durante 4 segundos <<<")
    ser.reset_input_buffer()
    ser.write(b"M0\n")            # activa el envío sin mover motor
    time.sleep(0.3)
    ser.reset_input_buffer()

    primeras, ultimas = None, None
    t_end = time.time() + 4.0
    while time.time() < t_end:
        l = ser.readline().decode("utf-8", "ignore").strip()
        if not l or l.startswith("#") or l.startswith("t_ms"):
            continue
        p = l.split(",")
        if len(p) == 4:
            try:
                vals = [float(x) for x in p]
            except ValueError:
                continue
            if primeras is None:
                primeras = vals
            ultimas = vals
    ser.write(b"T\n")

    if primeras is None:
        print("    ⚠ No se recibieron filas de 4 columnas.")
        print("      ¿Seguro que flasheaste exp1_motor.ino?")
        return
    d_izq = ultimas[2] - primeras[2]
    d_der = ultimas[3] - primeras[3]
    print(f"    Δ encoder izq: {d_izq:+.0f} pulsos")
    print(f"    Δ encoder der: {d_der:+.0f} pulsos")
    if abs(d_izq) > 5 or abs(d_der) > 5:
        print("    ✓ El encoder CUENTA al girar. Conversión a posición:")
        rad = d_izq * 2 * 3.14159 / 1945.0
        print(f"      {d_izq:+.0f} pulsos = {rad:+.3f} rad = {rad*180/3.14159:+.1f}°")
    else:
        print("    ✗ El encoder NO contó. Revisa:")
        print("      - cableado de los pines A/B del encoder")
        print("      - alimentación del encoder")
        print("      - que giraste la rueda correcta")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port")
    ap.add_argument("--puertos", action="store_true")
    ap.add_argument("--skip-encoder", action="store_true",
                    help="saltar prueba de encoder (si usas exp2_pendulo)")
    args = ap.parse_args()

    print("═"*55)
    print("  VERIFICACIÓN PREVIA A LA TOMA DE DATA")
    print("═"*55)

    if not check_librerias():
        sys.exit(1)

    import serial.tools.list_ports
    if args.puertos or not args.port:
        print("\n[PUERTOS DISPONIBLES]")
        encontrados = list(serial.tools.list_ports.comports())
        if not encontrados:
            print("    (ninguno — conecta el ESP32 por USB)")
        for p in encontrados:
            marca = " <- probablemente este" if ("ACM" in p.device or "USB" in p.device) else ""
            print(f"    {p.device:18s} {p.description}{marca}")
        if not args.port:
            print("\n    Vuelve a correr con:  --port <el de arriba>")
            return

    ser = check_puerto(args.port)
    if ser is None:
        sys.exit(1)

    check_esp32(ser)
    if not args.skip_encoder:
        check_encoder(ser)

    ser.close()
    print("\n" + "═"*55)
    print("  VERIFICACIÓN COMPLETA")
    print("═"*55)
    print("  Si todo salió ✓, ya puedes correr las fases.")


if __name__ == "__main__":
    main()
