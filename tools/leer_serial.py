"""
leer_serial.py — Recibe telemetría del ESP32 y guarda en .txt

Uso:
    python leer_serial.py              (usa COM3 a 115200 por defecto)
    python leer_serial.py COM5         (otro puerto)
    python leer_serial.py COM5 9600    (otro puerto y baud rate)

Instalar dependencia:
    pip install pyserial

Formato de cada línea recibida del ESP32:
    vd=0.100 v=0.098 th=0.0012 al=-0.003 wl=2.81 wr=2.83 ul=4.21 ur=4.18
"""

import serial
import sys
import os
from datetime import datetime

# ── Configuración ──────────────────────────────────────────────────────────────
PORT     = sys.argv[1] if len(sys.argv) > 1 else "COM3"
BAUDRATE = int(sys.argv[2]) if len(sys.argv) > 2 else 115200

# El archivo de salida lleva la fecha y hora para no sobreescribir datos previos
timestamp  = datetime.now().strftime("%Y%m%d_%H%M%S")
output_dir = os.path.dirname(os.path.abspath(__file__))
output_file = os.path.join(output_dir, f"datos_{timestamp}.txt")

# ── Cabecera del archivo ───────────────────────────────────────────────────────
HEADER = "tiempo_s\tvd\tv\ttheta\talpha\tomega_l\tomega_r\tul\tur\n"

# ── Conexión serial ────────────────────────────────────────────────────────────
print(f"Conectando a {PORT} @ {BAUDRATE} baud...")
try:
    ser = serial.Serial(PORT, BAUDRATE, timeout=2)
except serial.SerialException as e:
    print(f"ERROR: No se pudo abrir {PORT}. ¿Está conectada la ESP32?")
    print(f"       {e}")
    sys.exit(1)

print(f"Guardando datos en: {output_file}")
print("Presiona Ctrl+C para detener.\n")

t = 0.0          # Tiempo acumulado [s]
Ts = 0.1         # Periodo de impresión (10 ciclos × 10 ms = 100 ms)
lineas = 0

with open(output_file, "w", encoding="utf-8") as f:
    f.write(HEADER)

    try:
        while True:
            # Leer una línea del serial (bloqueante hasta timeout)
            raw = ser.readline()
            if not raw:
                continue   # Timeout sin datos, intentar de nuevo

            # Decodificar y limpiar espacios/saltos de línea
            line = raw.decode("utf-8", errors="replace").strip()

            # Filtrar líneas que no son telemetría (logs del ESP-IDF empiezan con 'I (')
            if not line.startswith("vd="):
                print(f"[LOG ESP32] {line}")
                continue

            # Parsear los 8 valores del formato:
            # "vd=0.100 v=0.098 th=0.0012 al=-0.003 wl=2.81 wr=2.83 ul=4.21 ur=4.18"
            try:
                partes = {}
                for token in line.split():
                    clave, valor = token.split("=")
                    partes[clave] = float(valor)

                vd  = partes["vd"]
                v   = partes["v"]
                th  = partes["th"]
                al  = partes["al"]
                wl  = partes["wl"]
                wr  = partes["wr"]
                ul  = partes["ul"]
                ur  = partes["ur"]

            except (ValueError, KeyError):
                print(f"[PARSE ERROR] Línea ignorada: {line}")
                continue

            # Escribir al archivo (separado por tabulaciones para abrir en Excel)
            f.write(f"{t:.3f}\t{vd:.4f}\t{v:.4f}\t{th:.5f}\t{al:.5f}\t"
                    f"{wl:.4f}\t{wr:.4f}\t{ul:.4f}\t{ur:.4f}\n")
            f.flush()   # Escribir a disco inmediatamente (no esperar al buffer)

            # Mostrar en pantalla
            lineas += 1
            print(f"t={t:7.2f}s | α={al:+.4f}rad | v={v:.3f}m/s | "
                  f"θ={th:+.4f}rad | ul={ul:+.2f}V ur={ur:+.2f}V  [{lineas} muestras]",
                  end="\r")

            t += Ts

    except KeyboardInterrupt:
        print(f"\n\nDetenido. {lineas} muestras guardadas en:")
        print(f"  {output_file}")

ser.close()
