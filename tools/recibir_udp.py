"""
recibir_udp.py — Receptor de telemetría del PISDRSL por UDP
============================================================

La ESP32 envía un paquete UDP binario cada 100 ms (10 ciclos × 10 ms).
Formato: 0xAA + 11 bytes = 12 bytes en total.

USO:
    python recibir_udp.py                → escucha en el puerto 5005
    python recibir_udp.py 6000           → escucha en otro puerto
    python recibir_udp.py 5005 300       → escucha 300 segundos y cierra

REQUISITOS:
    - La PC y la ESP32 deben estar en la MISMA red WiFi/LAN.
    - El firewall de Windows debe permitir UDP entrante en el puerto 5005.
"""

import socket
import struct
import sys
import os
from datetime import datetime

# ── Configuración ──────────────────────────────────────────────────────────────
PORT        = int(sys.argv[1]) if len(sys.argv) > 1 else 5005
MAX_SEG     = int(sys.argv[2]) if len(sys.argv) > 2 else 0

PKT_SIZE    = 12
HEADER_BYTE = 0xAA

# Factores de escala (deben coincidir con main.c)
KT_V  = 637.5
KT_TH = 426.6
KT_AL = 91.07
KT_OM = 8.67
KT_U  = 12.08

ts_str      = datetime.now().strftime("%Y%m%d_%H%M%S")
output_dir  = os.path.dirname(os.path.abspath(__file__))
output_file = os.path.join(output_dir, f"datos_{ts_str}.txt")

# ── Abrir socket UDP ───────────────────────────────────────────────────────────
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)

try:
    sock.bind(('', PORT))
except OSError as e:
    print(f"\n ERROR: No se puede abrir el puerto UDP {PORT}.")
    print(f"        {e}")
    sys.exit(1)

print(f"\n PISDRSL — Receptor UDP (protocolo binario)")
print(f" Escuchando en puerto {PORT} (todas las interfaces)...")
print(f" Guardando en: {output_file}")
if MAX_SEG > 0:
    print(f" Duración: {MAX_SEG} segundos")
print(f"\n Esperando paquetes de la ESP32...")
print(f" (Ctrl+C para detener)\n")
print(f" {'Tiempo':>8}  {'alpha(°)':>9}  {'v(m/s)':>8}  {'θ(°)':>8}  "
      f"{'ωl(r/s)':>8}  {'ωr(r/s)':>8}  {'ul(V)':>7}  {'ur(V)':>7}  {'SL SR':>6}")
print(f" " + "─" * 88)

HEADER_CSV = "tiempo_s,vd,v,theta,alpha,omegal,omegar,ul,ur,sl,sr\n"

t         = 0.0
Ts_telem  = 0.1
muestras  = 0
errores   = 0
RAD2DEG   = 57.2958

import time
t_inicio = time.time()

with open(output_file, "w", encoding="utf-8") as f:
    f.write(HEADER_CSV)

    try:
        while True:
            if MAX_SEG > 0 and (time.time() - t_inicio) >= MAX_SEG:
                print(f"\n\n Tiempo límite ({MAX_SEG}s) alcanzado.")
                break

            try:
                sock.settimeout(2.0)
                data, addr = sock.recvfrom(256)
            except socket.timeout:
                print(f" [esperando paquetes... ¿ESP32 encendida y conectada al WiFi?]",
                      end="\r")
                continue

            # Verificar paquete binario: 12 bytes, primer byte = 0xAA
            if len(data) < PKT_SIZE or data[0] != HEADER_BYTE:
                errores += 1
                print(f"\n [paquete ignorado: {len(data)} bytes, header=0x{data[0]:02X}]")
                continue

            b = data
            vd     = (b[1]  - 127) / KT_V
            v      = (b[2]  - 127) / KT_V
            theta  = (b[3]  - 128) / KT_TH
            alpha  = (b[4]  - 127) / KT_AL
            omegal = (b[5]  - 127) / KT_OM
            omegar = (b[6]  - 127) / KT_OM
            ul     = (b[7]  - 127) / KT_U
            ur     = (b[8]  - 127) / KT_U
            sl     = b[9]
            sr     = b[10]

            f.write(f"{t:.3f},{vd:.4f},{v:.4f},{theta:.5f},{alpha:.5f},"
                    f"{omegal:.4f},{omegar:.4f},{ul:.4f},{ur:.4f},{sl},{sr}\n")
            f.flush()

            muestras += 1
            caido = "CAIDO" if abs(alpha) > 1.33 else "     "
            print(f" {t:8.2f}s  {alpha*RAD2DEG:+8.2f}°  {v:8.4f}  "
                  f"{theta*RAD2DEG:+7.2f}°  {omegal:8.3f}  {omegar:8.3f}  "
                  f"{ul:+6.2f}  {ur:+6.2f}  {sl} {sr}  {caido}  [{muestras}]",
                  end="\r")

            t += Ts_telem

    except KeyboardInterrupt:
        pass

duracion = time.time() - t_inicio
print(f"\n\n {'─'*50}")
print(f"  Muestras recibidas : {muestras}")
print(f"  Paquetes ignorados : {errores}")
print(f"  Duración           : {duracion:.1f} s")
print(f"  Frecuencia real    : {muestras/duracion:.2f} Hz (esperado: 10 Hz)")
print(f"  Archivo guardado   : {output_file}")
print(f" {'─'*50}\n")

sock.close()
