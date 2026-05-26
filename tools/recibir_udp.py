"""
recibir_udp.py - Receptor de telemetria del PISDRSL por UDP
===========================================================

La ESP32 envia telemetria UDP a 5 Hz; el control sigue corriendo a 100 Hz.

Formato v2:
  0xAB + version + seq + 8 float32 + 2 ADC12 + flags

Tambien acepta el formato legacy:
  0xAA + 11 bytes = 12 bytes en total.

USO:
    python recibir_udp.py
    python recibir_udp.py 5005
    python recibir_udp.py 5005 300
"""

import os
import socket
import struct
import sys
import time
from datetime import datetime

PORT = int(sys.argv[1]) if len(sys.argv) > 1 else 5005
MAX_SEG = int(sys.argv[2]) if len(sys.argv) > 2 else 0

HEADER_LEGACY = 0xAA
PKT_SIZE_LEGACY = 12
HEADER_V2 = 0xAB
VERSION_V2 = 2
PKT_FMT_V2 = "<BBHffffffffHHB"
PKT_SIZE_V2 = struct.calcsize(PKT_FMT_V2)

# Factores legacy de main.c anterior.
KT_V = 637.5
KT_TH = 426.6
KT_AL = 91.07
KT_OM = 8.67
KT_U = 12.08


def decode_packet(data: bytes):
    if len(data) >= PKT_SIZE_V2 and data[0] == HEADER_V2:
        header, version, seq, vd, v, theta, alpha, omegal, omegar, ul, ur, sl, sr, flags = (
            struct.unpack(PKT_FMT_V2, data[:PKT_SIZE_V2])
        )
        if header != HEADER_V2 or version != VERSION_V2:
            return None
        return {
            "proto": "v2",
            "seq": seq,
            "vd": vd,
            "v": v,
            "theta": theta,
            "alpha": alpha,
            "omegal": omegal,
            "omegar": omegar,
            "ul": ul,
            "ur": ur,
            "sl": sl,
            "sr": sr,
            "modo_sol": bool(flags & 0x01),
        }

    if len(data) >= PKT_SIZE_LEGACY and data[0] == HEADER_LEGACY:
        b = data
        return {
            "proto": "legacy",
            "seq": None,
            "vd": (b[1] - 127) / KT_V,
            "v": (b[2] - 127) / KT_V,
            "theta": (b[3] - 128) / KT_TH,
            "alpha": (b[4] - 127) / KT_AL,
            "omegal": (b[5] - 127) / KT_OM,
            "omegar": (b[6] - 127) / KT_OM,
            "ul": (b[7] - 127) / KT_U,
            "ur": (b[8] - 127) / KT_U,
            "sl": b[9],
            "sr": b[10],
            "modo_sol": bool(b[11] & 0x01),
        }

    return None


ts_str = datetime.now().strftime("%Y%m%d_%H%M%S")
output_dir = os.path.dirname(os.path.abspath(__file__))
output_file = os.path.join(output_dir, f"datos_{ts_str}.txt")

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)

try:
    sock.bind(("", PORT))
except OSError as exc:
    print(f"\nERROR: No se puede abrir el puerto UDP {PORT}.")
    print(f"       {exc}")
    sys.exit(1)

print("\nPISDRSL - Receptor UDP")
print(f"Escuchando en puerto {PORT} (todas las interfaces)...")
print(f"Guardando en: {output_file}")
if MAX_SEG > 0:
    print(f"Duracion: {MAX_SEG} segundos")
print("\nEsperando paquetes de la ESP32...")
print("(Ctrl+C para detener)\n")
print(
    f"{'Tiempo':>8}  {'alpha(deg)':>10}  {'v(m/s)':>8}  {'theta(deg)':>10}  "
    f"{'wl':>8}  {'wr':>8}  {'ul(V)':>7}  {'ur(V)':>7}  {'SL SR':>11}  proto"
)
print(" " + "-" * 100)

HEADER_CSV = "tiempo_s,proto,seq,vd,v,theta,alpha,omegal,omegar,ul,ur,sl_adc,sr_adc,modo_sol\n"

t = 0.0
TS_TELEM = 0.20
muestras = 0
errores = 0
RAD2DEG = 57.2958
t_inicio = time.time()

with open(output_file, "w", encoding="utf-8") as f:
    f.write(HEADER_CSV)

    try:
        while True:
            if MAX_SEG > 0 and (time.time() - t_inicio) >= MAX_SEG:
                print(f"\n\nTiempo limite ({MAX_SEG}s) alcanzado.")
                break

            try:
                sock.settimeout(2.0)
                data, _addr = sock.recvfrom(256)
            except socket.timeout:
                print("[esperando paquetes... ESP32 encendida y conectada al WiFi?]", end="\r")
                continue

            pkt = decode_packet(data)
            if pkt is None:
                errores += 1
                hdr = data[0] if data else 0
                print(f"\n[paquete ignorado: {len(data)} bytes, header=0x{hdr:02X}]")
                continue

            seq = "" if pkt["seq"] is None else pkt["seq"]
            f.write(
                f"{t:.3f},{pkt['proto']},{seq},{pkt['vd']:.6f},{pkt['v']:.6f},"
                f"{pkt['theta']:.6f},{pkt['alpha']:.6f},"
                f"{pkt['omegal']:.6f},{pkt['omegar']:.6f},"
                f"{pkt['ul']:.6f},{pkt['ur']:.6f},"
                f"{pkt['sl']},{pkt['sr']},{int(pkt['modo_sol'])}\n"
            )
            f.flush()

            muestras += 1
            caido = "CAIDO" if abs(pkt["alpha"]) > 1.33 else "     "
            print(
                f"{t:8.2f}s  {pkt['alpha'] * RAD2DEG:+9.2f}  {pkt['v']:8.4f}  "
                f"{pkt['theta'] * RAD2DEG:+9.2f}  {pkt['omegal']:8.3f}  "
                f"{pkt['omegar']:8.3f}  {pkt['ul']:+6.2f}  {pkt['ur']:+6.2f}  "
                f"{pkt['sl']:4} {pkt['sr']:4}  {pkt['proto']} {caido}  [{muestras}]",
                end="\r",
            )

            t += TS_TELEM

    except KeyboardInterrupt:
        pass

duracion = time.time() - t_inicio
freq = muestras / duracion if duracion > 0 else 0.0
print(f"\n\n{'-' * 50}")
print(f"  Muestras recibidas : {muestras}")
print(f"  Paquetes ignorados : {errores}")
print(f"  Duracion           : {duracion:.1f} s")
print(f"  Frecuencia real    : {freq:.2f} Hz (esperado: 5 Hz)")
print(f"  Archivo guardado   : {output_file}")
print(f"{'-' * 50}\n")

sock.close()
