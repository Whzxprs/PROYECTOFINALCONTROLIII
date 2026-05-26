# -*- coding: utf-8 -*-
"""
Balancin Monitor  –  Dashboard modo oscuro / WiFi UDP
======================================================
Recibe telemetria de la ESP32 por UDP (inalambrico).
Muestra valores en tiempo real y permite grabar .txt para MATLAB.

Protocolo UDP (12 bytes por datagrama):
  [0]  0xAA  header
  [1]  vd    velocidad referencia  (offset 127, escala KT_V)
  [2]  v     velocidad real        (offset 127, escala KT_V)
  [3]  theta orientacion           (offset 128, escala KT_TH)
  [4]  alpha inclinacion           (offset 127, escala KT_AL)
  [5]  oml   omega izq             (offset 127, escala KT_OM)
  [6]  omr   omega der             (offset 127, escala KT_OM)
  [7]  ul    voltaje motor izq     (offset 127, escala KT_U)
  [8]  ur    voltaje motor der     (offset 127, escala KT_U)
  [9]  Sl    sensor linea izq      (0-255, sin offset)
  [10] Sr    sensor linea der      (0-255, sin offset)
  [11] flags bit0 = modo_sol

Instalar:
  pip install PyQt6 pyqtgraph numpy

Uso:
  1. Pon UDP_PORT igual al UDP_PORT del ESP32 (por defecto 5005)
  2. Ejecuta: python monitor.py
  3. Pulsa "Escuchar" – la app espera datagramas del balancin
  4. "Grabar" guarda un .txt compatible con MATLAB
"""

import sys
import socket
import struct
import threading
import collections
import datetime
import os

import numpy as np
import pyqtgraph as pg

from PyQt6.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
    QLabel, QPushButton, QLineEdit, QSplitter, QGroupBox,
    QScrollArea, QFrame, QFileDialog,
)
from PyQt6.QtCore import Qt, QTimer, QThread, pyqtSignal
from PyQt6.QtGui import QFont, QIntValidator

# ─────────────────────────────────────────────────────────────────────────────
#  CONFIGURACION – debe coincidir con los #define del ESP32
# ─────────────────────────────────────────────────────────────────────────────
UDP_PORT_DEFAULT = 5005    # Puerto UDP de escucha (igual que en main.c)
CMD_UDP_PORT     = 5006    # Puerto para enviar ganancias al ESP32
PKT_SIZE         = 12      # Bytes por datagrama: 0xAA + 11 bytes de datos
HEADER           = 0xAA

# Factores de escala — mismos que el firmware ESP32
KT_V   = 637.5
KT_TH  = 426.6
KT_AL  = 91.07
KT_OM  = 8.67
KT_U   = 12.08

BUF_LEN    = 600   # muestras visibles en graficas (~6 s a 100 Hz)
REFRESH_MS = 50    # refresco visual (20 fps)

# ─────────────────────────────────────────────────────────────────────────────
#  PALETA MODO OSCURO
# ─────────────────────────────────────────────────────────────────────────────
C = {
    "bg"    : "#0d1117", "bg2"  : "#161b22", "bg3"  : "#1c2128",
    "border": "#30363d", "text" : "#c9d1d9", "muted": "#7a8fa6",
    "dim"   : "#4a5568", "green": "#4ade80", "cyan" : "#22d3ee",
    "pink"  : "#f472b6", "purple":"#a78bfa", "orange":"#fb923c",
    "yellow": "#fbbf24", "blue" : "#60a5fa", "indigo":"#818cf8",
    "teal"  : "#34d399", "mint" : "#2dd4bf", "sun"  : "#ffaa00",
    "red"   : "#f87171", "accent":"#1f6feb", "rec"  : "#dc2626",
}

QSS = f"""
QMainWindow, QWidget {{
    background-color:{C['bg']}; color:{C['text']};
    font-family:'Segoe UI','Arial',sans-serif; font-size:12px;
}}
QGroupBox {{
    color:{C['muted']}; font-size:11px; font-weight:bold;
    border:1px solid {C['bg3']}; border-radius:6px;
    margin-top:10px; padding-top:6px;
}}
QGroupBox::title {{
    subcontrol-origin:margin; left:10px; padding:0 4px;
    background-color:{C['bg']};
}}
QLineEdit {{
    background:{C['bg2']}; border:1px solid {C['border']};
    border-radius:5px; padding:4px 8px; color:{C['text']};
}}
QPushButton {{
    background:{C['accent']}; color:white; border:none;
    border-radius:5px; padding:5px 14px; font-weight:bold;
}}
QPushButton:hover {{ background:#388bfd; }}
QPushButton:checked {{ background:{C['rec']}; }}
QPushButton:checked:hover {{ background:#ef4444; }}
QPushButton#small {{
    background:{C['bg3']}; font-size:13px;
    padding:3px 8px; border-radius:4px;
}}
QPushButton#small:hover {{ background:{C['border']}; }}
QScrollArea {{ border:none; }}
QSplitter::handle {{ background:{C['bg3']}; }}
"""

# ─────────────────────────────────────────────────────────────────────────────
#  HILO RECEPTOR UDP
# ─────────────────────────────────────────────────────────────────────────────
class UDPWorker(QThread):
    packet_ready = pyqtSignal(dict, str)   # data, ip_origen
    status_msg   = pyqtSignal(str, bool)   # mensaje, es_error

    def __init__(self, port: int):
        super().__init__()
        self._port = port
        self._stop = threading.Event()

    def stop(self):
        self._stop.set()

    def run(self):
        try:
            sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
            sock.settimeout(0.5)
            sock.bind(("", self._port))
            self.status_msg.emit(
                f"Escuchando en UDP:{self._port} — esperando ESP32...", False)
        except Exception as exc:
            self.status_msg.emit(f"Error al abrir UDP:{self._port} — {exc}", True)
            return

        while not self._stop.is_set():
            try:
                data, (ip, _) = sock.recvfrom(256)
            except socket.timeout:
                continue
            except Exception:
                continue

            # Buscar paquetes binarios 0xAA dentro del datagrama
            buf = bytearray(data)
            while len(buf) >= PKT_SIZE:
                idx = buf.find(HEADER)
                if idx < 0:
                    break
                if idx > 0:
                    del buf[:idx]
                if len(buf) < PKT_SIZE:
                    break
                parsed = self._decode(bytes(buf[:PKT_SIZE]))
                del buf[:PKT_SIZE]
                if parsed:
                    self.packet_ready.emit(parsed, ip)

        sock.close()
        self.status_msg.emit("UDP cerrado", False)

    def _decode(self, pkt: bytes):
        if pkt[0] != HEADER:
            return None
        return {
            "vd"      : (pkt[1]  - 127) / KT_V,
            "v"       : (pkt[2]  - 127) / KT_V,
            "theta"   : (pkt[3]  - 128) / KT_TH,
            "alpha"   : (pkt[4]  - 127) / KT_AL,
            "oml"     : (pkt[5]  - 127) / KT_OM,
            "omr"     : (pkt[6]  - 127) / KT_OM,
            "ul"      : (pkt[7]  - 127) / KT_U,
            "ur"      : (pkt[8]  - 127) / KT_U,
            "Sl"      : pkt[9],
            "Sr"      : pkt[10],
            "modo_sol": bool(pkt[11] & 0x01),
        }

# ─────────────────────────────────────────────────────────────────────────────
#  TARJETA DE VALOR INDIVIDUAL
# ─────────────────────────────────────────────────────────────────────────────
class ValueCard(QWidget):
    def __init__(self, label: str, unit: str = "", color: str = C["cyan"],
                 val_range: tuple = None):
        super().__init__()
        self._color = color
        self._range = val_range

        lay = QVBoxLayout(self)
        lay.setContentsMargins(10, 8, 10, 8)
        lay.setSpacing(3)

        row = QHBoxLayout()
        row.setSpacing(4)

        self._lbl = QLabel(label)
        self._lbl.setStyleSheet(f"color:{C['muted']}; font-size:11px;")

        self._val = QLabel("—")
        self._val.setAlignment(Qt.AlignmentFlag.AlignRight)
        f = QFont("Consolas", 16); f.setBold(True)
        self._val.setFont(f)
        self._val.setStyleSheet(f"color:{color};")

        self._unit = QLabel(unit)
        self._unit.setStyleSheet(f"color:{C['dim']}; font-size:10px;")
        self._unit.setAlignment(Qt.AlignmentFlag.AlignBottom)

        row.addWidget(self._lbl)
        row.addStretch()
        row.addWidget(self._val)
        row.addWidget(self._unit)
        lay.addLayout(row)

        if val_range is not None:
            self._bg = QFrame()
            self._bg.setFixedHeight(3)
            self._bg.setStyleSheet(f"background:{C['bg3']}; border-radius:1px;")
            self._fill = QFrame(self._bg)
            self._fill.setFixedHeight(3)
            self._fill.setStyleSheet(f"background:{color}; border-radius:1px;")
            lay.addWidget(self._bg)
        else:
            self._fill = None

        self.setStyleSheet(
            f"background:{C['bg2']}; border-radius:8px;"
            f"border:1px solid {C['bg3']};")

    def set_value(self, value):
        if isinstance(value, bool):
            txt   = "SOL"    if value else "SOMBRA"
            col   = C["sun"] if value else C["cyan"]
            brd   = C["sun"] if value else C["bg3"]
            bg    = "#1a1200" if value else C["bg2"]
            self._val.setText(txt)
            self._val.setStyleSheet(f"color:{col};")
            self.setStyleSheet(
                f"background:{bg}; border-radius:8px; border:1px solid {brd};")
        elif isinstance(value, int):
            self._val.setText(str(value))
        else:
            self._val.setText(f"{value:+.4f}")

        if self._fill and self._range and not isinstance(value, bool):
            lo, hi = self._range
            pct = max(0.0, min(1.0, (value - lo) / (hi - lo or 1)))
            w = self._bg.width()
            self._fill.setFixedWidth(max(2, int(pct * w)))

# ─────────────────────────────────────────────────────────────────────────────
#  PANEL IZQUIERDO – valores actuales
# ─────────────────────────────────────────────────────────────────────────────
class DataPanel(QWidget):
    def __init__(self):
        super().__init__()
        self.setFixedWidth(230)

        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        scroll.setHorizontalScrollBarPolicy(Qt.ScrollBarPolicy.ScrollBarAlwaysOff)

        inner = QWidget()
        vl = QVBoxLayout(inner)
        vl.setContentsMargins(8, 8, 8, 8)
        vl.setSpacing(8)

        def grp(title, cards):
            box = QGroupBox(title)
            gl = QVBoxLayout(box)
            gl.setSpacing(6); gl.setContentsMargins(6, 10, 6, 6)
            for c in cards: gl.addWidget(c)
            return box

        self.cards = {
            "vd"      : ValueCard("Vel. referencia", "m/s",   C["green"],  (-1.5, 1.5)),
            "v"       : ValueCard("Vel. real",       "m/s",   C["cyan"],   (-1.5, 1.5)),
            "alpha"   : ValueCard("Inclinacion a",   "rad",   C["pink"],   (-1.0, 1.0)),
            "theta"   : ValueCard("Orientacion th",  "rad",   C["purple"], (-1.0, 1.0)),
            "oml"     : ValueCard("w izquierdo",     "rad/s", C["orange"], (-20.0,20.0)),
            "omr"     : ValueCard("w derecho",       "rad/s", C["yellow"], (-20.0,20.0)),
            "ul"      : ValueCard("Motor izq",       "V",     C["blue"],   (-12.0,12.0)),
            "ur"      : ValueCard("Motor der",       "V",     C["indigo"], (-12.0,12.0)),
            "Sl"      : ValueCard("Sensor izq",      "",      C["teal"],   (0, 255)),
            "Sr"      : ValueCard("Sensor der",      "",      C["mint"],   (0, 255)),
            "modo_sol": ValueCard("Condicion solar", "",      C["sun"]),
        }

        vl.addWidget(grp("Velocidad",      [self.cards["vd"],  self.cards["v"]]))
        vl.addWidget(grp("Angulos IMU",    [self.cards["alpha"],self.cards["theta"]]))
        vl.addWidget(grp("Encoders",       [self.cards["oml"], self.cards["omr"]]))
        vl.addWidget(grp("Motores",        [self.cards["ul"],  self.cards["ur"]]))
        vl.addWidget(grp("Sensores linea", [self.cards["Sl"],  self.cards["Sr"]]))
        vl.addWidget(grp("Modo solar",     [self.cards["modo_sol"]]))
        vl.addStretch()

        scroll.setWidget(inner)
        rl = QVBoxLayout(self)
        rl.setContentsMargins(0, 0, 0, 0)
        rl.addWidget(scroll)

    def refresh(self, data: dict):
        for k, card in self.cards.items():
            if k in data:
                card.set_value(data[k])

# ─────────────────────────────────────────────────────────────────────────────
#  GRAFICO CON BUFFER CIRCULAR
# ─────────────────────────────────────────────────────────────────────────────
class RollingChart(pg.PlotWidget):
    def __init__(self, title: str, channels: dict,
                 y_label: str = "", y_range: tuple = None):
        super().__init__()
        self.setBackground(C["bg"])
        self.setTitle(title, color=C["muted"], size="9pt")
        self.showGrid(x=True, y=True, alpha=0.12)
        self.getAxis("left").setLabel(y_label, color=C["dim"], size="8pt")
        self.getAxis("left").setStyle(tickFont=QFont("Consolas", 7))
        self.getAxis("bottom").setStyle(tickFont=QFont("Consolas", 7))
        self.getAxis("bottom").setLabel("muestras", color=C["dim"], size="7pt")
        self.setMenuEnabled(False)
        self.getPlotItem().hideButtons()
        if y_range:
            self.setYRange(*y_range)

        leg = self.addLegend(offset=(6, 6), labelTextColor=C["muted"],
                             colCount=len(channels))
        leg.setBrush(pg.mkBrush(C["bg2"] + "cc"))
        leg.setPen(pg.mkPen(C["bg3"]))

        self._curves = {}
        for name, (buf, color) in channels.items():
            curve = self.plot(pen=pg.mkPen(color, width=1.8),
                              name=name, antialias=True)
            self._curves[name] = (curve, buf)

        self.setMinimumHeight(100)

    def refresh(self):
        for _, (curve, buf) in self._curves.items():
            if buf:
                curve.setData(np.array(buf))

# ─────────────────────────────────────────────────────────────────────────────
#  PANEL DE GRAFICAS
# ─────────────────────────────────────────────────────────────────────────────
class PlotPanel(QWidget):
    def __init__(self, bufs: dict):
        super().__init__()
        lay = QVBoxLayout(self)
        lay.setContentsMargins(4, 4, 4, 4)
        lay.setSpacing(4)
        b = bufs
        defs = [
            ("Velocidad lineal",
             {"vd":(b["vd"],C["green"]), "v":(b["v"],C["cyan"])},
             "m/s", (-1.5, 1.5)),
            ("Inclinacion alpha – balance",
             {"alpha":(b["alpha"],C["pink"])},
             "rad", (-1.0, 1.0)),
            ("Orientacion theta – seguimiento linea",
             {"theta":(b["theta"],C["purple"])},
             "rad", (-1.0, 1.0)),
            ("Velocidades angulares ruedas",
             {"wL":(b["oml"],C["orange"]), "wR":(b["omr"],C["yellow"])},
             "rad/s", (-20, 20)),
            ("Voltajes motor",
             {"uL":(b["ul"],C["blue"]), "uR":(b["ur"],C["indigo"])},
             "V", (-12, 12)),
            ("Sensores de linea TCRT5000",
             {"Izq":(b["Sl"],C["teal"]), "Der":(b["Sr"],C["mint"])},
             "ADC 0-255", (0, 255)),
        ]
        self._charts = []
        for title, channels, ylabel, yrange in defs:
            c = RollingChart(title, channels, ylabel, yrange)
            lay.addWidget(c)
            self._charts.append(c)

    def refresh(self):
        for c in self._charts: c.refresh()

# ─────────────────────────────────────────────────────────────────────────────
#  PANEL DE GANANCIAS Y REFERENCIAS
#  Envía: 0xBB + id(1B) + float32_LE(4B)  →  ESP32:5006
#  IDs: 0=kpi 1=kdi 2=kpv 3=kiv 4=kpo 5=kdo 6=vd 7=ramp 8=alphad
# ─────────────────────────────────────────────────────────────────────────────
_GAIN_DEFS = [
    (0, "kpi",    "P balance"),
    (1, "kdi",    "D balance"),
    (2, "kpv",    "P veloc."),
    (3, "kiv",    "I veloc."),
    (4, "kpo",    "P orient."),
    (5, "kdo",    "D orient."),
    (6, "vd",     "Vel. ref (m/s)"),
    (7, "ramp",   "Rampa"),
    (8, "α_ref",  "Alpha ref (rad)"),
]

class GainsBar(QWidget):
    def __init__(self, send_fn):
        super().__init__()
        self._send = send_fn
        self.setFixedHeight(82)
        self.setStyleSheet(
            f"background:{C['bg2']}; border-top:1px solid {C['bg3']};")

        outer = QVBoxLayout(self)
        outer.setContentsMargins(12, 5, 12, 5)
        outer.setSpacing(4)

        title = QLabel("GANANCIAS Y REFERENCIAS  —  Enter en un campo para enviar ese valor")
        title.setStyleSheet(f"color:{C['muted']}; font-size:10px; font-weight:bold;")
        outer.addWidget(title)

        row = QHBoxLayout()
        row.setSpacing(8)
        self._fields = {}

        for gid, name, tip in _GAIN_DEFS:
            col = QVBoxLayout()
            col.setSpacing(1)

            lbl = QLabel(name)
            lbl.setStyleSheet(f"color:{C['dim']}; font-size:9px;")
            lbl.setAlignment(Qt.AlignmentFlag.AlignCenter)

            edit = QLineEdit("0.0000")
            edit.setFixedWidth(72)
            edit.setAlignment(Qt.AlignmentFlag.AlignRight)
            edit.setToolTip(tip)
            edit.returnPressed.connect(lambda g=gid, e=edit: self._on_enter(g, e))

            col.addWidget(lbl)
            col.addWidget(edit)
            row.addLayout(col)
            self._fields[gid] = edit

        row.addStretch()

        btn = QPushButton("Enviar\ntodo")
        btn.setFixedSize(64, 46)
        btn.clicked.connect(self._send_all)
        row.addWidget(btn, alignment=Qt.AlignmentFlag.AlignVCenter)

        outer.addLayout(row)

    def _on_enter(self, gid: int, edit: QLineEdit):
        try:
            self._send(gid, float(edit.text()))
        except ValueError:
            pass

    def _send_all(self):
        for gid, edit in self._fields.items():
            try:
                self._send(gid, float(edit.text()))
            except ValueError:
                pass

# ─────────────────────────────────────────────────────────────────────────────
#  VENTANA PRINCIPAL
# ─────────────────────────────────────────────────────────────────────────────
class MainWindow(QMainWindow):

    # Cabecera del .txt (compatible con MATLAB readmatrix / readtable)
    TXT_HEADER = (
        "# Balancin telemetria – generado por monitor.py\n"
        "# Cargar en MATLAB: data = readmatrix('archivo.txt', 'CommentStyle','#');\n"
        "# Columnas:\n"
        "# t[s]\tvd[m/s]\tv[m/s]\ttheta[rad]\talpha[rad]"
        "\toml[rad/s]\tomr[rad/s]\tul[V]\tur[V]\tSl\tSr\tmodo_sol\n"
    )

    def __init__(self):
        super().__init__()
        self.setWindowTitle("Balancin Monitor  –  WiFi UDP")
        self.setMinimumSize(1100, 720)
        self.setStyleSheet(QSS)

        self._worker    = None
        self._pkt_count = 0
        self._latest    = {}
        self._esp32_ip  = "—"
        self._cmd_sock  = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

        # Grabacion .txt
        self._rec_file    = None
        self._rec_t       = 0.0
        self._rec_lock    = threading.Lock()

        self._bufs = {k: collections.deque(maxlen=BUF_LEN)
                      for k in ("vd","v","theta","alpha","oml","omr","ul","ur","Sl","Sr")}

        self._build_ui()

        self._timer = QTimer()
        self._timer.timeout.connect(self._refresh_ui)
        self._timer.start(REFRESH_MS)

        self._rate_timer = QTimer()
        self._rate_timer.timeout.connect(self._update_rate)
        self._rate_timer.start(1000)
        self._rate_last = 0

    # ─── Construccion UI ──────────────────────────────────────────────────
    def _build_ui(self):
        root = QWidget(); self.setCentralWidget(root)
        rl = QVBoxLayout(root)
        rl.setContentsMargins(0, 0, 0, 0)
        rl.setSpacing(0)

        # ── Barra superior ──
        bar = QWidget()
        bar.setStyleSheet(
            f"background:{C['bg2']}; border-bottom:1px solid {C['bg3']};")
        bar.setFixedHeight(48)
        bl = QHBoxLayout(bar)
        bl.setContentsMargins(14, 6, 14, 6)
        bl.setSpacing(8)

        logo = QLabel("BALANCIN")
        logo.setStyleSheet(
            f"color:{C['accent']}; font-size:15px; font-weight:bold;"
            f" letter-spacing:2px;")

        div = QFrame()
        div.setFrameShape(QFrame.Shape.VLine)
        div.setStyleSheet(f"color:{C['bg3']};")

        # Puerto UDP
        lbl_port = QLabel("Puerto UDP:")
        lbl_port.setStyleSheet(f"color:{C['muted']}; font-size:11px;")
        self._edit_port = QLineEdit(str(UDP_PORT_DEFAULT))
        self._edit_port.setFixedWidth(60)
        self._edit_port.setValidator(QIntValidator(1024, 65535))
        self._edit_port.setToolTip(
            "Debe coincidir con UDP_PORT en el ESP32 (main.c)")

        # Boton escuchar
        self._btn_listen = QPushButton("Escuchar")
        self._btn_listen.setCheckable(True)
        self._btn_listen.setFixedWidth(95)
        self._btn_listen.clicked.connect(self._toggle_listen)

        # IP de la ESP32
        lbl_ip = QLabel("ESP32:")
        lbl_ip.setStyleSheet(f"color:{C['muted']}; font-size:11px;")
        self._lbl_ip = QLabel("—")
        self._lbl_ip.setStyleSheet(
            f"color:{C['teal']}; font-size:11px; font-family:Consolas;")

        # Estado
        self._lbl_status = QLabel("Sin conexion")
        self._lbl_status.setStyleSheet(f"color:{C['dim']}; font-size:11px;")

        divv = QFrame()
        divv.setFrameShape(QFrame.Shape.VLine)
        divv.setStyleSheet(f"color:{C['bg3']};")

        # ── Boton grabar .txt ──
        self._btn_rec = QPushButton("Grabar .txt")
        self._btn_rec.setCheckable(True)
        self._btn_rec.setFixedWidth(105)
        self._btn_rec.setToolTip(
            "Graba telemetria en .txt compatible con MATLAB.\n"
            "Al pulsar de nuevo cierra el archivo.")
        self._btn_rec.clicked.connect(self._toggle_recording)

        self._lbl_rec = QLabel("")
        self._lbl_rec.setStyleSheet(f"color:{C['dim']}; font-size:10px;")

        # Limpiar graficas
        btn_clear = QPushButton("Limpiar")
        btn_clear.setObjectName("small")
        btn_clear.clicked.connect(self._clear)

        # Indicador modo sol
        self._lbl_sol = QLabel("  SOMBRA")
        self._lbl_sol.setStyleSheet(
            f"color:{C['cyan']}; font-weight:bold; font-size:12px;"
            f" background:{C['bg3']}; border-radius:10px; padding:2px 10px;")

        # Paquetes/s
        self._lbl_rate = QLabel("0 pkt/s")
        self._lbl_rate.setStyleSheet(f"color:{C['dim']}; font-size:11px;")

        bl.addWidget(logo)
        bl.addWidget(div)
        bl.addWidget(lbl_port)
        bl.addWidget(self._edit_port)
        bl.addWidget(self._btn_listen)
        bl.addSpacing(8)
        bl.addWidget(lbl_ip)
        bl.addWidget(self._lbl_ip)
        bl.addSpacing(8)
        bl.addWidget(self._lbl_status)
        bl.addWidget(divv)
        bl.addWidget(self._btn_rec)
        bl.addWidget(self._lbl_rec)
        bl.addWidget(btn_clear)
        bl.addStretch()
        bl.addWidget(self._lbl_rate)
        bl.addSpacing(10)
        bl.addWidget(self._lbl_sol)
        rl.addWidget(bar)

        # ── Cuerpo ──
        body = QSplitter(Qt.Orientation.Horizontal)
        body.setHandleWidth(2)
        self._data_panel = DataPanel()
        body.addWidget(self._data_panel)
        self._plot_panel = PlotPanel(self._bufs)
        body.addWidget(self._plot_panel)
        body.setStretchFactor(0, 0); body.setStretchFactor(1, 1)
        body.setSizes([230, 870])
        rl.addWidget(body)

        # ── Panel de ganancias ──
        self._gains_bar = GainsBar(self._send_gain)
        rl.addWidget(self._gains_bar)

        # ── Barra inferior ──
        self._botbar = QLabel("  Esperando datos del ESP32...")
        self._botbar.setFixedHeight(20)
        self._botbar.setStyleSheet(
            f"background:{C['bg2']}; color:{C['dim']}; font-size:10px;"
            f" border-top:1px solid {C['bg3']}; padding-left:8px;")
        rl.addWidget(self._botbar)

    # ─── UDP listener ──────────────────────────────────────────────────────
    def _toggle_listen(self, checked: bool):
        if checked:
            port = int(self._edit_port.text() or UDP_PORT_DEFAULT)
            self._edit_port.setEnabled(False)
            self._worker = UDPWorker(port)
            self._worker.packet_ready.connect(self._on_packet)
            self._worker.status_msg.connect(self._on_status)
            self._worker.start()
            self._btn_listen.setText("Detener")
        else:
            self._stop_listen()

    def _stop_listen(self):
        if self._worker:
            self._worker.stop()
            self._worker.wait()
            self._worker = None
        self._btn_listen.setText("Escuchar")
        self._btn_listen.setChecked(False)
        self._edit_port.setEnabled(True)
        self._stop_recording()

    def _on_status(self, msg: str, is_error: bool):
        col = C["red"] if is_error else C["muted"]
        self._lbl_status.setStyleSheet(f"color:{col}; font-size:11px;")
        self._lbl_status.setText(msg)

    def _on_packet(self, data: dict, ip: str):
        self._pkt_count += 1
        self._latest = data
        if ip != self._esp32_ip:
            self._esp32_ip = ip
            self._lbl_ip.setText(ip)

        for key, buf in self._bufs.items():
            if key in data:
                buf.append(data[key])

        # Grabacion .txt (en el hilo de senal, ya rapido)
        with self._rec_lock:
            if self._rec_file:
                try:
                    self._rec_t += 0.01   # Ts = 10 ms
                    row = (
                        f"{self._rec_t:.3f}\t"
                        f"{data['vd']:+.6f}\t{data['v']:+.6f}\t"
                        f"{data['theta']:+.6f}\t{data['alpha']:+.6f}\t"
                        f"{data['oml']:+.6f}\t{data['omr']:+.6f}\t"
                        f"{data['ul']:+.6f}\t{data['ur']:+.6f}\t"
                        f"{data['Sl']}\t{data['Sr']}\t"
                        f"{int(data['modo_sol'])}\n"
                    )
                    self._rec_file.write(row)
                except Exception:
                    pass

    # ─── Grabacion .txt ────────────────────────────────────────────────────
    def _toggle_recording(self, checked: bool):
        if checked:
            self._start_recording()
        else:
            self._stop_recording()

    def _start_recording(self):
        ts  = datetime.datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
        default = os.path.join(
            os.path.expanduser("~"), "Documents",
            f"balancin_{ts}.txt")
        path, _ = QFileDialog.getSaveFileName(
            self, "Guardar telemetria", default,
            "Archivos de texto (*.txt)")
        if not path:
            self._btn_rec.setChecked(False)
            return

        try:
            f = open(path, "w", encoding="utf-8")
            ts_str = datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S")
            f.write(f"# Balancin telemetria  {ts_str}\n")
            f.write("# Cargar en MATLAB:\n")
            f.write("#   data = readmatrix('archivo.txt', 'CommentStyle','#');\n")
            f.write("# Columnas:\n")
            f.write("# t[s]\tvd[m/s]\tv[m/s]\ttheta[rad]\talpha[rad]"
                    "\toml[rad/s]\tomr[rad/s]\tul[V]\tur[V]\tSl\tSr\tmodo_sol\n")
            with self._rec_lock:
                self._rec_file = f
                self._rec_t    = 0.0
            name = os.path.basename(path)
            self._lbl_rec.setText(f"  Grabando: {name}")
            self._lbl_rec.setStyleSheet(
                f"color:{C['rec']}; font-size:10px; font-weight:bold;")
            self._btn_rec.setText("Detener")
        except Exception as exc:
            self._btn_rec.setChecked(False)
            self._on_status(f"No se pudo crear archivo: {exc}", True)

    def _stop_recording(self):
        with self._rec_lock:
            if self._rec_file:
                self._rec_file.flush()
                self._rec_file.close()
                self._rec_file = None
        self._btn_rec.setChecked(False)
        self._btn_rec.setText("Grabar .txt")
        self._lbl_rec.setText("")

    # ─── Envío de ganancias al ESP32 ──────────────────────────────────────
    def _send_gain(self, gain_id: int, value: float):
        if self._esp32_ip == "—":
            self._on_status("Sin IP del ESP32 — recibe al menos un paquete primero", True)
            return
        try:
            pkt = struct.pack('<BBf', 0xBB, gain_id, value)
            self._cmd_sock.sendto(pkt, (self._esp32_ip, CMD_UDP_PORT))
            names = ["kpi","kdi","kpv","kiv","kpo","kdo","vd","ramp","α_ref"]
            name  = names[gain_id] if gain_id < len(names) else str(gain_id)
            self._on_status(f"Enviado {name} = {value:.4f}  →  {self._esp32_ip}:{CMD_UDP_PORT}", False)
        except Exception as exc:
            self._on_status(f"Error enviando ganancia: {exc}", True)

    # ─── Utilidades ────────────────────────────────────────────────────────
    def _clear(self):
        for buf in self._bufs.values(): buf.clear()
        self._pkt_count = 0

    # ─── Refresco visual (20 fps) ──────────────────────────────────────────
    def _refresh_ui(self):
        if not self._latest: return

        self._data_panel.refresh(self._latest)
        self._plot_panel.refresh()

        sol = self._latest.get("modo_sol", False)
        if sol:
            self._lbl_sol.setText("  SOL")
            self._lbl_sol.setStyleSheet(
                f"color:{C['bg']}; font-weight:bold; font-size:12px;"
                f" background:{C['sun']}; border-radius:10px; padding:2px 10px;")
        else:
            self._lbl_sol.setText("  SOMBRA")
            self._lbl_sol.setStyleSheet(
                f"color:{C['cyan']}; font-weight:bold; font-size:12px;"
                f" background:{C['bg3']}; border-radius:10px; padding:2px 10px;")

        n = len(next(iter(self._bufs.values())))
        self._botbar.setText(
            f"  ESP32: {self._esp32_ip}   |   "
            f"Total paquetes: {self._pkt_count}   |   "
            f"Buf: {n}/{BUF_LEN} muestras")

    def _update_rate(self):
        rate = self._pkt_count - self._rate_last
        self._rate_last = self._pkt_count
        self._lbl_rate.setText(f"{rate} pkt/s")

    def closeEvent(self, event):
        self._stop_recording()
        self._stop_listen()
        self._cmd_sock.close()
        event.accept()

# ─────────────────────────────────────────────────────────────────────────────
#  PUNTO DE ENTRADA
# ─────────────────────────────────────────────────────────────────────────────
def main():
    pg.setConfigOptions(antialias=True, foreground=C["muted"], background=C["bg"])
    app = QApplication(sys.argv)
    app.setFont(QFont("Segoe UI", 9))
    win = MainWindow()
    win.show()
    sys.exit(app.exec())

if __name__ == "__main__":
    main()
