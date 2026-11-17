"""
XM10 System Architecture Visualizer
=====================================
HARfinal 프로젝트 전체 시스템 구조와 제어 흐름을 시각화합니다.

탭 구성:
  1. 시스템 아키텍처  - 레이어별 소프트웨어 구조
  2. 제어 흐름       - 2ms IPO 루프 및 FreeRTOS 스레드
  3. 제어 알고리즘   - 5단계 Physical AI 파이프라인
  4. 데이터 흐름     - 센서 → 알고리즘 → 액추에이터
  5. CSV 분석        - 기존 CSV 파일 로드 및 시각화

의존성:
    pip install pyqt5 pyqtgraph numpy matplotlib networkx
"""

import sys
import os
import numpy as np
from PyQt5 import QtWidgets, QtGui, QtCore
from PyQt5.QtCore import Qt
import pyqtgraph as pg

# ─────────────────────────────────────────────────────────────────────────────
# Color Palette
# ─────────────────────────────────────────────────────────────────────────────
C = {
    'bg':        '#1e1e2e',
    'surface':   '#2a2a3c',
    'border':    '#45475a',
    'text':      '#cdd6f4',
    'subtext':   '#a6adc8',
    'blue':      '#89b4fa',
    'green':     '#a6e3a1',
    'yellow':    '#f9e2af',
    'red':       '#f38ba8',
    'orange':    '#fab387',
    'purple':    '#cba6f7',
    'teal':      '#94e2d5',
    'pink':      '#f5c2e7',
    'sky':       '#89dceb',
}

LAYER_COLORS = [C['blue'], C['teal'], C['green'], C['yellow'], C['orange'], C['red']]

# ─────────────────────────────────────────────────────────────────────────────
# Helper: rounded-rect item for pyqtgraph scene
# ─────────────────────────────────────────────────────────────────────────────
class BoxItem(QtWidgets.QGraphicsRectItem):
    def __init__(self, x, y, w, h, color, label, sublabel='', radius=8):
        super().__init__(x, y, w, h)
        self.label = label
        self.sublabel = sublabel
        self.color = color
        self.setAcceptHoverEvents(True)

        pen = QtGui.QPen(QtGui.QColor(color))
        pen.setWidth(2)
        self.setPen(pen)
        fill = QtGui.QColor(color)
        fill.setAlpha(40)
        self.setBrush(QtGui.QBrush(fill))

    def paint(self, painter, option, widget):
        painter.setRenderHint(QtGui.QPainter.Antialiasing)
        fill = QtGui.QColor(self.color)
        fill.setAlpha(40 if not self.isUnderMouse() else 80)
        pen = QtGui.QPen(QtGui.QColor(self.color))
        pen.setWidth(2)
        painter.setPen(pen)
        painter.setBrush(QtGui.QBrush(fill))
        r = self.rect()
        painter.drawRoundedRect(r, 8, 8)

        # Main label
        font = QtGui.QFont('Segoe UI', 9, QtGui.QFont.Bold)
        painter.setFont(font)
        painter.setPen(QtGui.QPen(QtGui.QColor(C['text'])))
        painter.drawText(r, Qt.AlignCenter | Qt.AlignVCenter,
                         self.label if not self.sublabel else self.label)

        # Sub label
        if self.sublabel:
            font2 = QtGui.QFont('Segoe UI', 7)
            painter.setFont(font2)
            painter.setPen(QtGui.QPen(QtGui.QColor(C['subtext'])))
            sub_r = QtCore.QRectF(r.x(), r.y() + r.height() * 0.55,
                                  r.width(), r.height() * 0.4)
            painter.drawText(sub_r, Qt.AlignCenter | Qt.AlignVCenter, self.sublabel)


class ArrowItem(QtWidgets.QGraphicsLineItem):
    def __init__(self, x1, y1, x2, y2, color=C['subtext'], bidirect=False):
        super().__init__(x1, y1, x2, y2)
        self.bidirect = bidirect
        pen = QtGui.QPen(QtGui.QColor(color))
        pen.setWidth(1)
        pen.setStyle(Qt.DashLine)
        self.setPen(pen)

    def paint(self, painter, option, widget):
        painter.setRenderHint(QtGui.QPainter.Antialiasing)
        super().paint(painter, option, widget)
        # Arrowhead at end
        self._draw_head(painter, self.line().p1(), self.line().p2())
        if self.bidirect:
            self._draw_head(painter, self.line().p2(), self.line().p1())

    def _draw_head(self, painter, frm, to):
        angle = np.arctan2(to.y() - frm.y(), to.x() - frm.x())
        size = 8
        p1 = QtCore.QPointF(
            to.x() - size * np.cos(angle - 0.4),
            to.y() - size * np.sin(angle - 0.4))
        p2 = QtCore.QPointF(
            to.x() - size * np.cos(angle + 0.4),
            to.y() - size * np.sin(angle + 0.4))
        poly = QtGui.QPolygonF([to, p1, p2])
        painter.setBrush(QtGui.QBrush(self.pen().color()))
        painter.setPen(self.pen())
        painter.drawPolygon(poly)


# ─────────────────────────────────────────────────────────────────────────────
# Tab 1: System Architecture
# ─────────────────────────────────────────────────────────────────────────────
def build_architecture_scene():
    scene = QtWidgets.QGraphicsScene()
    scene.setBackgroundBrush(QtGui.QBrush(QtGui.QColor(C['bg'])))

    W, H = 900, 80
    GAP = 14
    LAYERS = [
        ("USER APPLICATION LAYER",
         "XM_Apps/User_Algorithm — User_Setup() + User_Loop()",
         C['purple']),
        ("XM API FACADE",
         "xm_api_data  ·  xm_api_tsm  ·  xm_api_usb  ·  xm_api_external_io",
         C['blue']),
        ("SYSTEM CORE  (core_process.h)",
         "_FetchAllInputs()  ──  User_Loop()  ──  _FlushAllOutputs()   @2ms",
         C['teal']),
        ("DEVICE DRIVERS & SERVICES",
         "cm_drv (H10 exo)  ·  emg_hub_drv  ·  imu_hub_drv  ·  DOP Manager",
         C['green']),
        ("IOIF ABSTRACTION",
         "CAN-FD  ·  UART  ·  ADC  ·  GPIO  ·  USB OTG  ·  I2C  ·  DMA",
         C['yellow']),
        ("STM32 HAL  +  FreeRTOS",
         "UserTask(RT6)  ·  RxTask(RT7)  ·  StartupTask(RT7)  ·  DefaultTask(Low)",
         C['orange']),
        ("PHYSICAL HARDWARE",
         "STM32H743 @ 480MHz  ←CAN-FD→  KIT H10 Exoskeleton  ←USB→  PC",
         C['red']),
    ]

    y = 20
    items = []
    for i, (title, sub, color) in enumerate(LAYERS):
        box = BoxItem(10, y, W, H, color, title, sub)
        scene.addItem(box)
        items.append((box, y + H / 2))
        y += H + GAP

    # Bidirectional arrows between layers
    for i in range(len(items) - 1):
        _, y1 = items[i]
        _, y2 = items[i + 1]
        cx = W / 2 + 10
        arr = ArrowItem(cx, y1 + H / 2, cx, y2 - H / 2, C['subtext'], bidirect=True)
        scene.addItem(arr)

    total_h = y + 20
    scene.setSceneRect(0, 0, W + 20, total_h)
    return scene


# ─────────────────────────────────────────────────────────────────────────────
# Tab 2: Control Flow (IPO Loop)
# ─────────────────────────────────────────────────────────────────────────────
def build_flow_scene():
    scene = QtWidgets.QGraphicsScene()
    scene.setBackgroundBrush(QtGui.QBrush(QtGui.QColor(C['bg'])))

    # FreeRTOS tasks on the left
    task_data = [
        ("StartupTask\nRT-Priority 7", "Bootup, PnP device discovery", C['yellow'], 20, 20),
        ("RxTask\nRT-Priority 7", "CANFD PDO reception\n(preempts UserTask ~10-50µs)", C['orange'], 20, 150),
        ("UserTask\nRT-Priority 6", "Main 2ms IPO loop\n(your algorithm runs here)", C['blue'], 20, 310),
        ("DefaultTask\nLow-Priority", "System housekeeping\n(mostly suspended)", C['purple'], 20, 500),
    ]
    for label, sub, color, x, y in task_data:
        box = BoxItem(x, y, 230, 90, color, label, sub)
        scene.addItem(box)

    # IPO steps on the right
    ipo_x = 340
    ipo_steps = [
        (ipo_x, 80,  480, 100, C['teal'],   "INPUT PHASE",
         "_FetchAllInputs()\nH10 angle/torque · GRF/FSR · EMG · IMU · GPIO/ADC"),
        (ipo_x, 220, 480, 100, C['blue'],   "LOGIC PHASE  (User_Loop)",
         "Your Algorithm — 2ms budget\nRead XM.status.*  →  Compute  →  XM_Set*()"),
        (ipo_x, 360, 480, 100, C['green'],  "OUTPUT PHASE",
         "_FlushAllOutputs()\nCAN-FD P/I/F-Vectors · USB CDC/MSC stream · LED"),
        (ipo_x, 500, 480, 80,  C['yellow'], "BLOCKING WAIT",
         "Suspend UserTask until next H10 heartbeat\n(~2ms cadence, ensures deterministic timing)"),
    ]
    ipo_items = []
    for x, y, w, h, color, label, sub in ipo_steps:
        box = BoxItem(x, y, w, h, color, label, sub)
        scene.addItem(box)
        ipo_items.append((x, y, w, h))

    # Arrows between IPO steps
    for i in range(len(ipo_items) - 1):
        _, y1, w1, h1 = ipo_items[i]
        x2, y2, _, _ = ipo_items[i + 1]
        cx = ipo_x + 480 / 2
        arr = ArrowItem(cx, y1 + h1, cx, y2, C['teal'])
        scene.addItem(arr)

    # Loop-back arrow: WAIT → INPUT
    lx = ipo_x + 480 + 30
    x0, y0, w0, h0 = ipo_items[0]
    xl, yl, wl, hl = ipo_items[-1]
    arr_back = ArrowItem(lx, yl + hl / 2, lx, y0 + h0 / 2, C['orange'])
    scene.addItem(arr_back)
    arr_h1 = ArrowItem(lx, yl + hl / 2, xl + wl, yl + hl / 2, C['orange'])
    scene.addItem(arr_h1)
    arr_h2 = ArrowItem(lx, y0 + h0 / 2, x0 + w0, y0 + h0 / 2, C['orange'])
    scene.addItem(arr_h2)

    # Label for the loop-back
    lbl = scene.addText("2ms\nLoop", QtGui.QFont('Segoe UI', 8))
    lbl.setDefaultTextColor(QtGui.QColor(C['orange']))
    lbl.setPos(lx + 5, (y0 + yl) / 2)

    # Connections: UserTask → IPO box
    arr_ut = ArrowItem(250, 355, ipo_x, 280, C['blue'])
    scene.addItem(arr_ut)

    scene.setSceneRect(0, 0, 900, 650)
    return scene


# ─────────────────────────────────────────────────────────────────────────────
# Tab 3: Control Algorithms  (5-Stage Physical AI)
# ─────────────────────────────────────────────────────────────────────────────
def build_algo_scene():
    scene = QtWidgets.QGraphicsScene()
    scene.setBackgroundBrush(QtGui.QBrush(QtGui.QColor(C['bg'])))

    stages = [
        ("Stage 1: Physical Transparency",
         [("Impedance Control\n(Ex.20)", "τ = -Kp(θ-θd) - Kd·ω", C['teal']),
          ("Gravity Comp\n(Ex.21)",      "τ_grav = f(model)", C['teal']),
          ("FF+FB Hybrid\n(Ex.30)",      "Model FF + PD FB", C['teal']),
          ("DOB\n(Ex.31)",               "τ_ext_est = disturbance observer", C['teal'])]),
        ("Stage 2: Intent Sensing",
         [("TinyAI\n(Ex.16)",    "3-layer NN on-device", C['blue']),
          ("FSM Gait\n(Ex.17)",  "7-phase walking FSM", C['blue']),
          ("CPG Osc\n(Ex.22)",   "φ[k+1]=φ[k]+dt(ω+ε·F·cos φ)", C['blue']),
          ("GRF Phase\n(Ex.32)", "Heel-strike → gait phase", C['blue'])]),
        ("Stage 3: Adaptive Assistance",
         [("Active Assist\n(Ex.12)",     "Intent → torque scaling", C['green']),
          ("PD Control\n(Ex.14)",        "τ = Kp·e + Kd·ė", C['green']),
          ("Gait Adaptive\n(Ex.23)",     "sin(phase)·τ_max", C['green']),
          ("Admittance\n(Ex.28)",        "F_ext → Δx_d", C['green'])]),
        ("Stage 4: Machine Learning",
         [("Inv Pendulum\n(Ex.15)", "τ = MgL·sin(θ)", C['yellow']),
          ("HZD\n(Ex.24)",          "Bézier virtual constraints", C['yellow']),
          ("ILC\n(Ex.26)",          "Trial-by-trial learning", C['yellow']),
          ("MRAC\n(Ex.27)",         "Online adaptive control", C['yellow'])]),
        ("Stage 5: Shared Autonomy",
         [("Passive Mode\n(Ex.11)",   "P-Vector trajectories", C['orange']),
          ("Resistive\n(Ex.13)",      "Anti-motion (rehab)", C['orange']),
          ("Bilateral\n(Ex.29)",      "L↔R sync (gait training)", C['orange']),
          ("Kinesthetic\n(Ex.33)",    "Expert capture → VLA dataset", C['orange'])]),
    ]

    x_stage = 10
    y_start = 20
    stage_h = 160
    stage_gap = 24
    BOX_W, BOX_H = 160, 90

    for si, (stage_name, algs) in enumerate(stages):
        y_s = y_start + si * (stage_h + stage_gap)
        # Stage label
        lbl = scene.addText(stage_name, QtGui.QFont('Segoe UI', 10, QtGui.QFont.Bold))
        lbl.setDefaultTextColor(QtGui.QColor(LAYER_COLORS[si]))
        lbl.setPos(x_stage, y_s)

        for ai, (alg_name, formula, color) in enumerate(algs):
            ax = x_stage + ai * (BOX_W + 12)
            ay = y_s + 34
            box = BoxItem(ax, ay, BOX_W, BOX_H, color, alg_name, formula)
            scene.addItem(box)

        # Stage background band
        band_w = len(algs) * (BOX_W + 12) - 12
        band = QtWidgets.QGraphicsRectItem(x_stage - 6, y_s - 4, band_w + 12, stage_h)
        pen = QtGui.QPen(QtGui.QColor(LAYER_COLORS[si]))
        pen.setWidth(1)
        pen.setStyle(Qt.DotLine)
        band.setPen(pen)
        band.setBrush(QtGui.QBrush(Qt.NoBrush))
        scene.addItem(band)

    total_h = y_start + len(stages) * (stage_h + stage_gap) + 20
    scene.setSceneRect(0, 0, 780, total_h)
    return scene


# ─────────────────────────────────────────────────────────────────────────────
# Tab 4: Data Flow (Sensors → Algorithm → Actuators)
# ─────────────────────────────────────────────────────────────────────────────
def build_dataflow_scene():
    scene = QtWidgets.QGraphicsScene()
    scene.setBackgroundBrush(QtGui.QBrush(QtGui.QColor(C['bg'])))

    # Sensors (left column)
    sensors = [
        ("H10 Exoskeleton\n(KIT H10)", "Hip angle, torque, IMU\nCAN-FD PDO @ 2ms", C['blue']),
        ("GRF Shoes\n(MarvelDex)", "14ch FSR per foot\nCAN-FD serial", C['teal']),
        ("EMG Hub\n(8-channel)", "Bilateral EMG amplifiers\nCAN-FD", C['green']),
        ("External IMU\n(Xsens MTi-630)", "Global 9-DOF\nUART", C['yellow']),
        ("External GPIO/ADC\n(12-ch expansion)", "Analog/Digital sensors\nIOIF abstraction", C['orange']),
    ]

    # Actuators (right column)
    actuators = [
        ("Hip Torque\n(L / R)", "XM_SetAssistTorque()\n→ F-Vector CAN-FD SDO", C['blue']),
        ("Hip Position\n(P-Vector)", "XM_SendPVector()\n→ Trajectory profile", C['teal']),
        ("Hip Impedance\n(I-Vector)", "XM_SendIVector()\n→ Virtual spring/damper", C['green']),
        ("USB Data\n(PhAI Studio)", "XM_SendUsbDataWithId()\n→ Real-time visualization", C['yellow']),
        ("LED / Debug", "XM_SetLedState()\n→ Status indication", C['orange']),
    ]

    BW, BH = 200, 75
    SX, AX = 20, 620
    cy = 30

    sensor_mids = []
    for i, (name, sub, color) in enumerate(sensors):
        y = cy + i * (BH + 15)
        box = BoxItem(SX, y, BW, BH, color, name, sub)
        scene.addItem(box)
        sensor_mids.append(y + BH / 2)

    actuator_mids = []
    for i, (name, sub, color) in enumerate(actuators):
        y = cy + i * (BH + 15)
        box = BoxItem(AX, y, BW, BH, color, name, sub)
        scene.addItem(box)
        actuator_mids.append(y + BH / 2)

    # Central algorithm box
    CX, CY, CW, CH = 290, 140, 240, 200
    center = BoxItem(CX, CY, CW, CH, C['purple'], "USER ALGORITHM\n(User_Loop @ 2ms)",
                     "XM.status.* (read)\nXM_Set*() (write)\nFSM / CPG / PID / ML")
    scene.addItem(center)

    # Arrows: sensors → center
    for mid_y in sensor_mids:
        arr = ArrowItem(SX + BW, mid_y, CX, CY + CH / 2, C['teal'])
        scene.addItem(arr)

    # Arrows: center → actuators
    for mid_y in actuator_mids:
        arr = ArrowItem(CX + CW, CY + CH / 2, AX, mid_y, C['orange'])
        scene.addItem(arr)

    # XM API label
    api_lbl = scene.addText("XM API\nFacade", QtGui.QFont('Segoe UI', 8, QtGui.QFont.Bold))
    api_lbl.setDefaultTextColor(QtGui.QColor(C['subtext']))
    api_lbl.setPos(CX + CW / 2 - 30, CY - 40)

    scene.setSceneRect(0, 0, 850, 520)
    return scene


# ─────────────────────────────────────────────────────────────────────────────
# Tab 5: CSV Analyzer
# ─────────────────────────────────────────────────────────────────────────────
class CsvAnalyzerWidget(QtWidgets.QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.data = None
        self.cols = None
        self._build_ui()

    def _build_ui(self):
        layout = QtWidgets.QVBoxLayout(self)
        layout.setContentsMargins(8, 8, 8, 8)

        # Top bar
        top = QtWidgets.QHBoxLayout()
        layout.addLayout(top)
        self.lbl_path = QtWidgets.QLabel("CSV 파일을 열거나 여기로 드래그하세요")
        self.lbl_path.setStyleSheet(f"color: {C['subtext']};")
        top.addWidget(self.lbl_path, stretch=1)
        btn = QtWidgets.QPushButton("CSV 열기")
        btn.clicked.connect(self._open_csv)
        btn.setStyleSheet(f"background:{C['blue']}; color:#fff; border-radius:6px; padding:5px 12px;")
        top.addWidget(btn)

        # Channel selector
        mid = QtWidgets.QHBoxLayout()
        layout.addLayout(mid)
        mid.addWidget(QtWidgets.QLabel("채널 선택:"))
        self.ch_list = QtWidgets.QListWidget()
        self.ch_list.setSelectionMode(QtWidgets.QAbstractItemView.MultiSelection)
        self.ch_list.setMaximumHeight(60)
        self.ch_list.setStyleSheet(f"background:{C['surface']}; color:{C['text']};")
        mid.addWidget(self.ch_list, stretch=1)
        btn_plot = QtWidgets.QPushButton("그래프 업데이트")
        btn_plot.clicked.connect(self._update_plots)
        btn_plot.setStyleSheet(f"background:{C['green']}; color:#000; border-radius:6px; padding:5px 12px;")
        mid.addWidget(btn_plot)

        # Stats label
        self.stats_lbl = QtWidgets.QLabel("")
        self.stats_lbl.setStyleSheet(f"color:{C['yellow']}; font-size:9pt;")
        layout.addWidget(self.stats_lbl)

        # Plot grid
        self.plot_container = pg.GraphicsLayoutWidget()
        self.plot_container.setBackground(C['bg'])
        layout.addWidget(self.plot_container, stretch=1)

        self.setAcceptDrops(True)

    def dragEnterEvent(self, e):
        if e.mimeData().hasUrls():
            e.accept()

    def dropEvent(self, e):
        for url in e.mimeData().urls():
            path = url.toLocalFile()
            if path.endswith('.csv'):
                self._load(path)
                break

    def _open_csv(self):
        # Try to default to the data directory
        data_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'CDC', 'data')
        if not os.path.isdir(data_dir):
            data_dir = os.path.dirname(os.path.abspath(__file__))
        path, _ = QtWidgets.QFileDialog.getOpenFileName(
            self, "CSV 열기", data_dir, "CSV (*.csv);;All (*)")
        if path:
            self._load(path)

    def _load(self, path):
        try:
            with open(path, 'r') as f:
                header = f.readline().strip()
            self.cols = header.split(',')
            self.data = np.genfromtxt(path, delimiter=',', skip_header=1)
            if self.data.ndim == 1:
                self.data = self.data.reshape(1, -1)
            # Clamp to actual column count
            self.data = self.data[:, :len(self.cols)]
        except Exception as e:
            QtWidgets.QMessageBox.critical(self, "오류", str(e))
            return

        self.lbl_path.setText(os.path.basename(path))

        skip = {'time_s', 'pc_time_s', 'seq_id', 'module_id', 'tx_drops'}
        self.ch_list.clear()
        for i, c in enumerate(self.cols):
            if c not in skip:
                item = QtWidgets.QListWidgetItem(f"[{i}] {c}")
                item.setData(Qt.UserRole, i)
                self.ch_list.addItem(item)
        for i in range(self.ch_list.count()):
            self.ch_list.item(i).setSelected(True)

        rows, cols_n = self.data.shape
        # Compute basic stats for first data column
        data_cols_idx = [i for i, c in enumerate(self.cols) if c not in skip]
        if data_cols_idx and rows > 1:
            vals = self.data[:, data_cols_idx[0]]
            valid = vals[np.isfinite(vals)]
            self.stats_lbl.setText(
                f"행: {rows}  채널: {len(data_cols_idx)}  "
                f"[{self.cols[data_cols_idx[0]]}]  "
                f"min={valid.min():.3f}  max={valid.max():.3f}  mean={valid.mean():.3f}"
            )
        self._update_plots()

    def _update_plots(self):
        if self.data is None:
            return
        self.plot_container.clear()

        selected = [item.data(Qt.UserRole) for item in self.ch_list.selectedItems()]
        if not selected:
            return

        skip = {'time_s', 'pc_time_s', 'seq_id', 'module_id', 'tx_drops'}
        time_idx = next((i for i, c in enumerate(self.cols) if c == 'time_s'), 0)
        x = self.data[:, time_idx]
        if len(x) > 1:
            x = x - x[0]

        # Group channels: max 3 per plot, max 6 plots
        COLS_PER_PLOT = 3
        groups = [selected[i:i+COLS_PER_PLOT] for i in range(0, min(len(selected), 18), COLS_PER_PLOT)]
        PLOT_COLS = 2

        plots = []
        for gi, group in enumerate(groups[:6]):
            row, col = gi // PLOT_COLS, gi % PLOT_COLS
            p = self.plot_container.addPlot(row=row, col=col)
            p.setBackground(C['surface'])
            p.showGrid(x=True, y=True, alpha=0.2)
            p.addLegend(offset=(5, 5))
            p.getAxis('bottom').setPen(C['subtext'])
            p.getAxis('left').setPen(C['subtext'])
            p.getAxis('bottom').setTextPen(C['subtext'])
            p.getAxis('left').setTextPen(C['subtext'])

            for ki, ci in enumerate(group):
                if ci >= self.data.shape[1]:
                    continue
                y = self.data[:, ci].copy()
                # Clamp outliers
                y[~np.isfinite(y)] = np.nan
                mask = np.abs(y) < 1e6
                y[~mask] = np.nan
                color = pg.intColor(ki + gi * COLS_PER_PLOT, hues=max(9, len(selected)))
                name = self.cols[ci] if ci < len(self.cols) else f"ch{ci}"
                p.plot(x, y, pen=pg.mkPen(color, width=1.5), name=name, connect='finite')

            plots.append(p)

        # Link X-axes
        if len(plots) > 1:
            for p in plots[1:]:
                p.setXLink(plots[0])


# ─────────────────────────────────────────────────────────────────────────────
# Scrollable scene view
# ─────────────────────────────────────────────────────────────────────────────
class SceneTab(QtWidgets.QGraphicsView):
    def __init__(self, scene, parent=None):
        super().__init__(scene, parent)
        self.setRenderHint(QtGui.QPainter.Antialiasing)
        self.setDragMode(QtWidgets.QGraphicsView.ScrollHandDrag)
        self.setTransformationAnchor(QtWidgets.QGraphicsView.AnchorUnderMouse)
        self.setStyleSheet(f"background:{C['bg']}; border:none;")

    def wheelEvent(self, e):
        factor = 1.15 if e.angleDelta().y() > 0 else 0.87
        self.scale(factor, factor)


# ─────────────────────────────────────────────────────────────────────────────
# Main Window
# ─────────────────────────────────────────────────────────────────────────────
class MainWindow(QtWidgets.QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("XM10 / HARfinal — System Visualizer")
        self.resize(1100, 750)
        self._apply_style()

        tabs = QtWidgets.QTabWidget()
        tabs.setStyleSheet(f"""
            QTabBar::tab {{
                background:{C['surface']}; color:{C['subtext']};
                padding:8px 18px; border-radius:6px 6px 0 0;
            }}
            QTabBar::tab:selected {{
                background:{C['blue']}; color:#fff;
            }}
        """)

        tabs.addTab(SceneTab(build_architecture_scene()), "① 시스템 아키텍처")
        tabs.addTab(SceneTab(build_flow_scene()),         "② 제어 흐름 (IPO)")
        tabs.addTab(SceneTab(build_algo_scene()),         "③ 제어 알고리즘")
        tabs.addTab(SceneTab(build_dataflow_scene()),     "④ 데이터 흐름")
        tabs.addTab(CsvAnalyzerWidget(),                  "⑤ CSV 분석")

        self.setCentralWidget(tabs)
        self._add_hint_bar()

    def _apply_style(self):
        self.setStyleSheet(f"""
            QMainWindow {{ background:{C['bg']}; }}
            QWidget {{ background:{C['bg']}; color:{C['text']};
                       font-family:'Segoe UI',sans-serif; font-size:10pt; }}
            QLabel {{ color:{C['text']}; }}
            QPushButton {{ background:{C['blue']}; color:#fff;
                           border-radius:6px; padding:5px 12px; border:none; }}
            QPushButton:hover {{ background:{C['purple']}; }}
        """)

    def _add_hint_bar(self):
        bar = self.statusBar()
        bar.setStyleSheet(f"background:{C['surface']}; color:{C['subtext']}; border-top:1px solid {C['border']};")
        bar.showMessage("① ~ ④ 탭: 마우스 휠로 확대/축소, 드래그로 이동 | ⑤ 탭: CSV 파일 열기 또는 드래그&드롭")


# ─────────────────────────────────────────────────────────────────────────────
# Entry Point
# ─────────────────────────────────────────────────────────────────────────────
def main():
    app = QtWidgets.QApplication(sys.argv)
    app.setStyle("Fusion")
    pg.setConfigOptions(antialias=True)
    win = MainWindow()
    win.show()
    sys.exit(app.exec_())


if __name__ == '__main__':
    main()
