"""
User Algorithm Visualizer — XM_Apps/User_Algorithm
====================================================
XM_Apps/User_Algorithm 내 알고리즘의 로직을 인터랙티브하게 시각화합니다.

의존성:
    pip install pyqt5 pyqtgraph numpy
"""

import sys
import numpy as np
from PyQt5 import QtWidgets, QtGui, QtCore
from PyQt5.QtCore import Qt
import pyqtgraph as pg

# ─────────────────────────────────────────────────────────────────────────────
# Color Palette
# ─────────────────────────────────────────────────────────────────────────────
C = {
    'bg':      '#1e1e2e', 'surface': '#2a2a3c', 'border': '#45475a',
    'text':    '#cdd6f4', 'sub':     '#a6adc8',
    'blue':    '#89b4fa', 'teal':    '#94e2d5', 'green':  '#a6e3a1',
    'yellow':  '#f9e2af', 'orange':  '#fab387', 'red':    '#f38ba8',
    'purple':  '#cba6f7', 'pink':    '#f5c2e7', 'sky':    '#89dceb',
}

GLOBAL_STYLESHEET = f"""
QWidget       {{ background:{C['bg']}; color:{C['text']};
                 font-family:'Segoe UI',sans-serif; font-size:10pt; }}
QTabWidget::pane {{ border:1px solid {C['border']}; }}
QTabBar::tab  {{ background:{C['surface']}; color:{C['sub']};
                 padding:8px 16px; border-radius:6px 6px 0 0; }}
QTabBar::tab:selected {{ background:{C['blue']}; color:#fff; }}
QGroupBox     {{ border:1px solid {C['border']}; border-radius:8px;
                 margin-top:6px; padding:6px; }}
QGroupBox::title {{ color:{C['yellow']}; subcontrol-origin:margin; padding:2px 8px; }}
QLabel        {{ color:{C['text']}; }}
QScrollArea   {{ border:none; }}
"""

PAD = 10  # 박스 내부 텍스트 여백 (px)

# ─────────────────────────────────────────────────────────────────────────────
# Graphics helpers
# ─────────────────────────────────────────────────────────────────────────────
class RoundBox(QtWidgets.QGraphicsRectItem):
    """
    둥근 모서리 박스.
    - label  : 상단 굵은 글씨
    - sub    : 하단 작은 글씨 (없으면 label이 전체 영역 사용)
    - PAD    : 좌우 여백으로 삐져나가지 않음
    - Qt.TextWordWrap 으로 자동 줄바꿈
    """
    def __init__(self, x, y, w, h, bg, border, label, sub='', font_size=9):
        super().__init__(x, y, w, h)
        self._bg = bg; self._br = border
        self._label = label; self._sub = sub; self._fs = font_size
        self.setAcceptHoverEvents(True)

    def paint(self, p, opt, w):
        p.setRenderHint(QtGui.QPainter.Antialiasing)
        r = self.rect()

        # 배경
        fill = QtGui.QColor(self._bg)
        fill.setAlpha(65 if not self.isUnderMouse() else 100)
        p.setPen(QtGui.QPen(QtGui.QColor(self._br), 2))
        p.setBrush(QtGui.QBrush(fill))
        p.drawRoundedRect(r, 9, 9)

        # 텍스트 영역 (좌우 PAD 여백)
        inner = QtCore.QRectF(r.x() + PAD, r.y() + 4,
                              r.width() - PAD * 2, r.height() - 8)
        flags = Qt.AlignCenter | Qt.AlignVCenter | Qt.TextWordWrap

        if self._sub:
            # 상단 60% : label (굵게)
            top = QtCore.QRectF(inner.x(), inner.y(),
                                inner.width(), inner.height() * 0.55)
            p.setFont(QtGui.QFont('Segoe UI', self._fs, QtGui.QFont.Bold))
            p.setPen(QtGui.QPen(QtGui.QColor(C['text'])))
            p.drawText(top, flags, self._label)

            # 하단 40% : sub (작게)
            bot = QtCore.QRectF(inner.x(), inner.y() + inner.height() * 0.55,
                                inner.width(), inner.height() * 0.42)
            p.setFont(QtGui.QFont('Segoe UI', max(self._fs - 2, 7)))
            p.setPen(QtGui.QPen(QtGui.QColor(C['sub'])))
            p.drawText(bot, flags, self._sub)
        else:
            p.setFont(QtGui.QFont('Segoe UI', self._fs, QtGui.QFont.Bold))
            p.setPen(QtGui.QPen(QtGui.QColor(C['text'])))
            p.drawText(inner, flags, self._label)


def arrow(scene, x1, y1, x2, y2, color='sub', w=1, style=Qt.DashLine):
    pen = QtGui.QPen(QtGui.QColor(C[color]), w, style)
    scene.addLine(x1, y1, x2, y2, pen)
    ang = np.arctan2(y2 - y1, x2 - x1)
    sz = 7
    pts = [QtCore.QPointF(x2, y2),
           QtCore.QPointF(x2 - sz * np.cos(ang - 0.4), y2 - sz * np.sin(ang - 0.4)),
           QtCore.QPointF(x2 - sz * np.cos(ang + 0.4), y2 - sz * np.sin(ang + 0.4))]
    scene.addPolygon(QtGui.QPolygonF(pts),
                     QtGui.QPen(QtGui.QColor(C[color]), w),
                     QtGui.QBrush(QtGui.QColor(C[color])))


def lbl(scene, x, y, txt, color='sub', size=8, bold=False, max_width=0):
    """씬에 텍스트 아이템 추가. max_width > 0 이면 자동 줄바꿈."""
    t = scene.addText(txt, QtGui.QFont(
        'Segoe UI', size,
        QtGui.QFont.Bold if bold else QtGui.QFont.Normal))
    t.setDefaultTextColor(QtGui.QColor(C[color]))
    if max_width > 0:
        t.setTextWidth(max_width)
    t.setPos(x, y)
    return t


def box(scene, x, y, w, h, bg, border, label, sub='', fs=9):
    b = RoundBox(x, y, w, h, bg, border, label, sub, fs)
    scene.addItem(b)
    return b


def scene_view(scene):
    v = QtWidgets.QGraphicsView(scene)
    v.setRenderHint(QtGui.QPainter.Antialiasing)
    v.setDragMode(QtWidgets.QGraphicsView.ScrollHandDrag)
    v.setTransformationAnchor(QtWidgets.QGraphicsView.AnchorUnderMouse)
    v.setStyleSheet(f'background:{C["bg"]}; border:none;')

    def _wheel(e):
        f = 1.15 if e.angleDelta().y() > 0 else 0.87
        v.scale(f, f)
    v.wheelEvent = _wheel
    return v


# ─────────────────────────────────────────────────────────────────────────────
# Tab 1 — Algorithm Overview Cards
# ─────────────────────────────────────────────────────────────────────────────
def build_overview():
    scene = QtWidgets.QGraphicsScene()
    scene.setBackgroundBrush(QtGui.QBrush(QtGui.QColor(C['bg'])))

    # 카드 데이터: (파일명, 색, 입력내용, 처리내용, 출력내용, 파라미터내용)
    cards = [
        ("Final_Encoder_ex01.c", C['blue'],
         "모터 엔코더 각도\nleftHipMotorAngle",
         "sign 보정 → 오프셋 제거\n→ 임계값 비교 → 펄스 트리거",
         "XM_SetAssistTorqueLH/RH()\n[120ms 펄스]",
         "threshold=20°\nrearm=15°\npulse=120ms  limit=2.5Nm"),
        ("Final_FSR_Fuzzy_Logic.c", C['teal'],
         "FSR 4채널 ADC\n(PF3~PF6: LToe/LHeel/RToe/RHeel)",
         "캘리브레이션 → LPF\n→ 정규화 → tanh 퍼지\n→ 보행 위상 4종",
         "TERMINAL_STANCE → SWING\n전환 시 토크 펄스 출력",
         "heel/toe_threshold=0.35\nsensitivity=12\npulse=100ms"),
        ("Final_EMG.c", C['green'],
         "EMG 4채널 ADC\n(PF3~PF6, 바이어스 ~1.65V)",
         "바이어스 제거 → HPF(80Hz)\n→ 정류 → 포락선LPF(5Hz)\n→ 데드밴드 → 비례 변환",
         "포락선 크기 비례 연속 토크\n(proportional assist)\nTSM 상태 머신 적용",
         "deadband=0.02V\nassist_scale=0.5\nmax_torque=2.5Nm"),
        ("passive_mode.c", C['purple'],
         "H10 모드 전환 감지\n(STANDBY ↔ ASSIST)",
         "Homing(0°) →\n임피던스 설정 →\nP-Vector 왕복 큐잉",
         "XM_SendPVector()\n±25° 왕복 (250°/s)\nMSC 데이터 로깅",
         "MAX=+25°  MIN=-25°\nspeed=250°/s\nkp=80  kd=1"),
        ("pd_realtime_control.c", C['yellow'],
         "ADC 4채널 (PF3~PF6)\n+ BTN1 bias 캘리브레이션",
         "bias 제거 → clamp\n→ 계단식 램프 프로파일\n(0 → 10 → 0 Nm)",
         "ADC 전압 직접 → 토크\n(control_ON=1 시 활성화)\nUSB 스트리밍 병행",
         "ramp=1s  hold=2s\nstep=1Nm  max=10Nm\nlimit=2.5Nm"),
    ]

    CARD_W  = 860   # 카드 전체 너비
    CARD_H  = 190   # 카드 전체 높이
    HEAD_H  = 32    # 헤더 높이
    BOX_GAP = 8     # 열 사이 간격
    GAP     = 14    # 카드 사이 간격
    COL_N   = 4
    col_w   = (CARD_W - BOX_GAP * (COL_N + 1)) // COL_N
    col_h   = CARD_H - HEAD_H - BOX_GAP * 2

    col_titles = ['입력', '처리', '출력', '키 파라미터']
    col_colors = [C['teal'], C['blue'], C['green'], C['yellow']]

    y = 20
    for (fname, color, v1, v2, v3, v4) in cards:
        values = [v1, v2, v3, v4]

        # 헤더
        lbl(scene, 26, y + 7, fname, 'text', 10, bold=True)
        scene.addLine(20, y + HEAD_H, CARD_W + 20, y + HEAD_H,
                      QtGui.QPen(QtGui.QColor(color), 1))

        # 4개 열
        for ci, (title, val, col) in enumerate(zip(col_titles, values, col_colors)):
            cx = 20 + BOX_GAP + ci * (col_w + BOX_GAP)
            cy = y + HEAD_H + BOX_GAP
            box(scene, cx, cy, col_w, col_h, C['surface'], col,
                title, val, fs=9)

        # 외곽선
        outer = QtWidgets.QGraphicsRectItem(20, y, CARD_W, CARD_H)
        outer.setPen(QtGui.QPen(QtGui.QColor(color), 2))
        outer.setBrush(QtGui.QBrush(Qt.NoBrush))
        scene.addItem(outer)

        y += CARD_H + GAP

    scene.setSceneRect(0, 0, CARD_W + 40, y + 20)
    return scene_view(scene)


# ─────────────────────────────────────────────────────────────────────────────
# Tab 2 — Final_Encoder FSM
# ─────────────────────────────────────────────────────────────────────────────
def build_encoder_fsm():
    scene = QtWidgets.QGraphicsScene()
    scene.setBackgroundBrush(QtGui.QBrush(QtGui.QColor(C['bg'])))

    lbl(scene, 20, 10, 'Encoder Angle Trigger FSM  (좌/우 독립)', 'blue', 11, bold=True)
    lbl(scene, 20, 32, '마우스 휠: 확대/축소  |  드래그: 이동', 'sub', 8)

    # ── FSM 상태 박스 ──────────────────────────────────────────────────────
    BW, BH, GAP = 200, 85, 50
    states = [
        ('IDLE',      '(H10 비ASSIST 상태)',      C['sub'],    C['border']),
        ('ARMED',     '트리거 대기 중',             C['teal'],   C['teal']),
        ('PULSE ON',  '토크 출력 중',              C['green'],  C['green']),
        ('REARM',     '각도 복귀 대기',             C['yellow'], C['yellow']),
    ]
    SX, SY0 = 40, 75
    state_cy = {}
    for i, (name, desc, bg, br) in enumerate(states):
        sy = SY0 + i * (BH + GAP)
        box(scene, SX, sy, BW, BH, bg, br, name, desc, fs=10)
        state_cy[name] = sy + BH // 2

    # ── 전이 화살표 ────────────────────────────────────────────────────────
    AX = SX + BW + 15  # 화살표 x
    transitions = [
        ('IDLE',     'ARMED',    'H10 → ASSIST\n& control_ON=1', C['teal']),
        ('ARMED',    'PULSE ON', 'angle ≥ threshold(20°)',        C['green']),
        ('PULSE ON', 'REARM',    '경과 ≥ pulse_ms(120ms)',         C['yellow']),
        ('REARM',    'ARMED',    'angle ≤ rearm(15°)',             C['teal']),
    ]
    for (s1, s2, label, col) in transitions:
        y1, y2 = state_cy[s1], state_cy[s2]
        arrow(scene, AX, y1, AX, y2, 'sub', 1, Qt.SolidLine)
        lbl(scene, AX + 12, (y1 + y2) // 2 - 18, label, 'sub', 8,
            max_width=160)

    # 복귀 화살표 (REARM → ARMED) 는 오른쪽으로 우회
    # BTN / 안전 주석
    NOTE_X = SX + BW + 200
    lbl(scene, NOTE_X, state_cy['IDLE'] - 12,
        '← BTN3: control_ON=0 강제 해제', 'red', 8)
    lbl(scene, NOTE_X, state_cy['ARMED'] - 12,
        '← BTN1: control_ON 토글', 'yellow', 8)
    lbl(scene, NOTE_X, state_cy['PULSE ON'] - 12,
        '← HARD_MAX = 2.5 Nm clamp', 'orange', 8)

    # ── 타이밍 다이어그램 ─────────────────────────────────────────────────
    lbl(scene, 530, 10, '타이밍 다이어그램 (개념 시뮬레이션)', 'purple', 11, bold=True)

    pw = pg.PlotWidget(useOpenGL=False)
    pw.setBackground(C['surface'])
    pw.setFixedSize(460, 310)
    pw.showGrid(x=True, y=True, alpha=0.2)
    pw.setLabel('bottom', '시간 (ms)')
    pw.setLabel('left', '값')
    for ax in ('bottom', 'left'):
        pw.getAxis(ax).setPen(C['sub'])
        pw.getAxis(ax).setTextPen(C['sub'])

    t = np.linspace(0, 600, 3000)
    angle = np.clip(20 * np.sin(2 * np.pi * t / 500) + 0.3 * np.random.randn(len(t)),
                    -5, 35)
    threshold = np.full_like(t, 20.0)
    rearm     = np.full_like(t, 15.0)

    pulse = np.zeros_like(t)
    armed = True; pulse_end = -1
    for i in range(len(t)):
        if pulse_end > 0 and t[i] < pulse_end:
            pulse[i] = 5.0
        else:
            pulse_end = -1
            if armed and angle[i] >= 20:
                pulse_end = t[i] + 120
                pulse[i] = 5.0
                armed = False
            if not armed and angle[i] <= 15:
                armed = True

    pw.plot(t, angle,     pen=pg.mkPen(C['blue'],   width=2), name='Hip Angle (deg)')
    pw.plot(t, threshold, pen=pg.mkPen(C['red'],    width=1, style=Qt.DashLine),
            name='Threshold 20°')
    pw.plot(t, rearm,     pen=pg.mkPen(C['yellow'], width=1, style=Qt.DotLine),
            name='Rearm 15°')
    pw.plot(t, pulse,     pen=pg.mkPen(C['green'],  width=3),
            name='Pulse ON (임의 단위)')
    pw.addLegend(offset=(5, 5))

    proxy = QtWidgets.QGraphicsProxyWidget()
    proxy.setWidget(pw)
    proxy.setPos(525, 45)
    scene.addItem(proxy)

    scene.setSceneRect(0, 0, 1010, 640)
    return scene_view(scene)


# ─────────────────────────────────────────────────────────────────────────────
# Tab 3 — FSR Fuzzy Logic
# ─────────────────────────────────────────────────────────────────────────────
def build_fuzzy():
    scene = QtWidgets.QGraphicsScene()
    scene.setBackgroundBrush(QtGui.QBrush(QtGui.QColor(C['bg'])))

    lbl(scene, 20, 10, 'FSR Fuzzy Gait Phase Detection  +  Push-off Assist', 'teal', 11, bold=True)

    # ── 왼쪽 파이프라인 ───────────────────────────────────────────────────
    PW_L, PH_L, PY0, PX_L = 220, 88, 55, 20
    pipeline = [
        ('ADC 샘플링',       'PF3~PF6 4채널 (LToe/LHeel/RToe/RHeel)',  C['blue'],   C['blue']),
        ('저역통과필터',      'LPF 8 Hz  (α = dt/(rc+dt))',             C['teal'],   C['teal']),
        ('캘리브레이션',      'BTN1: Zero  |  BTN2: Full Load (1초)',   C['yellow'], C['yellow']),
        ('정규화',           'load = (v − off) / span  →  [0, 1.5]',   C['orange'], C['orange']),
        ('퍼지 멤버십',      'μ_large = tanh(k·(x−θ))×0.5+0.5',        C['purple'], C['purple']),
        ('보행 위상 결정',   '4상태 max-μ 선택 (Heel/Toe 조합)',         C['green'],  C['green']),
        ('토크 펄스 출력',   'TERMINAL_STANCE → SWING 전환 시 발생',     C['red'],    C['red']),
    ]
    prev_cy = None
    for i, (name, desc, bg, br) in enumerate(pipeline):
        py = PY0 + i * (PH_L + 12)
        box(scene, PX_L, py, PW_L, PH_L, bg, br, name, desc, fs=9)
        cy = py + PH_L // 2
        if prev_cy is not None:
            arrow(scene, PX_L + PW_L // 2, prev_cy, PX_L + PW_L // 2, py, 'sub', 1, Qt.SolidLine)
        prev_cy = py + PH_L

    # ── 오른쪽 상단: 퍼지 규칙표 ────────────────────────────────────────
    RX = 270
    lbl(scene, RX, 55, 'Fuzzy 규칙표  (Larsen product T-norm)', 'purple', 10, bold=True)

    # 헤더
    for cx, cw, title in [(RX, 170, '보행 위상'),
                           (RX + 180, 100, 'Heel Load'),
                           (RX + 290, 100, 'Toe Load')]:
        box(scene, cx, 78, cw, 30, C['surface'], C['border'], title, fs=8)

    rules = [
        ('HEEL_STRIKE',     'Large', 'Small', C['blue']),
        ('LOAD_RESPONSE',   'Large', 'Large', C['teal']),
        ('TERMINAL_STANCE', 'Small', 'Large', C['green']),
        ('SWING (Toe-off)', 'Small', 'Small', C['yellow']),
    ]
    for i, (phase, heel, toe, col) in enumerate(rules):
        ry = 116 + i * 44
        box(scene, RX,       ry, 170, 36, C['surface'], col, phase, fs=8)
        box(scene, RX + 180, ry, 100, 36, C['surface'], col, heel,  fs=9)
        box(scene, RX + 290, ry, 100, 36, C['surface'], col, toe,   fs=9)

    # ── 오른쪽 중단: tanh 그래프 ─────────────────────────────────────────
    lbl(scene, RX, 300, 'tanh 멤버십  μ(x) = 0.5·( tanh(k·(x−θ)) + 1 )',
        'teal', 9, max_width=420)

    pw_m = pg.PlotWidget(useOpenGL=False)
    pw_m.setBackground(C['surface'])
    pw_m.setFixedSize(400, 185)
    pw_m.showGrid(x=True, y=True, alpha=0.2)
    pw_m.setLabel('bottom', '정규화 하중')
    pw_m.setLabel('left', 'μ')
    for ax in ('bottom', 'left'):
        pw_m.getAxis(ax).setPen(C['sub'])
        pw_m.getAxis(ax).setTextPen(C['sub'])

    x = np.linspace(0, 1, 500)
    for sens, col, name in [(6, C['yellow'], 'k=6 (완만)'),
                             (12, C['teal'],   'k=12 (기본값)'),
                             (20, C['red'],    'k=20 (날카로움)')]:
        pw_m.plot(x, 0.5 * (np.tanh(sens * (x - 0.35)) + 1),
                  pen=pg.mkPen(col, width=2), name=name)
    pw_m.addLegend(offset=(5, 5))

    prx_m = QtWidgets.QGraphicsProxyWidget()
    prx_m.setWidget(pw_m)
    prx_m.setPos(RX, 320)
    scene.addItem(prx_m)

    # ── 오른쪽 하단: 보행 위상 타임라인 ──────────────────────────────────
    lbl(scene, RX, 522, '보행 주기 위상 시퀀스 (한 걸음)', 'green', 10, bold=True)
    phase_boxes = [
        ('HEEL\nSTRIKE',    C['blue'],    85),
        ('LOAD\nRESP',      C['teal'],    85),
        ('TERMINAL\nSTANCE',C['green'],   115),
        ('SWING\n(Toe-off)', C['yellow'], 85),
    ]
    px2 = RX
    for (pl, col, pw2) in phase_boxes:
        box(scene, px2, 543, pw2, 60, col, col, pl, fs=8)
        px2 += pw2 + 6

    # 펄스 트리거 annotation
    trigger_x = RX + 85 + 85 + 115 - 10
    arrow(scene, trigger_x, 603, trigger_x, 650, 'red', 2, Qt.SolidLine)
    lbl(scene, trigger_x + 6, 618,
        '← TS → SWING 전환 시\n   토크 펄스 발생', 'red', 8, max_width=170)

    scene.setSceneRect(0, 0, 790, 730)
    return scene_view(scene)


# ─────────────────────────────────────────────────────────────────────────────
# Tab 4 — EMG Pipeline + TSM
# ─────────────────────────────────────────────────────────────────────────────
def build_emg():
    scene = QtWidgets.QGraphicsScene()
    scene.setBackgroundBrush(QtGui.QBrush(QtGui.QColor(C['bg'])))

    lbl(scene, 20, 10, 'Final_EMG — 신호 처리 파이프라인 + Task State Machine', 'green', 11, bold=True)

    # ── 왼쪽: TSM ────────────────────────────────────────────────────────
    lbl(scene, 20, 40, 'Task State Machine', 'green', 10, bold=True)

    TSM_W, TSM_H, TSM_GAP = 200, 85, 55
    tsm_data = [
        ('OFF',     'CM 연결 대기',           C['border'], C['sub']),
        ('STANDBY', 'ASSIST 모드 전환 대기',   C['teal'],   C['teal']),
        ('ACTIVE',  '비례 토크 제어 실행 중',  C['green'],  C['green']),
    ]
    TSM_X, TSM_Y0 = 20, 65
    tsm_cy = {}
    for i, (name, desc, bg, br) in enumerate(tsm_data):
        sy = TSM_Y0 + i * (TSM_H + TSM_GAP)
        box(scene, TSM_X, sy, TSM_W, TSM_H, bg, br, name, desc, fs=10)
        tsm_cy[name] = sy + TSM_H // 2

    # 정방향 화살표
    cx = TSM_X + TSM_W // 2
    arrow(scene, cx, tsm_cy['OFF'] + TSM_H // 2,
          cx, tsm_cy['STANDBY'] - TSM_H // 2, 'teal', 1, Qt.SolidLine)
    lbl(scene, cx + 8, (tsm_cy['OFF'] + tsm_cy['STANDBY']) // 2 - 10,
        'CM 연결됨', 'teal', 8)

    arrow(scene, cx, tsm_cy['STANDBY'] + TSM_H // 2,
          cx, tsm_cy['ACTIVE'] - TSM_H // 2, 'green', 1, Qt.SolidLine)
    lbl(scene, cx + 8, (tsm_cy['STANDBY'] + tsm_cy['ACTIVE']) // 2 - 10,
        'ASSIST 감지', 'green', 8)

    # 역방향 화살표 (왼쪽)
    bx = TSM_X - 14
    arrow(scene, bx, tsm_cy['ACTIVE'], bx, tsm_cy['STANDBY'], 'yellow', 1, Qt.DashLine)
    lbl(scene, bx - 38, (tsm_cy['STANDBY'] + tsm_cy['ACTIVE']) // 2 - 12,
        'ASSIST\n해제', 'yellow', 7, max_width=40)

    arrow(scene, bx - 6, tsm_cy['STANDBY'], bx - 6, tsm_cy['OFF'], 'red', 1, Qt.DashLine)
    lbl(scene, bx - 44, (tsm_cy['OFF'] + tsm_cy['STANDBY']) // 2 - 12,
        'CM\n끊김', 'red', 7, max_width=40)

    # ACTIVE 내부 흐름
    lbl(scene, TSM_X, tsm_cy['ACTIVE'] + TSM_H // 2 + 18,
        'ACTIVE 내부 흐름', 'green', 9, bold=True)

    act_steps = [
        ('_SampleAdcChannels()',          '4채널 ADC 샘플링',          C['blue']),
        ('_UpdateCalibration()',           'BTN1:Rest  BTN2:Effort',    C['yellow']),
        ('_ProcessEmgSignals()',           '파이프라인 전체 실행',         C['teal']),
        ('_SelectTorquePair()',            'PF3/PF4 또는 PF5/PF6 선택', C['purple']),
        ('XM_SetAssistTorqueRH/LH()',     'control_ON 게이팅 후 출력',  C['green']),
    ]
    ay = tsm_cy['ACTIVE'] + TSM_H // 2 + 42
    for i, (fn, desc, col) in enumerate(act_steps):
        box(scene, TSM_X, ay, TSM_W, 60, C['surface'], col, fn, desc, fs=8)
        if i > 0:
            arrow(scene, TSM_X + TSM_W // 2, ay - 2,
                  TSM_X + TSM_W // 2, ay, 'sub', 1, Qt.SolidLine)
        ay += 68

    # ── 중간: EMG 파이프라인 ─────────────────────────────────────────────
    PX, PY0 = 250, 65
    PW2, PH2 = 290, 72
    lbl(scene, PX, 40, 'EMG 신호 처리 파이프라인 (채널별 독립)', 'blue', 10, bold=True)

    pipe = [
        ('Raw ADC',         '0 ~ 3.3 V,  중심전압 ≈ 1.65 V',          C['sub'],    C['border']),
        ('바이어스 제거',    'centered = raw − bias_v',                  C['blue'],   C['blue']),
        ('LPF 80 Hz',       'pre_lpf = LPF(centered)',                  C['teal'],   C['teal']),
        ('전파 정류',        'rectified = |pre_lpf|',                    C['purple'], C['purple']),
        ('포락선 추출',      'envelope = LPF_5Hz(rectified)',            C['green'],  C['green']),
        ('데드밴드 제거',    'active = max(envelope − 0.02, 0)',         C['yellow'], C['yellow']),
        ('비례 토크 변환',   'torque = (active/span) × scale × 2.5 Nm', C['orange'], C['orange']),
    ]
    prev_mid2 = None
    for i, (name, desc, bg, br) in enumerate(pipe):
        py = PY0 + i * (PH2 + 10)
        box(scene, PX, py, PW2, PH2, bg, br, name, desc, fs=9)
        if prev_mid2 is not None:
            arrow(scene, PX + PW2 // 2, prev_mid2,
                  PX + PW2 // 2, py, 'sub', 1, Qt.SolidLine)
        prev_mid2 = py + PH2

    # ── 오른쪽: 신호 파형 ────────────────────────────────────────────────
    lbl(scene, 570, 40, '신호 처리 단계별 파형 (시뮬레이션)', 'purple', 10, bold=True)

    t = np.linspace(0, 2, 2000)
    raw = (1.65
           + 0.3 * np.sin(2 * np.pi * 5 * t) * np.exp(-((t - 0.8) ** 2) / 0.04)
           + 0.3 * np.sin(2 * np.pi * 5 * t) * np.exp(-((t - 1.5) ** 2) / 0.04)
           + 0.04 * np.random.randn(len(t)))
    centered = raw - 1.65
    rect = np.abs(centered)
    alpha_env = 1 - np.exp(-2 * np.pi * 5 / 1000)
    envelope = np.zeros_like(rect)
    for i in range(1, len(rect)):
        envelope[i] = envelope[i - 1] + alpha_env * (rect[i] - envelope[i - 1])
    active = np.maximum(envelope - 0.02, 0)
    torque = (active / max(active.max(), 0.05)) * 1.25

    pw_e = pg.PlotWidget(useOpenGL=False)
    pw_e.setBackground(C['surface'])
    pw_e.setFixedSize(390, 460)
    pw_e.showGrid(x=True, y=True, alpha=0.2)
    pw_e.setLabel('bottom', '시간 (s)')
    pw_e.addLegend(offset=(5, 5))
    for ax in ('bottom', 'left'):
        pw_e.getAxis(ax).setPen(C['sub'])
        pw_e.getAxis(ax).setTextPen(C['sub'])

    for sig, col, name in [
        (raw,      C['sub'],   'Raw ADC (V)'),
        (centered, C['blue'],  'Centered (V)'),
        (envelope, C['teal'],  'Envelope (V)'),
        (torque,   C['green'], 'Torque (Nm)'),
    ]:
        pw_e.plot(t, sig, pen=pg.mkPen(col, width=1.5), name=name)

    prx_e = QtWidgets.QGraphicsProxyWidget()
    prx_e.setWidget(pw_e)
    prx_e.setPos(565, 65)
    scene.addItem(prx_e)

    scene.setSceneRect(0, 0, 980, 860)
    return scene_view(scene)


# ─────────────────────────────────────────────────────────────────────────────
# Tab 5 — Passive Mode + PD Ramp
# ─────────────────────────────────────────────────────────────────────────────
def build_passive_ramp():
    scene = QtWidgets.QGraphicsScene()
    scene.setBackgroundBrush(QtGui.QBrush(QtGui.QColor(C['bg'])))

    # ════════════════════════════════ 왼쪽: Passive Mode ══════════════════
    lbl(scene, 20, 10, 'passive_mode.c — P-Vector 왕복 운동', 'purple', 11, bold=True)

    # ── Homing FSM ────────────────────────────────────────────────────────
    lbl(scene, 20, 38, '① Homing 상태 머신 (ASSIST 진입 시 1회)', 'yellow', 9, bold=True)
    HW, HH, HX, HY0 = 210, 58, 20, 58
    homing = [
        ('ENTRY',           'P-Vector Reset 전송',          C['sub'],    C['border']),
        ('SET_IMPEDANCE',   'kp=80  kd=1  duration=50ms',   C['surface'],C['teal']),
        ('START_MOTION',    '현재 위치 → 0° P-Vector 전송', C['surface'],C['blue']),
        ('WAIT_FOR_DONE',   'isPVectorDone 플래그 대기',     C['surface'],C['yellow']),
        ('FINALIZE_DELAY',  '50ms 안정화 대기',              C['surface'],C['orange']),
        ('CLEANUP',         '→ ACTIVE 상태로 전환',          C['surface'],C['green']),
    ]
    for i, (name, desc, bg, br) in enumerate(homing):
        hy = HY0 + i * (HH + 10)
        box(scene, HX, hy, HW, HH, bg, br, name, desc, fs=8)
        if i > 0:
            arrow(scene, HX + HW // 2, hy - 2,
                  HX + HW // 2, hy, 'sub', 1, Qt.SolidLine)

    # ── Passive FSM ────────────────────────────────────────────────────────
    lbl(scene, 250, 38, '② Passive 왕복 상태 머신', 'purple', 9, bold=True)
    PFW, PFH, PFX, PFY0 = 210, 72, 250, 58
    passive = [
        ('SET_IMPEDANCE',   'kp=80  kd=1  적용',            C['surface'], C['teal']),
        ('START_MOTION',    '현재→MAX  MAX→MIN 선(pre-queue)',C['surface'], C['blue']),
        ('MOVING_TO_MIN',   'Done → MAX 궤적 큐잉',          C['surface'], C['purple']),
        ('MOVING_TO_MAX',   'Done → MIN 궤적 큐잉',          C['surface'], C['purple']),
    ]
    for i, (name, desc, bg, br) in enumerate(passive):
        pfy = PFY0 + i * (PFH + 14)
        box(scene, PFX, pfy, PFW, PFH, bg, br, name, desc, fs=8)
        if i > 0:
            arrow(scene, PFX + PFW // 2, pfy - 2,
                  PFX + PFW // 2, pfy, 'sub', 1, Qt.SolidLine)

    # MOVING_TO_MIN ↔ MOVING_TO_MAX 루프 화살표
    loop_x = PFX + PFW + 18
    y_min_mid = PFY0 + 2 * (PFH + 14) + PFH // 2
    y_max_mid = PFY0 + 3 * (PFH + 14) + PFH // 2
    arrow(scene, loop_x, y_min_mid, loop_x, y_max_mid, 'yellow', 1, Qt.DashLine)
    arrow(scene, loop_x, y_max_mid, loop_x, y_min_mid, 'yellow', 1, Qt.DashLine)
    lbl(scene, loop_x + 6, (y_min_mid + y_max_mid) // 2 - 14,
        '무한\n반복', 'yellow', 8)

    # ── P-Vector 파형 ──────────────────────────────────────────────────────
    GRAPH_Y = HY0 + len(homing) * (HH + 10) + 20
    lbl(scene, 20, GRAPH_Y - 18, 'P-Vector 각도 시뮬레이션 (왕복 ±25°)', 'purple', 9, bold=True)

    pw1 = pg.PlotWidget(useOpenGL=False)
    pw1.setBackground(C['surface'])
    pw1.setFixedSize(440, 195)
    pw1.showGrid(x=True, y=True, alpha=0.2)
    pw1.setLabel('bottom', '시간 (s)')
    pw1.setLabel('left', '각도 (°)')
    for ax in ('bottom', 'left'):
        pw1.getAxis(ax).setPen(C['sub'])
        pw1.getAxis(ax).setTextPen(C['sub'])

    t = np.linspace(0, 8, 4000)
    angle = 25 * np.sin(2 * np.pi * t / 4)
    pw1.plot(t, angle, pen=pg.mkPen(C['purple'], width=2), name='Hip Angle (deg)')
    pw1.addLine(y=25,  pen=pg.mkPen(C['red'], width=1, style=Qt.DashLine))
    pw1.addLine(y=-25, pen=pg.mkPen(C['red'], width=1, style=Qt.DashLine))
    pw1.addLegend(offset=(5, 5))

    prx1 = QtWidgets.QGraphicsProxyWidget()
    prx1.setWidget(pw1)
    prx1.setPos(20, GRAPH_Y)
    scene.addItem(prx1)

    # ════════════════════════════════ 오른쪽: PD Ramp ═════════════════════
    RX = 500
    lbl(scene, RX, 10, 'pd_realtime_control.c — 계단식 램프', 'yellow', 11, bold=True)

    # ── TSM ────────────────────────────────────────────────────────────────
    lbl(scene, RX, 38, 'Task State Machine', 'yellow', 9, bold=True)
    tsm_r = [
        ('OFF',     'CM 연결 대기',           C['border'],  C['sub']),
        ('STANDBY', 'ASSIST 모드 대기',        C['teal'],    C['teal']),
        ('ACTIVE',  '램프 토크 제어 실행 중',  C['yellow'],  C['yellow']),
    ]
    TW, TH, TX, TY0 = 200, 70, RX, 58
    tsm_r_cy = {}
    for i, (name, desc, bg, br) in enumerate(tsm_r):
        ty = TY0 + i * (TH + 28)
        box(scene, TX, ty, TW, TH, bg, br, name, desc, fs=9)
        tsm_r_cy[name] = ty + TH // 2

    tcx = TX + TW // 2
    arrow(scene, tcx, tsm_r_cy['OFF'] + TH // 2,
          tcx, tsm_r_cy['STANDBY'] - TH // 2, 'teal', 1, Qt.SolidLine)
    lbl(scene, tcx + 8, (tsm_r_cy['OFF'] + tsm_r_cy['STANDBY']) // 2 - 10,
        'CM 연결됨', 'teal', 8)
    arrow(scene, tcx, tsm_r_cy['STANDBY'] + TH // 2,
          tcx, tsm_r_cy['ACTIVE'] - TH // 2, 'yellow', 1, Qt.SolidLine)
    lbl(scene, tcx + 8, (tsm_r_cy['STANDBY'] + tsm_r_cy['ACTIVE']) // 2 - 10,
        'ASSIST 감지', 'yellow', 8)

    # ── 램프 위상 FSM ──────────────────────────────────────────────────────
    rfsm_y = TY0 + 3 * (TH + 28) + 10
    lbl(scene, RX, rfsm_y - 18, '램프 위상 FSM (ACTIVE 내부)', 'orange', 9, bold=True)
    ramp_phases = [
        ('PHASE_RAMP', 'from → to  (1 s 선형 보간)',   C['surface'], C['orange']),
        ('PHASE_HOLD', 'to 유지  (2 s)',                C['surface'], C['yellow']),
        ('PHASE_DONE', '프로파일 완료  →  0 유지',      C['surface'], C['sub']),
    ]
    RPW, RPH = 210, 60
    for i, (name, desc, bg, br) in enumerate(ramp_phases):
        ry = rfsm_y + i * (RPH + 12)
        box(scene, RX, ry, RPW, RPH, bg, br, name, desc, fs=8)
        if i > 0:
            arrow(scene, RX + RPW // 2, ry - 2,
                  RX + RPW // 2, ry, 'sub', 1, Qt.SolidLine)

    # 반복 화살표
    loop2_x = RX + RPW + 16
    y_ramp = rfsm_y + RPH // 2
    y_hold = rfsm_y + (RPH + 12) + RPH // 2
    arrow(scene, loop2_x, y_hold, loop2_x, y_ramp, 'orange', 1, Qt.DashLine)
    lbl(scene, loop2_x + 5, (y_ramp + y_hold) // 2 - 10,
        'step++\n(< 20)', 'orange', 8)

    # ── 램프 토크 파형 ──────────────────────────────────────────────────────
    ramp_graph_y = rfsm_y + 3 * (RPH + 12) + 18
    lbl(scene, RX, ramp_graph_y - 18,
        '계단식 램프 토크 프로파일 (0 → 10 → 0 Nm)', 'yellow', 9, bold=True)

    pw2 = pg.PlotWidget(useOpenGL=False)
    pw2.setBackground(C['surface'])
    pw2.setFixedSize(440, 195)
    pw2.showGrid(x=True, y=True, alpha=0.2)
    pw2.setLabel('bottom', '시간 (s)')
    pw2.setLabel('left', '토크 (Nm)')
    for ax in ('bottom', 'left'):
        pw2.getAxis(ax).setPen(C['sub'])
        pw2.getAxis(ax).setTextPen(C['sub'])

    ramp_t, ramp_tau = [], []
    t_cur = 0
    levels_up = list(range(0, 11))
    levels_dn = list(range(10, -1, -1))
    for (fr, to) in (list(zip(levels_up, levels_up[1:]))
                     + list(zip(levels_dn, levels_dn[1:]))):
        ts = np.linspace(0, 1, 100)
        ramp_t.extend(t_cur + ts)
        ramp_tau.extend(fr + (to - fr) * ts)
        t_cur += 1
        ramp_t.extend(t_cur + np.linspace(0, 2, 200))
        ramp_tau.extend([float(to)] * 200)
        t_cur += 2

    pw2.plot(np.array(ramp_t), np.array(ramp_tau),
             pen=pg.mkPen(C['yellow'], width=2), name='Torque Cmd (Nm)')
    pw2.addLine(y=2.5, pen=pg.mkPen(C['red'], width=1, style=Qt.DashLine))
    pw2.addLegend(offset=(5, 5))

    prx2 = QtWidgets.QGraphicsProxyWidget()
    prx2.setWidget(pw2)
    prx2.setPos(RX, ramp_graph_y)
    scene.addItem(prx2)

    total_h = max(GRAPH_Y + 200, ramp_graph_y + 200) + 30
    scene.setSceneRect(0, 0, RX + 460, total_h)
    return scene_view(scene)


# ─────────────────────────────────────────────────────────────────────────────
# Main Window
# ─────────────────────────────────────────────────────────────────────────────
class MainWindow(QtWidgets.QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle('XM10 — User Algorithm Visualizer')
        self.resize(1120, 800)

        tabs = QtWidgets.QTabWidget()
        tabs.addTab(build_overview(),    '① 알고리즘 개요')
        tabs.addTab(build_encoder_fsm(), '② Encoder FSM')
        tabs.addTab(build_fuzzy(),       '③ FSR Fuzzy')
        tabs.addTab(build_emg(),         '④ EMG Pipeline')
        tabs.addTab(build_passive_ramp(), '⑤ Passive / Ramp')
        self.setCentralWidget(tabs)

        bar = self.statusBar()
        bar.setStyleSheet(f'background:{C["surface"]}; color:{C["sub"]};'
                          f'border-top:1px solid {C["border"]};')
        bar.showMessage(
            '마우스 휠: 확대/축소  |  드래그: 이동  |  각 탭에서 알고리즘 로직 확인')


def main():
    app = QtWidgets.QApplication(sys.argv)
    app.setStyle('Fusion')
    app.setStyleSheet(GLOBAL_STYLESHEET)
    pg.setConfigOptions(antialias=True)
    win = MainWindow()
    win.show()
    sys.exit(app.exec_())


if __name__ == '__main__':
    main()
