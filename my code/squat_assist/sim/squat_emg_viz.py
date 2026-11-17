"""
squat_emg_viz.py
Squat_EMG_Assist.c 시뮬레이션 시각화
각 플롯을 별도 PNG 파일로 저장

저장 파일 목록:
  squat_viz_p1_timeline.png   - 스쿼트 타임라인 전체 개요
  squat_viz_p2_fsr.png        - FSR 신호 및 하중
  squat_viz_p3_emg.png        - EMG 파이프라인 (raw -> envelope -> norm)
  squat_viz_p4_encoder.png    - 엔코더 각도 및 속도
  squat_viz_p5_fsm.png        - FSM 상태 전이 + 토크 출력
  squat_viz_p6_torque.png     - 최종 토크 분해 (base + comp)
"""

import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
from matplotlib.patches import FancyArrowPatch
import matplotlib.gridspec as gridspec

# ============================================================
# 전역 폰트 / 스타일
# ============================================================
plt.rcParams.update({
    'font.size':        18,
    'axes.titlesize':   22,
    'axes.labelsize':   20,
    'xtick.labelsize':  17,
    'ytick.labelsize':  17,
    'legend.fontsize':  17,
    'figure.dpi':       150,
    'axes.grid':        True,
    'grid.alpha':       0.35,
    'axes.spines.top':  False,
    'axes.spines.right':False,
})

DPI   = 150
CMAP  = ['#2196F3', '#F44336', '#4CAF50', '#FF9800',
          '#9C27B0', '#00BCD4', '#795548', '#E91E63']

# ============================================================
# 시뮬레이션 파라미터
# ============================================================
DT          = 0.001          # 1 ms
T_END       = 3.5
t           = np.arange(0, T_END, DT)
N           = len(t)

# 캘리브레이션
FSR_ZERO    = 0.10
FSR_FULL    = 0.90
FSR_SPAN    = FSR_FULL - FSR_ZERO

EMG_BIAS    = 1.65
EMG_FS      = 0.50
EMG_DEAD    = 0.020
EMG_SPAN    = EMG_FS - EMG_DEAD

# 튜닝
ENTER_DEG   = 15.0
BOTTOM_DEG  = 45.0
STAND_DEG   = 5.0
DEBOUNCE_MS = 150
MAX_TQ      = 1.0
GRAV_MGL    = 1.0
COMP_SCALE  = 0.20
TQ_LIMIT    = 1.5
TWO_PI      = 6.28318530718
CTRL_DT     = 0.001

# ============================================================
# 엔코더 각도 프로파일 (좌우 동일)
# ============================================================
def make_angle_profile(t):
    angle = np.zeros(len(t))
    for i, ti in enumerate(t):
        if ti < 0.5:
            angle[i] = 2.0
        elif ti < 1.45:
            # 0.5 ~ 1.45s: 2 -> 51 deg
            frac = (ti - 0.5) / 0.95
            angle[i] = 2.0 + frac * 49.0
        elif ti < 1.60:
            angle[i] = 51.0
        elif ti < 2.80:
            # 1.6 ~ 2.8s: 51 -> 13 deg
            frac = (ti - 1.6) / 1.2
            angle[i] = 51.0 - frac * 38.0
        elif ti < 3.20:
            # 2.8 ~ 3.2s: 13 -> 1 deg
            frac = (ti - 2.8) / 0.4
            angle[i] = 13.0 - frac * 12.0
        else:
            angle[i] = 1.0
    return angle

angle_L = make_angle_profile(t)
angle_R = make_angle_profile(t) + 0.5   # 약간 비대칭

# 각속도 (수치 미분 + LPF)
def lpf(arr, cutoff_hz, dt=DT):
    rc    = 1.0 / (TWO_PI * cutoff_hz)
    alpha = dt / (rc + dt)
    out   = np.zeros_like(arr)
    out[0] = arr[0]
    for i in range(1, len(arr)):
        out[i] = out[i-1] + alpha * (arr[i] - out[i-1])
    return out

vel_L_raw = np.gradient(np.deg2rad(angle_L), DT)
vel_R_raw = np.gradient(np.deg2rad(angle_R), DT)
vel_L     = lpf(vel_L_raw, 5.0)
vel_R     = lpf(vel_R_raw, 5.0)
avg_angle = (angle_L + angle_R) / 2.0
avg_vel   = (vel_L   + vel_R)   / 2.0

# ============================================================
# FSM 상태 계산
# ============================================================
STAND=0; DESCENDING=1; BOTTOM=2; ASCENDING=3; RETURN=4
phase    = np.zeros(N, dtype=int)
phase[:] = STAND

state            = STAND
deb_active       = False
deb_tick         = 0

for i in range(1, N):
    ti        = t[i] * 1000  # ms
    ang       = avg_angle[i]
    vel       = avg_vel[i]
    enter_z   = ang >= ENTER_DEG
    bottom_z  = ang >= BOTTOM_DEG
    stand_z   = ang <= STAND_DEG

    if state == STAND:
        if enter_z:
            if not deb_active:
                deb_active = True
                deb_tick   = ti
            elif (ti - deb_tick) >= DEBOUNCE_MS:
                state      = DESCENDING
                deb_active = False
        else:
            deb_active = False

    elif state == DESCENDING:
        if bottom_z:
            state = BOTTOM
        elif not enter_z:
            state = RETURN

    elif state == BOTTOM:
        if vel < -0.05:
            state = ASCENDING

    elif state == ASCENDING:
        if not enter_z:
            state      = RETURN
            deb_active = False

    elif state == RETURN:
        if stand_z:
            if not deb_active:
                deb_active = True
                deb_tick   = ti
            elif (ti - deb_tick) >= DEBOUNCE_MS:
                state      = STAND
                deb_active = False
        else:
            if enter_z:
                state      = ASCENDING
                deb_active = False
            else:
                deb_active = False

    phase[i] = state

# ============================================================
# FSR 신호
# ============================================================
def make_fsr_lpf(t, angle):
    """각도에 비례해서 하중 증가"""
    raw = FSR_ZERO + (angle / 55.0) * (FSR_FULL - FSR_ZERO) * 0.9
    raw += np.random.randn(len(t)) * 0.005
    return lpf(raw, 8.0)

np.random.seed(42)
fsr_lt = make_fsr_lpf(t, angle_L * 0.9)
fsr_lh = make_fsr_lpf(t, angle_L * 1.0)
fsr_rt = make_fsr_lpf(t, angle_R * 0.9)
fsr_rh = make_fsr_lpf(t, angle_R * 1.0)

def to_load(v):
    span = FSR_SPAN if FSR_SPAN > 0.05 else 0.05
    return np.clip((v - FSR_ZERO) / span, 0, 1.5)

load_lt = to_load(fsr_lt)
load_lh = to_load(fsr_lh)
load_rt = to_load(fsr_rt)
load_rh = to_load(fsr_rh)

# ============================================================
# EMG 신호 파이프라인
# ============================================================
def make_emg_activation(t, scale=1.0):
    """스쿼트 구간에서 근육 활성화 증가"""
    act = np.zeros(len(t))
    for i, ti in enumerate(t):
        if 0.8 <= ti <= 2.8:
            if ti < 1.6:
                act[i] = 0.3 + 0.7 * (ti - 0.8) / 0.8
            elif ti < 2.0:
                act[i] = 1.0
            else:
                act[i] = 1.0 - 0.6 * (ti - 2.0) / 0.8
    return act * scale

act_rh = make_emg_activation(t, scale=0.35)
act_lh = make_emg_activation(t, scale=0.40)

# Raw EMG: bias + amplitude*sin (rectified EMG feel)
np.random.seed(7)
freq_emg = 120  # Hz
emg_rh_raw = EMG_BIAS + act_rh * np.sin(TWO_PI * freq_emg * t) \
             + np.random.randn(N) * 0.01
emg_lh_raw = EMG_BIAS + act_lh * np.sin(TWO_PI * freq_emg * t + 0.3) \
             + np.random.randn(N) * 0.01

def process_emg(raw, bias=EMG_BIAS):
    centered  = raw - bias
    pre_lpf   = lpf(centered,     80.0)
    rect      = np.abs(pre_lpf)
    envelope  = lpf(rect,          5.0)
    active    = envelope - EMG_DEAD
    active    = np.where(active > 0, active, 0.0)
    span      = EMG_SPAN if EMG_SPAN > 0.05 else 0.05
    norm      = np.clip(active / span, 0.0, 1.0)
    return centered, pre_lpf, rect, envelope, norm

rh_cent, rh_pre, rh_rect, rh_env, rh_norm = process_emg(emg_rh_raw)
lh_cent, lh_pre, lh_rect, lh_env, lh_norm = process_emg(emg_lh_raw)

# ============================================================
# 보상 토크
# ============================================================
angle_rad_L = np.deg2rad(angle_L)
angle_rad_R = np.deg2rad(angle_R)
grav_L = GRAV_MGL * np.sin(angle_rad_L)
grav_R = GRAV_MGL * np.sin(angle_rad_R)
comp_L = COMP_SCALE * grav_L
comp_R = COMP_SCALE * grav_R

# ============================================================
# 최종 토크
# ============================================================
base_L = np.where(phase == ASCENDING, lh_norm * MAX_TQ, 0.0)
base_R = np.where(phase == ASCENDING, rh_norm * MAX_TQ, 0.0)

assist_on = np.ones(N)  # control_ON=1, fsr_cal_ready=1

torque_L = np.clip((base_L + comp_L) * assist_on, -TQ_LIMIT, TQ_LIMIT)
torque_R = np.clip((base_R + comp_R) * assist_on, -TQ_LIMIT, TQ_LIMIT)

# ============================================================
# 헬퍼: 페이즈 배경 음영
# ============================================================
PHASE_COLORS = {
    STAND:      '#E3F2FD',
    DESCENDING: '#FFF9C4',
    BOTTOM:     '#FFCCBC',
    ASCENDING:  '#C8E6C9',
    RETURN:     '#F3E5F5',
}
PHASE_LABELS = {
    STAND:      'STAND',
    DESCENDING: 'DESCENDING',
    BOTTOM:     'BOTTOM',
    ASCENDING:  'ASCENDING',
    RETURN:     'RETURN',
}

def shade_phases(ax, t, phase, ymin, ymax):
    i = 0
    while i < len(t):
        j = i
        cur = phase[i]
        while j < len(t) and phase[j] == cur:
            j += 1
        ax.axvspan(t[i], t[j-1], alpha=0.25,
                   color=PHASE_COLORS[cur], zorder=0)
        i = j

def add_phase_legend(ax):
    handles = [mpatches.Patch(color=PHASE_COLORS[p], alpha=0.5,
                               label=PHASE_LABELS[p])
               for p in [STAND, DESCENDING, BOTTOM, ASCENDING, RETURN]]
    ax.legend(handles=handles, loc='upper right',
              fontsize=15, framealpha=0.8)

def save_fig(fig, fname):
    fig.savefig(fname, dpi=DPI, bbox_inches='tight',
                facecolor='white')
    plt.close(fig)
    print(f"  saved -> {fname}")

# ============================================================
# P1: 타임라인 전체 개요
# ============================================================
print("Generating P1: timeline overview ...")
fig, axes = plt.subplots(4, 1, figsize=(20, 22),
                          sharex=True,
                          gridspec_kw={'hspace': 0.08})
fig.suptitle('Squat EMG Assist — Full Cycle Overview', fontsize=26, fontweight='bold', y=0.98)

# 1) 각도
ax = axes[0]
shade_phases(ax, t, phase, 0, 55)
ax.plot(t, angle_L, color=CMAP[0], lw=2.2, label='Left hip angle')
ax.plot(t, angle_R, color=CMAP[1], lw=2.2, label='Right hip angle', ls='--')
ax.axhline(ENTER_DEG,  color='gray', lw=1.5, ls=':', label=f'Enter thr {ENTER_DEG}°')
ax.axhline(BOTTOM_DEG, color='gray', lw=1.5, ls='-.', label=f'Bottom thr {BOTTOM_DEG}°')
ax.set_ylabel('Angle (deg)', fontweight='bold')
ax.set_ylim(-2, 58)
ax.legend(loc='upper right', fontsize=15)
add_phase_legend(ax)

# 2) EMG norm
ax = axes[1]
shade_phases(ax, t, phase, 0, 1)
ax.plot(t, rh_norm, color=CMAP[2], lw=2.2, label='EMG RH norm')
ax.plot(t, lh_norm, color=CMAP[3], lw=2.2, label='EMG LH norm', ls='--')
ax.set_ylabel('EMG norm (0~1)', fontweight='bold')
ax.set_ylim(-0.05, 1.1)
ax.legend(loc='upper right', fontsize=15)

# 3) FSM state
ax = axes[2]
shade_phases(ax, t, phase, 0, 5)
ax.step(t, phase, color='#333', lw=2.5, where='post')
ax.set_yticks([0,1,2,3,4])
ax.set_yticklabels(['STAND','DESC','BOTTOM','ASCEND','RETURN'], fontsize=15)
ax.set_ylabel('FSM Phase', fontweight='bold')
ax.set_ylim(-0.4, 4.6)

# 4) 토크
ax = axes[3]
shade_phases(ax, t, phase, -0.1, 1.6)
ax.plot(t, torque_L, color=CMAP[0], lw=2.5, label='Left assist torque')
ax.plot(t, torque_R, color=CMAP[1], lw=2.5, label='Right assist torque', ls='--')
ax.set_ylabel('Torque (Nm)', fontweight='bold')
ax.set_xlabel('Time (s)', fontweight='bold')
ax.set_ylim(-0.05, 1.6)
ax.legend(loc='upper right', fontsize=15)

for ax in axes:
    ax.set_xlim(0, T_END)
plt.tight_layout(rect=[0, 0, 1, 0.97])
save_fig(fig, 'D:/HARfinal/PythonDecoder/squat_viz_p1_timeline.png')

# ============================================================
# P2: FSR 신호
# ============================================================
print("Generating P2: FSR signals ...")
fig, axes = plt.subplots(2, 1, figsize=(20, 14),
                          sharex=True,
                          gridspec_kw={'hspace': 0.10})
fig.suptitle('FSR Signals — Raw Voltage & Normalised Load', fontsize=26, fontweight='bold')

ax = axes[0]
shade_phases(ax, t, phase, 0, 1.0)
ax.plot(t, fsr_lt, color=CMAP[0], lw=2.0, label='PF3 LT (left toe)')
ax.plot(t, fsr_lh, color=CMAP[1], lw=2.0, label='PF4 LH (left heel)', ls='--')
ax.plot(t, fsr_rt, color=CMAP[2], lw=2.0, label='PF5 RT (right toe)')
ax.plot(t, fsr_rh, color=CMAP[3], lw=2.0, label='PF6 RH (right heel)', ls='--')
ax.axhline(FSR_ZERO, color='gray', ls=':', lw=1.5, label=f'zero_v = {FSR_ZERO} V')
ax.axhline(FSR_FULL, color='gray', ls='-.', lw=1.5, label=f'full_v = {FSR_FULL} V')
ax.set_ylabel('Voltage after LPF (V)', fontweight='bold')
ax.set_ylim(0.05, 0.98)
ax.legend(loc='lower right', fontsize=15, ncol=2)
add_phase_legend(ax)

ax = axes[1]
shade_phases(ax, t, phase, 0, 1.5)
ax.plot(t, load_lt, color=CMAP[0], lw=2.0, label='LT load')
ax.plot(t, load_lh, color=CMAP[1], lw=2.0, label='LH load', ls='--')
ax.plot(t, load_rt, color=CMAP[2], lw=2.0, label='RT load')
ax.plot(t, load_rh, color=CMAP[3], lw=2.0, label='RH load', ls='--')
ax.set_ylabel('Normalised Load (0~1)', fontweight='bold')
ax.set_xlabel('Time (s)', fontweight='bold')
ax.set_ylim(-0.02, 1.3)
ax.legend(loc='lower right', fontsize=15, ncol=2)

ax.text(0.02, 1.18,
        'FSR Load Formula:\n'
        '  load = (lpf_v - zero_v) / (full_v - zero_v)\n'
        '  span_min = 0.05 V,  clamp to [0, 1.5]',
        transform=ax.transAxes, fontsize=15,
        bbox=dict(boxstyle='round', facecolor='lightyellow', alpha=0.9))

for ax in axes:
    ax.set_xlim(0, T_END)
plt.tight_layout(rect=[0, 0, 1, 0.96])
save_fig(fig, 'D:/HARfinal/PythonDecoder/squat_viz_p2_fsr.png')

# ============================================================
# P3: EMG 파이프라인
# ============================================================
print("Generating P3: EMG pipeline ...")
fig, axes = plt.subplots(4, 1, figsize=(20, 24),
                          sharex=True,
                          gridspec_kw={'hspace': 0.10})
fig.suptitle('EMG Processing Pipeline', fontsize=26, fontweight='bold')

steps = [
    (axes[0], emg_rh_raw,  emg_lh_raw,  'Raw ADC voltage (V)',       (1.3, 2.0)),
    (axes[1], rh_pre,      lh_pre,       'After 80 Hz LPF + centered (V)', (-0.35, 0.35)),
    (axes[2], rh_env,      lh_env,       'Envelope after 5 Hz LPF (V)', (-0.01, 0.32)),
    (axes[3], rh_norm,     lh_norm,      'Normalised EMG  (0~1)',      (-0.02, 1.1)),
]

for ax, sig_rh, sig_lh, ylabel, ylim in steps:
    shade_phases(ax, t, phase, ylim[0], ylim[1])
    ax.plot(t, sig_rh, color=CMAP[0], lw=2.0, label='RH (PF7)')
    ax.plot(t, sig_lh, color=CMAP[1], lw=2.0, label='LH (PF8)', ls='--')
    ax.set_ylabel(ylabel, fontweight='bold')
    ax.set_ylim(*ylim)
    ax.legend(loc='upper right', fontsize=15)

# deadband line on envelope plot
axes[2].axhline(EMG_DEAD, color='red', ls=':', lw=2.0,
                label=f'Deadband = {EMG_DEAD} V')
axes[2].legend(loc='upper right', fontsize=15)

# 파이프라인 설명 박스
axes[0].text(0.01, 0.97,
    'Step 1: raw ADC\n'
    'Step 2: subtract bias (1.65 V) -> 80 Hz LPF -> full-wave rect\n'
    'Step 3: 5 Hz envelope LPF\n'
    'Step 4: deadband(0.02 V) remove -> normalise to [0,1]',
    transform=axes[0].transAxes, fontsize=15, va='top',
    bbox=dict(boxstyle='round', facecolor='#E3F2FD', alpha=0.9))

axes[3].set_xlabel('Time (s)', fontweight='bold')
add_phase_legend(axes[0])

for ax in axes:
    ax.set_xlim(0, T_END)
plt.tight_layout(rect=[0, 0, 1, 0.97])
save_fig(fig, 'D:/HARfinal/PythonDecoder/squat_viz_p3_emg.png')

# ============================================================
# P4: 엔코더 각도 + 속도
# ============================================================
print("Generating P4: encoder ...")
fig, axes = plt.subplots(2, 1, figsize=(20, 14),
                          sharex=True,
                          gridspec_kw={'hspace': 0.10})
fig.suptitle('Encoder — Hip Angle & Angular Velocity', fontsize=26, fontweight='bold')

ax = axes[0]
shade_phases(ax, t, phase, -2, 58)
ax.plot(t, angle_L, color=CMAP[0], lw=2.5, label='Left control angle')
ax.plot(t, angle_R, color=CMAP[1], lw=2.5, label='Right control angle', ls='--')
ax.axhline(ENTER_DEG,  color='#888', lw=1.8, ls=':',  label=f'Enter {ENTER_DEG}°')
ax.axhline(BOTTOM_DEG, color='#888', lw=1.8, ls='-.', label=f'Bottom {BOTTOM_DEG}°')
ax.axhline(STAND_DEG,  color='#888', lw=1.8, ls='--', label=f'Stand {STAND_DEG}°')
ax.set_ylabel('Hip angle (deg)', fontweight='bold')
ax.set_ylim(-3, 60)
ax.legend(loc='upper right', fontsize=15)
add_phase_legend(ax)

ax = axes[1]
shade_phases(ax, t, phase, -2, 2)
ax.plot(t, vel_L, color=CMAP[0], lw=2.5, label='Left angular velocity')
ax.plot(t, vel_R, color=CMAP[1], lw=2.5, label='Right angular velocity', ls='--')
ax.axhline(-0.05, color='red', lw=2.0, ls='--',
           label='BOTTOM->ASCENDING threshold (-0.05 rad/s)')
ax.axhline(0, color='black', lw=1.0)
ax.set_ylabel('Angular velocity (rad/s)', fontweight='bold')
ax.set_xlabel('Time (s)', fontweight='bold')
ax.set_ylim(-1.2, 1.2)
ax.legend(loc='upper right', fontsize=15)

ax.text(0.01, 0.05,
        'velocity < -0.05 rad/s (at BOTTOM state)\n'
        '=> ASCENDING transition triggered',
        transform=ax.transAxes, fontsize=15,
        bbox=dict(boxstyle='round', facecolor='#FFCCBC', alpha=0.9))

for ax in axes:
    ax.set_xlim(0, T_END)
plt.tight_layout(rect=[0, 0, 1, 0.96])
save_fig(fig, 'D:/HARfinal/PythonDecoder/squat_viz_p4_encoder.png')

# ============================================================
# P5: FSM 상태 전이 다이어그램 (정적) + 전이 이벤트 마킹
# ============================================================
print("Generating P5: FSM state diagram ...")
fig = plt.figure(figsize=(22, 14))
fig.suptitle('Squat FSM — State Transitions (Simulation)', fontsize=26, fontweight='bold')

gs = gridspec.GridSpec(1, 2, figure=fig, width_ratios=[1.1, 1], wspace=0.12)

# 왼쪽: 타임라인에서 전이 이벤트 마킹
ax_left = fig.add_subplot(gs[0])
shade_phases(ax_left, t, phase, -0.5, 4.5)
ax_left.step(t, phase, color='black', lw=3, where='post', label='FSM phase')
ax_left.set_yticks([0,1,2,3,4])
ax_left.set_yticklabels(['STAND','DESCENDING','BOTTOM','ASCENDING','RETURN'], fontsize=16)
ax_left.set_xlabel('Time (s)', fontweight='bold')
ax_left.set_ylabel('FSM Phase', fontweight='bold')
ax_left.set_xlim(0, T_END)
ax_left.set_ylim(-0.5, 4.5)

# 전이 시간 표시
transitions = []
for i in range(1, N):
    if phase[i] != phase[i-1]:
        transitions.append((t[i], phase[i-1], phase[i]))

trans_conditions = {
    (STAND, DESCENDING):    'angle >= 15°\n+ 150ms deb',
    (DESCENDING, BOTTOM):   'angle >= 45°',
    (BOTTOM, ASCENDING):    'velocity < -0.05',
    (ASCENDING, RETURN):    'angle < 15°',
    (RETURN, STAND):        'angle <= 5°\n+ 150ms deb',
}
for (ti, frm, to) in transitions:
    cond = trans_conditions.get((frm, to), '')
    ax_left.axvline(ti, color='red', lw=1.8, ls='--', alpha=0.7)
    ax_left.annotate(f't={ti:.2f}s\n{cond}',
                     xy=(ti, (frm + to) / 2),
                     xytext=(ti + 0.08, (frm + to) / 2),
                     fontsize=13,
                     arrowprops=dict(arrowstyle='->', color='red'),
                     color='red',
                     bbox=dict(boxstyle='round,pad=0.2',
                               facecolor='white', alpha=0.85))

# 오른쪽: 상태 다이어그램 (박스 + 화살표)
ax_right = fig.add_subplot(gs[1])
ax_right.set_xlim(0, 10)
ax_right.set_ylim(0, 12)
ax_right.axis('off')
ax_right.set_title('State Transition Diagram', fontsize=20, fontweight='bold', pad=12)

STATE_POS = {
    STAND:      (5.0, 10.5),
    DESCENDING: (5.0,  8.0),
    BOTTOM:     (5.0,  5.5),
    ASCENDING:  (5.0,  3.0),
    RETURN:     (5.0,  0.8),
}
BOX_COLOR = {
    STAND:      '#BBDEFB',
    DESCENDING: '#FFF9C4',
    BOTTOM:     '#FFCCBC',
    ASCENDING:  '#C8E6C9',
    RETURN:     '#E1BEE7',
}

for st, (x, y) in STATE_POS.items():
    ax_right.add_patch(mpatches.FancyBboxPatch(
        (x - 1.5, y - 0.55), 3.0, 1.1,
        boxstyle='round,pad=0.15',
        facecolor=BOX_COLOR[st], edgecolor='#444', lw=2.0))
    ax_right.text(x, y, PHASE_LABELS[st],
                  ha='center', va='center', fontsize=17, fontweight='bold')

arrows = [
    (STAND,      DESCENDING, 'angle>=15°\n+150ms deb', 'left'),
    (DESCENDING, BOTTOM,     'angle>=45°',              'left'),
    (BOTTOM,     ASCENDING,  'vel < -0.05',             'left'),
    (ASCENDING,  RETURN,     'angle<15°',               'left'),
    (RETURN,     STAND,      'angle<=5°\n+150ms deb',   'right'),
]

for (frm, to, label, side) in arrows:
    x0, y0 = STATE_POS[frm]
    x1, y1 = STATE_POS[to]
    dx = 0.0
    if side == 'left':
        ox, ox1 = x0 - 0.3, x1 - 0.3
        tx = x0 - 1.8
    else:
        ox, ox1 = x0 + 0.3, x1 + 0.3
        tx = x0 + 1.8
    ax_right.annotate('', xy=(ox1, y1 + 0.55), xytext=(ox, y0 - 0.55),
                      arrowprops=dict(arrowstyle='->', color='#333',
                                      lw=2.0,
                                      connectionstyle='arc3,rad=0.0'))
    ty = (y0 + y1) / 2
    ax_right.text(tx, ty, label, ha='center', va='center',
                  fontsize=13, color='#333',
                  bbox=dict(boxstyle='round,pad=0.15',
                            facecolor='white', alpha=0.8))

# ASCENDING 에 토크 표시
x, y = STATE_POS[ASCENDING]
ax_right.text(x + 2.0, y,
              'EMG\nTorque\nON',
              ha='center', va='center', fontsize=14,
              color='white', fontweight='bold',
              bbox=dict(boxstyle='round,pad=0.4',
                        facecolor='#388E3C', alpha=0.95))

add_phase_legend(ax_left)
plt.tight_layout(rect=[0, 0, 1, 0.95])
save_fig(fig, 'D:/HARfinal/PythonDecoder/squat_viz_p5_fsm.png')

# ============================================================
# P6: 토크 분해 (base + comp + final)
# ============================================================
print("Generating P6: torque breakdown ...")
fig, axes = plt.subplots(3, 1, figsize=(20, 20),
                          sharex=True,
                          gridspec_kw={'hspace': 0.10})
fig.suptitle('Torque Breakdown — Base (EMG) + Compensation + Final',
             fontsize=26, fontweight='bold')

# 1) EMG base torque
ax = axes[0]
shade_phases(ax, t, phase, -0.05, 1.1)
ax.plot(t, base_L, color=CMAP[0], lw=2.5, label='Base LH  = emg_norm_LH x 1.0 Nm')
ax.plot(t, base_R, color=CMAP[1], lw=2.5, label='Base RH  = emg_norm_RH x 1.0 Nm', ls='--')
ax.set_ylabel('EMG base torque (Nm)', fontweight='bold')
ax.set_ylim(-0.03, 1.1)
ax.legend(loc='upper right', fontsize=15)
ax.text(0.01, 0.92,
        'Active ONLY during ASCENDING phase\n'
        'base = emg_norm * squat_max_torque_nm',
        transform=ax.transAxes, fontsize=15,
        bbox=dict(boxstyle='round', facecolor='#C8E6C9', alpha=0.9))
add_phase_legend(ax)

# 2) 보상 토크
ax = axes[1]
shade_phases(ax, t, phase, -0.05, 0.35)
ax.plot(t, comp_L, color=CMAP[0], lw=2.5, label='comp_L = 0.20 x grav_L')
ax.plot(t, comp_R, color=CMAP[1], lw=2.5, label='comp_R = 0.20 x grav_R', ls='--')
ax.plot(t, grav_L, color=CMAP[0], lw=1.5, ls=':', alpha=0.6, label='gravity_L (pre-scale)')
ax.set_ylabel('Compensation torque (Nm)', fontweight='bold')
ax.set_ylim(-0.02, 0.35)
ax.legend(loc='upper right', fontsize=15)
ax.text(0.01, 0.88,
        'comp = compensation_scale(0.20) x gravity_mgl_nm(1.0) x sin(angle)\n'
        'Applied in ALL phases (friction_compensation_ON = 0)',
        transform=ax.transAxes, fontsize=15,
        bbox=dict(boxstyle='round', facecolor='#FFF9C4', alpha=0.9))

# 3) 최종 토크
ax = axes[2]
shade_phases(ax, t, phase, -0.05, 1.6)
ax.fill_between(t, 0, torque_L, alpha=0.25, color=CMAP[0])
ax.fill_between(t, 0, torque_R, alpha=0.25, color=CMAP[1])
ax.plot(t, torque_L, color=CMAP[0], lw=2.8, label='Left final torque')
ax.plot(t, torque_R, color=CMAP[1], lw=2.8, label='Right final torque', ls='--')
ax.axhline(TQ_LIMIT, color='red', lw=1.8, ls='--',
           label=f'assist_torque_limit = {TQ_LIMIT} Nm')
ax.axhline(2.5, color='darkred', lw=1.8, ls=':',
           label='HARD_MAX = 2.5 Nm')
ax.set_ylabel('Final assist torque (Nm)', fontweight='bold')
ax.set_xlabel('Time (s)', fontweight='bold')
ax.set_ylim(-0.03, 1.7)
ax.legend(loc='upper right', fontsize=15)

ax.text(0.01, 0.88,
        'final = clamp(base + comp, -limit, +limit)\n'
        'Safety gate: control_ON=1 AND fsr_cal_ready=1 AND H10=ASSIST',
        transform=ax.transAxes, fontsize=15,
        bbox=dict(boxstyle='round', facecolor='#FFCCBC', alpha=0.9))

# 피크 값 어노테이션
peak_idx = np.argmax(torque_L)
ax.annotate(f'Peak: {torque_L[peak_idx]:.2f} Nm',
            xy=(t[peak_idx], torque_L[peak_idx]),
            xytext=(t[peak_idx] - 0.4, torque_L[peak_idx] + 0.15),
            fontsize=15, color=CMAP[0], fontweight='bold',
            arrowprops=dict(arrowstyle='->', color=CMAP[0], lw=2.0))

for ax in axes:
    ax.set_xlim(0, T_END)
plt.tight_layout(rect=[0, 0, 1, 0.96])
save_fig(fig, 'D:/HARfinal/PythonDecoder/squat_viz_p6_torque.png')

print("\nAll done.")
print("  squat_viz_p1_timeline.png")
print("  squat_viz_p2_fsr.png")
print("  squat_viz_p3_emg.png")
print("  squat_viz_p4_encoder.png")
print("  squat_viz_p5_fsm.png")
print("  squat_viz_p6_torque.png")
