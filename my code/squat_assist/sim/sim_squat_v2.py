"""Simulate Squat_EMG_Assist2.c logic on MaxTorque0.csv input.

Reproduces v2 algorithm:
  - User lookup table for ASCENDING/BOTTOM(HOLD) torque
  - Sliding-window dwell-time intent detection
  - 500ms ramp-in on base torque
  - Compensation: 0.5 * 8 * sin(theta), clamped to non-negative
  - Final clamp to +/- 10 Nm

Output: PNG matching the user's reference figure layout.
"""
import math
import sys
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import matplotlib

matplotlib.rcParams['font.family'] = ['Malgun Gothic', 'sans-serif']
matplotlib.rcParams['axes.unicode_minus'] = False

CSV_PATH = r'C:\Users\user\Documents\카카오톡 받은 파일\MaxTorque0.csv'

# --- v2 parameters (must match Squat_EMG_Assist2.c) ---
DEG_TO_RAD = math.pi / 180.0

GRAVITY_MGL_NM = 8.0
COMPENSATION_SCALE = 0.5

SQUAT_ENTER_DEG = 15.0
SQUAT_STAND_DEG = 5.0
SQUAT_DEBOUNCE_MS = 150

DWELL_ANGLE_DEAD_DEG = 1.5
DWELL_THRESHOLD_MS = 200
DWELL_EXIT_ANGLE_DEG = 2.5

ASSIST_RAMP_TIME_MS = 500

TORQUE_LIMIT = 10.0
HARD_MAX = 10.0

SQUAT_MAX_TORQUE_NM = 6.0  # default; used as gain (lookup designed for 6.0)

# ASCEND lookup (5°-70°, 5° step, Nm)
USER_LOOKUP_ASCEND = [
    3.94, 3.80, 3.96, 3.51, 2.74, 2.16, 2.07,
    1.20, 0.42, 0.21, 0.00, 0.00, 0.00, 0.00,
]
LOOKUP_BASE_DEG = 5.0
LOOKUP_STEP_DEG = 5.0

# Phase IDs (match FSM)
STAND, DESCENDING, BOTTOM, ASCENDING, RETURN_PH = 0, 1, 2, 3, 4
# Intent IDs
INT_DESCEND, INT_HOLD, INT_ASCEND = 0, 1, 2


def lookup_user_ascend(angle_deg):
    if angle_deg <= 0.0:
        return 0.0
    if angle_deg <= LOOKUP_BASE_DEG:
        return USER_LOOKUP_ASCEND[0] * (angle_deg / LOOKUP_BASE_DEG)
    idx_f = (angle_deg - LOOKUP_BASE_DEG) / LOOKUP_STEP_DEG
    idx_low = int(idx_f)
    if idx_low >= len(USER_LOOKUP_ASCEND) - 1:
        return USER_LOOKUP_ASCEND[-1]
    frac = idx_f - idx_low
    v = USER_LOOKUP_ASCEND[idx_low] * (1.0 - frac) + USER_LOOKUP_ASCEND[idx_low + 1] * frac
    return max(0.0, v)


def clamp(x, lo, hi):
    return max(lo, min(hi, x))


def simulate(df):
    # Each row treated as 1ms step (seq_id based)
    n = len(df)
    out = {
        'time_s': df['time_s'].values,
        'L_Angle': df['L_Angle'].values,
        'R_Angle': df['R_Angle'].values,
        'EMG_LH': df['EMG_LH_Env'].values,
        'EMG_RH': df['EMG_RH_Env'].values,
        'phase': np.zeros(n, dtype=int),
        'intent': np.zeros(n, dtype=int),
        'L_Torque': np.zeros(n),
        'R_Torque': np.zeros(n),
        'base': np.zeros(n),
        'ramp_mul': np.zeros(n),
        'comp_lh': np.zeros(n),
        'comp_rh': np.zeros(n),
    }

    # State
    phase = STAND
    debounce_active = False
    debounce_tick = 0
    intent = INT_DESCEND
    dwell_win_start_tick = 0
    dwell_win_start_angle = (df['L_Angle'].iloc[0] + df['R_Angle'].iloc[0]) * 0.5
    ramp_active = False
    ramp_tick = 0
    prev_base_active = False

    # CSV rows are ~10ms apart in real time. Use time_s for accurate timing.
    times_s = df['time_s'].values
    t0 = times_s[0]

    for i in range(n):
        now = int((times_s[i] - t0) * 1000.0)  # ms since start
        lh = df['L_Angle'].iloc[i]
        rh = df['R_Angle'].iloc[i]
        avg_angle = (lh + rh) * 0.5

        # --- Update User Intent (sliding-window dwell) ---
        if phase in (STAND, RETURN_PH):
            intent = INT_DESCEND
            dwell_win_start_tick = now
            dwell_win_start_angle = avg_angle
        else:
            elapsed = now - dwell_win_start_tick
            if intent == INT_HOLD:
                change = avg_angle - dwell_win_start_angle
                if abs(change) > DWELL_EXIT_ANGLE_DEG:
                    intent = INT_DESCEND if change > 0 else INT_ASCEND
                    dwell_win_start_tick = now
                    dwell_win_start_angle = avg_angle
            else:
                if elapsed >= DWELL_THRESHOLD_MS:
                    change = avg_angle - dwell_win_start_angle
                    ach = abs(change)
                    if ach < DWELL_ANGLE_DEAD_DEG:
                        intent = INT_HOLD
                    elif change > 0:
                        intent = INT_DESCEND
                    else:
                        intent = INT_ASCEND
                    dwell_win_start_tick = now
                    dwell_win_start_angle = avg_angle

        # --- FSM ---
        enter_zone = avg_angle >= SQUAT_ENTER_DEG
        stand_zone = avg_angle <= SQUAT_STAND_DEG

        if phase == STAND:
            if enter_zone:
                if not debounce_active:
                    debounce_active = True
                    debounce_tick = now
                elif (now - debounce_tick) >= SQUAT_DEBOUNCE_MS:
                    phase = DESCENDING
                    debounce_active = False
            else:
                debounce_active = False

        elif phase == DESCENDING:
            if intent == INT_HOLD and enter_zone:
                phase = BOTTOM
                debounce_active = False
            elif not enter_zone:
                phase = RETURN_PH
                debounce_active = False

        elif phase == BOTTOM:
            if intent == INT_ASCEND:
                phase = ASCENDING

        elif phase == ASCENDING:
            if not enter_zone:
                phase = RETURN_PH
                debounce_active = False

        elif phase == RETURN_PH:
            if stand_zone:
                if not debounce_active:
                    debounce_active = True
                    debounce_tick = now
                elif (now - debounce_tick) >= SQUAT_DEBOUNCE_MS:
                    phase = STAND
                    debounce_active = False
            elif enter_zone:
                phase = ASCENDING
                debounce_active = False
            else:
                debounce_active = False

        # --- Compensation (apply per leg) ---
        lh_rad = lh * DEG_TO_RAD
        rh_rad = rh * DEG_TO_RAD
        gravity_lh = GRAVITY_MGL_NM * math.sin(lh_rad)
        gravity_rh = GRAVITY_MGL_NM * math.sin(rh_rad)
        gravity_lh = max(0.0, gravity_lh)  # neg clamp from v1 carried over
        gravity_rh = max(0.0, gravity_rh)
        comp_lh = COMPENSATION_SCALE * gravity_lh
        comp_rh = COMPENSATION_SCALE * gravity_rh

        # --- Base torque (v2 lookup) ---
        if phase == ASCENDING:
            target_base = lookup_user_ascend(avg_angle)
        elif phase == BOTTOM:
            target_base = lookup_user_ascend(avg_angle) if intent == INT_HOLD else 0.0
        else:
            target_base = 0.0

        target_base *= (SQUAT_MAX_TORQUE_NM / 6.0)

        # --- Ramp ---
        base_active = target_base > 0.05
        if base_active:
            if not prev_base_active:
                ramp_tick = now
                ramp_active = True
            prev_base_active = True
            if ramp_active:
                el = now - ramp_tick
                if el >= ASSIST_RAMP_TIME_MS:
                    ramp_mul = 1.0
                    ramp_active = False
                else:
                    ramp_mul = el / float(ASSIST_RAMP_TIME_MS)
            else:
                ramp_mul = 1.0
        else:
            ramp_active = False
            prev_base_active = False
            ramp_mul = 0.0

        base_lh = target_base * ramp_mul
        base_rh = target_base * ramp_mul

        # --- Final clamp ---
        l_torque = clamp(base_lh + comp_lh, -TORQUE_LIMIT, TORQUE_LIMIT)
        r_torque = clamp(base_rh + comp_rh, -TORQUE_LIMIT, TORQUE_LIMIT)

        out['phase'][i] = phase
        out['intent'][i] = intent
        out['base'][i] = target_base * ramp_mul
        out['ramp_mul'][i] = ramp_mul
        out['comp_lh'][i] = comp_lh
        out['comp_rh'][i] = comp_rh
        out['L_Torque'][i] = l_torque
        out['R_Torque'][i] = r_torque

    return out


def main():
    df = pd.read_csv(CSV_PATH)
    print(f'Loaded {len(df)} rows from {CSV_PATH}')
    print(f'Time range: {df.time_s.min():.2f} - {df.time_s.max():.2f} s')

    sim = simulate(df)

    # --- Plot in 4-panel layout ---
    fig, axes = plt.subplots(4, 1, figsize=(11, 12), sharex=True)

    t = sim['time_s']
    # restrict to 10-40s like the reference
    mask = (t >= 10.0) & (t <= 41.0)

    # 1. EMG Envelope
    ax = axes[0]
    ax.plot(t[mask], sim['EMG_LH'][mask], 'b-', label='Left Hip (LH)', linewidth=0.8)
    ax.plot(t[mask], sim['EMG_RH'][mask], 'r-', label='Right Hip (RH)', linewidth=0.8)
    ax.set_ylabel('Voltage (V)')
    ax.set_title('1. EMG Envelope (근활성도)')
    ax.legend(loc='upper right')
    ax.grid(True, alpha=0.3)

    # 2. Hip Joint Control Angle
    ax = axes[1]
    ax.plot(t[mask], sim['L_Angle'][mask], 'b-', label='Left Hip (LH)', linewidth=1.0)
    ax.plot(t[mask], sim['R_Angle'][mask], 'r-', label='Right Hip (RH)', linewidth=1.0)
    ax.set_ylabel('Angle (deg)')
    ax.set_title('2. Hip Joint Control Angle (영점 조정 각도)')
    ax.legend(loc='lower right')
    ax.grid(True, alpha=0.3)
    ax.axhline(0, color='k', linewidth=0.3)

    # 3. Applied Assist Torque (SIMULATED v2)
    ax = axes[2]
    ax.plot(t[mask], sim['L_Torque'][mask], 'b-', label='Left Hip (LH)', linewidth=1.0)
    ax.plot(t[mask], sim['R_Torque'][mask], 'r-', label='Right Hip (RH)', linewidth=1.0)
    ax.set_ylabel('Torque (Nm)')
    ax.set_title('3. Applied Assist Torque (보조 토크 명령) — v2 시뮬레이션')
    ax.legend(loc='upper right')
    ax.grid(True, alpha=0.3)
    ax.axhline(0, color='k', linewidth=0.3)

    # 4. Squat FSM Phase & Intent
    ax = axes[3]
    ax.plot(t[mask], sim['phase'][mask], 'k-', label='Squat Phase (0:STAND, 1:DESC, 2:BTM, 3:ASC, 4:RET)', linewidth=1.2)
    # Map intent to a position for visualization
    ax.plot(t[mask], sim['intent'][mask] + 0.1, 'g--', label='User Intent (0:DESC, 1:HOLD, 2:ASC)', linewidth=1.0, alpha=0.7)
    ax.axhline(1, color='m', linestyle='--', linewidth=0.8, label='Control Active (1:ON)')
    ax.set_ylabel('ID / State')
    ax.set_xlabel('Time (seconds)')
    ax.set_title('4. Squat FSM Phase, User Intent & Control Toggle')
    ax.set_ylim(-0.5, 4.5)
    ax.legend(loc='upper left', fontsize=8)
    ax.grid(True, alpha=0.3)

    plt.suptitle('Squat_EMG_Assist2.c 시뮬레이션 결과 — MaxTorque0.csv 입력', fontsize=12, y=0.995)
    plt.tight_layout()

    out_path = r'D:\HARfinal\PythonDecoder\sim_v2_torque0.png'
    plt.savefig(out_path, dpi=110, bbox_inches='tight')
    print(f'Saved: {out_path}')

    # Print summary stats
    print('\n--- Simulation summary (in 10-41s window) ---')
    print(f'Phase distribution:')
    phase_names = {0: 'STAND', 1: 'DESC', 2: 'BOTTOM', 3: 'ASC', 4: 'RETURN'}
    for p in range(5):
        n_in = int((sim['phase'][mask] == p).sum())
        pct = 100.0 * n_in / mask.sum()
        print(f'  {phase_names[p]:8s}: {n_in:5d} ({pct:5.1f}%)')
    print(f'Torque stats (L_Torque):')
    print(f'  max  = {sim["L_Torque"][mask].max():.2f} Nm')
    print(f'  mean (when in ASC) = {sim["L_Torque"][mask & (sim["phase"]==3)].mean():.2f} Nm')
    print(f'  mean (when in BOTTOM) = {sim["L_Torque"][mask & (sim["phase"]==2)].mean():.2f} Nm')
    print(f'  mean (when in DESC) = {sim["L_Torque"][mask & (sim["phase"]==1)].mean():.2f} Nm')


if __name__ == '__main__':
    main()
