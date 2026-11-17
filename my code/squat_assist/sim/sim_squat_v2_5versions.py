"""
Squat_EMG_Assist2.c 5-version lookup table simulation (v2 FSM full re-simulation).
Input: MaxTorque0.csv (EMG envelope, hip angles).
- CSV의 Squat_Ph는 무시: v2 FSM(dwell intent 기반)을 처음부터 재시뮬레이션.
- Output: 8-panel plot (2 col x 4 row).
"""

import csv
import numpy as np
import matplotlib.pyplot as plt
import matplotlib
matplotlib.rcParams["font.family"] = ["Malgun Gothic", "NanumGothic", "DejaVu Sans"]
matplotlib.rcParams["axes.unicode_minus"] = False

CSV = r"C:\Users\user\Documents\카카오톡 받은 파일\MaxTorque0.csv"

# Firmware constants / Live-Expr defaults
GRAVITY_MGL = 8.0
SQUAT_MAX_TORQUE = 6.0
HARD_MAX = 10.0
ASSIST_LIMIT = 10.0
ASSIST_RAMP_MS = 500.0
DWELL_ANGLE_DEAD = 1.5
DWELL_THRESHOLD_MS = 200
DWELL_EXIT_ANGLE = 2.5
LOOKUP_BASE_DEG = 5.0
LOOKUP_STEP_DEG = 5.0
SQUAT_ENTER_DEG = 15.0
SQUAT_STAND_DEG = 5.0
SQUAT_DEBOUNCE_MS = 150
EMG_ASCEND_THRESHOLD = 0.5

# Phase IDs
STAND, DESC, BOTTOM, ASC, RETURN = 0, 1, 2, 3, 4
# Intent IDs
INT_DESC, INT_HOLD, INT_ASC = 0, 1, 2

# 5 lookup versions (smoothed median based)
LOOKUP_A = [7.89, 7.79, 8.21, 8.63, 8.15, 6.81, 5.62, 4.71, 3.31, 1.84, 0.67, 0.00, 0.00, 0.58]
LOOKUP_B = [4.60, 4.39, 4.51, 4.63, 4.21, 3.29, 2.45, 1.80, 0.86, 0.00, 0.00, 0.00, 0.00, 0.00]
LOOKUP_C = [8.24, 8.48, 9.24, 10.00, 9.84, 8.81, 7.91, 7.28, 6.14, 4.91, 3.94, 3.23, 3.45, 4.34]
LOOKUP_D = [4.95, 5.09, 5.54, 6.00, 5.90, 5.29, 4.75, 4.37, 3.69, 2.94, 2.37, 1.94, 2.07, 2.60]
LOOKUP_E = [7.55, 7.09, 7.17, 7.26, 6.46, 4.81, 3.32, 2.14, 0.49, 0.00, 0.00, 0.00, 0.00, 0.00]

VERSIONS = {
    "A": dict(lut=LOOKUP_A, comp_scale=0.5, comp_on=1, color="#1f77b4",
              title="Torque A (peak 10, comp x 0.5)"),
    "B": dict(lut=LOOKUP_B, comp_scale=0.5, comp_on=1, color="#ff7f0e",
              title="Torque B (peak 6,  comp x 0.5)"),
    "C": dict(lut=LOOKUP_C, comp_scale=0.0, comp_on=0, color="#2ca02c",
              title="Torque C (peak 10, NO comp)"),
    "D": dict(lut=LOOKUP_D, comp_scale=0.0, comp_on=0, color="#d62728",
              title="Torque D (peak 6,  NO comp)"),
    "E": dict(lut=LOOKUP_E, comp_scale=1.0, comp_on=1, color="#9467bd",
              title="Torque E (peak 10, comp x 1.0)"),
}


def lookup_interp(lut, angle_deg):
    if angle_deg <= LOOKUP_BASE_DEG:
        if angle_deg <= 0.0:
            return 0.0
        return lut[0] * (angle_deg / LOOKUP_BASE_DEG)
    idx_f = (angle_deg - LOOKUP_BASE_DEG) / LOOKUP_STEP_DEG
    idx_low = int(idx_f)
    if idx_low >= len(lut) - 1:
        return lut[-1]
    frac = idx_f - idx_low
    v = lut[idx_low] * (1.0 - frac) + lut[idx_low + 1] * frac
    return max(0.0, v)


def clamp(x, lo, hi):
    return max(lo, min(hi, x))


def simulate_fsm_and_torque(times_ms, angles, emg_avg, version):
    """v2 FSM (dwell intent) + base/ramp/comp 전체 재시뮬레이션."""
    lut = version["lut"]
    cs = version["comp_scale"]
    comp_on = version["comp_on"]

    n = len(times_ms)
    phases = np.zeros(n, dtype=int)
    intents = np.zeros(n, dtype=int)
    torques = np.zeros(n)

    # FSM state
    phase = STAND
    deb_active = False
    deb_t = times_ms[0]

    # Intent state
    intent = INT_DESC
    win_start_t = times_ms[0]
    win_start_angle = angles[0]

    # Ramp state
    prev_base_active = False
    ramp_start_t = times_ms[0]
    ramp_active = False

    for i in range(n):
        t = times_ms[i]
        ang = angles[i]
        emg = emg_avg[i]

        enter_zone = (ang >= SQUAT_ENTER_DEG)
        stand_zone = (ang <= SQUAT_STAND_DEG)

        # --- Intent update (v2 dwell) ---
        if phase in (STAND, RETURN):
            intent = INT_DESC
            win_start_t = t
            win_start_angle = ang
        else:
            elapsed = t - win_start_t
            if intent == INT_HOLD:
                change = ang - win_start_angle
                if abs(change) > DWELL_EXIT_ANGLE:
                    intent = INT_DESC if change > 0 else INT_ASC
                    win_start_t = t
                    win_start_angle = ang
            else:
                if elapsed >= DWELL_THRESHOLD_MS:
                    change = ang - win_start_angle
                    if abs(change) < DWELL_ANGLE_DEAD:
                        intent = INT_HOLD
                    elif change > 0:
                        intent = INT_DESC
                    else:
                        intent = INT_ASC
                    win_start_t = t
                    win_start_angle = ang

        # --- Phase FSM (v2) ---
        if phase == STAND:
            if enter_zone:
                if not deb_active:
                    deb_active = True
                    deb_t = t
                elif (t - deb_t) >= SQUAT_DEBOUNCE_MS:
                    phase = DESC
                    deb_active = False
            else:
                deb_active = False

        elif phase == DESC:
            if intent == INT_HOLD and enter_zone:
                phase = BOTTOM
                deb_active = False
            elif not enter_zone:
                phase = RETURN
                deb_active = False

        elif phase == BOTTOM:
            emg_push = (EMG_ASCEND_THRESHOLD > 0) and (emg >= EMG_ASCEND_THRESHOLD)
            if intent == INT_ASC or emg_push:
                phase = ASC

        elif phase == ASC:
            if not enter_zone:
                phase = RETURN
                deb_active = False

        elif phase == RETURN:
            if stand_zone:
                if not deb_active:
                    deb_active = True
                    deb_t = t
                elif (t - deb_t) >= SQUAT_DEBOUNCE_MS:
                    phase = STAND
                    deb_active = False
            elif enter_zone:
                phase = ASC
                deb_active = False
            else:
                deb_active = False

        phases[i] = phase
        intents[i] = intent

        # --- base target ---
        if phase == ASC:
            target_base = lookup_interp(lut, ang)
        elif phase == BOTTOM and intent == INT_HOLD:
            target_base = lookup_interp(lut, ang)
        else:
            target_base = 0.0

        target_base *= (SQUAT_MAX_TORQUE / 6.0)
        base_active = target_base > 0.05

        # --- ramp ---
        if base_active:
            if not prev_base_active:
                ramp_start_t = t
                ramp_active = True
            if ramp_active:
                el = t - ramp_start_t
                if el >= ASSIST_RAMP_MS:
                    ramp_active = False
                    mul = 1.0
                else:
                    mul = el / ASSIST_RAMP_MS
            else:
                mul = 1.0
        else:
            mul = 0.0
            ramp_active = False
        prev_base_active = base_active

        base = target_base * mul

        # --- compensation ---
        theta = np.deg2rad(ang)
        gravity = max(0.0, GRAVITY_MGL * np.sin(theta))
        comp = (cs * gravity) if comp_on else 0.0

        torques[i] = clamp(base + comp, -ASSIST_LIMIT, ASSIST_LIMIT)

    return phases, intents, torques


def main():
    with open(CSV) as f:
        rows = list(csv.DictReader(f))

    t_s = np.array([float(r["time_s"]) for r in rows])
    emg_lh = np.array([float(r["EMG_LH_Env"]) for r in rows])
    emg_rh = np.array([float(r["EMG_RH_Env"]) for r in rows])
    ang_l = np.array([float(r["L_Angle"]) for r in rows])
    ang_r = np.array([float(r["R_Angle"]) for r in rows])
    ctrl = np.array([int(float(r["Ctrl_ON"])) for r in rows])

    t_ms = t_s * 1000.0
    avg_ang = (ang_l + ang_r) / 2.0
    emg_avg = (emg_lh + emg_rh) / 2.0  # for emg_ascend_threshold check (envelope V)

    # Re-simulate FSM/torque for all 5 versions
    # Phase/intent depend on version slightly (EMG push), but for plot 3 we show A's result.
    results = {}
    for vid, ver in VERSIONS.items():
        results[vid] = simulate_fsm_and_torque(t_ms, avg_ang, emg_avg, ver)

    # Use Version A's phase/intent for plot 3 (representative)
    phases_A, intents_A, _ = results["A"]

    fig, axes = plt.subplots(4, 2, figsize=(15, 14), sharex=True)
    fig.suptitle("Squat_EMG_Assist2.c - 5 lookup version simulation (v2 FSM re-sim, MaxTorque0.csv)",
                 fontsize=13, fontweight="bold")

    ax = axes[0, 0]
    ax.plot(t_s, emg_lh, color="#1f77b4", label="Left Hip", lw=1.2)
    ax.plot(t_s, emg_rh, color="#d62728", label="Right Hip", lw=1.2)
    ax.set_ylabel("EMG (V)")
    ax.set_title("1. EMG Envelope")
    ax.legend(loc="upper right", fontsize=8)
    ax.grid(alpha=0.3)

    ax = axes[0, 1]
    ax.plot(t_s, ang_l, color="#1f77b4", label="Left Hip", lw=1.2)
    ax.plot(t_s, ang_r, color="#d62728", label="Right Hip", lw=1.2)
    ax.set_ylabel("Angle (deg)")
    ax.set_title("2. Hip Joint Control Angle")
    ax.legend(loc="upper right", fontsize=8)
    ax.grid(alpha=0.3)
    ax.axhline(0, color="k", lw=0.5)

    ax = axes[1, 0]
    ax.plot(t_s, phases_A, color="black",
            label="Squat Phase (0:STAND 1:DESC 2:BTM 3:ASC 4:RET)", lw=1.5)
    ax.plot(t_s, intents_A, color="#2ca02c",
            label="User Intent (0:DESC 1:HOLD 2:ASC)", lw=1.0, linestyle="--")
    ax.plot(t_s, ctrl, color="#9467bd",
            label="Control Active", lw=0.8, linestyle=":")
    ax.set_ylabel("ID / State")
    ax.set_title("3. v2 FSM Phase, User Intent (re-simulated)")
    ax.legend(loc="upper right", fontsize=7)
    ax.grid(alpha=0.3)
    ax.set_ylim(-0.5, 4.5)

    torque_positions = [("A", 1, 1), ("B", 2, 0), ("C", 2, 1), ("D", 3, 0), ("E", 3, 1)]
    plot_idx = 4
    for vid, r, c in torque_positions:
        ax = axes[r, c]
        ver = VERSIONS[vid]
        _, _, tq = results[vid]
        ax.plot(t_s, tq, color=ver["color"], lw=1.3, label=vid)
        ax.fill_between(t_s, 0, tq, color=ver["color"], alpha=0.15)
        ax.set_ylabel("Torque (Nm)")
        ax.set_title(f"{plot_idx}. {ver['title']}")
        ax.legend(loc="upper right", fontsize=8)
        ax.grid(alpha=0.3)
        ax.set_ylim(-0.5, 11)
        ax.axhline(0, color="k", lw=0.5)
        ax.axhline(HARD_MAX, color="red", lw=0.5, linestyle=":", alpha=0.5)
        plot_idx += 1

    axes[3, 0].set_xlabel("Time (s)")
    axes[3, 1].set_xlabel("Time (s)")

    plt.tight_layout(rect=[0, 0, 1, 0.97])

    out_path = r"D:\HARfinal\PythonDecoder\sim_5versions_result.png"
    plt.savefig(out_path, dpi=120, bbox_inches="tight")
    print(f"Saved: {out_path}")

    print("\n=== Peak torque summary (Nm) ===")
    for vid in VERSIONS.keys():
        _, _, tq = results[vid]
        active = tq[tq > 0.5]
        m = active.mean() if active.size > 0 else 0
        print(f"  Version {vid}: peak {tq.max():.2f}, mean(active) {m:.2f}")


if __name__ == "__main__":
    main()
