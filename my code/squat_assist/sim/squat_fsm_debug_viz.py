"""
Squat FSM Debug Visualization
==============================
Compares OLD (buggy) vs NEW (fixed) parameter sets:
  - Motor twitching cause  (noise > threshold -> instant DESCENDING -> safety reset loop)
  - No ascending torque    (gain designed for raw uV, but emg_val is now 0~1 normalized)
  - Fix: 150ms debounce + gain x100 correction + threshold 0.25->0.30

Run:
    pip install matplotlib numpy
    python squat_fsm_debug_viz.py
"""

import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
from matplotlib.gridspec import GridSpec

plt.rcParams["font.family"] = "DejaVu Sans"

# ──────────────────────────────────────────
# Parameters  (mirrors C code values)
# ──────────────────────────────────────────
DT_S = 0.001   # 1 ms loop

OLD = dict(
    gain_resistance    = 0.002,
    gain_assist        = 0.003,
    emg_down_threshold = 0.25,
    emg_up_threshold   = 0.50,
    debounce_ms        = 0,        # no debounce
    max_resistance_nm  = 0.3,
    torque_limit       = 1.0,
    theta_stand        = 5.0,
    theta_sit          = -40.0,
)

NEW = dict(
    gain_resistance    = 0.20,
    gain_assist        = 0.30,
    emg_down_threshold = 0.30,
    emg_up_threshold   = 0.50,
    debounce_ms        = 150,      # 150 ms debounce
    max_resistance_nm  = 0.3,
    torque_limit       = 1.0,
    theta_stand        = 5.0,
    theta_sit          = -40.0,
)

PHASE_NAMES  = ["STAND", "DESCENDING", "BOTTOM", "ASCENDING", "RETURN"]
PHASE_COLORS = ["#4CAF50", "#FF9800", "#F44336", "#2196F3", "#9C27B0"]


# ──────────────────────────────────────────
# Synthetic human signal
# ──────────────────────────────────────────
def make_signals(t):
    """
    One squat + standing noise segments.
    Returns emg (0~1 normalised), angle (deg).
    """
    n     = len(t)
    emg   = np.zeros(n)
    angle = np.full(n, 10.0)

    # Timeline (sec)
    # 0.0 ~ 0.5  : standing  (noise only)
    # 0.5 ~ 1.2  : descend   (EMG rises, angle decreases)
    # 1.2 ~ 2.0  : bottom    (holding)
    # 2.0 ~ 3.0  : ascend    (strong EMG, angle increases)
    # 3.0 ~ 4.0  : return to upright
    # 4.0 ~ 5.0  : standing  (noise only)

    for i, ti in enumerate(t):
        if ti < 0.5:
            noise    = 0.20 + 0.15 * np.sin(2 * np.pi * 8  * ti) + \
                              0.08 * np.sin(2 * np.pi * 23 * ti)
            emg[i]   = np.clip(noise, 0, 1)
            angle[i] = 10.0

        elif ti < 1.2:
            progress = (ti - 0.5) / 0.7
            emg[i]   = 0.45 + 0.20 * progress
            angle[i] = 10.0 - 50.0 * progress

        elif ti < 2.0:
            emg[i]   = 0.30 + 0.10 * np.sin(2 * np.pi * 3 * ti)
            angle[i] = -40.0

        elif ti < 3.0:
            emg[i]   = 0.70 + 0.15 * np.sin(2 * np.pi * 2 * ti)
            progress = (ti - 2.0) / 1.0
            angle[i] = -40.0 + 50.0 * progress

        elif ti < 4.0:
            progress = (ti - 3.0) / 1.0
            emg[i]   = 0.70 * (1 - progress)
            angle[i] = 10.0

        else:
            noise    = 0.18 + 0.12 * np.sin(2 * np.pi * 11 * ti) + \
                              0.07 * np.sin(2 * np.pi * 31 * ti)
            emg[i]   = np.clip(noise, 0, 1)
            angle[i] = 10.0

    return emg, angle


# ──────────────────────────────────────────
# FSM simulator
# ──────────────────────────────────────────
def simulate_fsm(t, emg, angle, cfg):
    n         = len(t)
    phase     = np.zeros(n, dtype=int)
    torque    = np.zeros(n)
    cur_phase = 0
    debounce  = 0

    for i in range(n):
        e   = emg[i]
        a   = angle[i]
        cmd = 0.0

        if cur_phase == 0:       # STAND
            if e > cfg["emg_down_threshold"] and a > cfg["theta_stand"]:
                debounce += 1
                if debounce >= cfg["debounce_ms"]:
                    cur_phase = 1
                    debounce  = 0
            else:
                debounce = 0
            cmd = 0.0

        elif cur_phase == 1:     # DESCENDING
            cmd = cfg["gain_resistance"] * e
            if a < cfg["theta_sit"]:
                cur_phase = 2
            elif e < cfg["emg_down_threshold"] * 0.3:
                cur_phase = 0
                debounce  = 0

        elif cur_phase == 2:     # BOTTOM
            cmd = cfg["max_resistance_nm"]
            if e > cfg["emg_up_threshold"]:
                cur_phase = 3

        elif cur_phase == 3:     # ASCENDING
            cmd = -(cfg["gain_assist"] * e)
            if a > cfg["theta_stand"]:
                cur_phase = 4

        elif cur_phase == 4:     # RETURN
            cmd = -(cfg["gain_assist"] * e * 0.3)
            if e < cfg["emg_down_threshold"] * 0.3:
                cur_phase = 0
                debounce  = 0

        # Emergency safety: angle back to stand during descent/bottom
        if a > cfg["theta_stand"] and cur_phase in (1, 2):
            cur_phase = 0
            debounce  = 0
            cmd       = 0.0

        lim      = cfg["torque_limit"]
        cmd      = max(-lim, min(lim, cmd))
        phase[i] = cur_phase
        torque[i]= cmd

    return phase, torque


# ──────────────────────────────────────────
# Plot
# ──────────────────────────────────────────
def plot_all():
    t             = np.arange(0, 5.0, DT_S)
    emg, angle    = make_signals(t)
    phase_old, torque_old = simulate_fsm(t, emg, angle, OLD)
    phase_new, torque_new = simulate_fsm(t, emg, angle, NEW)

    fig = plt.figure(figsize=(17, 15))
    fig.suptitle(
        "Squat FSM Debug  |  OLD (left, buggy) vs NEW (right, fixed)",
        fontsize=13, fontweight="bold", y=0.99)

    gs = GridSpec(5, 2, figure=fig,
                  hspace=0.60, wspace=0.32,
                  left=0.08, right=0.96, top=0.94, bottom=0.07)

    axes_l = [fig.add_subplot(gs[r, 0]) for r in range(5)]
    axes_r = [fig.add_subplot(gs[r, 1]) for r in range(5)]

    ROW_LABELS = [
        "1. EMG signal (normalised 0~1)",
        "2. Hip angle (deg)",
        "3. FSM state",
        "4. Output torque (Nm)",
        "5. Twitch zoom  0 ~ 0.6 s",
    ]

    def phase_bg(ax, ph, ta):
        for i in range(len(ta) - 1):
            ax.axvspan(ta[i], ta[i+1],
                       color=PHASE_COLORS[ph[i]], alpha=0.12, linewidth=0)

    def style(ax, ylabel, title, xlim=(0, 5)):
        ax.set_xlim(xlim)
        ax.set_ylabel(ylabel, fontsize=8)
        ax.set_title(title, fontsize=9, pad=3)
        ax.tick_params(labelsize=8)
        ax.grid(True, alpha=0.3, linewidth=0.5)
        ax.set_xlabel("Time (s)", fontsize=7)

    for col, (axes, phase, torque, cfg, tag) in enumerate([
        (axes_l, phase_old, torque_old, OLD, "OLD"),
        (axes_r, phase_new, torque_new, NEW, "NEW"),
    ]):
        # ── Row 1: EMG ─────────────────────────────────────
        axes[0].plot(t, emg, color="#E91E63", linewidth=0.8, label="EMG norm")
        axes[0].axhline(cfg["emg_down_threshold"], color="darkorange",
                        linestyle="--", linewidth=1.2,
                        label=f'down_thr={cfg["emg_down_threshold"]}')
        axes[0].axhline(cfg["emg_up_threshold"], color="royalblue",
                        linestyle="--", linewidth=1.2,
                        label=f'up_thr={cfg["emg_up_threshold"]}')
        phase_bg(axes[0], phase, t)
        axes[0].legend(fontsize=7, loc="upper right")
        style(axes[0], "EMG", f"[{tag}]  {ROW_LABELS[0]}")

        # ── Row 2: Angle ────────────────────────────────────
        axes[1].plot(t, angle, color="#795548", linewidth=0.9)
        axes[1].axhline(cfg["theta_stand"], color="green",
                        linestyle="--", linewidth=1.0,
                        label=f'stand={cfg["theta_stand"]} deg')
        axes[1].axhline(cfg["theta_sit"], color="red",
                        linestyle="--", linewidth=1.0,
                        label=f'sit={cfg["theta_sit"]} deg')
        phase_bg(axes[1], phase, t)
        axes[1].legend(fontsize=7, loc="upper right")
        style(axes[1], "Angle (deg)", f"[{tag}]  {ROW_LABELS[1]}")

        # ── Row 3: FSM state ────────────────────────────────
        n_trans = int(np.sum(np.diff(phase) != 0))
        color   = "red" if (col == 0 and n_trans > 20) else "black"
        axes[2].step(t, phase, where="post", color="#333333", linewidth=1.0)
        phase_bg(axes[2], phase, t)
        axes[2].set_yticks(range(5))
        axes[2].set_yticklabels(PHASE_NAMES, fontsize=7)
        style(axes[2], "Phase", f"[{tag}]  {ROW_LABELS[2]}  ({n_trans} transitions)")
        axes[2].title.set_color(color)

        # ── Row 4: Torque ───────────────────────────────────
        axes[3].fill_between(t, torque, 0, where=(torque > 0),
                             color="#FF5722", alpha=0.6, label="Resistance (+)")
        axes[3].fill_between(t, torque, 0, where=(torque < 0),
                             color="#2196F3", alpha=0.6, label="Assist (-)")
        axes[3].plot(t, torque, color="#333333", linewidth=0.5)
        axes[3].axhline(0, color="black", linewidth=0.7)
        axes[3].legend(fontsize=7, loc="upper right")
        phase_bg(axes[3], phase, t)
        mx, mn = torque.max(), torque.min()
        style(axes[3], "Torque (Nm)",
              f"[{tag}]  {ROW_LABELS[3]}   peak+ {mx:.3f} / peak- {mn:.3f} Nm")

        # ── Row 5: Zoom 0~0.6 s ─────────────────────────────
        zm   = t < 0.6
        zt, ze, zp = t[zm], emg[zm], phase[zm]
        axes[4].step(zt, zp, where="post", color="#333333", linewidth=1.2)
        ax_r = axes[4].twinx()
        ax_r.plot(zt, ze, color="#E91E63", linewidth=0.8, alpha=0.7)
        ax_r.axhline(cfg["emg_down_threshold"], color="darkorange",
                     linestyle="--", linewidth=1.0)
        ax_r.set_ylabel("EMG", fontsize=7, color="#E91E63")
        ax_r.tick_params(labelsize=7, colors="#E91E63")
        ax_r.set_ylim(0, 1.1)
        phase_bg(axes[4], zp, zt)
        axes[4].set_yticks(range(5))
        axes[4].set_yticklabels(PHASE_NAMES, fontsize=6)
        axes[4].set_xlim(0, 0.6)
        axes[4].set_xlabel("Time (s)", fontsize=7)
        axes[4].tick_params(labelsize=8)
        axes[4].grid(True, alpha=0.3, linewidth=0.5)
        zt_trans = int(np.sum(np.diff(zp) != 0))
        tc       = "red" if zt_trans > 5 else "green"
        axes[4].set_title(
            f"[{tag}]  {ROW_LABELS[4]}  ({zt_trans} transitions in 0.6 s)",
            fontsize=9, pad=3, color=tc)

    # ── Phase legend ─────────────────────────────────────────────────
    patches = [mpatches.Patch(color=PHASE_COLORS[i], alpha=0.6, label=PHASE_NAMES[i])
               for i in range(5)]
    fig.legend(handles=patches, loc="lower center", ncol=5,
               fontsize=8, title="Phase color key",
               bbox_to_anchor=(0.5, 0.005))

    # ── Root-cause summary box ────────────────────────────────────────
    note = (
        "[BUG 1 - Twitching]  EMG noise spike > threshold(0.25) "
        "-> instant DESCENDING -> safety check (angle>5 deg) resets to STAND "
        "-> repeats every 1 ms loop\n"
        "[BUG 2 - No ascent torque]  gain=0.003 was tuned for raw uV (0~200), "
        "but emg_val became 0~1 normalized  ->  max torque = 0.003 x 1.0 = 0.003 Nm  (imperceptible)\n"
        "[FIX]  debounce 150 ms (must sustain > threshold to enter DESCENDING)  +  "
        "gain x100  (0.002->0.20, 0.003->0.30)  +  threshold 0.25->0.30"
    )
    fig.text(0.5, 0.028, note,
             ha="center", va="bottom", fontsize=7.5,
             bbox=dict(boxstyle="round,pad=0.45",
                       facecolor="#FFF9C4", edgecolor="#F9A825", linewidth=1.2))

    plt.savefig("squat_fsm_debug.png", dpi=150, bbox_inches="tight")
    print("Saved: squat_fsm_debug.png")
    plt.show()


if __name__ == "__main__":
    plot_all()
