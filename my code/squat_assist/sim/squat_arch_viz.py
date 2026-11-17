"""
Squat EMG Assist — Architecture Visualization
Saves: squat_arch_viz.png
"""

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
from matplotlib.patches import FancyBboxPatch, FancyArrowPatch
from matplotlib.gridspec import GridSpec
import matplotlib.patheffects as pe

# ── colour palette ──────────────────────────────────────────────────
C = {
    "fsr":   "#4CAF50",   # green
    "emg":   "#2196F3",   # blue
    "fsm":   "#FF9800",   # orange
    "comp":  "#9C27B0",   # purple
    "gate":  "#F44336",   # red
    "cdc":   "#607D8B",   # grey-blue
    "sys":   "#795548",   # brown
    "pin":   "#009688",   # teal
    "bg":    "#FAFAFA",
    "panel": "#F5F5F5",
    "head":  "#37474F",
    "take":  "#E8F5E9",
    "skip":  "#FFEBEE",
    "warn":  "#FFF9C4",
}

fig = plt.figure(figsize=(26, 34), facecolor=C["bg"])
fig.suptitle(
    "Squat EMG Assist  —  Architecture & Code-Assembly Map",
    fontsize=18, fontweight="bold", color=C["head"], y=0.992
)

gs = GridSpec(5, 3, figure=fig,
              hspace=0.55, wspace=0.30,
              left=0.03, right=0.97, top=0.975, bottom=0.01)

# ============================================================
# helper: rounded box
# ============================================================
def rbox(ax, x, y, w, h, color, alpha=0.85, lw=1.2, ec=None):
    ec = ec or color
    p = FancyBboxPatch((x, y), w, h,
                       boxstyle="round,pad=0.015",
                       facecolor=color, edgecolor=ec,
                       linewidth=lw, alpha=alpha,
                       transform=ax.transAxes, clip_on=False)
    ax.add_patch(p)

def hdr(ax, x, y, w, h, text, color, fontsize=9):
    rbox(ax, x, y, w, h, color, alpha=0.92)
    ax.text(x + w/2, y + h/2, text,
            ha="center", va="center",
            fontsize=fontsize, fontweight="bold",
            color="white", transform=ax.transAxes)

def cell(ax, x, y, w, h, text, bg="#FFFFFF", fontsize=7.5,
         ha="left", bold=False, color="#212121"):
    rbox(ax, x, y, w, h, bg, alpha=0.80, lw=0.6)
    fw = "bold" if bold else "normal"
    ax.text(x + 0.012, y + h/2, text,
            ha=ha, va="center",
            fontsize=fontsize, fontweight=fw, color=color,
            transform=ax.transAxes)

def blank(ax):
    ax.set_xlim(0, 1); ax.set_ylim(0, 1)
    ax.axis("off")

# ============================================================
# PANEL 1  (row 0, col 0-2 span)  — Source File Map
# ============================================================
ax1 = fig.add_subplot(gs[0, :])
blank(ax1)
ax1.text(0.5, 0.97, "(1) Source Files  →  Functions to Extract",
         ha="center", va="top", fontsize=13, fontweight="bold", color=C["head"])

files = [
    ("Final_FSR_Fuzzy_Logic.c",  C["fsr"],  [
        ("[+] _SampleFsr()",              "ADC read + 8 Hz LPF for PF3-PF6",                        True),
        ("[+] _UpdateCalibration()",      "FSR cal: BTN1/2 PRESSED → zero / full-load",             True),
        ("[+] _StartCalibration()",       "accumulate 1 s average into s_fsr_off_v / s_fsr_on_v",   True),
        ("[+] _ResetCalibration()",       "restore defaults, clear flags",                           True),
        ("[+] _UpdateLoads()",            "normalize: (lpf-off)/(on-off), clamp 0~1.5",             True),
        ("[+] CDC macro system",          "CDC_STREAM_CHANNELS → struct + meta JSON + _SendStream", True),
        ("[+] _ClampFloat()",             "utility — keep ONE copy only",                            True),
        ("[-] _UpdateFootDetector()",     "gait phase detection — NOT needed for squat",             False),
        ("[-] _MembershipLarge()",        "tanh fuzzy membership — NOT needed",                      False),
        ("[-] push-off pulse vars",       "s_left/right_push_off_remaining_ms — NOT needed",         False),
    ]),
    ("Final_EMG.c",              C["emg"],  [
        ("[+] _SampleAdcChannels()",      "read PF7/PF8 (XM_EXT_ADC_9/10) into s_emg_raw_v[2]",    True),
        ("[+] _ProcessEmgSignals()",      "bias→80Hz LPF→rectify→5Hz envelope (per channel)",       True),
        ("[+] _LowPassUpdate()",          "IIR: prev + alpha*(input-prev)",                          True),
        ("[+] _AbsFloat()",               "full-wave rectification helper",                          True),
        ("[+] EMG calibration trio",      "_StartRestCal / _StartEffortCal / _ResetEmgCal  (CLICK)",True),
        ("[+] _EnvelopeToTorque()  →mod", "change output: raw Nm  →  0~1 normalised only",          True),
        ("[-] TSM state machine",         "Off/Standby/Active — unnecessary complexity",             False),
        ("[-] _SelectTorquePair()",       "pair routing not needed; squat uses both channels",       False),
        ("[-] _UpdateUsbDebug()",         "replaced by CDC macro stream",                            False),
    ]),
    ("Final_Compensation_AddOn.c", C["comp"], [
        ("[+] _UpdateCompensation()",     "gravity = mgl*sin(θ), friction = coulomb+viscous",        True),
        ("[+] angle + velocity read",     "leftHipAngle + differentiator + LPF velocity",            True),
        ("[+] _SignFloat()",              "utility: sign(x) ∈ {-1,0,+1}",                            True),
        ("[-] BTN toggle logic",          "BTN1/2 toggle pattern — replaced by our button scheme",   False),
    ]),
    ("Final_Encoder_ex01.c",     C["sys"],  [
        ("[+] _UpdateButtons() pattern",  "read all 3 buttons ONCE, store in local struct",          True),
        ("[+] s_assist_session_active",   "first-entry init pattern on ASSIST mode entry",            True),
        ("[+] mode-change safety flush",  "zero torque on ASSIST→STANDBY transition",                True),
        ("[-] pulse trigger/rearm logic", "angle-threshold pulse — replaced by squat FSM",           False),
    ]),
    ("Final_FSR.c",              C["pin"],  [
        ("[+] contact hysteresis consts", "FSR_ON_THRESHOLD=0.35, FSR_OFF_THRESHOLD=0.20",           True),
        ("[+] _LowPassUpdate() variant",  "same formula — merge into single shared copy",            True),
        ("[-] _LoadToTorque()",           "FSR→torque direct — squat uses EMG for torque",           False),
        ("[-] TSM / Active_Loop()",       "TSM not used in new file",                                False),
    ]),
]

n_files = len(files)
col_w = 1.0 / n_files
row_h = 0.06
top_y = 0.90

for fi, (fname, fc, items) in enumerate(files):
    fx = fi * col_w + 0.003
    fw = col_w - 0.006
    hdr(ax1, fx, top_y, fw, 0.07, fname, fc, fontsize=8)
    for ri, (func, desc, keep) in enumerate(items):
        bg = C["take"] if keep else C["skip"]
        ry = top_y - (ri + 1) * row_h - 0.005
        rbox(ax1, fx, ry, fw, row_h - 0.005, bg, lw=0.5)
        icon = "[+]" if keep else "[-]"
        ax1.text(fx + 0.004, ry + (row_h-0.005)/2,
                 f"{icon} {func}",
                 va="center", fontsize=6.2, fontweight="bold" if keep else "normal",
                 color="#1B5E20" if keep else "#B71C1C",
                 transform=ax1.transAxes)
        ax1.text(fx + 0.004, ry + (row_h-0.005)/2 - 0.028,
                 f"   {desc}",
                 va="center", fontsize=5.5, color="#424242",
                 transform=ax1.transAxes)

# legend
leg = [mpatches.Patch(fc=C["take"], ec="#4CAF50", label="Use (keep / adapt)"),
       mpatches.Patch(fc=C["skip"], ec="#F44336", label="Discard")]
ax1.legend(handles=leg, loc="lower right", fontsize=8, framealpha=0.9)

# ============================================================
# PANEL 2  (row 1, col 0)  — Pin Assignment
# ============================================================
ax2 = fig.add_subplot(gs[1, 0])
blank(ax2)
ax2.text(0.5, 0.98, "(2) Pin Assignment  (PF3–PF8)", ha="center", va="top",
         fontsize=11, fontweight="bold", color=C["head"])

cols_p = ["Pin", "XM_EXT_ADC", "XM_EXT_DIO", "Role", "Module"]
cw = [0.12, 0.18, 0.17, 0.30, 0.18]
rows_p = [
    ("PF3", "ADC_5",  "DIO_1", "FSR Left Toe",    "FSR"),
    ("PF4", "ADC_6",  "DIO_2", "FSR Left Heel",   "FSR"),
    ("PF5", "ADC_7",  "DIO_3", "FSR Right Toe",   "FSR"),
    ("PF6", "ADC_8",  "DIO_4", "FSR Right Heel",  "FSR"),
    ("PF7", "ADC_9",  "DIO_5", "EMG Right Hip",   "EMG"),
    ("PF8", "ADC_10", "DIO_6", "EMG Left Hip",    "EMG"),
]
col_colors = {"FSR": C["fsr"], "EMG": C["emg"]}

y0 = 0.86; rh = 0.10
x0 = 0.01
for ci, (cname, cw_) in enumerate(zip(cols_p, cw)):
    xc = x0 + sum(cw[:ci])
    hdr(ax2, xc, y0, cw_, rh, cname, C["head"], fontsize=8)
for ri, row in enumerate(rows_p):
    mod = row[-1]
    bg = "#E8F5E9" if mod == "FSR" else "#E3F2FD"
    ry = y0 - (ri + 1) * rh - 0.01
    for ci, (val, cw_) in enumerate(zip(row, cw)):
        xc = x0 + sum(cw[:ci])
        bcolor = col_colors[mod] if ci == 4 else bg
        tc = "white" if ci == 4 else "#212121"
        rbox(ax2, xc, ry, cw_, rh - 0.01, bcolor, lw=0.5)
        ax2.text(xc + cw_/2, ry + (rh-0.01)/2, val,
                 ha="center", va="center", fontsize=7.5,
                 fontweight="bold" if ci in (0,4) else "normal",
                 color=tc, transform=ax2.transAxes)

# Setup code snippet
ax2.text(0.01, 0.25,
    "User_Setup() — DIO → ADC switch:\n"
    "  XM_SwitchDioToAdc(XM_EXT_DIO_1);  // PF3\n"
    "  XM_SwitchDioToAdc(XM_EXT_DIO_2);  // PF4\n"
    "  XM_SwitchDioToAdc(XM_EXT_DIO_3);  // PF5\n"
    "  XM_SwitchDioToAdc(XM_EXT_DIO_4);  // PF6\n"
    "  XM_SwitchDioToAdc(XM_EXT_DIO_5);  // PF7  ← NEW\n"
    "  XM_SwitchDioToAdc(XM_EXT_DIO_6);  // PF8  ← NEW",
    va="top", fontsize=7, family="monospace",
    color="#212121", transform=ax2.transAxes,
    bbox=dict(boxstyle="round", fc="#F5F5F5", ec="#BDBDBD", lw=0.8))

# ============================================================
# PANEL 3  (row 1, col 1)  — Variable Conflict Map
# ============================================================
ax3 = fig.add_subplot(gs[1, 1])
blank(ax3)
ax3.text(0.5, 0.98, "(3) Variable Conflict Resolution",
         ha="center", va="top", fontsize=11, fontweight="bold", color=C["head"])

conflicts = [
    # (old name,        appears in,               new name,               prefix_color)
    ("s_cal_state",    "FSR_Fuzzy, EMG, FSR.c",  "s_fsr_cal_state\ns_emg_cal_state",  C["fsr"]),
    ("s_cal_sum[]",    "FSR_Fuzzy, EMG, FSR.c",  "s_fsr_cal_sum[]\ns_emg_cal_sum[]",  C["emg"]),
    ("s_cal_count",    "3 files",                "s_fsr_cal_count\ns_emg_cal_count",  C["fsr"]),
    ("s_raw_v[]",      "FSR_Fuzzy, EMG, FSR.c",  "s_fsr_raw_v[]\ns_emg_raw_v[]",      C["emg"]),
    ("CalState_t",     "3 files",                "FsrCalState_t\nEmgCalState_t",       C["fsr"]),
    ("CH_PF3..PF6",    "EMG, FSR.c",             "FsrCh_t {FSR_LT..RH}\nEmgCh_t {EMG_RH, EMG_LH}", C["emg"]),
    ("_ClampFloat()",  "ALL 5 files",            "1 copy only (static)",               C["gate"]),
    ("_LowPassUpdate()","EMG, FSR.c",            "1 copy only (static)",               C["gate"]),
    ("_UpdateCalibration()","FSR_Fuzzy, EMG, FSR.c","_UpdateFsrCal()\n_UpdateEmgCal()", C["fsr"]),
    ("control_ON",     "ALL 5 files",            "1 global — keep as-is",              C["sys"]),
    ("assist_enable",  "4 files",                "1 global — keep as-is",              C["sys"]),
    ("s_filter_initialized","FSR_Fuzzy, FSR.c",  "s_fsr_filter_ok  (rename)",          C["fsr"]),
]

rh = 0.068
y0 = 0.91
hdr(ax3, 0.0, y0, 0.32, 0.065, "Old Name (conflict)",  "#F44336", 8)
hdr(ax3, 0.32, y0, 0.35, 0.065, "Appears In",           C["head"],  8)
hdr(ax3, 0.67, y0, 0.33, 0.065, "New Name",             "#1B5E20",  8)

for ri, (old, src, new, pc) in enumerate(conflicts):
    ry = y0 - (ri + 1) * rh - 0.003
    bg_r = "#FFEBEE"
    rbox(ax3, 0.0,  ry, 0.32, rh-0.003, bg_r, lw=0.5)
    rbox(ax3, 0.32, ry, 0.35, rh-0.003, C["warn"], lw=0.5)
    rbox(ax3, 0.67, ry, 0.33, rh-0.003, C["take"], lw=0.5)
    ax3.text(0.005, ry + (rh-0.003)/2, old,
             va="center", fontsize=6.5, fontweight="bold",
             color="#B71C1C", transform=ax3.transAxes, family="monospace")
    ax3.text(0.325, ry + (rh-0.003)/2, src,
             va="center", fontsize=6.2, color="#5D4037", transform=ax3.transAxes)
    ax3.text(0.675, ry + (rh-0.003)/2, new,
             va="center", fontsize=6.2, color="#1B5E20",
             transform=ax3.transAxes, family="monospace")

# ============================================================
# PANEL 4  (row 1, col 2)  — Button Assignment
# ============================================================
ax4 = fig.add_subplot(gs[1, 2])
blank(ax4)
ax4.text(0.5, 0.98, "(4) Button Assignment",
         ha="center", va="top", fontsize=11, fontweight="bold", color=C["head"])

btns = [
    ("BTN1", "PRESSED", "FSR zero cal\n(unloaded 1s)",   C["fsr"]),
    ("BTN2", "PRESSED", "FSR full-load cal\n(loaded 1s)", C["fsr"]),
    ("BTN3", "PRESSED", "FSR cal reset",                   C["fsr"]),
    ("BTN1", "CLICK",   "EMG rest cal\n(relax 3s)",        C["emg"]),
    ("BTN2", "CLICK",   "EMG effort cal\n(contract 3s)",   C["emg"]),
    ("BTN3", "CLICK",   "EMG cal reset",                   C["emg"]),
]

y0 = 0.88; rh = 0.11
hdr(ax4, 0.0,  y0, 0.15, 0.09, "Button", C["head"], 8)
hdr(ax4, 0.15, y0, 0.25, 0.09, "Event",  C["head"], 8)
hdr(ax4, 0.40, y0, 0.60, 0.09, "Action", C["head"], 8)

event_col = {"PRESSED": "#E65100", "CLICK": "#1565C0"}
for ri, (btn, ev, action, mc) in enumerate(btns):
    ry = y0 - (ri+1) * rh - 0.005
    rbox(ax4, 0.0,  ry, 0.15, rh-0.005, mc, alpha=0.25, lw=0.5)
    rbox(ax4, 0.15, ry, 0.25, rh-0.005, event_col[ev], alpha=0.18, lw=0.5)
    rbox(ax4, 0.40, ry, 0.60, rh-0.005, mc, alpha=0.12, lw=0.5)
    ax4.text(0.075, ry + (rh-0.005)/2, btn,
             ha="center", va="center", fontsize=8, fontweight="bold",
             color=mc, transform=ax4.transAxes)
    ax4.text(0.275, ry + (rh-0.005)/2, ev,
             ha="center", va="center", fontsize=7.5, fontweight="bold",
             color=event_col[ev], transform=ax4.transAxes)
    ax4.text(0.41, ry + (rh-0.005)/2, action,
             va="center", fontsize=7, color="#212121", transform=ax4.transAxes)

ax4.text(0.5, 0.02,
    "Key fix: XM_GetButtonEvent() called ONCE per loop\n"
    "Result passed as BtnEvents_t struct to both _UpdateFsrCal() and _UpdateEmgCal()\n"
    "→ eliminates double-consume bug from previous code",
    ha="center", va="bottom", fontsize=7,
    color="#37474F", transform=ax4.transAxes,
    bbox=dict(boxstyle="round", fc=C["warn"], ec="#F9A825", lw=0.8))

# ============================================================
# PANEL 5  (row 2, col 0-2 span)  — User_Loop Pipeline
# ============================================================
ax5 = fig.add_subplot(gs[2, :])
blank(ax5)
ax5.text(0.5, 0.98, "(5) User_Loop()  —  Execution Pipeline  (1 ms / 1 kHz)",
         ha="center", va="top", fontsize=12, fontweight="bold", color=C["head"])

steps = [
    # (step#, label,                   detail,                                          color,      x_norm)
    ("(1)",  "XM_IO_Update()",          "button debounce\ntimer tick",                   C["sys"],   0.02),
    ("(2)",  "_ReadButtons()",          "btn1/2/3 event\nread ONCE → struct",            C["sys"],   0.13),
    ("(3)",  "Assist\nMode?",           "is_assist_mode\ncheck",                         C["gate"],  0.24),
    ("(4)",  "_SampleFsr()",           "PF3-6 ADC\n+ 8Hz LPF",                          C["fsr"],   0.34),
    ("(5)",  "_UpdateFsrCal()",        "BTN PRESSED\nFSR cal FSM",                       C["fsr"],   0.44),
    ("(6)",  "_UpdateLoads()",         "normalize\n(lpf-off)/(on-off)",                  C["fsr"],   0.54),
    ("(7)",  "_SampleEmg()",           "PF7-8 ADC\nraw voltage",                         C["emg"],   0.64),
    ("(8)",  "_ProcessEmgSignals()",   "80Hz LPF→rectify\n→5Hz envelope",               C["emg"],   0.74),
    ("(9)",  "_UpdateEmgCal()",        "BTN CLICK\nEMG cal FSM",                         C["emg"],   0.84),
]

steps2 = [
    ("(10)",  "_SampleAngle()",         "leftHipAngle\n+ velocity LPF",                  C["comp"],  0.02),
    ("(11)",  "_GetEmgNorm()",          "deadband→normalize\n→ 0~1 per channel",          C["emg"],   0.13),
    ("(12)",  "_UpdateSquatFsm()",      "5-phase FSM\nangle+EMG→torque",                  C["fsm"],   0.24),
    ("(13)",  "_UpdateCompensation()",  "gravity+friction\nfeedforward",                   C["comp"],  0.34),
    ("(14)",  "_ApplyTorque()",         "safety gate\n+ XM_SetAssist*()",                 C["gate"],  0.44),
    ("(15)",  "_UpdatePublicSignals()","Live Expressions\nvars update",                    C["sys"],   0.54),
    ("(16)",  "_SendStream()",          "CDC macro\n→ USB 100Hz",                          C["cdc"],   0.64),
]

bw = 0.093; bh = 0.32; by = 0.57

for step_row, sy in [(steps, 0.62), (steps2, 0.24)]:
    for (num, name, detail, color, xpos) in step_row:
        rbox(ax5, xpos, sy - bh/2, bw, bh, color, alpha=0.85)
        ax5.text(xpos + bw/2, sy - bh/2 + bh*0.78, num,
                 ha="center", va="center", fontsize=9, fontweight="bold",
                 color="white", transform=ax5.transAxes)
        ax5.text(xpos + bw/2, sy - bh/2 + bh*0.53, name,
                 ha="center", va="center", fontsize=7.5, fontweight="bold",
                 color="white", transform=ax5.transAxes)
        ax5.text(xpos + bw/2, sy - bh/2 + bh*0.22, detail,
                 ha="center", va="center", fontsize=6.2,
                 color="#F5F5F5", transform=ax5.transAxes)
        # arrow
        if xpos + bw + 0.005 < 0.95:
            ax5.annotate("", xy=(xpos + bw + 0.005, sy),
                         xytext=(xpos + bw, sy),
                         xycoords="axes fraction",
                         arrowprops=dict(arrowstyle="->", color="#555", lw=1.2))

# row label
ax5.text(0.965, 0.62, "Row 1", ha="left", va="center", fontsize=8,
         color=C["head"], fontweight="bold", transform=ax5.transAxes)
ax5.text(0.965, 0.24, "Row 2", ha="left", va="center", fontsize=8,
         color=C["head"], fontweight="bold", transform=ax5.transAxes)

# divider
ax5.axhline(0.44, color="#BDBDBD", lw=1.0, linestyle="--")
ax5.text(0.5, 0.44, "  ↓ continues  ", ha="center", va="center",
         fontsize=8, color="#757575", transform=ax5.transAxes,
         bbox=dict(fc="white", ec="none"))

# source tags
src_map = {
    0.02: "Encoder_ex01", 0.13: "NEW", 0.24: "FSR_Fuzzy",
    0.34: "FSR_Fuzzy", 0.44: "FSR_Fuzzy", 0.54: "FSR_Fuzzy",
    0.64: "EMG", 0.74: "EMG", 0.84: "EMG"
}
src_map2 = {
    0.02: "Comp_AddOn", 0.13: "EMG (mod)", 0.24: "NEW",
    0.34: "Comp_AddOn", 0.44: "FSR_Fuzzy", 0.54: "ALL",
    0.64: "FSR_Fuzzy"
}
for step_row, sy, smap in [(steps, 0.62, src_map), (steps2, 0.24, src_map2)]:
    for (num, name, detail, color, xpos) in step_row:
        src = smap.get(xpos, "")
        ax5.text(xpos + bw/2, sy - bh/2 - 0.03, src,
                 ha="center", va="top", fontsize=5.5,
                 color="#616161", style="italic", transform=ax5.transAxes)

# ============================================================
# PANEL 6  (row 3, col 0)  — Squat FSM
# ============================================================
ax6 = fig.add_subplot(gs[3, 0])
blank(ax6)
ax6.text(0.5, 0.98, "(6) Squat FSM — 5 Phases",
         ha="center", va="top", fontsize=11, fontweight="bold", color=C["head"])

phases = [
    ("STAND\n(0)",      C["fsr"],   0.50, 0.82),
    ("DESCENDING\n(1)", C["emg"],   0.15, 0.57),
    ("BOTTOM\n(2)",     C["gate"],  0.50, 0.42),
    ("ASCENDING\n(3)",  "#2196F3",  0.85, 0.57),
    ("RETURN\n(4)",     C["comp"],  0.50, 0.12),
]

pw = 0.18; ph_h = 0.13
for (name, col, px, py) in phases:
    rbox(ax6, px - pw/2, py - ph_h/2, pw, ph_h, col, alpha=0.88)
    ax6.text(px, py, name, ha="center", va="center",
             fontsize=8, fontweight="bold", color="white", transform=ax6.transAxes)

arrows_fsm = [
    # (from_xy, to_xy, label, color)
    ((0.50, 0.755), (0.24, 0.625),
     "EMG>down_thr\nAND angle>θ_stand\n(debounce 150ms)", C["emg"]),
    ((0.24, 0.505), (0.41, 0.425),
     "angle < θ_sit", C["gate"]),
    ((0.59, 0.42),  (0.76, 0.505),
     "EMG > up_thr", "#2196F3"),
    ((0.85, 0.625), (0.62, 0.765),
     "angle > θ_stand", C["fsr"]),
    ((0.50, 0.355), (0.50, 0.185),
     "angle > θ_stand", C["comp"]),
    ((0.50, 0.055), (0.50, 0.755),
     "EMG < 0.3×down_thr\n(hysteresis)", "#9E9E9E"),
    # emergency reset
    ((0.24, 0.57),  (0.44, 0.82),
     "EMERGENCY:\nangle>θ_stand", C["gate"]),
]

for (fx, fy), (tx, ty), lbl, col in arrows_fsm:
    ax6.annotate("", xy=(tx, ty), xytext=(fx, fy),
                 xycoords="axes fraction",
                 arrowprops=dict(arrowstyle="->", color=col, lw=1.3,
                                 connectionstyle="arc3,rad=0.15"))
    mx, my = (fx+tx)/2, (fy+ty)/2
    ax6.text(mx, my, lbl, ha="center", va="center",
             fontsize=5.5, color=col, transform=ax6.transAxes,
             bbox=dict(fc="white", ec=col, lw=0.5, boxstyle="round,pad=0.1", alpha=0.85))

# torque annotations
torque_ann = [
    (0.15, 0.38, "τ = gain_res × emg_norm\n(+Nm, resist)"),
    (0.50, 0.28, "τ = max_res_nm\n(fixed +Nm)"),
    (0.85, 0.38, "τ = -(gain_assist × emg_norm)\n(-Nm, assist)"),
    (0.50, 0.01, "τ = -(gain_assist × emg_norm × 0.3)\n(fade-out)"),
]
for (tx, ty, txt) in torque_ann:
    ax6.text(tx, ty, txt, ha="center", va="bottom", fontsize=5.8,
             color="#E65100", transform=ax6.transAxes,
             bbox=dict(fc="#FFF3E0", ec="#FF9800", lw=0.5,
                       boxstyle="round,pad=0.15", alpha=0.9))

# ============================================================
# PANEL 7  (row 3, col 1)  — EMG Pipeline
# ============================================================
ax7 = fig.add_subplot(gs[3, 1])
blank(ax7)
ax7.text(0.5, 0.98, "(7) EMG Signal Pipeline  (per channel)",
         ha="center", va="top", fontsize=11, fontweight="bold", color=C["head"])

pipeline = [
    ("Raw ADC\ns_emg_raw_v[ch]",     "XM_AnalogReadMillivolts\n(PF7/PF8, 0~3.3V)",       "#CFD8DC"),
    ("Bias removal\ncentered_v",      "centered = raw - s_emg_bias_v[ch]\n(default 1.65V)", "#B3E5FC"),
    ("80 Hz LPF\npre_lpf_v",         "_LowPassUpdate(prev, centered, 80Hz)\nalpha=dt/(RC+dt)", "#B3E5FC"),
    ("Rectify\nrect_v",               "rect = |pre_lpf_v|\n(_AbsFloat)",                   "#B2EBF2"),
    ("5 Hz envelope\nenv_v",          "_LowPassUpdate(prev, rect, 5Hz)\nsmooth activation", "#B2DFDB"),
    ("Deadband\nactive_v",            "active = env - deadband\nif active<=0: return 0",   "#DCEDC8"),
    ("Normalise\nemg_norm [0~1]",     "norm = active / (full_scale - deadband)\nclamp 0~1", "#C8E6C9"),
]

y0 = 0.88; rh = 0.115
for ri, (name, detail, bg) in enumerate(pipeline):
    ry = y0 - ri * (rh + 0.005)
    rbox(ax7, 0.01, ry, 0.98, rh, bg, lw=0.7, ec=C["emg"])
    ax7.text(0.28, ry + rh/2, name,
             va="center", ha="center", fontsize=7.5, fontweight="bold",
             color="#0D47A1", transform=ax7.transAxes)
    ax7.text(0.55, ry + rh/2, detail,
             va="center", fontsize=6.5, color="#212121",
             transform=ax7.transAxes, family="monospace")
    if ri < len(pipeline) - 1:
        ax7.annotate("", xy=(0.5, ry - 0.003),
                     xytext=(0.5, ry),
                     xycoords="axes fraction",
                     arrowprops=dict(arrowstyle="->", color=C["emg"], lw=1.1))

# cal note
ax7.text(0.5, 0.005,
    "Calibration:  BTN1 CLICK → rest 3s → s_emg_bias_v[] = mean(raw)\n"
    "              BTN2 CLICK → effort 3s → s_emg_full_scale_v[] = peak_env\n"
    "              BTN3 CLICK → reset to defaults (1.65V bias, 1.0V full-scale)",
    ha="center", va="bottom", fontsize=6.5,
    transform=ax7.transAxes,
    bbox=dict(boxstyle="round", fc=C["warn"], ec="#F9A825", lw=0.8))

# ============================================================
# PANEL 8  (row 3, col 2)  — Key Public Variables
# ============================================================
ax8 = fig.add_subplot(gs[3, 2])
blank(ax8)
ax8.text(0.5, 0.98, "(8) Key Public Variables  (Live Expressions)",
         ha="center", va="top", fontsize=11, fontweight="bold", color=C["head"])

pub_vars = [
    # (group, color, vars_list)
    ("FSR", C["fsr"], [
        "float  pf3_volt .. pf6_volt      // raw ADC volts",
        "float  fsr_lt/lh/rt/rh_load     // 0~1.5 normalised",
        "uint16 fsr_cal_ready             // 1 = both steps done",
    ]),
    ("EMG", C["emg"], [
        "float  emg_rh_raw_v, emg_lh_raw_v  // ADC volts",
        "float  emg_rh_env_v, emg_lh_env_v  // envelope (V)",
        "float  emg_rh_norm,  emg_lh_norm   // 0~1",
        "uint16 emg_cal_done                 // 1 = cal done",
    ]),
    ("Squat FSM", C["fsm"], [
        "float    squat_theta_stand_deg  =  5.0",
        "float    squat_theta_sit_deg    = -40.0",
        "float    squat_emg_down_thr     =  0.30",
        "float    squat_emg_up_thr       =  0.50",
        "float    squat_gain_resistance  =  0.20",
        "float    squat_gain_assist      =  0.30",
        "float    squat_max_res_nm       =  0.30",
        "uint16   squat_entry_debounce_ms = 150",
        "uint16   squat_phase            // 0~4 (monitor)",
    ]),
    ("Compensation", C["comp"], [
        "float  gravity_mgl_nm      =  1.0",
        "float  compensation_scale  =  0.20",
        "uint16 compensation_ON     =  0     // toggle",
        "float  comp_torque_rh/lh_nm        // feedforward",
        "float  hip_velocity_rads           // filtered",
    ]),
    ("Control / Output", C["gate"], [
        "uint16 control_ON          =  0   // set 1 to enable",
        "float  assist_torque_limit_nm = 1.0",
        "float  left/right_assist_torque_nm // final cmd",
        "uint16 assist_enable              // 1 = active",
        "uint16 assist_mode_active         // H10 ASSIST?",
    ]),
    ("CDC Stream", C["cdc"], [
        "uint16 cdc_stream_enable    =  1",
        "uint16 cdc_stream_period_ms = 10  // 100 Hz",
        "// Channels: angle, emg_norm×2, fsr_load×4,",
        "// squat_phase, torque×2, comp×2, flags×3",
    ]),
]

y = 0.91; rh_g = 0.02; rh_v = 0.048
for group, gc, vlist in pub_vars:
    rbox(ax8, 0.0, y - rh_g, 1.0, rh_g, gc, alpha=0.90)
    ax8.text(0.01, y - rh_g/2, group,
             va="center", fontsize=7.5, fontweight="bold",
             color="white", transform=ax8.transAxes)
    y -= rh_g
    for v in vlist:
        rbox(ax8, 0.0, y - rh_v, 1.0, rh_v - 0.003, "#FAFAFA", lw=0.4, ec="#E0E0E0")
        ax8.text(0.012, y - rh_v/2, v,
                 va="center", fontsize=6.0, color="#212121",
                 transform=ax8.transAxes, family="monospace")
        y -= rh_v
    y -= 0.005

# ============================================================
# PANEL 9  (row 4, col 0-2 span)  — CDC Channels + Safety Gate
# ============================================================
ax9 = fig.add_subplot(gs[4, :])
blank(ax9)
ax9.text(0.5, 0.98, "(9) CDC Stream Channels  +  Safety Gate Logic",
         ha="center", va="top", fontsize=12, fontweight="bold", color=C["head"])

# CDC channels table (left half)
cdc_chans = [
    ("1",  "Hip Angle",    "deg",  "XM.status.h10.leftHipAngle",      C["comp"]),
    ("2",  "EMG RH Norm",  "-",    "emg_rh_norm",                      C["emg"]),
    ("3",  "EMG LH Norm",  "-",    "emg_lh_norm",                      C["emg"]),
    ("4",  "FSR LT",       "-",    "fsr_lt_load",                      C["fsr"]),
    ("5",  "FSR LH",       "-",    "fsr_lh_load",                      C["fsr"]),
    ("6",  "FSR RT",       "-",    "fsr_rt_load",                      C["fsr"]),
    ("7",  "FSR RH",       "-",    "fsr_rh_load",                      C["fsr"]),
    ("8",  "Squat Phase",  "id",   "squat_phase",                      C["fsm"]),
    ("9",  "Cmd Torque RH","Nm",   "right_assist_torque_nm",           C["gate"]),
    ("10", "Cmd Torque LH","Nm",   "left_assist_torque_nm",            C["gate"]),
    ("11", "Comp RH",      "Nm",   "comp_torque_rh_nm",                C["comp"]),
    ("12", "Comp LH",      "Nm",   "comp_torque_lh_nm",                C["comp"]),
    ("13", "Cal Ready",    "bool", "fsr_cal_ready",                    C["fsr"]),
    ("14", "Control ON",   "bool", "control_ON",                       C["sys"]),
]

rh = 0.055; y0 = 0.91
hdr(ax9, 0.0,  y0, 0.04, 0.07, "ch", C["head"], 8)
hdr(ax9, 0.04, y0, 0.12, 0.07, "Name", C["head"], 8)
hdr(ax9, 0.16, y0, 0.07, 0.07, "Unit", C["head"], 8)
hdr(ax9, 0.23, y0, 0.27, 0.07, "Source Variable", C["head"], 8)
for ri, (ch, name, unit, src, cc) in enumerate(cdc_chans):
    ry = y0 - (ri+1)*rh - 0.01
    for xp, wp, txt in [(0.0,0.04,ch),(0.04,0.12,name),(0.16,0.07,unit),(0.23,0.27,src)]:
        bg = "#E3F2FD" if ri%2==0 else "#FAFAFA"
        rbox(ax9, xp, ry, wp, rh-0.004, bg, lw=0.4)
        ax9.text(xp+0.005, ry+(rh-0.004)/2, txt,
                 va="center", fontsize=6.5, color="#212121",
                 transform=ax9.transAxes,
                 family="monospace" if xp==0.23 else "sans-serif")
    rbox(ax9, 0.50, ry, 0.004, rh-0.004, cc, alpha=0.85, lw=0)  # color bar

# Safety gate (right half)
ax9.text(0.55, 0.95, "Safety Gate  (_ApplyTorque)", fontsize=9,
         fontweight="bold", color=C["gate"], transform=ax9.transAxes)

gate_code = (
    "torque_limit = clamp(assist_torque_limit_nm, 0, HARD_MAX=2.5 Nm)\n\n"
    "assist_ok = (control_ON == 1)\n"
    "         AND (fsr_cal_ready == 1)          // FSR zero + full-load done\n"
    "         AND (emg_cal_done == 1)           // EMG rest + effort done\n"
    "         AND (h10Mode == ASSIST)\n\n"
    "if assist_ok:\n"
    "    if NOT torque_mode_active:\n"
    "        XM_SetH10AssistExistingMode(false)\n"
    "        XM_SetControlMode(XM_CTRL_TORQUE)\n"
    "        torque_mode_active = true\n"
    "    assist_enable = 1\n"
    "    rh = clamp(squat_torque_rh + comp_torque_rh, -limit, +limit)\n"
    "    lh = clamp(squat_torque_lh + comp_torque_lh, -limit, +limit)\n"
    "    XM_SetAssistTorqueRH(rh)\n"
    "    XM_SetAssistTorqueLH(lh)\n"
    "else:\n"
    "    assist_enable = 0\n"
    "    XM_SetControlMode(XM_CTRL_MONITOR)  // if was active\n"
    "    XM_SetAssistTorqueRH/LH(0.0f)"
)
ax9.text(0.55, 0.90, gate_code,
         va="top", fontsize=6.5, family="monospace",
         color="#212121", transform=ax9.transAxes,
         bbox=dict(boxstyle="round", fc="#FFEBEE", ec=C["gate"], lw=1.0))

plt.savefig("squat_arch_viz.png", dpi=140, bbox_inches="tight",
            facecolor=C["bg"])
print("Saved: squat_arch_viz.png")
