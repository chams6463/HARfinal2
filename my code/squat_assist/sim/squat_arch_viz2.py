"""
Squat EMG Assist - Architecture Visualization (Multi-Page)
Saves: squat_arch_p1.png ~ squat_arch_p6.png
"""

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
from matplotlib.patches import FancyBboxPatch
from matplotlib.gridspec import GridSpec

C = {
    "fsr":   "#4CAF50",
    "emg":   "#2196F3",
    "fsm":   "#FF9800",
    "comp":  "#9C27B0",
    "gate":  "#F44336",
    "cdc":   "#607D8B",
    "sys":   "#795548",
    "pin":   "#009688",
    "bg":    "#FAFAFA",
    "take":  "#E8F5E9",
    "skip":  "#FFEBEE",
    "warn":  "#FFF9C4",
    "head":  "#37474F",
}

def new_fig(title, figsize=(20, 13)):
    fig, ax = plt.subplots(figsize=figsize, facecolor=C["bg"])
    ax.set_xlim(0, 1)
    ax.set_ylim(0, 1)
    ax.axis("off")
    fig.suptitle(title, fontsize=16, fontweight="bold",
                 color=C["head"], y=0.97)
    return fig, ax

def rbox(ax, x, y, w, h, color, alpha=0.85, lw=1.2, ec=None):
    ec = ec or color
    p = FancyBboxPatch((x, y), w, h,
                       boxstyle="round,pad=0.012",
                       facecolor=color, edgecolor=ec,
                       linewidth=lw, alpha=alpha,
                       transform=ax.transAxes, clip_on=False)
    ax.add_patch(p)

def hdr(ax, x, y, w, h, text, color, fontsize=11):
    rbox(ax, x, y, w, h, color, alpha=0.92)
    ax.text(x + w/2, y + h/2, text, ha="center", va="center",
            fontsize=fontsize, fontweight="bold", color="white",
            transform=ax.transAxes)

def save(fig, fname):
    fig.savefig(fname, dpi=150, bbox_inches="tight", facecolor=C["bg"])
    plt.close(fig)
    print(f"Saved: {fname}")

# =============================================================
# PAGE 1  —  Source File Function Map
# =============================================================
fig1, ax1 = new_fig("Page 1 / 6   Source Files  ->  Functions to Extract", figsize=(24, 18))

files = [
    ("Final_FSR_Fuzzy_Logic.c", C["fsr"], [
        ("[+] _SampleFsr()",           "ADC read (PF3-6) + 8Hz LPF",                          True),
        ("[+] _UpdateCalibration()",   "FSR cal FSM   BTN1/2 PRESSED -> zero / full-load",    True),
        ("[+] _StartCalibration()",    "accumulate 1s avg -> s_fsr_off_v / s_fsr_on_v",       True),
        ("[+] _ResetCalibration()",    "restore defaults, clear zero/fullload flags",           True),
        ("[+] _UpdateLoads()",         "normalize: (lpf-off)/(on-off), clamp 0~1.5",          True),
        ("[+] CDC macro system",       "CDC_STREAM_CHANNELS macro -> struct+meta+_SendStream", True),
        ("[+] _ClampFloat()",          "utility  (keep ONE copy, discard all duplicates)",      True),
        ("[-] _UpdateFootDetector()",  "fuzzy gait phase detect -- NOT needed for squat",      False),
        ("[-] _MembershipLarge()",     "tanh membership -- NOT needed for squat",               False),
        ("[-] push-off pulse vars",    "s_left/right_push_off_remaining_ms -- NOT needed",     False),
        ("[-] _UpdateAssistTorque()",  "gait-pulse torque output -- replaced by squat FSM",    False),
    ]),
    ("Final_EMG.c", C["emg"], [
        ("[+] _SampleAdcChannels()",   "read PF7/PF8 (XM_EXT_ADC_9/10) -> s_emg_raw_v[2]",  True),
        ("[+] _ProcessEmgSignals()",   "bias -> 80Hz LPF -> rectify -> 5Hz envelope",         True),
        ("[+] _LowPassUpdate()",       "IIR LPF: prev + alpha*(input-prev)",                   True),
        ("[+] _AbsFloat()",            "full-wave rectification helper",                        True),
        ("[+] _StartRestCalibration()","BTN1 CLICK -> rest 3s -> save bias",                   True),
        ("[+] _StartEffortCalibration()","BTN2 CLICK -> effort 3s -> save full-scale",         True),
        ("[+] _ResetCalibrationDefaults()","BTN3 CLICK -> restore EMG defaults",               True),
        ("[+] _EnvelopeToTorque() MOD","adapt: return 0~1 norm instead of raw Nm",             True),
        ("[-] TSM (Off/Standby/Active)","state machine -- unnecessary complexity",              False),
        ("[-] _SelectTorquePair()",    "pair routing -- squat uses both channels directly",    False),
        ("[-] _UpdateUsbDebug()",      "500ms printf debug -- replaced by CDC macro",          False),
    ]),
    ("Final_Compensation_AddOn.c", C["comp"], [
        ("[+] _UpdateCompensation()",  "gravity=mgl*sin(theta), friction=coulomb+viscous",     True),
        ("[+] angle+velocity read",    "leftHipAngle + differentiator + LPF -> vel_rads",      True),
        ("[+] _SignFloat()",           "sign(x) in {-1,0,+1}  (utility)",                      True),
        ("[-] BTN toggle pattern",     "BTN1/2 toggle control_ON -- replaced by our scheme",   False),
    ]),
    ("Final_Encoder_ex01.c", C["sys"], [
        ("[+] _UpdateButtons() pattern","read all 3 btns ONCE per loop -> local struct",       True),
        ("[+] s_assist_session_active", "first-entry init pattern on ASSIST mode entry",       True),
        ("[+] mode-exit safety flush",  "zero torque on ASSIST->STANDBY transition",           True),
        ("[-] pulse trigger/rearm",     "angle-threshold pulse -- replaced by squat FSM",      False),
    ]),
    ("Final_FSR.c", C["pin"], [
        ("[+] contact hysteresis vals", "FSR_ON_THRESHOLD=0.35  FSR_OFF_THRESHOLD=0.20",       True),
        ("[+] _LowPassUpdate() formula","same as EMG -- merge into ONE shared copy",            True),
        ("[-] _LoadToTorque()",         "FSR->Nm direct -- squat torque comes from EMG",       False),
        ("[-] TSM / Active_Loop()",     "TSM structure not used in new file",                   False),
    ]),
]

n = len(files)
col_w = 1.0 / n
row_h = 0.055
top_y = 0.91

for fi, (fname, fc, items) in enumerate(files):
    fx = fi * col_w + 0.004
    fw = col_w - 0.008
    hdr(ax1, fx, top_y, fw, 0.065, fname, fc, fontsize=9)
    for ri, (func, desc, keep) in enumerate(items):
        bg = C["take"] if keep else C["skip"]
        ry = top_y - (ri + 1) * row_h - 0.004
        rbox(ax1, fx, ry, fw, row_h - 0.003, bg, lw=0.5)
        col_t = "#1B5E20" if keep else "#B71C1C"
        ax1.text(fx + 0.006, ry + (row_h - 0.003) * 0.68,
                 func, va="center", fontsize=7.2,
                 fontweight="bold" if keep else "normal",
                 color=col_t, transform=ax1.transAxes)
        ax1.text(fx + 0.006, ry + (row_h - 0.003) * 0.25,
                 "  " + desc, va="center", fontsize=6.2,
                 color="#424242", transform=ax1.transAxes)

leg = [mpatches.Patch(fc=C["take"], ec=C["fsr"], label="[+] Use / adapt"),
       mpatches.Patch(fc=C["skip"], ec=C["gate"], label="[-] Discard")]
ax1.legend(handles=leg, loc="lower right", fontsize=10, framealpha=0.9)
save(fig1, "squat_arch_p1.png")


# =============================================================
# PAGE 2  —  Pin Map + Variable Conflict Table + Button Map
# =============================================================
fig2 = plt.figure(figsize=(22, 16), facecolor=C["bg"])
fig2.suptitle("Page 2 / 6   Pin Assignment  |  Variable Conflicts  |  Button Map",
              fontsize=15, fontweight="bold", color=C["head"], y=0.97)
gs2 = GridSpec(1, 3, figure=fig2, left=0.03, right=0.97,
               top=0.91, bottom=0.03, wspace=0.06)

# ---- 2A: Pin table ----
ax2a = fig2.add_subplot(gs2[0, 0])
ax2a.set_xlim(0,1); ax2a.set_ylim(0,1); ax2a.axis("off")
ax2a.text(0.5, 0.97, "Pin Assignment  (PF3 - PF8)",
          ha="center", va="top", fontsize=12, fontweight="bold", color=C["head"])

pin_rows = [
    ("PF3", "XM_EXT_ADC_5",  "XM_EXT_DIO_1", "FSR Left Toe",   C["fsr"]),
    ("PF4", "XM_EXT_ADC_6",  "XM_EXT_DIO_2", "FSR Left Heel",  C["fsr"]),
    ("PF5", "XM_EXT_ADC_7",  "XM_EXT_DIO_3", "FSR Right Toe",  C["fsr"]),
    ("PF6", "XM_EXT_ADC_8",  "XM_EXT_DIO_4", "FSR Right Heel", C["fsr"]),
    ("PF7", "XM_EXT_ADC_9",  "XM_EXT_DIO_5", "EMG Right Hip",  C["emg"]),
    ("PF8", "XM_EXT_ADC_10", "XM_EXT_DIO_6", "EMG Left Hip",   C["emg"]),
]
col_hdrs = ["Pin", "ADC Pin", "DIO Switch", "Role"]
col_xs   = [0.0, 0.14, 0.38, 0.62]
col_ws   = [0.13, 0.23, 0.23, 0.37]
rh = 0.075
y0 = 0.88
for ci, (ch, cw) in enumerate(zip(col_hdrs, col_ws)):
    hdr(ax2a, col_xs[ci], y0, cw-0.01, rh, ch, C["head"], 9)
for ri, (pin, adc, dio, role, rc) in enumerate(pin_rows):
    ry = y0 - (ri+1)*(rh+0.008)
    bg = "#E8F5E9" if rc == C["fsr"] else "#E3F2FD"
    for ci, (val, cw) in enumerate(zip([pin, adc, dio, role], col_ws)):
        rbox(ax2a, col_xs[ci], ry, cw-0.01, rh, bg, lw=0.5,
             ec=rc if ci==3 else "#E0E0E0")
        ax2a.text(col_xs[ci] + (cw-0.01)/2, ry + rh/2, val,
                  ha="center", va="center", fontsize=8.5,
                  fontweight="bold" if ci in (0,3) else "normal",
                  color=rc if ci==3 else "#212121",
                  transform=ax2a.transAxes)

ax2a.text(0.5, 0.22,
    "User_Setup() — pin switch:\n\n"
    "XM_SwitchDioToAdc(XM_EXT_DIO_1);  // PF3\n"
    "XM_SwitchDioToAdc(XM_EXT_DIO_2);  // PF4\n"
    "XM_SwitchDioToAdc(XM_EXT_DIO_3);  // PF5\n"
    "XM_SwitchDioToAdc(XM_EXT_DIO_4);  // PF6\n"
    "XM_SwitchDioToAdc(XM_EXT_DIO_5);  // PF7  NEW\n"
    "XM_SwitchDioToAdc(XM_EXT_DIO_6);  // PF8  NEW",
    ha="center", va="center", fontsize=8.5, family="monospace",
    transform=ax2a.transAxes,
    bbox=dict(boxstyle="round", fc="#F5F5F5", ec="#9E9E9E", lw=1))

# ---- 2B: Variable conflict table ----
ax2b = fig2.add_subplot(gs2[0, 1])
ax2b.set_xlim(0,1); ax2b.set_ylim(0,1); ax2b.axis("off")
ax2b.text(0.5, 0.97, "Variable Conflict Resolution",
          ha="center", va="top", fontsize=12, fontweight="bold", color=C["head"])

conflicts = [
    ("s_cal_state",     "FSR_Fuzzy\nEMG  FSR.c",  "s_fsr_cal_state\ns_emg_cal_state"),
    ("s_cal_sum[]",     "FSR_Fuzzy\nEMG  FSR.c",  "s_fsr_cal_sum[]\ns_emg_cal_sum[]"),
    ("s_cal_count",     "FSR_Fuzzy\nEMG  FSR.c",  "s_fsr_cal_count\ns_emg_cal_count"),
    ("s_cal_tick",      "FSR_Fuzzy\nEMG  FSR.c",  "s_fsr_cal_tick\ns_emg_cal_tick"),
    ("s_raw_v[]",       "FSR_Fuzzy\nEMG  FSR.c",  "s_fsr_raw_v[4]\ns_emg_raw_v[2]"),
    ("s_lpf_v[]",       "FSR_Fuzzy\nFSR.c",       "s_fsr_lpf_v[4]\n(EMG has pre_lpf_v)"),
    ("CalState_t",      "FSR_Fuzzy\nEMG  FSR.c",  "FsrCalState_t\nEmgCalState_t"),
    ("AdcChIdx_t\nCH_PF3..6", "EMG  FSR.c",       "FsrCh_t {FSR_LT,LH,RT,RH}\nEmgCh_t {EMG_RH, EMG_LH}"),
    ("_ClampFloat()",   "ALL 5 files",             "1 static copy\n(delete 4 duplicates)"),
    ("_LowPassUpdate()","EMG  FSR.c",              "1 static copy\n(delete 1 duplicate)"),
    ("_UpdateCalibration()", "FSR_Fuzzy\nEMG FSR.c", "_UpdateFsrCal(BtnEvents_t*)\n_UpdateEmgCal(BtnEvents_t*)"),
    ("control_ON",      "ALL 5 files",             "1 global uint16_t\n(keep as-is)"),
    ("assist_enable",   "4 files",                 "1 global uint16_t\n(keep as-is)"),
    ("s_filter_initialized","FSR_Fuzzy FSR.c",     "s_fsr_filter_ok\n(rename to avoid clash)"),
]

hdr(ax2b, 0.0,  0.90, 0.32, 0.06, "Conflict Name",  "#F44336", 9)
hdr(ax2b, 0.33, 0.90, 0.25, 0.06, "Appears In",     C["head"],  9)
hdr(ax2b, 0.59, 0.90, 0.41, 0.06, "Resolved As",    "#1B5E20",  9)
rh = 0.058
for ri, (old, src, new) in enumerate(conflicts):
    ry = 0.89 - (ri+1)*(rh+0.002)
    bg_alt = "#FFEBEE" if ri%2==0 else "#FFF8F8"
    rbox(ax2b, 0.0,  ry, 0.32, rh, bg_alt,    lw=0.4)
    rbox(ax2b, 0.33, ry, 0.25, rh, C["warn"], lw=0.4)
    rbox(ax2b, 0.59, ry, 0.41, rh, C["take"], lw=0.4)
    ax2b.text(0.005, ry+rh/2, old, va="center", fontsize=7.5,
              fontweight="bold", color="#B71C1C",
              family="monospace", transform=ax2b.transAxes)
    ax2b.text(0.335, ry+rh/2, src, va="center", fontsize=7,
              color="#5D4037", transform=ax2b.transAxes)
    ax2b.text(0.595, ry+rh/2, new, va="center", fontsize=7,
              color="#1B5E20", family="monospace",
              transform=ax2b.transAxes)

# ---- 2C: Button map ----
ax2c = fig2.add_subplot(gs2[0, 2])
ax2c.set_xlim(0,1); ax2c.set_ylim(0,1); ax2c.axis("off")
ax2c.text(0.5, 0.97, "Button Assignment",
          ha="center", va="top", fontsize=12, fontweight="bold", color=C["head"])

btns = [
    ("BTN1", "PRESSED", C["fsr"],  "#E65100",
     "FSR zero calibration",
     "unloaded 1 s  ->  s_fsr_off_v[] = avg(lpf)"),
    ("BTN2", "PRESSED", C["fsr"],  "#E65100",
     "FSR full-load calibration",
     "full weight 1 s  ->  s_fsr_on_v[] = avg(lpf)"),
    ("BTN3", "PRESSED", C["fsr"],  "#E65100",
     "FSR calibration reset",
     "restore off=0V  on=1V  clear fsr_cal_ready"),
    ("BTN1", "CLICK",   C["emg"],  "#1565C0",
     "EMG rest calibration",
     "relax 3 s  ->  s_emg_bias_v[] = mean(raw)"),
    ("BTN2", "CLICK",   C["emg"],  "#1565C0",
     "EMG effort calibration",
     "contract 3 s  ->  s_emg_full_scale_v[] = peak(env)"),
    ("BTN3", "CLICK",   C["emg"],  "#1565C0",
     "EMG calibration reset",
     "restore bias=1.65V  full_scale=1.0V"),
]

y0 = 0.90; rh = 0.108
hdr(ax2c, 0.0,  y0, 0.12, 0.065, "Btn",    C["head"], 9)
hdr(ax2c, 0.13, y0, 0.17, 0.065, "Event",  C["head"], 9)
hdr(ax2c, 0.31, y0, 0.69, 0.065, "Action + Detail", C["head"], 9)

for ri, (btn, ev, mc, ec, action, detail) in enumerate(btns):
    ry = y0 - (ri+1)*(rh+0.005)
    rbox(ax2c, 0.0,  ry, 0.12, rh, mc, alpha=0.25, lw=0.5)
    rbox(ax2c, 0.13, ry, 0.17, rh, ec, alpha=0.18, lw=0.5)
    rbox(ax2c, 0.31, ry, 0.69, rh, mc, alpha=0.10, lw=0.5)
    ax2c.text(0.06,  ry+rh/2, btn, ha="center", va="center",
              fontsize=10, fontweight="bold", color=mc, transform=ax2c.transAxes)
    ax2c.text(0.215, ry+rh/2, ev, ha="center", va="center",
              fontsize=8.5, fontweight="bold", color=ec, transform=ax2c.transAxes)
    ax2c.text(0.32,  ry+rh*0.70, action, va="center",
              fontsize=8.5, fontweight="bold", color="#212121", transform=ax2c.transAxes)
    ax2c.text(0.32,  ry+rh*0.28, detail, va="center",
              fontsize=7.5, color="#424242", transform=ax2c.transAxes)

rbox(ax2c, 0.0, 0.01, 1.0, 0.10, C["warn"], lw=1, ec="#F9A825")
ax2c.text(0.5, 0.06,
    "KEY FIX: XM_GetButtonEvent() called ONCE at top of User_Loop()\n"
    "-> stored in BtnEvents_t struct -> passed to both _UpdateFsrCal() and _UpdateEmgCal()\n"
    "-> eliminates the double-consume bug from the previous merged code",
    ha="center", va="center", fontsize=8, color="#37474F",
    transform=ax2c.transAxes)

save(fig2, "squat_arch_p2.png")


# =============================================================
# PAGE 3  —  User_Loop() Pipeline
# =============================================================
fig3, ax3 = new_fig("Page 3 / 6   User_Loop()  Execution Pipeline  (1 ms / 1 kHz)",
                    figsize=(22, 14))

steps = [
    # (num, name, detail_lines, color, src)
    ("(1)",  "XM_IO_Update()",
     ["button debounce timer tick", "MUST be first call"],
     C["sys"], "Encoder_ex01"),

    ("(2)",  "btn = _ReadButtons()",
     ["XM_GetButtonEvent() x3", "stored in BtnEvents_t", "called ONLY ONCE"],
     C["sys"], "NEW"),

    ("(3)",  "Assist mode check",
     ["is_assist_mode", "= h10Mode==ASSIST", "session init on entry"],
     C["gate"], "FSR_Fuzzy"),

    ("(4)",  "_SampleFsr()",
     ["PF3-6 ADC read", "8 Hz LPF update", "cal accumulate if busy"],
     C["fsr"], "FSR_Fuzzy"),

    ("(5)",  "_UpdateFsrCal(&btn)",
     ["PRESSED events only", "FSR cal state FSM", "LED1/2 feedback"],
     C["fsr"], "FSR_Fuzzy"),

    ("(6)",  "_UpdateLoads()",
     ["(lpf - off) / (on - off)", "clamp 0 ~ 1.5", "-> s_fsr_load[4]"],
     C["fsr"], "FSR_Fuzzy"),

    ("(7)",  "_SampleEmg()",
     ["PF7/PF8 ADC read", "-> s_emg_raw_v[2]", "if cal: accumulate"],
     C["emg"], "EMG"),

    ("(8)",  "_ProcessEmgSignals()",
     ["bias removal", "80Hz LPF + rectify", "5Hz envelope LPF"],
     C["emg"], "EMG"),

    ("(9)",  "_UpdateEmgCal(&btn)",
     ["CLICK events only", "EMG cal state FSM", "LED3 feedback"],
     C["emg"], "EMG"),

    ("(10)", "_SampleAngle()",
     ["leftHipAngle read", "differentiate -> vel", "velocity LPF"],
     C["comp"], "Comp_AddOn"),

    ("(11)", "_GetEmgNorm(ch)",
     ["deadband subtract", "/ (full_scale-db)", "clamp 0~1 per ch"],
     C["emg"], "EMG (mod)"),

    ("(12)", "_UpdateSquatFsm()",
     ["emg_rh, emg_lh, angle", "5-phase state machine", "-> squat_torque_rh/lh"],
     C["fsm"], "NEW"),

    ("(13)", "_UpdateCompensation()",
     ["gravity=mgl*sin(th)", "friction if enabled", "-> comp_torque_rh/lh"],
     C["comp"], "Comp_AddOn"),

    ("(14)", "_ApplyTorque()",
     ["safety gate check", "sum squat + comp", "XM_SetAssistTorque*()"],
     C["gate"], "FSR_Fuzzy"),

    ("(15)", "_UpdatePublicSignals()",
     ["copy statics to", "public globals", "Live Expressions"],
     C["sys"], "ALL"),

    ("(16)", "_SendStream()",
     ["CDC macro fill", "XM_SendUsbDataWithId", "100 Hz rate"],
     C["cdc"], "FSR_Fuzzy"),
]

n = len(steps)
cols = 8
rows = 2
bw = 1.0 / cols - 0.008
bh = 0.36

for si, (num, name, details, color, src) in enumerate(steps):
    row = si // cols
    col = si % cols
    x = col * (1.0/cols) + 0.004
    y = 0.88 - row * 0.46

    # box
    rbox(ax3, x, y-bh, bw, bh, color, alpha=0.88)

    # step number
    ax3.text(x + bw/2, y - bh*0.08, num,
             ha="center", va="center", fontsize=11,
             fontweight="bold", color="white", transform=ax3.transAxes)
    # function name
    ax3.text(x + bw/2, y - bh*0.28, name,
             ha="center", va="center", fontsize=8,
             fontweight="bold", color="white", transform=ax3.transAxes)
    # detail lines
    for di, dl in enumerate(details):
        ax3.text(x + bw/2, y - bh*(0.50 + di*0.155), dl,
                 ha="center", va="center", fontsize=6.8,
                 color="#F5F5F5", transform=ax3.transAxes)
    # source tag
    ax3.text(x + bw/2, y - bh*1.05, src,
             ha="center", va="top", fontsize=6.5,
             style="italic", color="#616161", transform=ax3.transAxes)

    # arrow to next (within row)
    if col < cols - 1 and si < n - 1 and (si+1) // cols == row:
        ax3.annotate("", xy=(x+bw+0.006, y-bh/2),
                     xytext=(x+bw, y-bh/2),
                     xycoords="axes fraction",
                     arrowprops=dict(arrowstyle="->", color="#757575", lw=1.5))

# row divider
ax3.axhline(0.46, color="#BDBDBD", lw=1.2, linestyle="--")
ax3.text(0.5, 0.455, "  continues ->  ",
         ha="center", va="center", fontsize=9, color="#9E9E9E",
         transform=ax3.transAxes,
         bbox=dict(fc="white", ec="none"))

save(fig3, "squat_arch_p3.png")


# =============================================================
# PAGE 4  —  Squat FSM
# =============================================================
fig4, ax4 = new_fig("Page 4 / 6   Squat FSM  (5-Phase State Machine)", figsize=(20, 16))

phases_pos = {
    "STAND\n(0)":       (0.50, 0.88, C["fsr"]),
    "DESCENDING\n(1)":  (0.15, 0.62, C["emg"]),
    "BOTTOM\n(2)":      (0.50, 0.50, C["gate"]),
    "ASCENDING\n(3)":   (0.85, 0.62, "#1565C0"),
    "RETURN\n(4)":      (0.50, 0.18, C["comp"]),
}

pw = 0.16; ph = 0.10
for name, (px, py, col) in phases_pos.items():
    rbox(ax4, px-pw/2, py-ph/2, pw, ph, col, alpha=0.90)
    ax4.text(px, py, name, ha="center", va="center",
             fontsize=11, fontweight="bold", color="white",
             transform=ax4.transAxes)

# torque boxes per phase
torque_info = [
    (0.15, 0.44, C["emg"],
     "DESCENDING torque:\ncmd = +gain_res * emg_norm\ngain_res = 0.20  [resist flexion]"),
    (0.50, 0.37, C["gate"],
     "BOTTOM torque:\ncmd = +squat_max_res_nm\n= 0.30 Nm  [fixed hold]"),
    (0.85, 0.44, "#1565C0",
     "ASCENDING torque:\ncmd = -gain_assist * emg_norm\ngain_assist = 0.30  [assist extension]"),
    (0.50, 0.055, C["comp"],
     "RETURN torque:\ncmd = -gain_assist * emg_norm * 0.3\n[fade-out to 30%]"),
]
for (tx, ty, tc, txt) in torque_info:
    rbox(ax4, tx-0.14, ty-0.065, 0.28, 0.13, tc, alpha=0.12, lw=0.8, ec=tc)
    ax4.text(tx, ty, txt, ha="center", va="center",
             fontsize=8, color="#212121", transform=ax4.transAxes,
             bbox=dict(fc="none", ec="none"))

# transitions
transitions = [
    # (fx, fy, tx, ty, label, color, rad)
    (0.50, 0.835, 0.23, 0.665,
     "emg_norm > down_thr (0.30)\nAND angle > theta_stand (5 deg)\nsustained >= 150 ms debounce",
     C["emg"], 0.2),
    (0.23, 0.575, 0.42, 0.515,
     "angle < theta_sit\n(-40 deg)",
     C["gate"], 0.15),
    (0.58, 0.505, 0.77, 0.575,
     "emg_norm > up_thr\n(0.50)",
     "#1565C0", 0.15),
    (0.77, 0.665, 0.60, 0.835,
     "angle > theta_stand\n(5 deg)",
     C["fsr"], 0.2),
    (0.50, 0.445, 0.50, 0.285,
     "angle > theta_stand",
     C["comp"], 0.0),
    (0.50, 0.135, 0.50, 0.075,
     "",
     "#9E9E9E", 0.0),
    # return -> stand
    (0.42, 0.165, 0.41, 0.835,
     "emg_norm < down_thr * 0.3\n[hysteresis reset]",
     "#9E9E9E", -0.4),
    # descending -> stand (EMG lost)
    (0.07, 0.595, 0.41, 0.855,
     "emg_norm < down_thr * 0.3\n[Schmitt trigger]",
     "#FF5722", -0.35),
]

for (fx, fy, tx, ty, lbl, col, rad) in transitions:
    ax4.annotate("", xy=(tx, ty), xytext=(fx, fy),
                 xycoords="axes fraction",
                 arrowprops=dict(arrowstyle="->", color=col, lw=1.6,
                                 connectionstyle=f"arc3,rad={rad}"))
    if lbl:
        mx, my = (fx+tx)/2, (fy+ty)/2
        ax4.text(mx, my, lbl, ha="center", va="center",
                 fontsize=7.5, color=col, transform=ax4.transAxes,
                 bbox=dict(fc="white", ec=col, lw=0.8,
                           boxstyle="round,pad=0.2", alpha=0.92))

# emergency reset box
rbox(ax4, 0.01, 0.01, 0.40, 0.12, C["gate"], alpha=0.10, ec=C["gate"], lw=1.5)
ax4.text(0.21, 0.07,
    "EMERGENCY RESET  (checked every loop):\n"
    "if angle > theta_stand AND phase in {DESCENDING, BOTTOM}:\n"
    "    phase = STAND   cmd = 0   debounce_counter = 0",
    ha="center", va="center", fontsize=8.5,
    color="#B71C1C", transform=ax4.transAxes)

# tuning params box
rbox(ax4, 0.62, 0.01, 0.37, 0.22, C["fsm"], alpha=0.08, ec=C["fsm"], lw=1.2)
ax4.text(0.805, 0.12,
    "Tuning Parameters (Live Expressions):\n\n"
    "squat_theta_stand_deg     =   5.0\n"
    "squat_theta_sit_deg       = -40.0\n"
    "squat_emg_down_threshold  =   0.30\n"
    "squat_emg_up_threshold    =   0.50\n"
    "squat_gain_resistance     =   0.20\n"
    "squat_gain_assist         =   0.30\n"
    "squat_max_resistance_nm   =   0.30\n"
    "squat_entry_debounce_ms   = 150",
    ha="center", va="center", fontsize=8.5,
    family="monospace", color="#212121", transform=ax4.transAxes)

save(fig4, "squat_arch_p4.png")


# =============================================================
# PAGE 5  —  EMG Signal Pipeline
# =============================================================
fig5, ax5 = new_fig("Page 5 / 6   EMG Signal Pipeline  (per channel: RH and LH)", figsize=(20, 16))

pipeline = [
    ("Step 1",  "Raw ADC Read",
     "s_emg_raw_v[ch] = XM_AnalogReadMillivolts(s_emg_pins[ch]) * 0.001f",
     "PF7=EMG_RH  PF8=EMG_LH   range: 0 ~ 3.3 V   (biased raw EMG, center ~1.65V)",
     "#ECEFF1", "#607D8B"),

    ("Step 2",  "Bias Removal",
     "centered_v = raw - s_emg_bias_v[ch]",
     "Removes resting DC offset.  Default bias = 1.65V (set by BTN1 CLICK rest cal)",
     "#E3F2FD", C["emg"]),

    ("Step 3",  "80 Hz Low-Pass Filter",
     "pre_lpf_v = _LowPassUpdate(pre_lpf_v, centered_v, 80.0f)",
     "alpha = dt / (1/(2*pi*80) + dt)   Removes high-freq noise before rectification",
     "#B3E5FC", C["emg"]),

    ("Step 4",  "Full-Wave Rectification",
     "rect_v = _AbsFloat(pre_lpf_v)",
     "Makes negative muscle signal positive.  Equivalent to |signal|",
     "#B2EBF2", C["pin"]),

    ("Step 5",  "5 Hz Envelope LPF",
     "s_emg_env_v[ch] = _LowPassUpdate(env_v, rect_v, 5.0f)",
     "alpha = dt / (1/(2*pi*5) + dt)   Smooth muscle activation envelope",
     "#B2DFDB", C["fsr"]),

    ("Step 6",  "Deadband Removal",
     "active_v = env_v - EMG_ENVELOPE_DEADBAND_V   if active_v <= 0: return 0",
     "Default deadband = 0.020V.  Suppresses residual noise at rest",
     "#DCEDC8", "#558B2F"),

    ("Step 7",  "Normalise  ->  0 ~ 1",
     "span = s_emg_full_scale_v[ch] - deadband\nnorm = clamp(active_v / span, 0, 1)",
     "full_scale set by BTN2 CLICK effort cal (peak envelope during max contraction)",
     "#C8E6C9", "#1B5E20"),
]

y0 = 0.94
rh = 0.10
gap = 0.015

for ri, (step, name, formula, note, bg, ec) in enumerate(pipeline):
    ry = y0 - ri * (rh + gap)
    rbox(ax5, 0.0, ry-rh, 1.0, rh, bg, lw=1.2, ec=ec)

    # step label
    ax5.text(0.035, ry-rh/2, step, ha="center", va="center",
             fontsize=9, fontweight="bold", color=ec, transform=ax5.transAxes)

    # name
    ax5.text(0.115, ry-rh/2, name, ha="left", va="center",
             fontsize=11, fontweight="bold", color="#212121", transform=ax5.transAxes)

    # formula
    ax5.text(0.38, ry-rh*0.35, formula, ha="left", va="center",
             fontsize=9, family="monospace", color="#0D47A1", transform=ax5.transAxes)

    # note
    ax5.text(0.38, ry-rh*0.72, note, ha="left", va="center",
             fontsize=8.5, color="#424242", transform=ax5.transAxes)

    # arrow
    if ri < len(pipeline)-1:
        ay = ry - rh - gap/2
        ax5.annotate("", xy=(0.5, ay-0.001),
                     xytext=(0.5, ay+0.001),
                     xycoords="axes fraction",
                     arrowprops=dict(arrowstyle="->", color="#9E9E9E", lw=1.5))

# calibration note at bottom
rbox(ax5, 0.0, 0.005, 1.0, 0.085, C["warn"], lw=1.2, ec="#F9A825")
ax5.text(0.5, 0.048,
    "Calibration procedure:   "
    "BTN1 CLICK -> keep muscles RELAXED 3s -> s_emg_bias_v[ch] = mean(raw)     |     "
    "BTN2 CLICK -> MAX CONTRACTION 3s -> s_emg_full_scale_v[ch] = peak(env)     |     "
    "BTN3 CLICK -> reset to defaults  (bias=1.65V, full_scale=1.0V)",
    ha="center", va="center", fontsize=9, color="#37474F", transform=ax5.transAxes)

save(fig5, "squat_arch_p5.png")


# =============================================================
# PAGE 6  —  Public Variables + CDC Channels + Safety Gate
# =============================================================
fig6 = plt.figure(figsize=(22, 16), facecolor=C["bg"])
fig6.suptitle("Page 6 / 6   Public Variables  |  CDC Stream  |  Safety Gate",
              fontsize=15, fontweight="bold", color=C["head"], y=0.97)
gs6 = GridSpec(1, 2, figure=fig6, left=0.03, right=0.97,
               top=0.92, bottom=0.03, wspace=0.06)

# ---- 6A: Public variables ----
ax6a = fig6.add_subplot(gs6[0, 0])
ax6a.set_xlim(0,1); ax6a.set_ylim(0,1); ax6a.axis("off")
ax6a.text(0.5, 0.98, "Public Variables  (Live Expressions)",
          ha="center", va="top", fontsize=12, fontweight="bold", color=C["head"])

groups = [
    ("FSR Sensor", C["fsr"], [
        ("float",    "pf3_volt .. pf6_volt",        "raw ADC voltages (V)"),
        ("float",    "fsr_lt/lh/rt/rh_load",        "normalised load  0 ~ 1.5"),
        ("uint16_t", "fsr_cal_ready",                "1 = zero + full-load done"),
        ("uint16_t", "calibration_zero_done",        "BTN1 step complete"),
        ("uint16_t", "calibration_full_load_done",   "BTN2 step complete"),
    ]),
    ("EMG", C["emg"], [
        ("float",    "emg_rh_raw_v, emg_lh_raw_v",  "raw ADC volts (V)"),
        ("float",    "emg_rh_env_v, emg_lh_env_v",  "5Hz envelope (V)"),
        ("float",    "emg_rh_norm,  emg_lh_norm",   "normalised 0 ~ 1"),
        ("uint16_t", "emg_cal_done",                 "1 = rest + effort cal done"),
    ]),
    ("Squat FSM Tuning", C["fsm"], [
        ("float",    "squat_theta_stand_deg  =  5.0",  "stand threshold (deg)"),
        ("float",    "squat_theta_sit_deg    = -40.0", "bottom threshold (deg)"),
        ("float",    "squat_emg_down_thr     =  0.30", "STAND -> DESCEND trigger"),
        ("float",    "squat_emg_up_thr       =  0.50", "BOTTOM -> ASCEND trigger"),
        ("float",    "squat_gain_resistance  =  0.20", "resist torque gain"),
        ("float",    "squat_gain_assist      =  0.30", "assist torque gain"),
        ("float",    "squat_max_res_nm       =  0.30", "bottom fixed torque"),
        ("uint16_t", "squat_entry_debounce_ms = 150",  "anti-twitch debounce"),
        ("uint16_t", "squat_phase",                   "0~4 monitor only"),
    ]),
    ("Compensation", C["comp"], [
        ("float",    "gravity_mgl_nm     =  1.0",  "M*g*L model value"),
        ("float",    "compensation_scale =  0.20", "add-on scale 0~1"),
        ("uint16_t", "compensation_ON    =  0",    "0=off 1=gravity+fric"),
        ("uint16_t", "friction_compensation_ON = 0", "friction component"),
        ("float",    "hip_velocity_rads",           "filtered joint velocity"),
        ("float",    "comp_torque_rh/lh_nm",        "feedforward output"),
    ]),
    ("Control", C["gate"], [
        ("uint16_t", "control_ON          =  0",   "set 1 to enable torque"),
        ("float",    "assist_torque_limit_nm = 1.0","adjustable limit (Nm)"),
        ("float",    "left/right_assist_torque_nm", "final torque commanded"),
        ("uint16_t", "assist_enable",               "1 = gate open + active"),
        ("uint16_t", "assist_mode_active",           "1 = H10 in ASSIST mode"),
    ]),
    ("CDC Stream", C["cdc"], [
        ("uint16_t", "cdc_stream_enable    =  1",  "0=off 1=on"),
        ("uint16_t", "cdc_stream_period_ms = 10",  "send period (100 Hz)"),
    ]),
]

y = 0.94
for gname, gc, vlist in groups:
    gh = 0.028
    rbox(ax6a, 0.0, y-gh, 1.0, gh, gc, alpha=0.88)
    ax6a.text(0.01, y-gh/2, gname, va="center", fontsize=8.5,
              fontweight="bold", color="white", transform=ax6a.transAxes)
    y -= gh
    for (vtype, vname, vdesc) in vlist:
        vh = 0.038
        rbox(ax6a, 0.0, y-vh, 1.0, vh, "#FAFAFA", lw=0.3, ec="#E0E0E0")
        ax6a.text(0.01, y-vh/2, vtype, va="center", fontsize=7,
                  color="#7B1FA2", family="monospace", transform=ax6a.transAxes)
        ax6a.text(0.16, y-vh/2, vname, va="center", fontsize=7.5,
                  fontweight="bold", family="monospace",
                  color="#212121", transform=ax6a.transAxes)
        ax6a.text(0.70, y-vh/2, vdesc, va="center", fontsize=7,
                  color="#546E7A", transform=ax6a.transAxes)
        y -= vh
    y -= 0.008

# ---- 6B: CDC channels + safety gate ----
ax6b = fig6.add_subplot(gs6[0, 1])
ax6b.set_xlim(0,1); ax6b.set_ylim(0,1); ax6b.axis("off")
ax6b.text(0.5, 0.98, "CDC Stream Channels  +  Safety Gate",
          ha="center", va="top", fontsize=12, fontweight="bold", color=C["head"])

cdc_chans = [
    ("1",  "Hip Angle",      "deg",   "XM.status.h10.leftHipAngle",      C["comp"]),
    ("2",  "EMG RH Norm",    "-",     "emg_rh_norm",                      C["emg"]),
    ("3",  "EMG LH Norm",    "-",     "emg_lh_norm",                      C["emg"]),
    ("4",  "FSR LT Load",    "-",     "fsr_lt_load",                      C["fsr"]),
    ("5",  "FSR LH Load",    "-",     "fsr_lh_load",                      C["fsr"]),
    ("6",  "FSR RT Load",    "-",     "fsr_rt_load",                      C["fsr"]),
    ("7",  "FSR RH Load",    "-",     "fsr_rh_load",                      C["fsr"]),
    ("8",  "Squat Phase",    "id",    "squat_phase",                       C["fsm"]),
    ("9",  "Torque RH",      "Nm",    "right_assist_torque_nm",            C["gate"]),
    ("10", "Torque LH",      "Nm",    "left_assist_torque_nm",             C["gate"]),
    ("11", "Comp RH",        "Nm",    "comp_torque_rh_nm",                 C["comp"]),
    ("12", "Comp LH",        "Nm",    "comp_torque_lh_nm",                 C["comp"]),
    ("13", "Cal Ready",      "bool",  "fsr_cal_ready",                     C["fsr"]),
    ("14", "Control ON",     "bool",  "control_ON",                        C["sys"]),
]

y0 = 0.94; rh = 0.042
hdr(ax6b, 0.00, y0, 0.07, 0.04, "ch",      C["head"], 9)
hdr(ax6b, 0.08, y0, 0.22, 0.04, "Name",    C["head"], 9)
hdr(ax6b, 0.31, y0, 0.09, 0.04, "Unit",    C["head"], 9)
hdr(ax6b, 0.41, y0, 0.55, 0.04, "Source Variable", C["head"], 9)

for ri, (ch, name, unit, src, cc) in enumerate(cdc_chans):
    ry = y0 - (ri+1)*(rh+0.003) - 0.005
    bg = "#E3F2FD" if ri%2==0 else "#FAFAFA"
    rbox(ax6b, 0.00, ry, 0.07, rh, bg, lw=0.4)
    rbox(ax6b, 0.08, ry, 0.22, rh, bg, lw=0.4)
    rbox(ax6b, 0.31, ry, 0.09, rh, bg, lw=0.4)
    rbox(ax6b, 0.41, ry, 0.55, rh, bg, lw=0.4)
    rbox(ax6b, 0.965,ry, 0.01, rh, cc, alpha=0.85, lw=0)
    ax6b.text(0.035, ry+rh/2, ch, ha="center", va="center",
              fontsize=8.5, fontweight="bold", color=cc, transform=ax6b.transAxes)
    ax6b.text(0.085, ry+rh/2, name, va="center", fontsize=8.5,
              color="#212121", transform=ax6b.transAxes)
    ax6b.text(0.315, ry+rh/2, unit, ha="center", va="center",
              fontsize=8, color="#546E7A", transform=ax6b.transAxes)
    ax6b.text(0.415, ry+rh/2, src, va="center", fontsize=8,
              family="monospace", color="#0D47A1", transform=ax6b.transAxes)

# Safety gate
rbox(ax6b, 0.0, 0.01, 1.0, 0.36, C["gate"], alpha=0.06, lw=1.5, ec=C["gate"])
ax6b.text(0.5, 0.37, "Safety Gate  (_ApplyTorque)",
          ha="center", va="top", fontsize=10, fontweight="bold",
          color=C["gate"], transform=ax6b.transAxes)
ax6b.text(0.04, 0.34,
    "torque_limit = clamp(assist_torque_limit_nm, 0, HARD_MAX=2.5)\n\n"
    "assist_ok =  (control_ON == 1)\n"
    "         AND (fsr_cal_ready == 1)        // zero + full-load done\n"
    "         AND (emg_cal_done == 1)         // rest + effort done\n"
    "         AND (h10Mode == XM_H10_MODE_ASSIST)\n\n"
    "if assist_ok:\n"
    "    if NOT torque_mode_active:\n"
    "        XM_SetH10AssistExistingMode(false)\n"
    "        XM_SetControlMode(XM_CTRL_TORQUE)\n"
    "        torque_mode_active = true\n"
    "    assist_enable = 1\n"
    "    rh = clamp(squat_torque_rh + comp_torque_rh, -lim, +lim)\n"
    "    lh = clamp(squat_torque_lh + comp_torque_lh, -lim, +lim)\n"
    "    XM_SetAssistTorqueRH(rh);  XM_SetAssistTorqueLH(lh)\n\n"
    "else:\n"
    "    assist_enable = 0\n"
    "    if torque_mode_active:\n"
    "        XM_SetControlMode(XM_CTRL_MONITOR)\n"
    "        torque_mode_active = false\n"
    "    XM_SetAssistTorqueRH(0.0f);  XM_SetAssistTorqueLH(0.0f)",
    va="top", fontsize=8.8, family="monospace",
    color="#212121", transform=ax6b.transAxes)

save(fig6, "squat_arch_p6.png")

print("\nAll pages saved:")
for i in range(1, 7):
    print(f"  squat_arch_p{i}.png")
