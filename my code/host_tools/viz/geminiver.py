import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.gridspec import GridSpec
from matplotlib.patches import FancyBboxPatch

# --- 고해상도 환경 설정을 위한 넓은 도화지 ---
fig = plt.figure(figsize=(26, 34), facecolor="#F5F5F5")
fig.suptitle("Squat EMG Assist: Full Architecture & Logic Map", fontsize=24, fontweight="bold", y=0.98)

# 5x3 그리드로 배치하여 상세 내용 수용
gs = GridSpec(5, 3, figure=fig, hspace=0.3, wspace=0.2, left=0.04, right=0.96, top=0.94, bottom=0.03)

def create_panel(gs_pos, title, content_list, bg_color="#FFFFFF"):
    ax = fig.add_subplot(gs_pos)
    ax.set_title(title, fontsize=14, fontweight="bold", pad=10)
    
    # 텍스트 박스 스타일링
    y = 0.92
    for item in content_list:
        ax.text(0.02, y, item, fontsize=8, family="monospace", va="top", transform=ax.transAxes)
        y -= 0.05
    ax.axis("off")
    # 패널 박스 테두리
    patch = FancyBboxPatch((0, 0), 1, 1, boxstyle="round,pad=0.02", facecolor=bg_color, edgecolor="#BDBDBD", transform=ax.transAxes)
    ax.add_patch(patch)
    return ax

# [섹션별 전체 데이터 복원]
# Panel 1: Source Files (전체 기능 리스트 복원)
create_panel(gs[0, :], "1. Source Files & Function Map", [
    "[+] Final_FSR_Fuzzy_Logic.c: _SampleFsr, _UpdateCalibration, _UpdateLoads, _ClampFloat",
    "[+] Final_EMG.c: _SampleAdcChannels, _ProcessEmgSignals, _LowPassUpdate, _EnvelopeToTorque",
    "[+] Final_Compensation_AddOn.c: _UpdateCompensation, _SignFloat",
    "[+] Final_Encoder_ex01.c: _UpdateButtons, s_assist_session_active, safety_flush",
    "[+] Final_FSR.c: Hysteresis constants, shared LPF copy"
])

# Panel 2: Pin Assignment (PF3~PF8)
create_panel(gs[1, 0], "2. Pin Assignment (PF3-PF8)", [
    "PF3 (ADC5) : FSR Left Toe",
    "PF4 (ADC6) : FSR Left Heel",
    "PF5 (ADC7) : FSR Right Toe",
    "PF6 (ADC8) : FSR Right Heel",
    "PF7 (ADC9) : EMG Right Hip",
    "PF8 (ADC10): EMG Left Hip",
    "Note: XM_SwitchDioToAdc() required for all pins"
])

# Panel 3: Variable Resolution
create_panel(gs[1, 1], "3. Conflict Resolution (Prefix Mapping)", [
    "s_cal_state  -> s_fsr_cal_state / s_emg_cal_state",
    "s_cal_sum[]  -> s_fsr_cal_sum[] / s_emg_cal_sum[]",
    "s_raw_v[]    -> s_fsr_raw_v[]   / s_emg_raw_v[]",
    "CalState_t   -> FsrCalState_t   / EmgCalState_t",
    "Global Shared: _ClampFloat, _LowPassUpdate, control_ON, assist_enable"
])

# Panel 4: Button Assignment
create_panel(gs[1, 2], "4. Button Event Mapping", [
    "BTN1 PRESSED: FSR zero cal (1s)",
    "BTN2 PRESSED: FSR full-load cal (1s)",
    "BTN3 PRESSED: FSR cal reset",
    "BTN1 CLICK  : EMG rest cal (3s)",
    "BTN2 CLICK  : EMG effort cal (3s)",
    "BTN3 CLICK  : EMG cal reset"
])

# Panel 5: Execution Pipeline (전체 16단계 복원)
create_panel(gs[2, :], "5. User_Loop Pipeline (1ms / 1kHz Execution)", [
    "1.IO_Update | 2.ReadButtons | 3.CheckAssistMode | 4.SampleFsr | 5.UpdateFsrCal | 6.UpdateLoads",
    "7.SampleEmg | 8.ProcessEmgSignals | 9.UpdateEmgCal | 10.SampleAngle | 11.GetEmgNorm | 12.UpdateSquatFsm",
    "13.UpdateCompensation | 14.ApplyTorque | 15.UpdatePublicSignals | 16.SendStream(CDC 100Hz)"
])

# Panel 6: Squat FSM
create_panel(gs[3, 0], "6. Squat FSM (5 Phases)", [
    "Phase 0: STAND     -> (Trigger: EMG>down_thr & angle>theta_stand)",
    "Phase 1: DESCENDING-> (Trigger: angle < theta_sit)",
    "Phase 2: BOTTOM    -> (Trigger: EMG > up_thr)",
    "Phase 3: ASCENDING -> (Trigger: angle > theta_stand)",
    "Phase 4: RETURN    -> (Trigger: EMG < 0.3*down_thr)"
])

# Panel 7: EMG Pipeline
create_panel(gs[3, 1], "7. EMG Signal Pipeline", [
    "Raw -> Bias Removal -> 80Hz LPF -> Rectify -> 5Hz Envelope -> Deadband -> Normalize(0~1)"
])

# Panel 8: CDC Stream & Safety Gate
create_panel(gs[3, 2], "8. CDC Stream & Safety Gate", [
    "Stream: angle, emg_norm(2), fsr_load(4), squat_phase, torque(2), comp(2), flags(3)",
    "Gate: Check(control_ON && fsr_cal && emg_cal && h10Mode==ASSIST)",
    "Torque Limit: +/- 2.5Nm (Hard Limit)"
])

plt.savefig("squat_arch_viz_full.png", dpi=150, bbox_inches="tight")
print("Full detailed visualization saved.")