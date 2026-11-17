"""
test_squat_fsm.py
=================
Final_FSR_Fuzzy_Logic.c — Squat Assist User_Loop() 로직 테스트
--------------------------------------------------------------
C 코드를 Python으로 1:1 미러링하여 하드웨어 없이 VS Code에서 검증합니다.

실행 방법:
    python test_squat_fsm.py          ← 텍스트 결과 + 플롯
    python test_squat_fsm.py --noplot ← 텍스트 결과만

테스트 항목:
    TC-01  정상 스쿼트 1회 (STAND→DESC→BOT→ASC→RETURN→STAND)
    TC-02  비상 안전장치 (DESCENDING 중 갑자기 각도 복귀)
    TC-03  EMG 의도 소멸 히스테리시스 (DESCENDING 중 EMG 소멸 → STAND 복귀)
    TC-04  safety gate OFF (control_ON=0 이면 토크 0 유지)
    TC-05  토크 클램프 (gain 크게 해도 limit 초과 불가)
    TC-06  연속 3회 스쿼트 사이클
"""

import sys
import math
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec

# ─────────────────────────────────────────────────────────────────
# 1. XM API 목(Mock) — C 구조체 / 전역변수 Python 버전
# ─────────────────────────────────────────────────────────────────

HARD_MAX_ASSIST_TORQUE_NM = 2.5   # C: #define

class XmEmgHubData:
    def __init__(self):
        self.envelope_uv = 0.0

class XmH10Data:
    def __init__(self):
        self.leftHipAngle = 0.0
        self.h10Mode      = 1       # 1 = XM_H10_MODE_ASSIST

class XmInput:
    def __init__(self):
        self.h10     = XmH10Data()
        self.emg_hub = XmEmgHubData()

class XmOutput:
    def __init__(self):
        self.assist_torque_rh = 0.0
        self.assist_torque_lh = 0.0

class XmRobot:
    def __init__(self):
        self.status  = XmInput()
        self.command = XmOutput()

XM = XmRobot()

# ─────────────────────────────────────────────────────────────────
# 2. C 전역 변수 미러링
# ─────────────────────────────────────────────────────────────────

# --- 캘리브레이션 플래그 (테스트에서는 완료 상태로 초기화) ---
s_zero_captured        = True
s_full_load_captured   = True
s_torque_mode_active   = False
s_assist_session_active= False

# --- 공개 신호 ---
assist_enable          = 0
assist_mode_active     = 0
control_ON             = 1          # Live Expressions 설정값
assist_torque_limit_nm = 1.5        # Nm
left_assist_torque_nm  = 0.0
right_assist_torque_nm = 0.0

# --- 스쿼트 FSM 파라미터 (Live Expressions 튜닝 변수) ---
squat_theta_stand_deg   =  5.0
squat_theta_sit_deg     = -40.0
# 안전장치 전용 임계값: STAND 기준보다 여유를 두어 DESCENDING 진입 직후 오발동 방지
squat_safety_reset_deg  = 15.0   # deg: 이 각도 이상이면 "명백히 서 있음" 판정
squat_emg_down_threshold = 50.0     # µV
squat_emg_up_threshold   = 80.0     # µV
squat_gain_resistance    = 0.002    # Nm/µV
squat_gain_assist        = 0.003    # Nm/µV
squat_max_resistance_nm  = 0.3      # Nm

# ─────────────────────────────────────────────────────────────────
# 3. 상태 열거형
# ─────────────────────────────────────────────────────────────────
SQUAT_STAND      = 0
SQUAT_DESCENDING = 1
SQUAT_BOTTOM     = 2
SQUAT_ASCENDING  = 3
SQUAT_RETURN     = 4

PHASE_NAME = {
    0: "STAND",
    1: "DESCENDING",
    2: "BOTTOM",
    3: "ASCENDING",
    4: "RETURN",
}

# ─────────────────────────────────────────────────────────────────
# 4. XM API 함수 목(Mock)
# ─────────────────────────────────────────────────────────────────
_last_rh = 0.0
_last_lh = 0.0

def XM_SetAssistTorqueRH(rh):
    global _last_rh
    _last_rh = rh
    XM.command.assist_torque_rh = rh

def XM_SetAssistTorqueLH(lh):
    global _last_lh
    _last_lh = lh
    XM.command.assist_torque_lh = lh

def XM_SetControlMode(mode):
    pass    # 목: 무시

def XM_SetH10AssistExistingMode(flag):
    pass    # 목: 무시

def _ClampFloat(value, lo, hi):
    """C 코드의 _ClampFloat() 1:1 복사"""
    if value < lo:
        return lo
    if value > hi:
        return hi
    return value

# ─────────────────────────────────────────────────────────────────
# 5. User_Loop() — C 코드 1:1 Python 미러
# ─────────────────────────────────────────────────────────────────
squat_phase         = SQUAT_STAND
squat_phase_monitor = 0

def User_Loop():
    """
    Final_FSR_Fuzzy_Logic.c 의 User_Loop() Python 버전.
    C 코드와 동일한 로직, 동일한 변수명 사용.
    반환값: (squat_phase, cmd_torque_rh, cmd_torque_lh, assist_enable)
    """
    global squat_phase, squat_phase_monitor
    global s_torque_mode_active, s_assist_session_active
    global assist_enable, assist_mode_active
    global left_assist_torque_nm, right_assist_torque_nm

    # ── 0. H10 ASSIST 모드 확인
    is_assist_mode = (XM.status.h10.h10Mode == 1)
    assist_mode_active = 1 if is_assist_mode else 0

    if not is_assist_mode:
        if s_assist_session_active:
            squat_phase = SQUAT_STAND
            s_assist_session_active = False
        if s_torque_mode_active:
            XM_SetControlMode(0)    # MONITOR
            XM_SetH10AssistExistingMode(True)
            s_torque_mode_active = False
        assist_enable = 0
        left_assist_torque_nm  = 0.0
        right_assist_torque_nm = 0.0
        XM_SetAssistTorqueLH(0.0)
        XM_SetAssistTorqueRH(0.0)
        return squat_phase, 0.0, 0.0, 0

    if not s_assist_session_active:
        squat_phase = SQUAT_STAND
        s_assist_session_active = True

    # ── 3. 센서 읽기
    emg_val = XM.status.emg_hub.envelope_uv
    angle   = XM.status.h10.leftHipAngle

    # ── 4. 스쿼트 5-Phase FSM
    cmd_torque_rh = 0.0
    cmd_torque_lh = 0.0

    if squat_phase == SQUAT_STAND:
        cmd_torque_rh = 0.0
        cmd_torque_lh = 0.0
        if emg_val > squat_emg_down_threshold and angle > squat_theta_stand_deg:
            squat_phase = SQUAT_DESCENDING

    elif squat_phase == SQUAT_DESCENDING:
        cmd_torque_rh = squat_gain_resistance * emg_val
        cmd_torque_lh = squat_gain_resistance * emg_val
        if angle < squat_theta_sit_deg:
            squat_phase = SQUAT_BOTTOM
        elif emg_val < squat_emg_down_threshold * 0.3:
            squat_phase = SQUAT_STAND

    elif squat_phase == SQUAT_BOTTOM:
        cmd_torque_rh = squat_max_resistance_nm
        cmd_torque_lh = squat_max_resistance_nm
        if emg_val > squat_emg_up_threshold:
            squat_phase = SQUAT_ASCENDING

    elif squat_phase == SQUAT_ASCENDING:
        cmd_torque_rh = -(squat_gain_assist * emg_val)
        cmd_torque_lh = -(squat_gain_assist * emg_val)
        if angle > squat_theta_stand_deg:
            squat_phase = SQUAT_RETURN

    elif squat_phase == SQUAT_RETURN:
        cmd_torque_rh = -(squat_gain_assist * emg_val * 0.3)
        cmd_torque_lh = -(squat_gain_assist * emg_val * 0.3)
        if emg_val < squat_emg_down_threshold * 0.3:
            squat_phase = SQUAT_STAND

    else:
        squat_phase = SQUAT_STAND

    # ── 5. 비상 안전장치
    # squat_safety_reset_deg(15°) 사용: theta_stand(5°)와 분리해
    # DESCENDING 진입 직후 각도가 아직 높을 때 오발동하지 않도록 방지
    if (angle > squat_safety_reset_deg and
            squat_phase in (SQUAT_DESCENDING, SQUAT_BOTTOM)):
        squat_phase   = SQUAT_STAND
        cmd_torque_rh = 0.0
        cmd_torque_lh = 0.0

    squat_phase_monitor = squat_phase

    # ── 6. 안전 게이트 + 토크 출력
    torque_limit = _ClampFloat(assist_torque_limit_nm, 0.0, HARD_MAX_ASSIST_TORQUE_NM)
    assist_requested = (control_ON == 1) and s_zero_captured and s_full_load_captured

    if assist_requested:
        if not s_torque_mode_active:
            XM_SetH10AssistExistingMode(False)
            XM_SetControlMode(1)    # TORQUE
            s_torque_mode_active = True
        assist_enable = 1
        right_assist_torque_nm = _ClampFloat(cmd_torque_rh, -torque_limit, torque_limit)
        left_assist_torque_nm  = _ClampFloat(cmd_torque_lh, -torque_limit, torque_limit)
        XM_SetAssistTorqueRH(right_assist_torque_nm)
        XM_SetAssistTorqueLH(left_assist_torque_nm)
    else:
        assist_enable = 0
        right_assist_torque_nm = 0.0
        left_assist_torque_nm  = 0.0
        if s_torque_mode_active:
            XM_SetControlMode(0)
            XM_SetH10AssistExistingMode(True)
            s_torque_mode_active = False
        XM_SetAssistTorqueRH(0.0)
        XM_SetAssistTorqueLH(0.0)

    return squat_phase, right_assist_torque_nm, left_assist_torque_nm, assist_enable


# ─────────────────────────────────────────────────────────────────
# 6. 시뮬레이션 헬퍼
# ─────────────────────────────────────────────────────────────────
def reset_globals():
    """각 테스트 케이스 전 전역 상태 초기화"""
    global squat_phase, squat_phase_monitor
    global s_torque_mode_active, s_assist_session_active
    global assist_enable, left_assist_torque_nm, right_assist_torque_nm
    global control_ON, assist_torque_limit_nm

    squat_phase          = SQUAT_STAND
    squat_phase_monitor  = 0
    s_torque_mode_active = False
    s_assist_session_active = False
    assist_enable        = 0
    left_assist_torque_nm  = 0.0
    right_assist_torque_nm = 0.0
    control_ON           = 1
    assist_torque_limit_nm = 1.5
    XM.status.h10.h10Mode = 1
    XM.status.h10.leftHipAngle   = squat_theta_stand_deg + 2.0
    XM.status.emg_hub.envelope_uv = 0.0


def run_scenario(steps):
    """
    steps: list of dict → {'angle': float, 'emg': float}
    각 스텝을 2ms 루프 1회로 취급.
    반환: 각 스텝의 (phase, torque_rh, assist_enable) 리스트
    """
    reset_globals()
    log = []
    for s in steps:
        XM.status.h10.leftHipAngle    = s['angle']
        XM.status.emg_hub.envelope_uv = s['emg']
        phase, rh, lh, ae = User_Loop()
        log.append({'phase': phase, 'torque_rh': rh, 'torque_lh': lh,
                    'assist_enable': ae,
                    'angle': s['angle'], 'emg': s['emg']})
    return log


# ─────────────────────────────────────────────────────────────────
# 7. 테스트 케이스 정의 및 실행
# ─────────────────────────────────────────────────────────────────
PASS = "\033[92m PASS\033[0m"
FAIL = "\033[91m FAIL\033[0m"

results = []

def check(name, condition, detail=""):
    status = PASS if condition else FAIL
    results.append((name, condition))
    marker = "✓" if condition else "✗"
    print(f"  [{marker}] {name}{(' — ' + detail) if detail else ''}")
    return condition


print("=" * 62)
print("  Squat Assist FSM — Unit Test")
print("=" * 62)

# ──────────────────────────────────────────────
# TC-01: 정상 스쿼트 1회 전체 사이클
# ──────────────────────────────────────────────
print("\n[TC-01] 정상 스쿼트 1회 전체 사이클")

def make_squat_scenario():
    """angle + emg 시나리오 생성 (실제 착용 모사)"""
    steps = []
    # Phase 0: 서 있기 (EMG 없음)
    for _ in range(10):
        steps.append({'angle': 8.0, 'emg': 10.0})
    # Phase 0→1: EMG 발생 + 아직 각도 높음
    for _ in range(5):
        steps.append({'angle': 8.0, 'emg': 60.0})
    # Phase 1: DESCENDING (각도 점점 내려감)
    for i in range(20):
        a = 8.0 - i * 2.5        # 8 → -42
        steps.append({'angle': a, 'emg': 65.0})
    # Phase 2: BOTTOM (각도 최저, EMG 낮음 유지)
    for _ in range(10):
        steps.append({'angle': -45.0, 'emg': 30.0})
    # Phase 2→3: 올라가기 의도 EMG 발생
    for _ in range(5):
        steps.append({'angle': -45.0, 'emg': 90.0})
    # Phase 3: ASCENDING (각도 상승)
    for i in range(20):
        a = -45.0 + i * 2.5      # -45 → 5
        steps.append({'angle': a, 'emg': 90.0})
    # Phase 4: RETURN (각도 복귀, EMG 감소)
    for i in range(10):
        steps.append({'angle': 8.0, 'emg': max(90.0 - i * 10, 5.0)})
    # Phase 0 복귀
    for _ in range(5):
        steps.append({'angle': 8.0, 'emg': 5.0})
    return steps

log01 = run_scenario(make_squat_scenario())

phases_seen = [r['phase'] for r in log01]
check("STAND 초기 상태 유지",
      all(r['phase'] == SQUAT_STAND for r in log01[:10]))
check("DESCENDING 진입",
      SQUAT_DESCENDING in phases_seen)
check("BOTTOM 도달",
      SQUAT_BOTTOM in phases_seen)
check("ASCENDING 진입",
      SQUAT_ASCENDING in phases_seen)
check("RETURN 진입",
      SQUAT_RETURN in phases_seen)
check("STAND 복귀",
      log01[-1]['phase'] == SQUAT_STAND)
check("DESCENDING 구간 저항 토크 > 0",
      all(r['torque_rh'] >= 0 for r in log01 if r['phase'] == SQUAT_DESCENDING))
# BOTTOM→ASCENDING 전환 틱에서는 BOTTOM의 토크(양수)가 한 번 남아 있음.
# C switch-case 동일 특성 — 전환 첫 틱 제외하고 검증
asc_steps = [r for r in log01 if r['phase'] == SQUAT_ASCENDING]
check("ASCENDING 구간 보조 토크 < 0 (신전 방향, 전환 첫 틱 제외)",
      all(r['torque_rh'] <= 0 for r in asc_steps[1:]),
      f"전환 첫 틱 torque={asc_steps[0]['torque_rh']:.3f} Nm (BOTTOM 잔류, 정상)"
      if asc_steps else "ASCENDING 없음")

# ──────────────────────────────────────────────
# TC-02: 비상 안전장치 — DESCENDING 중 갑자기 각도 복귀
# ──────────────────────────────────────────────
print("\n[TC-02] 비상 안전장치 (DESCENDING 중 강제 각도 복귀)")

steps02 = []
# STAND → DESCENDING 진입 (각도 8°, safety_reset=15° 미만)
for _ in range(5): steps02.append({'angle': 8.0,  'emg': 60.0})
# DESCENDING 중 — 각도가 safety_reset_deg(15°) 이상으로 올라감 → 안전장치 발동
for _ in range(3): steps02.append({'angle': 12.0, 'emg': 60.0})
for _ in range(3): steps02.append({'angle': 18.0, 'emg': 60.0})  # 15° 초과 → 리셋

log02 = run_scenario(steps02)
last_phases = [r['phase'] for r in log02[-3:]]
check("비상 안전장치 STAND 복귀",
      all(p == SQUAT_STAND for p in last_phases),
      f"마지막 3 phase: {last_phases}")
check("비상 복귀 후 토크 0",
      all(r['torque_rh'] == 0.0 for r in log02[-3:]))

# ──────────────────────────────────────────────
# TC-03: EMG 히스테리시스 — DESCENDING 중 EMG 소멸 → STAND 복귀
# ──────────────────────────────────────────────
print("\n[TC-03] EMG 히스테리시스 (DESCENDING 중 EMG 소멸)")

steps03 = []
for _ in range(5): steps03.append({'angle': 8.0, 'emg': 60.0})  # DESCENDING 진입
for _ in range(5): steps03.append({'angle': 0.0, 'emg': 5.0})   # EMG 급감 (< 30%)

log03 = run_scenario(steps03)
# EMG < threshold*0.3 = 15µV 이면 STAND 복귀
check("EMG 소멸 시 STAND 복귀",
      log03[-1]['phase'] == SQUAT_STAND,
      f"최종 phase: {PHASE_NAME[log03[-1]['phase']]}")

# ──────────────────────────────────────────────
# TC-04: safety gate — control_ON = 0
# ──────────────────────────────────────────────
print("\n[TC-04] Safety Gate (control_ON = 0)")

# run_scenario 은 내부에서 reset_globals() → control_ON=1 로 덮어씀.
# TC-04 는 수동으로 루프를 돌려야 control_ON=0 유지 가능.
reset_globals()
control_ON = 0   # reset_globals 호출 직후 덮어쓰기

steps04 = [{'angle': 8.0, 'emg': 60.0} for _ in range(10)]
log04 = []
for s in steps04:
    XM.status.h10.leftHipAngle    = s['angle']
    XM.status.emg_hub.envelope_uv = s['emg']
    phase, rh, lh, ae = User_Loop()
    log04.append({'phase': phase, 'torque_rh': rh, 'torque_lh': lh,
                  'assist_enable': ae, 'angle': s['angle'], 'emg': s['emg']})

check("control_ON=0 시 토크 항상 0",
      all(r['torque_rh'] == 0.0 for r in log04))
check("control_ON=0 시 assist_enable=0",
      all(r['assist_enable'] == 0 for r in log04))

control_ON = 1  # 복원

# ──────────────────────────────────────────────
# TC-05: 토크 클램프 검증
# ──────────────────────────────────────────────
print("\n[TC-05] 토크 클램프 (gain 크게 설정해도 limit 초과 불가)")

old_gain = squat_gain_resistance
squat_gain_resistance = 999.0    # 극단적으로 큰 값

steps05 = []
# STAND → DESCENDING 진입 (각도 8°, safety_reset=15° 미만이므로 안전장치 미발동)
for _ in range(5): steps05.append({'angle': 8.0, 'emg': 60.0})
# DESCENDING 유지 (각도 내려가는 중)
for _ in range(5): steps05.append({'angle': 0.0, 'emg': 60.0})

log05 = run_scenario(steps05)

max_torque = max(abs(r['torque_rh']) for r in log05)
check("HARD_MAX 초과 불가",
      max_torque <= HARD_MAX_ASSIST_TORQUE_NM,
      f"최대 토크: {max_torque:.3f} Nm (limit: {HARD_MAX_ASSIST_TORQUE_NM} Nm)")
check("assist_torque_limit_nm 초과 불가",
      max_torque <= assist_torque_limit_nm,
      f"최대 토크: {max_torque:.3f} Nm (limit: {assist_torque_limit_nm} Nm)")

squat_gain_resistance = old_gain    # 복원

# ──────────────────────────────────────────────
# TC-06: 연속 3회 스쿼트 사이클
# ──────────────────────────────────────────────
print("\n[TC-06] 연속 3회 스쿼트 사이클")

def make_multi_squat(n=3):
    steps = []
    for _ in range(n):
        for _ in range(5):  steps.append({'angle': 8.0,  'emg': 10.0})  # 대기
        for _ in range(5):  steps.append({'angle': 8.0,  'emg': 60.0})  # 의도
        for i in range(20): steps.append({'angle': 8.0 - i*2.5, 'emg': 65.0})
        for _ in range(5):  steps.append({'angle': -45.0,'emg': 30.0})
        for _ in range(5):  steps.append({'angle': -45.0,'emg': 90.0})
        for i in range(20): steps.append({'angle': -45.0 + i*2.5,'emg': 90.0})
        for i in range(10): steps.append({'angle': 8.0,  'emg': max(90-i*10,5)})
    return steps

log06 = run_scenario(make_multi_squat(3))
stand_returns = sum(1 for i in range(1, len(log06))
                    if log06[i]['phase'] == SQUAT_STAND
                    and log06[i-1]['phase'] == SQUAT_RETURN)
check("3회 모두 STAND 복귀",
      stand_returns >= 3,
      f"RETURN→STAND 전환 횟수: {stand_returns}")
check("마지막 상태 STAND",
      log06[-1]['phase'] == SQUAT_STAND)


# ─────────────────────────────────────────────────────────────────
# 8. 요약 출력
# ─────────────────────────────────────────────────────────────────
passed = sum(1 for _, ok in results if ok)
total  = len(results)

print("\n" + "=" * 62)
print(f"  결과: {passed}/{total} 통과")
if passed == total:
    print("  \033[92m모든 테스트 통과 ✓\033[0m")
else:
    failed = [name for name, ok in results if not ok]
    print(f"  \033[91m실패: {failed}\033[0m")
print("=" * 62)


# ─────────────────────────────────────────────────────────────────
# 9. 플롯 — TC-01 정상 사이클 + TC-06 연속 사이클
# ─────────────────────────────────────────────────────────────────
if "--noplot" not in sys.argv:

    PHASE_COLOR = {
        SQUAT_STAND:      '#a6e3a1',
        SQUAT_DESCENDING: '#f9e2af',
        SQUAT_BOTTOM:     '#f38ba8',
        SQUAT_ASCENDING:  '#89b4fa',
        SQUAT_RETURN:     '#cba6f7',
    }
    PHASE_LABEL = {
        0: "STAND", 1: "DESCENDING", 2: "BOTTOM",
        3: "ASCENDING", 4: "RETURN"
    }
    BG   = '#1e1e2e'
    GRID = '#45475a'
    TXT  = '#cdd6f4'

    def plot_log(ax_list, log, title):
        t  = np.arange(len(log)) * 2    # ms (2ms 루프)
        angles  = [r['angle']    for r in log]
        emgs    = [r['emg']      for r in log]
        torques = [r['torque_rh']for r in log]
        phases  = [r['phase']    for r in log]

        ax_a, ax_e, ax_t, ax_p = ax_list

        # 배경 phase 색 칠하기
        for ax in ax_list:
            prev_ph, start = phases[0], 0
            for i, ph in enumerate(phases):
                if ph != prev_ph or i == len(phases) - 1:
                    end = i if ph != prev_ph else i + 1
                    ax.axvspan(t[start], t[min(end, len(t)-1)],
                               color=PHASE_COLOR[prev_ph], alpha=0.15)
                    start   = i
                    prev_ph = ph

        ax_a.plot(t, angles,  color='#89dceb', lw=1.5, label='Hip Angle (deg)')
        ax_a.axhline(squat_theta_stand_deg, color='#f9e2af', lw=1, ls='--', label=f'θ_stand={squat_theta_stand_deg}°')
        ax_a.axhline(squat_theta_sit_deg,   color='#f38ba8', lw=1, ls='--', label=f'θ_sit={squat_theta_sit_deg}°')
        ax_a.set_ylabel('Angle (deg)', color=TXT)
        ax_a.legend(fontsize=7, loc='lower right')
        ax_a.set_title(title, color=TXT, fontsize=10)

        ax_e.plot(t, emgs, color='#a6e3a1', lw=1.5, label='EMG envelope (µV)')
        ax_e.axhline(squat_emg_down_threshold, color='#f9e2af', lw=1, ls='--',
                     label=f'down_th={squat_emg_down_threshold}µV')
        ax_e.axhline(squat_emg_up_threshold,   color='#fab387', lw=1, ls='--',
                     label=f'up_th={squat_emg_up_threshold}µV')
        ax_e.set_ylabel('EMG (µV)', color=TXT)
        ax_e.legend(fontsize=7, loc='upper right')

        ax_t.plot(t, torques, color='#f38ba8', lw=1.5, label='Torque RH (Nm)')
        ax_t.axhline(0, color=GRID, lw=0.8, ls='-')
        ax_t.axhline( assist_torque_limit_nm, color='#45475a', lw=1, ls=':')
        ax_t.axhline(-assist_torque_limit_nm, color='#45475a', lw=1, ls=':')
        ax_t.set_ylabel('Torque (Nm)', color=TXT)
        ax_t.legend(fontsize=7)

        ax_p.step(t, phases, where='post', color='#cba6f7', lw=1.5)
        ax_p.set_yticks(list(PHASE_LABEL.keys()))
        ax_p.set_yticklabels(list(PHASE_LABEL.values()), fontsize=7)
        ax_p.set_ylabel('FSM Phase', color=TXT)
        ax_p.set_xlabel('Time (ms)', color=TXT)

        for ax in ax_list:
            ax.set_facecolor(BG)
            ax.tick_params(colors=TXT)
            ax.spines['bottom'].set_color(GRID)
            ax.spines['left'].set_color(GRID)
            ax.spines['top'].set_visible(False)
            ax.spines['right'].set_visible(False)
            ax.grid(True, color=GRID, alpha=0.4, lw=0.5)
            ax.set_xlim(t[0], t[-1])

    fig = plt.figure(figsize=(16, 9), facecolor=BG)
    fig.suptitle("Squat Assist FSM — Simulation", color=TXT, fontsize=13)
    gs  = gridspec.GridSpec(4, 2, figure=fig, hspace=0.55, wspace=0.35)

    axes_l = [fig.add_subplot(gs[i, 0]) for i in range(4)]
    axes_r = [fig.add_subplot(gs[i, 1]) for i in range(4)]

    plot_log(axes_l, log01, "TC-01  단일 스쿼트 사이클")
    plot_log(axes_r, log06, "TC-06  연속 3회 스쿼트")

    # 범례: phase 색상
    from matplotlib.patches import Patch
    legend_elements = [Patch(facecolor=PHASE_COLOR[k], label=PHASE_LABEL[k])
                       for k in sorted(PHASE_COLOR)]
    fig.legend(handles=legend_elements, loc='lower center',
               ncol=5, frameon=False, fontsize=8,
               labelcolor=TXT, facecolor=BG)

    plt.savefig("D:/HARfinal/PythonDecoder/squat_fsm_test_result.png",
                dpi=150, bbox_inches='tight', facecolor=BG)
    print("\n  플롯 저장: D:/HARfinal/PythonDecoder/squat_fsm_test_result.png")
    plt.show()
