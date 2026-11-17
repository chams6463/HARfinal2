# 마인크래프트 게임 출력 모듈 구현 명세

## 0. 배경

H10 웨어러블 외골격으로 마인크래프트를 조작합니다.
**키보드·마우스를 전혀 쓰지 않고 H10 착용만으로 플레이합니다.**
이 제약이 설계 곳곳에 영향을 주므로 염두에 두세요.

```
[H10] → [판정부: 이미 있는 코드] → [게임 출력부: 지금 만들 것] → [마인크래프트]
              50ms마다 dict 하나
```

게임 구동·서버 설정은 이쪽에서 처리합니다. **파이썬 모듈만 완성해 주면 됩니다.**

판정 dict의 필드(`state`, `walk_intensity`, `jump_intensity`, `turn_rate_dps`,
`turn_direction`, `turn_angle_deg`, `arm_emg`, `confidence`)는 그쪽에서 정한
그대로 씁니다. 다만 아래는 지켜 주세요.

- `state`는 대소문자·공백 무시하고 비교
- `turn_direction`은 `cw/ccw` 외에 `right/left`, `clockwise/counterclockwise`도 허용
- `None`·키 누락·타입 불일치는 **예외를 던지지 말고** 기본값 대체 + 경고 1회
- **회전과 팔 클릭은 `state`와 독립.** 걸으면서 돌고 클릭하는 게 정상입니다

---

## 1. 인터페이스

```python
from mc_game_output import GameOutput

out = GameOutput(player="minseogong3",
                 rcon_host="127.0.0.1", rcon_port=25575,
                 rcon_password="emgdemo",
                 tuning_path="mc_tuning.json")
try:
    while True:
        decision = 판정부가_만든_dict()      # 50ms 주기
        out.update(decision)
finally:
    out.close()
```

접속 정보는 전부 **생성자 인자**로. 코드에 하드코딩하지 마세요.
판정부 코드는 수정하지 말고 `out.update(decision)` 한 줄만 추가합니다.

---

## 2. 출력 수단

| 수단 | 무엇에 |
|---|---|
| RCON (`mcrcon`) | 이동 속도, 점프력 |
| 키보드 (`pydirectinput`) | `W` 전진, `Shift` 웅크리기, `Space` 점프 |
| 마우스 이동 | 좌우 시야 회전 |
| 마우스 버튼 | 좌클릭(캐기), 우클릭(설치/사용) |

```python
import pydirectinput
pydirectinput.PAUSE = 0.0      # 기본 0.1초 슬립. 끄지 않으면 50ms 루프가 무너진다
pydirectinput.FAILSAFE = False # 마우스가 화면 모서리에 가면 예외를 던지는 기능
```

---

## 3. 동작별 명세

### 3.1 걷기 — 4단계 속도

`state == "walk"` 인 동안:
1. `W`를 **누른 채 유지** (상태 변화 시에만 `keyDown`/`keyUp`, 매 틱 호출 금지)
2. `walk_intensity`(0~1)를 앵커 4개 사이에서 **선형 보간**
3. RCON: `attribute <player> minecraft:movement_speed base set <값>`

앵커: `[0.06, 0.10, 0.14, 0.19]` — 바닐라 기본이 `0.10`이므로 60/100/140/190%.
`walk_intensity` 0.0→1단계, 0.333→2단계, 0.667→3단계, 1.0→4단계.
판정부가 이산값을 주든 연속값을 주든 둘 다 동작해야 합니다.

> 달리기(sprint)는 구현하지 않습니다. `Ctrl` 키를 쓰지 마세요. 속도 변화만 있습니다.

**속도 스무딩(필수)** — 판정값을 그대로 꽂으면 캐릭터가 덜컹거립니다.
지수 평활을 **비대칭**으로: 올릴 때 60ms, 내릴 때 180ms.
근육 신호는 걸음마다 맥동하는데, 내리는 쪽을 느리게 하면 그 골을 메웁니다.

```
tau   = attack_ms if 목표 > 현재 else release_ms
alpha = 1 - exp(-dt / (tau/1000))
현재 += alpha * (목표 - 현재)
```

**단, `state`가 `walk`가 아니게 되면 스무딩을 무시하고 즉시 정지.**
릴리즈 180ms를 그대로 적용하면 멈추라고 한 뒤에도 계속 걸어가 절벽에서 떨어집니다.

### 3.2 점프

`state`가 `jump`로 **바뀌는 순간**에만 발동.

```
1. 점프력 설정 (아래 7.1의 두 방식 중 하나)
2. 설정이 서버에 반영될 때까지 대기   ← RCON 동기 호출이라 자동
3. pydirectinput.press("space")
```

RCON은 요청-응답이라 `mcr.command()`가 반환된 시점엔 서버가 이미 처리를
끝냈습니다. 이 순서면 점프력이 확실히 반영됩니다. 점프에 수십 ms 지연이
생기는데 **감수하기로 한 사항입니다.**

점프력 앵커: `[0.42, 0.52, 0.62]` (바닐라 기본 `0.42`).
`jump_intensity` 0~1을 이 사이에서 보간.
불응기간 `jump_refractory_ms`(기본 400)를 튜닝 파일에 두세요.

### 3.3 점프 관성 ← 놓치기 쉬움

**착용자는 키보드를 쓸 수 없습니다.** 사람이 점프하면서 동시에 `W`를 누를 수
없으므로 소프트웨어가 대신해야 합니다.

**`state`가 `jump`인 동안, 직전 상태가 `walk`였다면 `W`를 계속 누른 채로 두고
이동속도도 유지하세요.** 마인크래프트는 `W`를 누른 채 점프하면 자동으로 앞으로
나아가므로, `W`를 떼지 않는 것만으로 관성이 구현됩니다.

```
직전이 walk?  예   → 점프 동안 W 유지 + 마지막 walk_intensity 속도 유지
              아니오 → 제자리 점프 (W 누르지 않음)
```

### 3.4 앉기

`state == "crouch"` 인 동안 `Shift`를 **누른 채 유지**. 토글이 아니라 홀드입니다.
착용자가 앉은 채로 걷지는 못하므로 `Shift`+`W` 조합은 고려하지 않아도 됩니다.

### 3.5 회전 — `state`와 독립

```
rate = turn_rate_dps
if |rate| < turn_deadzone_dps: rate = 0
rate = clamp(rate, -turn_max_dps, +turn_max_dps)
픽셀 = rate * turn_gain * dt
```

**부호 규약:** `+`가 시계방향(우회전) = 마우스 `+x`. 반대면 `turn_invert`로 뒤집습니다.

**소수점 누적(필수)** — 마우스는 정수 픽셀만 움직입니다. `0.4`픽셀을 버리면
느린 회전이 아예 안 먹습니다.

```python
self._accum += 픽셀
step = int(self._accum)
self._accum -= step
if step:
    pydirectinput.moveRel(step, 0, relative=True)
```

**드리프트 보정(`turn_angle_deg`)** — 각속도만 적분하면 오차가 쌓여 몸의 방향과
게임 시야가 서서히 어긋납니다. 2초에 한 번:

```
1. RCON: data get entity <player> Rotation   → [yaw, pitch]
2. 오차 = turn_angle_deg - 게임yaw           (-180~180으로 접기)
3. |오차| > turn_sync_th_deg 면 오차의 turn_sync_gain 배만큼만 추가 이동
```

한 번에 다 맞추면 화면이 홱 튑니다. **일부만 조금씩** 당기세요.
`turn_sync_enable`로 켜고 끌 수 있게 하고 기본값은 `false`.

### 3.6 팔 EMG로 마우스 클릭 — `state`와 독립

| 동작 | 조건 | 게임에서 |
|---|---|---|
| 짧은 우클릭 | 약한 수축 (0.15 이상 ~ 0.4 미만) | 블록 설치 / 아이템 사용 |
| 긴 좌클릭 | 강한 수축 (0.4 이상) | 블록 캐기 (누르는 동안 계속) |

**핵심 문제:** 0.4를 넘으려면 0.15를 반드시 지나갑니다. 임계값을 넘는 즉시
우클릭을 쏘면 세게 힘줄 때 **둘 다 발동**합니다.

**해결: 버스트가 끝날 때 우클릭을 판정합니다.**

```
[대기]
   arm_emg >= arm_click_th(0.15)  →  버스트 시작, 최대값 추적 시작

[버스트 중]
   arm_emg >= arm_hold_th(0.4)             → 좌클릭 mouseDown (아직 안 눌렀으면)
   좌클릭 눌린 상태에서
       arm_emg < arm_hold_release_th(0.35) → 좌클릭 mouseUp
   최대값 갱신

[버스트 종료]  arm_emg < arm_click_th(0.15)
   좌클릭이 눌려 있으면 → mouseUp
   그 버스트의 최대값이 arm_hold_th 를 한 번도 안 넘었고
   버스트 길이 <= arm_click_max_burst_ms(800) 이면
       → 짧은 우클릭 1회 (click, button="right")
   불응기간 arm_click_refractory_ms(300) 시작
```

**근거**
- 버스트 종료 시 판정하므로 강한 수축에서 우클릭이 오발동하지 않습니다.
  대가는 우클릭이 수축 종료까지 늦어지는 것인데, 짧은 수축이면 지연도 짧습니다.
- 좌클릭 해제 임계값(0.35)을 진입값(0.4)보다 낮게 둔 것은 히스테리시스입니다.
  같으면 0.4 근처에서 떨릴 때 클릭이 붙었다 떨어졌다를 반복합니다.
- `arm_click_max_burst_ms` — 0.15~0.4 사이에 몇 초씩 머무는 건 의도된 클릭이
  아니라 팔에 힘이 들어간 상태입니다. 우클릭으로 치지 않습니다.

**단위를 가정하지 마세요.** `arm_emg`가 정규화된 0~1이라는 전제이지만, 원시
전압을 받게 되면 튜닝 파일의 임계값만 바꿔서 대응할 수 있어야 합니다.

### 3.7 확신도

`confidence < min_confidence`(기본 0.5)면 **`state`를 갱신하지 않고 직전 상태
유지.** 회전과 팔 클릭은 별개이므로 그대로 반영합니다.

---

## 4. 안전장치 — 전부 필수

**키보드 없이 플레이하므로, 소프트웨어가 멈추면 사용자가 캐릭터를 멈출 수단이
없습니다.** 생략 불가입니다.

| 상황 | 처리 |
|---|---|
| dict가 200ms 이상 안 들어옴 | 즉시 정지 (모든 키·마우스 버튼 해제, 속도 원복) |
| `Ctrl+C` | `close()` |
| 콘솔 창 닫기 / taskkill | 시그널 핸들러로 `close()` 강제 실행 |
| RCON 끊김 | 2초마다 재시도, 그동안 키 입력 중단 |

```python
import atexit, signal
atexit.register(self.close)

def _term(signum, frame):
    raise SystemExit(0)          # atexit 를 타도록 SystemExit 로 변환

for name in ("SIGTERM", "SIGBREAK", "SIGHUP"):
    sig = getattr(signal, name, None)
    if sig:
        try:
            signal.signal(sig, _term)
        except (ValueError, OSError):
            pass
```

> 윈도우에서 콘솔 창을 닫으면 `atexit`가 실행되지 않고 죽습니다. 그러면
> **이동속도가 마지막 값에 고정되고 `W` 키와 마우스 버튼이 눌린 채 남습니다.**
> 실측으로 확인된 문제입니다.

`close()`:
```
W, Shift, Space  keyUp
마우스 좌·우 버튼 mouseUp                    ← 빠뜨리기 쉬움
movement_speed 를 base_speed(0.1) 로 원복
점프력을 base_jump(0.42) 로 원복
RCON 연결 종료
```

---

## 5. RCON 송신 억제

마인크래프트 서버는 **초당 20틱, 즉 1틱이 50ms**입니다. 판정 주기도 50ms이므로
**커맨드를 50ms보다 자주 보내는 것은 의미가 없습니다.**

```
직전 전송값과의 차이 < tx_deadband(0.005) 이고
직전 전송으로부터 1초 미만                     → 보내지 않음
직전 전송으로부터 tx_min_interval_s(0.05) 미만 → 보내지 않음
```

`< 1초` 조건은 하트비트입니다. 값이 안 변해도 1초에 한 번은 재전송해서 서버가
재시작되어도 속도가 복구되게 합니다.

---

## 6. 튜닝 파일 (핫리로드)

모든 계수를 `mc_tuning.json`에 두고 **수정시각을 0.5초마다 확인해 바뀌면 다시
읽으세요.** 프로그램 재시작 없이 게임을 켠 채로 감각을 맞출 수 있습니다.

```json
{
  "walk_speed_levels": [0.06, 0.10, 0.14, 0.19],
  "attack_ms": 60,
  "release_ms": 180,

  "jump_mode": "attribute",
  "jump_power_levels": [0.42, 0.52, 0.62],
  "jump_boost_levels": [0, 1, 2],
  "jump_refractory_ms": 400,

  "turn_mode": "mouse",
  "turn_deadzone_dps": 8.0,
  "turn_gain": 3.0,
  "turn_max_dps": 180.0,
  "turn_invert": false,
  "turn_sync_enable": false,
  "turn_sync_th_deg": 12.0,
  "turn_sync_gain": 0.25,

  "arm_click_th": 0.15,
  "arm_hold_th": 0.40,
  "arm_hold_release_th": 0.35,
  "arm_click_refractory_ms": 300,
  "arm_click_max_burst_ms": 800,

  "min_confidence": 0.5,
  "base_speed": 0.1,
  "base_jump": 0.42,
  "tx_deadband": 0.005,
  "tx_min_interval_s": 0.05
}
```

**JSON 파싱 실패 시 절대 예외를 밖으로 내보내지 마세요.** 편집기가 저장하는
순간 파일이 반쪽만 쓰여 있을 수 있는데, 거기서 죽으면 4절의 사고가 그대로
일어납니다. 실패하면 경고만 찍고 이전 값을 유지하세요.

---

## 7. 양쪽 다 구현해야 하는 것 2개

아래 두 가지는 **아직 실기 검증이 안 됐습니다.** 검증은 이쪽에서 하므로,
**두 방식을 모두 구현하고 튜닝 파일 값으로 전환**되게 만들어 주세요.

### 7.1 점프력 — `jump_mode`

| 값 | 방식 |
|---|---|
| `"attribute"` | `attribute <player> minecraft:jump_strength base set <값>` — `jump_power_levels` 사용 |
| `"effect"` | `effect give <player> minecraft:jump_boost 1 <레벨> true` — `jump_boost_levels` 사용 |

`jump_strength`는 원래 말(horse) 전용이었다가 최근 버전에서 플레이어로 확장된
속성이라 현재 버전에서 먹는지 불확실합니다. 안 되면 `effect`로 전환합니다.

### 7.2 회전 — `turn_mode`

| 값 | 방식 |
|---|---|
| `"mouse"` | `pydirectinput.moveRel` (권장, 부드러움) |
| `"command"` | `rotate <player> ~<도> ~0` — 서버 틱을 타서 50ms 단위로 끊김 |

`"command"` 모드에서는 픽셀 누적 대신 **도(degree) 단위로 누적**하고, 5절의
송신 억제를 똑같이 적용하세요.

---

## 8. 함정

| 함정 | 대응 |
|---|---|
| `pydirectinput.PAUSE` 기본 0.1초 | `0.0`으로. 안 하면 50ms 루프가 무너짐 |
| 매 틱 `keyDown`/`mouseDown` 호출 | 상태 변화 시에만 |
| 마우스 이동 소수점 버림 | 나머지 누적 필수 |
| `close()`에서 마우스 버튼 안 뗌 | 좌클릭이 눌린 채 남아 계속 블록을 캠 |
| 창 닫기 시 키 잔존 | 시그널 핸들러 필수 (4절) |
| 회전 부호 반대 | `turn_invert` |
| 클릭 임계값 겹침 | 3.6절 버스트 종료 판정 |
| 콘솔 출력 인코딩 | 시작 시 `sys.stdout.reconfigure(encoding="utf-8", errors="replace")` |

---

## 9. 인도물

1. `mc_game_output.py` — `GameOutput` 클래스
2. `mc_tuning.json` — 6절 내용
3. 판정 루프에 `out.update(decision)` 한 줄 추가한 형태
4. 콘솔에 현재 상태를 한 줄로 표시
   예: `walk[3] spd=0.14 W=O turn=-35dps mouse=-21 jump=. arm=0.42 LMB=O conf=0.99`
