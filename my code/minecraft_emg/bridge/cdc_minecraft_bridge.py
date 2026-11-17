#!/usr/bin/env python3
"""
cdc_minecraft_bridge.py — XM10 -> 마인크래프트 동작 컨트롤러

H10 의 EMG 2채널 + IMU 를 받아 "서기/걷기/뛰기", "좌회전/우회전" 을 판정하고
마인크래프트 캐릭터를 움직인다.

    [H10/XM10] --USB--> [이 파일] --+--> 키보드 W   (전진/정지)
                                    +--> 마우스 좌우 (방향 전환)
                                    +--> RCON       (이동 속도)

펌웨어가 보내야 하는 9개 값 (모듈 0xF0, 50Hz, mc_features.CHANNELS 순서):
    0 emg_thigh  1 emg_calf  2 yaw_l  3 yaw_r
    4 gyr_l      5 gyr_r     6 thigh_l  7 thigh_r  8 fwd_vel

사용법
------
    # 하드웨어 없이 전 구간 점검 (동작을 순환하며 흉내낸다)
    python cdc_minecraft_bridge.py --source sim --sink rcon \
           --rcon-pass emgdemo --player minseogong3

    # 실제 센서 + 규칙 판정 (학습 데이터 없이도 동작)
    python cdc_minecraft_bridge.py --source serial --port COM6 --sink rcon \
           --rcon-pass emgdemo --player minseogong3

    # 학습된 모델로 판정
    python cdc_minecraft_bridge.py --source serial --port COM6 --sink rcon \
           --rcon-pass emgdemo --player minseogong3 --model mc_model.joblib

    # 신호만 보기 (마크 연결 없음)
    python cdc_minecraft_bridge.py --source serial --port COM6 --sink none

의존성
------
    pip install pyserial                (필수)
    pip install mcrcon pydirectinput    (--sink rcon)
    pip install scikit-learn joblib     (--model)

튜닝은 mc_tuning.json. 저장하면 1초 안에 반영된다 (재시작 불필요).
"""

import argparse
import atexit
import csv
import json
import math
import os
import signal
import struct
import sys
import time
from collections import deque
from datetime import datetime

import numpy as np

import mc_features as F
from mc_detectors import (ControlMapper, ModelDetector, RuleDetector,
                          SignalBuffer)

try:
    import serial
    import serial.tools.list_ports
except ImportError:
    serial = None


# ============================================================================
# PhAI V2.2 프로토콜 — cdc_phai_receiver.py 와 동일
# ============================================================================

PHAI_SOF         = 0xAA
PHAI_HEADER_SIZE = 6      # SOF(1) LEN(1) SEQ(2) MODULE(1) STATUS(1)
PHAI_CRC_SIZE    = 2
PHAI_MAX_LEN     = 255
DEFAULT_BAUD     = 921600
SERIAL_TIMEOUT   = 0.02

EMG_MODULE_ID    = 0xF0
N_SLOTS          = len(F.CHANNELS)     # 9


def _build_crc16_table():
    """CRC16-CCITT (poly 0x1021, MSB-first)."""
    table = []
    for i in range(256):
        crc = i << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if (crc & 0x8000) else (crc << 1) & 0xFFFF
        table.append(crc)
    return table


CRC16_TABLE = _build_crc16_table()

# cdc_phai_receiver.py 의 하드코딩 테이블 선두와 대조. 어긋나면 모든 프레임이
# CRC 실패로 조용히 버려져 "데이터가 아예 안 온다"처럼 보인다 — 원인 찾기가
# 지독하게 어려우므로 임포트 시점에 바로 터뜨린다.
assert CRC16_TABLE[:8] == [0x0000, 0x1021, 0x2042, 0x3063,
                           0x4084, 0x50A5, 0x60C6, 0x70E7], "CRC16 테이블 불일치"


def crc16_ccitt(data):
    crc = 0xFFFF
    for b in data:
        crc = ((crc << 8) & 0xFFFF) ^ CRC16_TABLE[((crc >> 8) ^ b) & 0xFF]
    return crc


def cobs_decode(encoded):
    """COBS 디코드 (뒤의 0x00 구분자는 빼고 넘길 것).

    우리는 0x00 을 프레임 끝 표시로 쓰는데 float 안에는 0x00 이 흔하다
    (1.0f = 00 00 80 3F). COBS 가 데이터에서 0x00 을 미리 없애 준다.
    """
    out = bytearray()
    i = 0
    while i < len(encoded):
        code = encoded[i]
        i += 1
        if code == 0:
            break
        for _ in range(1, code):
            if i >= len(encoded):
                break
            out.append(encoded[i])
            i += 1
        if code < 0xFF and i < len(encoded):
            out.append(0x00)
    return bytes(out)


# ============================================================================
# 튜닝 파일 핫리로드
# ============================================================================

DEFAULT_TUNING = {
    "thigh_rest": 0.050, "thigh_mvc": 0.600,
    "calf_rest": 0.040,  "calf_mvc": 0.450,
    "emg_thigh_weight": 0.8, "emg_calf_weight": 0.2,
    "stand_gyro_start": 30.0, "stand_gyro_stop": 20.0,
    "run_gyro_th": 110.0, "run_vel_th": 1.9,
    "turn_deadzone_dps": 8.0, "turn_gain": 3.0, "turn_max_dps": 180.0,
    "walk_band_lo": 0.25, "walk_band_hi": 0.55,
    "run_band_lo": 0.60,  "run_band_hi": 1.00,
    "move_threshold": 0.05,
    "attack_ms": 60, "release_ms": 180,
    "base_speed": 0.1, "speed_min": 0.3, "speed_max": 2.5,
    "attribute_id": "minecraft:movement_speed",
    "model_min_conf": 0.55,
    "tx_deadband": 0.01, "tx_min_interval_s": 0.05,
}


class Tuning:
    """mc_tuning.json 을 파일 수정시각 감시로 다시 읽는다.

    파싱에 실패해도 예외를 밖으로 내보내지 않는다. 편집기가 저장하는 순간
    파일이 반쪽만 쓰여 있을 수 있는데, 거기서 브릿지가 죽으면 플레이어
    속도가 마지막 값에 고정된 채 남는다.
    """

    CHECK_INTERVAL = 0.5

    def __init__(self, path):
        self.path = path
        self.cfg = dict(DEFAULT_TUNING)
        self._last_check = 0.0
        self._last_mtime = None
        self.reload(force=True, quiet=True)

    def maybe_reload(self, now):
        if now - self._last_check < self.CHECK_INTERVAL:
            return
        self._last_check = now
        self.reload()

    def reload(self, force=False, quiet=False):
        try:
            mtime = os.path.getmtime(self.path)
        except OSError:
            return
        if not force and mtime == self._last_mtime:
            return
        try:
            with open(self.path, "r", encoding="utf-8") as f:
                loaded = json.load(f)
        except Exception as e:
            print("\n[tuning] 무시 — 파싱 실패 ({}). 이전 값 유지.".format(e))
            return
        merged = dict(DEFAULT_TUNING)
        merged.update({k: v for k, v in loaded.items() if not k.startswith("_")})
        self.cfg = merged
        self._last_mtime = mtime
        if not quiet:
            print("\n[tuning] 재적용: gyro {}/{}  turn_dz={}  gain={}".format(
                merged["stand_gyro_start"], merged["stand_gyro_stop"],
                merged["turn_deadzone_dps"], merged["turn_gain"]))

    def __getitem__(self, key):
        return self.cfg[key]


# ============================================================================
# 신호원
# ============================================================================

class SerialSource:
    """USB CDC 에서 모듈 0xF0 프레임을 뽑아 9개짜리 샘플로 준다."""

    def __init__(self, port, baud=DEFAULT_BAUD):
        if serial is None:
            sys.exit("pyserial 이 없습니다:  pip install pyserial")
        self.port = port
        self.baud = baud
        self.ser = None
        self.buf = bytearray()
        self.stat_good = self.stat_crc = self.stat_sync = 0
        self.stat_shortpkt = 0
        self.last_seq = None
        self.seq_gaps = 0
        self.tx_drops = 0
        self._warned_short = False
        self._connect()

    def _connect(self):
        try:
            self.ser = serial.Serial(self.port, self.baud, timeout=SERIAL_TIMEOUT)
            print("[serial] {} @ {} 연결됨".format(self.port, self.baud))
        except Exception as e:
            self.ser = None
            print("[serial] {} 열기 실패: {}".format(self.port, e))

    def poll(self):
        if self.ser is None:
            time.sleep(1.0)
            self._connect()
            return []
        try:
            chunk = self.ser.read(2048)
        except Exception as e:
            print("\n[serial] 포트 소실: {} — 재연결 시도".format(e))
            try:
                self.ser.close()
            except Exception:
                pass
            self.ser = None
            return []

        if not chunk:
            return []
        self.buf.extend(chunk)

        # USB CDC 는 스트림이라 패킷 경계가 없다. 한 번의 read() 가 패킷
        # 반쪽을 줄 수도, 세 개를 뭉쳐 줄 수도 있어 직접 잘라야 한다.
        samples = []
        while True:
            delim = self.buf.find(b"\x00")
            if delim < 0:
                break
            if delim > 0:
                s = self._parse(cobs_decode(bytes(self.buf[:delim])))
                if s is not None:
                    samples.append(s)
            del self.buf[:delim + 1]
        return samples

    def _parse(self, frame):
        if len(frame) < PHAI_HEADER_SIZE + PHAI_CRC_SIZE or frame[0] != PHAI_SOF:
            self.stat_sync += 1
            return None
        n = frame[1]
        if n == 0 or n > PHAI_MAX_LEN:
            self.stat_sync += 1
            return None
        total = PHAI_HEADER_SIZE + n * 4 + PHAI_CRC_SIZE
        if len(frame) < total:
            self.stat_sync += 1
            return None

        crc_recv = frame[total - 2] | (frame[total - 1] << 8)
        if crc16_ccitt(frame[:total - PHAI_CRC_SIZE]) != crc_recv:
            self.stat_crc += 1
            return None

        if frame[4] != EMG_MODULE_ID:
            return None                  # 다른 모듈(IMU/모터/디버그) — 정상이므로 조용히 무시

        if n < N_SLOTS:
            # 펌웨어가 아직 9개를 안 보내고 있다. 조용히 버리면 "왜 아무것도
            # 안 되지" 로 헤매게 되므로 한 번은 분명히 알린다.
            self.stat_shortpkt += 1
            if not self._warned_short:
                self._warned_short = True
                print("\n[!] 펌웨어가 슬롯 {}개만 보냅니다. {}개가 필요합니다.\n"
                      "    필요 순서: {}".format(n, N_SLOTS, ", ".join(F.CHANNELS)))
            return None

        seq = frame[2] | (frame[3] << 8)
        self.tx_drops += frame[5] & 0x7F
        if self.last_seq is not None and ((seq - self.last_seq) & 0xFFFF) != 1:
            self.seq_gaps += 1
        self.last_seq = seq

        vals = struct.unpack_from("<{}f".format(n), frame, PHAI_HEADER_SIZE)
        self.stat_good += 1
        return np.asarray(vals[:N_SLOTS], dtype=np.float64)

    def close(self):
        if self.ser is not None:
            try:
                self.ser.close()
            except Exception:
                pass


class SimSource:
    """하드웨어 없이 전 구간을 점검하기 위한 가짜 센서.

    STAND -> WALK -> WALK+좌회전 -> RUN -> RUN+우회전 을 순환한다.
    캐릭터가 그 순서대로 움직이면 브릿지 -> 마크 경로가 살아있다는 뜻이다.
    """

    SCRIPT = [
        # (이름, 지속초, 보행주파수Hz, 스윙진폭deg, 전방속도, 회전율deg/s, EMG배율)
        ("STAND",      6, 0.00,  2.0, 0.02,   0.0, 0.05),
        ("WALK",       8, 0.95, 22.0, 1.15,   0.0, 0.40),
        ("WALK+LEFT",  8, 0.95, 22.0, 1.10, -35.0, 0.42),
        ("RUN",        8, 1.55, 34.0, 2.45,   0.0, 0.80),
        ("RUN+RIGHT",  8, 1.55, 34.0, 2.35,  35.0, 0.82),
        ("STAND+RIGHT", 6, 0.00, 3.0, 0.02,  40.0, 0.07),
    ]

    def __init__(self, tuning):
        self.t = tuning
        self.t0 = time.perf_counter()
        self._next = 0.0
        self._yaw = 0.0
        self.seq_gaps = 0
        self.tx_drops = 0
        self.stat_shortpkt = 0
        self.phase_name = "-"
        self._total = sum(s[1] for s in self.SCRIPT)
        self._rng = np.random.default_rng(3)

    def poll(self):
        now = time.perf_counter()
        if now < self._next:
            time.sleep(0.004)
            return []
        dt = 0.02
        self._next = now + dt                       # 50 Hz

        el = (now - self.t0) % self._total
        acc = 0.0
        seg = self.SCRIPT[-1]
        for s in self.SCRIPT:
            if el < acc + s[1]:
                seg = s
                break
            acc += s[1]
        name, _, freq, amp, vel, rate, egain = seg
        self.phase_name = name

        t = now - self.t0
        ph = 2 * math.pi * freq * t
        self._yaw += rate * dt

        rest_t = self.t["thigh_rest"]
        rest_c = self.t["calf_rest"]
        span_t = self.t["thigh_mvc"] - rest_t
        span_c = self.t["calf_mvc"] - rest_c
        puls = 0.5 * (1 + math.sin(ph)) if freq > 0 else 0.0

        s = np.zeros(N_SLOTS)
        s[F.CH["emg_thigh"]] = rest_t + span_t * egain * (0.55 + 0.45 * puls)
        s[F.CH["emg_calf"]] = rest_c + span_c * egain * 0.85 * (0.45 + 0.55 * puls)
        yw = (self._yaw + 180) % 360 - 180
        s[F.CH["yaw_l"]] = yw + 3.0 * math.sin(ph)
        s[F.CH["yaw_r"]] = yw - 3.0 * math.sin(ph)
        s[F.CH["gyr_l"]] = -amp * 2 * math.pi * freq * math.cos(ph)
        s[F.CH["gyr_r"]] = amp * 2 * math.pi * freq * math.cos(ph)
        s[F.CH["thigh_l"]] = -80 + amp * math.sin(ph + math.pi)
        s[F.CH["thigh_r"]] = -80 + amp * math.sin(ph)
        s[F.CH["fwd_vel"]] = vel
        s += self._rng.normal(0, 0.004, N_SLOTS) * np.abs(s).clip(0.05, None)
        return [s]

    def close(self):
        pass


# ============================================================================
# 출력 어댑터
# ============================================================================

class NullSink:
    name = "none"

    def send(self, moving, attr, turn_px):
        pass

    def close(self):
        pass


class RconSink:
    """전진=키보드 W, 회전=마우스 좌우, 속도=RCON /attribute.

    회전을 커맨드(/rotate)가 아니라 마우스로 보내는 이유: RCON 커맨드는
    서버 틱(초당 20회)에서 처리되어 회전이 뚝뚝 끊긴다. 마우스는 서버를
    거치지 않아 부드럽고 지연도 없다.
    """

    name = "rcon"

    def __init__(self, host, port, password, player, tuning):
        try:
            from mcrcon import MCRcon
        except ImportError:
            sys.exit("mcrcon 이 없습니다:  pip install mcrcon")
        try:
            import pydirectinput
        except ImportError:
            sys.exit("pydirectinput 이 없습니다:  pip install pydirectinput")

        self._MCRcon = MCRcon
        self._kb = pydirectinput
        self._kb.PAUSE = 0.0        # 기본 0.1초 슬립이 있어 끄지 않으면 루프가 죽는다
        self._kb.FAILSAFE = False

        self.host, self.port, self.password = host, port, password
        self.player = player
        self.t = tuning

        self.mcr = None
        self._last_attr = None
        self._last_tx = 0.0
        self._moving = False
        self._next_retry = 0.0
        self._connect()

    def _connect(self):
        now = time.perf_counter()
        if now < self._next_retry:
            return False
        try:
            self.mcr = self._MCRcon(self.host, self.password, port=self.port)
            self.mcr.connect()
            print("[rcon] {}:{} 연결됨 (player={})".format(
                self.host, self.port, self.player))
            self._last_attr = None
            return True
        except Exception as e:
            self.mcr = None
            self._next_retry = now + 2.0
            print("\n[rcon] 연결 실패: {} — 2초 후 재시도".format(e))
            return False

    def _cmd(self, command):
        if self.mcr is None and not self._connect():
            return
        try:
            self.mcr.command(command)
        except Exception as e:
            print("\n[rcon] 송신 실패: {} — 재연결".format(e))
            self.mcr = None

    def send(self, moving, attr, turn_px):
        # --- 전진: 상태가 바뀔 때만 (매 루프 keyDown 은 게임이 싫어한다) ---
        if moving != self._moving:
            try:
                if moving:
                    self._kb.keyDown("w")
                else:
                    self._kb.keyUp("w")
            except Exception as e:
                print("\n[key] 실패: {}".format(e))
            self._moving = moving

        # --- 회전: 마우스 상대 이동 ---
        if turn_px:
            try:
                self._kb.moveRel(int(turn_px), 0, relative=True)
            except Exception as e:
                print("\n[mouse] 실패: {}".format(e))

        # --- 속도: 데드밴드 + 20Hz 상한 ---
        # 값이 그대로여도 1초에 한 번은 재송신한다(서버 재시작 대비 하트비트).
        now = time.perf_counter()
        if (self._last_attr is not None
                and abs(attr - self._last_attr) < self.t["tx_deadband"]
                and now - self._last_tx < 1.0):
            return
        if now - self._last_tx < self.t["tx_min_interval_s"]:
            return
        self._cmd("attribute {} {} base set {:.4f}".format(
            self.player, self.t["attribute_id"], attr))
        self._last_attr = attr
        self._last_tx = now

    def close(self):
        """정상 종료 경로. 강제종료(kill -9)까지는 못 막는다 — 그때는 게임에서
        /attribute <player> minecraft:movement_speed base set 0.1 로 수동 복구."""
        try:
            self._kb.keyUp("w")
        except Exception:
            pass
        if self.mcr is not None:
            try:
                self.mcr.command("attribute {} {} base set {:.4f}".format(
                    self.player, self.t["attribute_id"], self.t["base_speed"]))
                print("\n[rcon] 속도 원복 완료")
            except Exception:
                print("\n[rcon] 원복 실패 — 게임에서 수동으로 base set 0.1 하세요")
            try:
                self.mcr.disconnect()
            except Exception:
                pass


# ============================================================================
# 메인
# ============================================================================

WATCHDOG_S = 0.20     # 펌웨어가 50Hz 이므로 10패킷 분량. 이만큼 끊기면 정지.


def list_ports():
    if serial is None:
        sys.exit("pyserial 이 없습니다:  pip install pyserial")
    ports = list(serial.tools.list_ports.comports())
    if not ports:
        print("시리얼 포트 없음")
        return
    for p in ports:
        print("  {:10s}  {}".format(p.device, p.description))


def make_meter(v, width=14):
    n = int(round(min(max(v, 0.0), 1.0) * width))
    return "█" * n + "░" * (width - n)


def main():
    # Windows 기본 콘솔/파이프 인코딩이 cp949 라 em-dash 나 블록문자에서 죽는다.
    for stream in (sys.stdout, sys.stderr):
        try:
            stream.reconfigure(encoding="utf-8", errors="replace")
        except Exception:
            pass

    here = os.path.dirname(os.path.abspath(__file__))

    ap = argparse.ArgumentParser(description="XM10 -> Minecraft 동작 컨트롤러")
    ap.add_argument("--source", choices=["serial", "sim"], default="serial")
    ap.add_argument("--port", default=None, help="COM 포트 (--source serial)")
    ap.add_argument("--baud", type=int, default=DEFAULT_BAUD)
    ap.add_argument("--sink", choices=["rcon", "none"], default="rcon")
    ap.add_argument("--rcon-host", default="127.0.0.1")
    ap.add_argument("--rcon-port", type=int, default=25575)
    ap.add_argument("--rcon-pass", default=None)
    ap.add_argument("--player", default=None)
    ap.add_argument("--model", default=None,
                    help="mc_train.py 가 만든 .joblib. 없으면 규칙 판정")
    ap.add_argument("--tuning", default=os.path.join(here, "mc_tuning.json"))
    ap.add_argument("--log", default=None)
    ap.add_argument("--no-log", action="store_true")
    ap.add_argument("--list-ports", action="store_true")
    args = ap.parse_args()

    if args.list_ports:
        list_ports()
        return

    tuning = Tuning(args.tuning)
    print("[tuning] {}".format(args.tuning))

    # --- 신호원 ---
    if args.source == "sim":
        source = SimSource(tuning)
        print("[source] SIM — STAND/WALK/WALK+좌/RUN/RUN+우 순환 (하드웨어 없음)")
    else:
        if not args.port:
            sys.exit("--port 를 지정하세요 (--list-ports 로 확인)")
        source = SerialSource(args.port, args.baud)

    # --- 판정기 ---
    if args.model:
        path = args.model if os.path.isabs(args.model) else os.path.join(here, args.model)
        detector = ModelDetector(path, tuning)
        cv = detector.cv
        print("[detector] 모델 — {}".format(os.path.basename(path)))
        if cv.get("loco") is not None:
            print("           교차검증 정확도  loco={:.1%}  turn={:.1%}".format(
                cv.get("loco") or 0, cv.get("turn") or 0))
    else:
        detector = RuleDetector(tuning)
        print("[detector] 규칙 (임계값) — 학습 모델을 쓰려면 --model")

    # --- 출력 ---
    if args.sink == "rcon":
        if not args.rcon_pass or not args.player:
            sys.exit("--rcon-pass 와 --player 가 필요합니다")
        sink = RconSink(args.rcon_host, args.rcon_port, args.rcon_pass,
                        args.player, tuning)
    else:
        sink = NullSink()

    buf = SignalBuffer()
    mapper = ControlMapper(tuning)

    # --- CSV 로깅 ---
    writer = fh = None
    if not args.no_log:
        path = args.log
        if path is None:
            d = os.path.join(here, "data")
            os.makedirs(d, exist_ok=True)
            path = os.path.join(d, "mcbridge_{:%Y%m%d_%H%M%S}.csv".format(datetime.now()))
        fh = open(path, "w", newline="", encoding="utf-8")
        writer = csv.writer(fh)
        # 중간단계를 전부 남긴다 — "왜 저기서 튀었지" 를 사후에 추적하려면
        # 원본값·판정·조작량이 모두 있어야 한다.
        writer.writerow(["t"] + F.CHANNELS +
                        ["loco", "turn", "yaw_rate", "effort", "conf",
                         "speed", "attr", "moving", "turn_px",
                         "seq_gaps", "tx_drops"])
        print("[log] {}".format(path))

    def cleanup():
        try:
            sink.send(False, tuning["base_speed"], 0)
        except Exception:
            pass
        sink.close()
        source.close()
        if fh is not None:
            fh.close()

    atexit.register(cleanup)

    # SIGTERM(콘솔 창 닫기, taskkill)은 기본적으로 atexit 를 거치지 않고 죽는다.
    # 그대로 두면 속도가 마지막 값에 고정되고 W 키도 눌린 채 남는다.
    def _term(signum, frame):
        raise SystemExit(0)

    for name in ("SIGTERM", "SIGBREAK", "SIGHUP"):
        sig = getattr(signal, name, None)
        if sig is not None:
            try:
                signal.signal(sig, _term)
            except (ValueError, OSError):
                pass

    print("[run] Ctrl+C 로 종료. 마인크래프트 창을 클릭해 활성화해 두세요.\n")

    t0 = time.perf_counter()
    last_rx = t0
    next_decide = 0.0
    next_meter = 0.0
    next_flush = 0.0
    stale = False
    dec = None
    moving, attr, turn_px = False, tuning["base_speed"], 0

    try:
        while True:
            samples = source.poll()
            now = time.perf_counter()

            for s in samples:
                buf.push(s)
            if samples:
                last_rx = now
                if stale:
                    print("\n[watchdog] 신호 복구")
                    stale = False

            if now - last_rx > WATCHDOG_S:
                # USB 가 끊겼는데 마지막 값이 남아 있으면 캐릭터가 영원히 달린다
                if not stale:
                    print("\n[watchdog] {:.0f}ms 무수신 — 정지".format(WATCHDOG_S * 1000))
                    stale = True
                moving, attr, turn_px = mapper.force_stop()
                sink.send(moving, attr, turn_px)

            elif buf.ready and now >= next_decide:
                # 판정은 홉 주기(0.2초)마다. 매 샘플마다 하면 낭비이고,
                # 학습 때 쓴 주기와 달라져 모델이 본 적 없는 조건이 된다.
                next_decide = now + F.HOP_S
                win = buf.array()
                dec = detector.update(win, now)
                moving, attr, turn_px = mapper.update(dec, now)
                sink.send(moving, attr, turn_px)

                if writer is not None:
                    writer.writerow(
                        ["{:.4f}".format(now - t0)]
                        + ["{:.5f}".format(x) for x in win[-1]]
                        + [dec.loco, dec.turn, "{:.2f}".format(dec.yaw_rate),
                           "{:.4f}".format(dec.effort), "{:.3f}".format(dec.confidence),
                           "{:.4f}".format(mapper.speed), "{:.4f}".format(attr),
                           int(moving), turn_px,
                           source.seq_gaps, source.tx_drops])

            if fh is not None and now >= next_flush:
                next_flush = now + 2.0
                fh.flush()          # 비정상 종료해도 직전 2초까지는 남긴다

            tuning.maybe_reload(now)

            if now >= next_meter:
                next_meter = now + 0.1
                if dec is None:
                    sys.stdout.write("\r버퍼 채우는 중... ({}/{})   ".format(
                        len(buf.buf), buf.n))
                else:
                    sim = getattr(source, "phase_name", None)
                    sys.stdout.write(
                        "\r{:>5s}/{:<5s} yaw={:+6.1f} eff={:.2f} |{}| "
                        "attr={:.3f} W={} mouse={:+3d} {}  ".format(
                            dec.loco, dec.turn, dec.yaw_rate, dec.effort,
                            make_meter(mapper.speed), attr,
                            "O" if moving else ".", turn_px,
                            "[sim:{}]".format(sim) if sim else ""))
                sys.stdout.flush()

    except KeyboardInterrupt:
        print("\n[run] 종료")


if __name__ == "__main__":
    main()
