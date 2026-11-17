#!/usr/bin/env python3
"""XM10 USB-CDC 센서 스트림을 받아 50 ms마다 H10 동작을 판정한다.

Module 0xF0 payload는 아래 13개 float가 정확히 이 순서라고 가정한다.

emg_thigh, emg_calf, l_angle, r_angle,
imu_x, imu_y, imu_z, gyro_x, gyro_y, gyro_z, torque_l, torque_r,
arm_emg

``time_s``는 PC가 패킷을 받은 뒤의 경과시간이고, ``seq_id``는 PhAI 헤더에서
얻는다. 학습 모델 입력에는 EMG·각도·IMU·gyro 10개만 들어가며 torque는 기록과
진단, arm_emg는 게임 클릭 출력에만 사용한다.
"""
from __future__ import annotations

import argparse
from collections import Counter
import csv
from dataclasses import dataclass
import json
import math
import os
from pathlib import Path
import struct
import sys
import time
from typing import Any, Iterable, Mapping

import numpy as np
import pandas as pd

try:
    from tools.h10_stream_motion_model import (
        HOP_MS,
        HOP_SAMPLES,
        SAMPLE_RATE_HZ,
        CausalMotionRuntime,
        H10MotionModelError,
        load_stream_model,
    )
except ModuleNotFoundError:
    from h10_stream_motion_model import (  # type: ignore[no-redef]
        HOP_MS,
        HOP_SAMPLES,
        SAMPLE_RATE_HZ,
        CausalMotionRuntime,
        H10MotionModelError,
        load_stream_model,
    )


PHAI_SOF = 0xAA
PHAI_HEADER_BYTES = 6
PHAI_CRC_BYTES = 2
DEFAULT_MODULE_ID = 0xF0
DEFAULT_BAUD = 921600
DEFAULT_INPUT_RATE_HZ = 1000.0
DEFAULT_STALE_MS = 250.0
PAYLOAD_COLUMNS = (
    "emg_thigh",
    "emg_calf",
    "l_angle",
    "r_angle",
    "imu_x",
    "imu_y",
    "imu_z",
    "gyro_x",
    "gyro_y",
    "gyro_z",
    "torque_l",
    "torque_r",
    "arm_emg",
)
MODEL_SENSOR_COLUMNS = PAYLOAD_COLUMNS[:10]
CSV_COLUMNS = ("time_s", "seq_id", *PAYLOAD_COLUMNS)
EXPECTED_PAYLOAD_BYTES = 4 * len(PAYLOAD_COLUMNS)
PACKAGED_MODEL_PATH = Path(__file__).resolve().with_name("h10_stream_motion_model.json")
REPOSITORY_MODEL_PATH = (
    Path(__file__).resolve().parents[1]
    / "outputs/h10_continuous_motion_classifier_v9_20260826_01a03742"
    / "h10_stream_motion_model.json"
)
DEFAULT_MODEL_PATH = (
    PACKAGED_MODEL_PATH if PACKAGED_MODEL_PATH.is_file() else REPOSITORY_MODEL_PATH
)
DEFAULT_TUNING_PATH = Path(__file__).resolve().with_name("mc_tuning.json")


class LiveMotionError(RuntimeError):
    """USB 입력, 패킷 또는 실시간 모델 계약이 맞지 않을 때 발생한다."""


class CobsDecodeError(LiveMotionError):
    pass


class PhaiFrameError(LiveMotionError):
    pass


class PhaiCrcError(LiveMotionError):
    pass


@dataclass(frozen=True)
class PhaiFrame:
    seq_id: int
    module_id: int
    status: int
    payload: bytes


@dataclass(frozen=True)
class SensorRow:
    time_s: float
    seq_id: int
    status: int
    values: tuple[float, ...]

    def sensor_dict(self) -> dict[str, float]:
        return dict(zip(PAYLOAD_COLUMNS, self.values))

    def csv_values(self) -> list[str | int]:
        return [
            f"{self.time_s:.6f}",
            self.seq_id,
            *(f"{value:.8f}" for value in self.values),
        ]


def crc16_ccitt(data: bytes) -> int:
    """PhAI V2.2 CRC16-CCITT: poly 0x1021, init 0xFFFF."""
    crc = 0xFFFF
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            if crc & 0x8000:
                crc = ((crc << 1) ^ 0x1021) & 0xFFFF
            else:
                crc = (crc << 1) & 0xFFFF
    return crc


def cobs_decode(encoded: bytes) -> bytes:
    if not encoded:
        raise CobsDecodeError("빈 COBS frame입니다.")
    if 0 in encoded:
        raise CobsDecodeError("COBS frame 안에 delimiter가 있습니다.")

    decoded = bytearray()
    cursor = 0
    while cursor < len(encoded):
        code = encoded[cursor]
        cursor += 1
        if code == 0:
            raise CobsDecodeError("COBS code가 0입니다.")
        end = cursor + code - 1
        if end > len(encoded):
            raise CobsDecodeError("COBS block이 중간에서 끝났습니다.")
        decoded.extend(encoded[cursor:end])
        cursor = end
        if code != 0xFF and cursor < len(encoded):
            decoded.append(0)
    return bytes(decoded)


def parse_phai_frame(encoded: bytes) -> PhaiFrame:
    raw = cobs_decode(encoded)
    minimum = PHAI_HEADER_BYTES + PHAI_CRC_BYTES
    if len(raw) < minimum:
        raise PhaiFrameError("PhAI frame이 header+CRC보다 짧습니다.")
    if raw[0] != PHAI_SOF:
        raise PhaiFrameError(f"PhAI SOF 오류: 0x{raw[0]:02X}")

    payload_bytes = raw[1] * 4
    expected_bytes = PHAI_HEADER_BYTES + payload_bytes + PHAI_CRC_BYTES
    if len(raw) != expected_bytes:
        raise PhaiFrameError(
            f"PhAI 길이 오류: decoded={len(raw)} expected={expected_bytes}"
        )
    payload_end = PHAI_HEADER_BYTES + payload_bytes
    received_crc = struct.unpack_from("<H", raw, payload_end)[0]
    calculated_crc = crc16_ccitt(raw[:payload_end])
    if received_crc != calculated_crc:
        raise PhaiCrcError(
            f"CRC 오류: received={received_crc:04X} calculated={calculated_crc:04X}"
        )
    return PhaiFrame(
        seq_id=struct.unpack_from("<H", raw, 2)[0],
        module_id=raw[4],
        status=raw[5],
        payload=raw[PHAI_HEADER_BYTES:payload_end],
    )


class PhaiStreamParser:
    """임의 크기의 serial read를 0x00 구분 PhAI frame들로 복원한다."""

    def __init__(self) -> None:
        self.buffer = bytearray()
        self.stats: Counter[str] = Counter()

    def feed(self, chunk: bytes) -> list[PhaiFrame]:
        self.stats["serial_bytes"] += len(chunk)
        self.buffer.extend(chunk)
        frames: list[PhaiFrame] = []
        while True:
            try:
                delimiter = self.buffer.index(0)
            except ValueError:
                if len(self.buffer) > 8192:
                    self.buffer.clear()
                    self.stats["framing_errors"] += 1
                break

            encoded = bytes(self.buffer[:delimiter])
            del self.buffer[: delimiter + 1]
            if not encoded:
                self.stats["empty_delimiters"] += 1
                continue
            try:
                frame = parse_phai_frame(encoded)
            except CobsDecodeError:
                self.stats["cobs_errors"] += 1
            except PhaiCrcError:
                self.stats["crc_errors"] += 1
            except PhaiFrameError:
                self.stats["framing_errors"] += 1
            else:
                self.stats["valid_frames"] += 1
                frames.append(frame)
        return frames


def sensor_row_from_frame(frame: PhaiFrame, time_s: float) -> SensorRow:
    if len(frame.payload) != EXPECTED_PAYLOAD_BYTES:
        raise LiveMotionError(
            "0xF0 payload는 정확히 13 float/52 byte여야 합니다: "
            f"received={len(frame.payload)} byte"
        )
    values = struct.unpack(f"<{len(PAYLOAD_COLUMNS)}f", frame.payload)
    if not all(math.isfinite(value) for value in values):
        raise LiveMotionError("센서 payload에 NaN 또는 무한대가 있습니다.")
    return SensorRow(
        time_s=float(time_s),
        seq_id=frame.seq_id,
        status=frame.status,
        values=tuple(float(value) for value in values),
    )


class FixedRateBatcher:
    """입력 한 tick 분량을 학습 모델의 1 kHz 50행으로 인과 재표본화한다."""

    def __init__(self, input_rate_hz: float) -> None:
        if not math.isfinite(input_rate_hz) or input_rate_hz <= 0.0:
            raise LiveMotionError("--input-rate-hz는 양수여야 합니다.")
        exact_count = input_rate_hz * HOP_MS / 1000.0
        rounded_count = int(round(exact_count))
        if rounded_count < 1 or not math.isclose(
            exact_count, rounded_count, rel_tol=0.0, abs_tol=1.0e-9
        ):
            raise LiveMotionError(f"{input_rate_hz:g} Hz는 {HOP_MS} ms당 정수 개 패킷이 아닙니다.")
        self.input_rate_hz = float(input_rate_hz)
        self.samples_per_result = rounded_count
        self.rows: list[SensorRow] = []

    def reset(self) -> None:
        self.rows.clear()

    def push(self, row: SensorRow) -> tuple[pd.DataFrame, SensorRow] | None:
        self.rows.append(row)
        if len(self.rows) < self.samples_per_result:
            return None
        if len(self.rows) > self.samples_per_result:
            raise LiveMotionError("50 ms 입력 batch 크기가 내부 계약을 초과했습니다.")

        source_rows = self.rows
        self.rows = []
        source = np.asarray([item.values for item in source_rows], dtype=np.float64)
        target = self._resample_to_model_rate(source)
        frame = pd.DataFrame(target, columns=PAYLOAD_COLUMNS)
        return frame, source_rows[-1]

    def _resample_to_model_rate(self, source: np.ndarray) -> np.ndarray:
        if source.shape != (self.samples_per_result, len(PAYLOAD_COLUMNS)):
            raise LiveMotionError(f"입력 batch 배열 크기 오류: {source.shape}")
        if self.samples_per_result == HOP_SAMPLES:
            return source.copy()

        # 현재 50 ms가 끝난 뒤 그 안에서 이미 받은 표본만으로 보간한다.
        # 예: 200 Hz이면 10행을 1 kHz 모델용 50행으로 바꾼다.
        source_times = (
            np.arange(1, self.samples_per_result + 1, dtype=np.float64)
            / self.input_rate_hz
        )
        target_times = np.arange(1, HOP_SAMPLES + 1, dtype=np.float64) / SAMPLE_RATE_HZ
        result = np.column_stack(
            [
                np.interp(
                    target_times,
                    source_times,
                    source[:, column],
                    left=source[0, column],
                    right=source[-1, column],
                )
                for column in range(source.shape[1])
            ]
        )
        if result.shape != (HOP_SAMPLES, len(PAYLOAD_COLUMNS)):
            raise LiveMotionError(f"재표본화 배열 크기 오류: {result.shape}")
        return result


class LiveMotionPipeline:
    """센서 패킷을 50 ms result JSON 계약으로 연결한다."""

    def __init__(
        self,
        model: Mapping[str, Any],
        input_rate_hz: float,
        runtime: Any | None = None,
        *,
        disable_crouch: bool = False,
        jump_enter_threshold: float | None = None,
    ) -> None:
        model_input_rate = int(model.get("input_rate_hz", SAMPLE_RATE_HZ))
        if model_input_rate != SAMPLE_RATE_HZ:
            raise LiveMotionError(
                f"모델 입력률이 코드와 다릅니다: {model_input_rate} != {SAMPLE_RATE_HZ}"
            )
        self.runtime = runtime or CausalMotionRuntime(
            model,
            disable_crouch=disable_crouch,
            jump_enter_threshold=jump_enter_threshold,
        )
        self.batcher = FixedRateBatcher(input_rate_hz)
        self.input_rate_hz = float(input_rate_hz)

    def reset(self) -> None:
        self.runtime.reset()
        self.batcher.reset()

    def push(
        self, row: SensorRow, *, with_sensors: bool = False
    ) -> dict[str, Any] | None:
        batch = self.batcher.push(row)
        if batch is None:
            return None
        frame, latest = batch
        model_result = dict(self.runtime.update_50ms(frame))
        probabilities = model_result.pop("probabilities", None)
        result: dict[str, Any] = {
            "valid": True,
            "time_s": latest.time_s,
            "seq_id": latest.seq_id,
            **model_result,
            # PF5 filtered envelope voltage. Do not calibrate or clamp to 0~1.
            "arm_emg": max(float(latest.sensor_dict()["arm_emg"]), 0.0),
            "input_rate_hz": self.input_rate_hz,
            "result_period_ms": HOP_MS,
        }
        if probabilities is not None:
            result["probabilities"] = probabilities
        if with_sensors:
            result["latest_sensors"] = latest.sensor_dict()
        return result


class CsvRecorder:
    def __init__(self, path: Path) -> None:
        if path.exists() or path.is_symlink():
            raise LiveMotionError(f"기존 CSV를 덮어쓰지 않습니다: {path}")
        path.parent.mkdir(parents=True, exist_ok=True)
        self.path = path
        self.stream = path.open("x", newline="", encoding="utf-8")
        self.writer = csv.writer(self.stream)
        self.writer.writerow(CSV_COLUMNS)
        self.rows = 0

    def write(self, row: SensorRow) -> None:
        self.writer.writerow(row.csv_values())
        self.rows += 1
        if self.rows % 1000 == 0:
            self.stream.flush()

    def close(self) -> None:
        self.stream.flush()
        self.stream.close()


def _load_serial_module():
    try:
        import serial
        from serial.tools import list_ports
    except ModuleNotFoundError as exc:
        raise LiveMotionError("pyserial이 없습니다: python -m pip install pyserial") from exc
    return serial, list_ports


def available_ports() -> list[tuple[str, str, str]]:
    _, list_ports = _load_serial_module()
    return [
        (port.device, port.description or "", port.hwid or "")
        for port in list_ports.comports()
    ]


def choose_port(requested: str | None) -> str:
    if requested:
        return requested
    ports = available_ports()
    preferred = [
        device
        for device, description, hwid in ports
        if (
            "usbmodem" in device.lower()
            or "ttyacm" in device.lower()
            or "usb" in description.lower()
            or "usb" in hwid.lower()
        )
    ]
    if len(preferred) == 1:
        return preferred[0]
    if len(ports) == 1:
        return ports[0][0]
    shown = ", ".join(device for device, _, _ in ports) or "없음"
    raise LiveMotionError(f"XM10 포트를 하나로 고를 수 없습니다: {shown}. --port COM번호를 지정하세요.")


def invalid_result(reason: str, time_s: float | None = None) -> dict[str, Any]:
    """입력이 끊기면 게임 쪽에서 모든 키를 놓게 하는 fail-safe result."""
    return {
        "valid": False,
        "time_s": time_s,
        "state": "stand",
        "state_changed": True,
        "jump_started": False,
        "key_command": "NONE",
        "walk_intensity": None,
        "jump_intensity": None,
        "turn_rate_dps": 0.0,
        "turn_direction": "none",
        "arm_emg": 0.0,
        "reason": reason,
    }


def _emit_result(
    result: dict[str, Any],
    *,
    events_only: bool,
    with_probabilities: bool,
) -> bool:
    if (
        events_only
        and result.get("valid", False)
        and not (result.get("state_changed") or result.get("jump_started"))
    ):
        return False
    if not with_probabilities:
        result.pop("probabilities", None)
    print(json.dumps(result, ensure_ascii=False, separators=(",", ":")), flush=True)
    return True


def run_live(args: argparse.Namespace) -> int:
    try:
        model = load_stream_model(args.model)
        pipeline = LiveMotionPipeline(
            model,
            args.input_rate_hz,
            disable_crouch=args.disable_crouch,
            jump_enter_threshold=args.jump_enter_threshold,
        )
    except H10MotionModelError as exc:
        raise LiveMotionError(f"동작 모델 설정 오류: {exc}") from exc
    port = choose_port(args.port)
    serial, _ = _load_serial_module()
    parser = PhaiStreamParser()
    recorder = CsvRecorder(args.record) if args.record is not None else None
    game_output = None
    if args.minecraft_output:
        password = args.rcon_password or os.environ.get("MC_RCON_PASSWORD")
        if not args.player:
            raise LiveMotionError("--minecraft-output에는 --player가 필요합니다.")
        if password is None:
            raise LiveMotionError("--rcon-password 또는 MC_RCON_PASSWORD 환경변수가 필요합니다.")
        try:
            try:
                from tools.mc_game_output import GameOutput
            except ModuleNotFoundError:
                from mc_game_output import GameOutput  # type: ignore[no-redef]
            game_output = GameOutput(
                player=args.player,
                rcon_host=args.rcon_host,
                rcon_port=args.rcon_port,
                rcon_password=password,
                tuning_path=args.tuning,
            )
        except Exception as exc:
            if recorder is not None:
                recorder.close()
            raise LiveMotionError(f"Minecraft 출력 초기화 실패: {exc}") from exc

    def consume_result(result: dict[str, Any]) -> bool:
        if game_output is not None:
            game_output.update(result)
        return _emit_result(
            result,
            events_only=args.events_only,
            with_probabilities=args.with_probabilities,
        )

    connection = None
    matched_packets = 0
    payload_errors = 0
    firmware_tx_drops = 0
    emitted_results = 0
    started = time.monotonic()
    rate_started = started
    rate_packets = 0
    last_matched = started
    last_keepalive = 0.0
    stale_reported = False
    stop_reason = "user_interrupt"
    try:
        connection = serial.Serial(
            port=port,
            baudrate=args.baud,
            timeout=0.02,
        )
        connection.dtr = True
        connection.rts = True
        connection.reset_input_buffer()
        connection.write(b"AGRB MON START\r\n")
        print(
            f"연결됨: {port} @ {args.baud}, module=0x{args.module:02X}, "
            f"input={args.input_rate_hz:g} Hz, result={HOP_MS} ms, "
            f"jump_th={args.jump_enter_threshold if args.jump_enter_threshold is not None else 'model'}",
            file=sys.stderr,
            flush=True,
        )
        if args.input_rate_hz != SAMPLE_RATE_HZ:
            print(
                f"주의: {args.input_rate_hz:g} Hz 입력을 {SAMPLE_RATE_HZ} Hz로 "
                "재표본화합니다. 이 조건의 실제 정확도는 검증되지 않았습니다.",
                file=sys.stderr,
                flush=True,
            )

        while True:
            now = time.monotonic()
            if args.duration is not None and now - started >= args.duration:
                stop_reason = "duration_complete"
                break

            chunk = connection.read(max(int(connection.in_waiting), 1))
            now = time.monotonic()
            if not chunk:
                if now - last_keepalive >= 1.0:
                    connection.write(b"AGRB MON START\r\n")
                    last_keepalive = now
                if (
                    now - last_matched
                ) * 1000.0 >= args.stale_ms and not stale_reported:
                    pipeline.reset()
                    stale_reported = True
                    if consume_result(
                        invalid_result("sensor_stream_stale", now - started)
                    ):
                        emitted_results += 1
                continue

            for phai_frame in parser.feed(chunk):
                if phai_frame.module_id != args.module:
                    continue
                reported_drops = phai_frame.status & 0x7F
                if reported_drops:
                    firmware_tx_drops += reported_drops
                    if firmware_tx_drops == reported_drops or firmware_tx_drops % 100 == 0:
                        print(
                            f"주의: XM10 firmware Tx drop 누계={firmware_tx_drops}",
                            file=sys.stderr,
                            flush=True,
                        )
                elapsed = now - started
                try:
                    sensor_row = sensor_row_from_frame(phai_frame, elapsed)
                except LiveMotionError as exc:
                    payload_errors += 1
                    if payload_errors == 1 or payload_errors % 100 == 0:
                        print(f"payload 오류 {payload_errors}: {exc}", file=sys.stderr)
                    continue

                matched_packets += 1
                rate_packets += 1
                last_matched = now
                stale_reported = False
                if recorder is not None:
                    recorder.write(sensor_row)
                result = pipeline.push(sensor_row, with_sensors=args.with_sensors)
                if result is not None and consume_result(result):
                    emitted_results += 1

            rate_elapsed = now - rate_started
            if rate_elapsed >= 2.0:
                observed_rate = rate_packets / rate_elapsed
                relative_error = (
                    abs(observed_rate - args.input_rate_hz) / args.input_rate_hz
                )
                if relative_error > 0.20:
                    print(
                        f"주의: 관측 0x{args.module:02X} rate={observed_rate:.1f} Hz, "
                        f"설정={args.input_rate_hz:g} Hz. --input-rate-hz를 확인하세요.",
                        file=sys.stderr,
                        flush=True,
                    )
                rate_started = now
                rate_packets = 0
    except KeyboardInterrupt:
        print(file=sys.stderr)
    except Exception as exc:
        stop_reason = f"error:{type(exc).__name__}"
        print(f"실시간 판정 오류: {exc}", file=sys.stderr)
        return 1
    finally:
        if connection is not None:
            connection.close()
        if game_output is not None:
            game_output.close()
        if recorder is not None:
            recorder.close()
        print(
            f"종료: reason={stop_reason}, packets={matched_packets}, "
            f"results={emitted_results}, payload_errors={payload_errors}, "
            f"firmware_tx_drops={firmware_tx_drops}, "
            f"parser={dict(parser.stats)}"
            + (f", csv={recorder.path}" if recorder is not None else ""),
            file=sys.stderr,
            flush=True,
        )
    return 0 if matched_packets > 0 and emitted_results > 0 else 1


def _parse_int(text: str) -> int:
    return int(text, 0)


def argument_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="H10 13채널 USB-CDC를 받아 50 ms마다 동작 result JSON 출력"
    )
    parser.add_argument("--port", help="Windows 예: COM6")
    parser.add_argument("--baud", type=int, default=DEFAULT_BAUD)
    parser.add_argument("--module", type=_parse_int, default=DEFAULT_MODULE_ID)
    parser.add_argument(
        "--model",
        type=Path,
        default=DEFAULT_MODEL_PATH,
        help="h10_stream_motion_model.json",
    )
    parser.add_argument(
        "--input-rate-hz",
        type=float,
        default=DEFAULT_INPUT_RATE_HZ,
        help="0xF0 패킷 송신률. 학습 당시 기본값 1000",
    )
    parser.add_argument(
        "--stale-ms",
        type=float,
        default=DEFAULT_STALE_MS,
        help="이 시간 동안 센서가 없으면 valid=false/NONE 출력",
    )
    parser.add_argument("--record", type=Path, help="동시에 저장할 새 CSV 경로")
    parser.add_argument(
        "--minecraft-output",
        action="store_true",
        help="result를 Minecraft RCON/DirectInput 출력에도 전달",
    )
    parser.add_argument("--player", help="Minecraft player 이름")
    parser.add_argument("--rcon-host", default="127.0.0.1")
    parser.add_argument("--rcon-port", type=int, default=25575)
    parser.add_argument("--rcon-password")
    parser.add_argument(
        "--tuning",
        type=Path,
        default=DEFAULT_TUNING_PATH,
        help="mc_tuning.json 경로",
    )
    parser.add_argument(
        "--events-only",
        action="store_true",
        help="상태 변경, 점프 시작, 입력 오류만 출력",
    )
    parser.add_argument(
        "--with-probabilities",
        action="store_true",
        help="상태별 원시 확률도 JSON에 포함",
    )
    parser.add_argument(
        "--with-sensors",
        action="store_true",
        help="마지막 13개 센서값도 각 result JSON에 포함",
    )
    parser.add_argument(
        "--disable-crouch",
        action="store_true",
        help="완전 굴곡 full-crouch gate와 Shift 출력을 끔",
    )
    parser.add_argument(
        "--jump-enter-threshold",
        type=float,
        help=(
            "점프 사건 진입 임계값. 모델 rearm 임계값보다 커야 하며, "
            "미지정 시 모델 저장값 사용"
        ),
    )
    parser.add_argument("--duration", type=float, help="지정 초 뒤 자동 종료")
    parser.add_argument("--list-ports", action="store_true")
    return parser


def main(argv: Iterable[str] | None = None) -> int:
    args = argument_parser().parse_args(argv)
    try:
        if args.list_ports:
            ports = available_ports()
            if not ports:
                print("시리얼 포트가 없습니다.")
            for device, description, hwid in ports:
                print(f"{device}\t{description}\t{hwid}")
            return 0
        if not args.model.is_file():
            raise LiveMotionError(f"모델 파일이 없습니다: {args.model}")
        if args.stale_ms <= 0.0:
            raise LiveMotionError("--stale-ms는 양수여야 합니다.")
        if args.duration is not None and args.duration <= 0.0:
            raise LiveMotionError("--duration은 양수여야 합니다.")
        if args.jump_enter_threshold is not None and not (
            0.0 <= args.jump_enter_threshold <= 1.0
        ):
            raise LiveMotionError("--jump-enter-threshold는 0~1이어야 합니다.")
        return run_live(args)
    except LiveMotionError as exc:
        print(f"오류: {exc}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
