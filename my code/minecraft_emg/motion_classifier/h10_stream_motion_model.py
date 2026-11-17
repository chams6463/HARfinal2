#!/usr/bin/env python3
"""H10 1 kHz 센서용 50 ms 인과 스트리밍 동작 판정 공통 코드."""
from __future__ import annotations

import json
import math
from pathlib import Path
from typing import Any, Mapping

import numpy as np
import pandas as pd

try:
    from tools.h10_motion_segment_model import (
        H10MotionModelError,
        REQUIRED_SENSOR_COLUMNS,
        STATE_CLASSES,
        fit_emg_scaling,
        softmax,
    )
except ModuleNotFoundError:
    from h10_motion_segment_model import (  # type: ignore[no-redef]
        H10MotionModelError,
        REQUIRED_SENSOR_COLUMNS,
        STATE_CLASSES,
        fit_emg_scaling,
        softmax,
    )


LEGACY_MODEL_SCHEMA_VERSION = "h10-motion-stream-emg-linear-1.1"
MODEL_SCHEMA_VERSION = "h10-motion-stream-emg-tree-2.0"
SAMPLE_RATE_HZ = 1000
HOP_MS = 50
HOP_SAMPLES = 50
# 학습된 bilateral q95 굴곡 span의 95%를 실시간 "최대 굴곡 plateau"로 본다.
# 92%보다 정점 의미가 분명하고, 97%보다 시험 tail 검출을 안정적으로 유지했다.
FULL_CROUCH_FRACTION = 0.95
HISTORY_WINDOWS_MS = (20, 40, 80, 160, 320)
MAX_HISTORY_SAMPLES = max(HISTORY_WINDOWS_MS)

SIGNAL_NAMES = (
    "emg_thigh_scaled",
    "emg_calf_scaled",
    "l_angle_rel",
    "r_angle_rel",
    "mean_angle_rel",
    "diff_angle_rel",
    "imu_x_rel",
    "imu_y_rel",
    "imu_z_rel",
    "acc_norm_rel",
    "gyro_x_rel",
    "gyro_y_rel",
    "gyro_z_rel",
    "gyro_norm",
)
STAT_NAMES = ("mean", "std", "range", "delta")
FEATURE_NAMES = tuple(
    f"{signal}_{stat}_{window_ms}ms"
    for window_ms in HISTORY_WINDOWS_MS
    for stat in STAT_NAMES
    for signal in SIGNAL_NAMES
)
# 좌우 회전은 독립 출력이므로 상태·강도 모델에서는 yaw와 yaw가 포함된 norm을 뺀다.
DECISION_FEATURE_NAMES = tuple(
    name
    for name in FEATURE_NAMES
    if not name.startswith("gyro_y_rel_") and not name.startswith("gyro_norm_")
)
DECISION_FEATURE_INDICES = np.asarray(
    [FEATURE_NAMES.index(name) for name in DECISION_FEATURE_NAMES], dtype=np.int64
)

CONTINUOUS_DERIVED_WINDOWS_MS = (20, 50, 100, 200, 320)
CONTINUOUS_DERIVED_STAT_NAMES = (
    "hip_corr",
    "hip_velocity_corr",
    "common_angle_range",
    "diff_angle_range",
    "acc_norm_range",
    "acc_norm_std",
    "acc_norm_min_rel",
    "acc_norm_max_rel",
    "same_direction_fraction",
)
CONTINUOUS_DERIVED_FEATURE_NAMES = tuple(
    f"{name}_{window_ms}ms"
    for window_ms in CONTINUOUS_DERIVED_WINDOWS_MS
    for name in CONTINUOUS_DERIVED_STAT_NAMES
)
CONTINUOUS_FEATURE_NAMES = DECISION_FEATURE_NAMES + CONTINUOUS_DERIVED_FEATURE_NAMES


def fit_stream_calibration(
    frames: list[pd.DataFrame], rest_frames: list[pd.DataFrame]
) -> dict[str, Any]:
    """학습 구간만 사용해 EMG와 중립 자세 기준을 계산한다."""
    emg = fit_emg_scaling(frames, rest_frames)
    neutral: dict[str, float] = {}
    for channel in (
        "l_angle",
        "r_angle",
        "imu_x",
        "imu_y",
        "imu_z",
        "gyro_x",
        "gyro_y",
        "gyro_z",
    ):
        values = np.concatenate(
            [frame[channel].to_numpy(dtype=np.float64) for frame in rest_frames]
        )
        neutral[channel] = float(np.median(values))
    return {
        "emg": emg,
        "neutral": neutral,
        "authority": "training_rest_only_session_relative",
    }


def _scaled_emg(
    values: np.ndarray, channel: str, calibration: Mapping[str, Any]
) -> np.ndarray:
    contract = calibration["emg"]
    channel_contract = contract["channels"][channel]
    return np.clip(
        (values - float(channel_contract["rest_median"]))
        / float(channel_contract["activation_p95"]),
        float(contract["clip_min"]),
        float(contract["clip_max"]),
    )


def transform_sensor_frame(
    frame: pd.DataFrame, calibration: Mapping[str, Any]
) -> np.ndarray:
    """원시 센서를 세션 상대 EMG와 중립 상대 운동 신호 14개로 바꾼다."""
    missing = sorted(set(REQUIRED_SENSOR_COLUMNS) - set(frame.columns))
    if missing:
        raise H10MotionModelError(f"필수 센서 열 누락: {missing}")
    if len(frame) == 0:
        raise H10MotionModelError("빈 센서 프레임은 판정할 수 없습니다.")
    sensor_values = frame.loc[:, REQUIRED_SENSOR_COLUMNS].to_numpy(
        dtype=np.float64
    )
    if not np.isfinite(sensor_values).all():
        raise H10MotionModelError("센서 열에 NaN 또는 무한대가 있습니다.")
    neutral = calibration["neutral"]
    thigh = _scaled_emg(
        frame["emg_thigh"].to_numpy(dtype=np.float64),
        "emg_thigh",
        calibration,
    )
    calf = _scaled_emg(
        frame["emg_calf"].to_numpy(dtype=np.float64),
        "emg_calf",
        calibration,
    )
    left = frame["l_angle"].to_numpy(dtype=np.float64) - float(
        neutral["l_angle"]
    )
    right = frame["r_angle"].to_numpy(dtype=np.float64) - float(
        neutral["r_angle"]
    )
    imu = frame[["imu_x", "imu_y", "imu_z"]].to_numpy(dtype=np.float64)
    imu_neutral = np.asarray(
        [neutral["imu_x"], neutral["imu_y"], neutral["imu_z"]],
        dtype=np.float64,
    )
    gyro = frame[["gyro_x", "gyro_y", "gyro_z"]].to_numpy(dtype=np.float64)
    gyro_neutral = np.asarray(
        [neutral["gyro_x"], neutral["gyro_y"], neutral["gyro_z"]],
        dtype=np.float64,
    )
    gyro_rel = gyro - gyro_neutral
    result = np.column_stack(
        (
            thigh,
            calf,
            left,
            right,
            0.5 * (left + right),
            left - right,
            imu - imu_neutral,
            np.linalg.norm(imu, axis=1) - np.linalg.norm(imu_neutral),
            gyro_rel,
            np.linalg.norm(gyro_rel, axis=1),
        )
    )
    if result.shape != (len(frame), len(SIGNAL_NAMES)) or not np.isfinite(result).all():
        raise H10MotionModelError(f"스트리밍 변환 배열 오류: {result.shape}")
    return result


def causal_feature_vector(history: np.ndarray) -> np.ndarray:
    """현재 시점 이전 값만 사용해 20~320 ms 다중 시간창 특징을 만든다."""
    values = np.asarray(history, dtype=np.float64)
    if values.shape != (MAX_HISTORY_SAMPLES, len(SIGNAL_NAMES)):
        raise H10MotionModelError(f"인과 버퍼 크기 오류: {values.shape}")
    output: list[np.ndarray] = []
    for window_ms in HISTORY_WINDOWS_MS:
        window = values[-window_ms:]
        output.extend(
            (
                np.mean(window, axis=0),
                np.std(window, axis=0, ddof=0),
                np.ptp(window, axis=0),
                window[-1] - window[0],
            )
        )
    result = np.concatenate(output)
    if result.shape != (len(FEATURE_NAMES),) or not np.isfinite(result).all():
        raise H10MotionModelError(f"인과 특징 크기 오류: {result.shape}")
    return result


def _safe_corr(left: np.ndarray, right: np.ndarray) -> float:
    """상수에 가까운 짧은 창에서도 NaN 없이 상관계수를 계산한다."""
    if len(left) < 3:
        return 0.0
    left_centered = left - np.mean(left)
    right_centered = right - np.mean(right)
    denominator = float(
        np.sqrt(np.sum(left_centered * left_centered) * np.sum(right_centered * right_centered))
    )
    if denominator <= 1.0e-12:
        return 0.0
    return float(np.clip(np.sum(left_centered * right_centered) / denominator, -1.0, 1.0))


def continuous_feature_vector(history: np.ndarray) -> np.ndarray:
    """연속 동작용 특징.

    기존 다중 시간창 특징에 점프의 양측 동상(in-phase) 움직임을 직접 표현하는
    상관·각도차·가속도 특징을 붙인다. 모든 값은 현재 시점 이전 이력만 사용한다.
    """
    values = np.asarray(history, dtype=np.float64)
    if values.shape != (MAX_HISTORY_SAMPLES, len(SIGNAL_NAMES)):
        raise H10MotionModelError(f"인과 버퍼 크기 오류: {values.shape}")
    output = [causal_feature_vector(values)[DECISION_FEATURE_INDICES]]
    derived: list[float] = []
    for window_ms in CONTINUOUS_DERIVED_WINDOWS_MS:
        window = values[-window_ms:]
        left = window[:, 2]
        right = window[:, 3]
        mean_angle = window[:, 4]
        diff_angle = window[:, 5]
        acc_norm = window[:, 9]
        left_velocity = np.diff(left)
        right_velocity = np.diff(right)
        same_direction = (
            float(np.mean(np.sign(left_velocity) * np.sign(right_velocity)))
            if len(left_velocity)
            else 0.0
        )
        derived.extend(
            (
                _safe_corr(left, right),
                _safe_corr(left_velocity, right_velocity),
                float(np.ptp(mean_angle)),
                float(np.ptp(diff_angle)),
                float(np.ptp(acc_norm)),
                float(np.std(acc_norm, ddof=0)),
                float(np.min(acc_norm)),
                float(np.max(acc_norm)),
                same_direction,
            )
        )
    output.append(np.asarray(derived, dtype=np.float64))
    result = np.concatenate(output)
    if result.shape != (len(CONTINUOUS_FEATURE_NAMES),) or not np.isfinite(result).all():
        raise H10MotionModelError(f"연속 동작 특징 크기 오류: {result.shape}")
    return result


def causal_feature_rows(
    frame: pd.DataFrame, calibration: Mapping[str, Any]
) -> tuple[np.ndarray, np.ndarray]:
    """동작 시작 전 중립 신호를 가정하고 50 ms마다 인과 특징을 만든다."""
    signals = transform_sensor_frame(frame, calibration)
    padded = np.vstack(
        (
            np.zeros((MAX_HISTORY_SAMPLES, len(SIGNAL_NAMES)), dtype=np.float64),
            signals,
        )
    )
    end_samples = np.arange(HOP_SAMPLES, len(frame) + 1, HOP_SAMPLES)
    features = np.vstack(
        [
            causal_feature_vector(
                padded[
                    MAX_HISTORY_SAMPLES + int(end) - MAX_HISTORY_SAMPLES :
                    MAX_HISTORY_SAMPLES + int(end)
                ]
            )
            for end in end_samples
        ]
    )
    return end_samples, features


def continuous_feature_rows(
    frame: pd.DataFrame, calibration: Mapping[str, Any]
) -> tuple[np.ndarray, np.ndarray]:
    """원본 파일 전체의 history를 유지하며 50 ms 연속 특징을 만든다."""
    signals = transform_sensor_frame(frame, calibration)
    padded = np.vstack(
        (
            np.zeros((MAX_HISTORY_SAMPLES, len(SIGNAL_NAMES)), dtype=np.float64),
            signals,
        )
    )
    end_samples = np.arange(HOP_SAMPLES, len(frame) + 1, HOP_SAMPLES)
    features = np.vstack(
        [
            continuous_feature_vector(
                padded[
                    MAX_HISTORY_SAMPLES + int(end) - MAX_HISTORY_SAMPLES :
                    MAX_HISTORY_SAMPLES + int(end)
                ]
            )
            for end in end_samples
        ]
    )
    return end_samples, features


def linear_probabilities(
    standardized_features: np.ndarray, linear_model: Mapping[str, Any]
) -> np.ndarray:
    weights = np.asarray(linear_model["weights"], dtype=np.float64)
    bias = np.asarray(linear_model["bias"], dtype=np.float64)
    temperature = float(linear_model.get("temperature", 1.0))
    if not math.isfinite(temperature) or temperature <= 0.0:
        raise H10MotionModelError("선형 모델 temperature가 유효하지 않습니다.")
    logits = np.asarray(standardized_features, dtype=np.float64) @ weights + bias
    return softmax(logits / temperature)


def tree_ensemble_probabilities(
    features: np.ndarray, forest: Mapping[str, Any]
) -> np.ndarray:
    """JSON으로 저장한 ExtraTrees/RandomForest 분류기를 순수 NumPy로 실행한다."""
    values = np.asarray(features, dtype=np.float64)
    if values.ndim != 1:
        raise H10MotionModelError(f"트리 입력은 1차원이어야 합니다: {values.shape}")
    expected = int(forest.get("n_features", -1))
    if values.shape != (expected,):
        raise H10MotionModelError(
            f"트리 특징 수가 다릅니다: input={len(values)}, model={expected}"
        )
    classes = tuple(forest.get("classes", ()))
    if not classes:
        raise H10MotionModelError("트리 분류기의 classes가 비었습니다.")
    total = np.zeros(len(classes), dtype=np.float64)
    trees = forest.get("trees")
    if not isinstance(trees, list) or not trees:
        raise H10MotionModelError("트리 분류기의 trees가 비었습니다.")
    for tree in trees:
        feature = tree["feature"]
        threshold = tree["threshold"]
        left = tree["children_left"]
        right = tree["children_right"]
        node = 0
        while int(left[node]) >= 0:
            feature_index = int(feature[node])
            node = (
                int(left[node])
                if values[feature_index] <= float(threshold[node])
                else int(right[node])
            )
        leaf = np.asarray(tree["leaf_probability"][node], dtype=np.float64)
        if leaf.shape != total.shape:
            raise H10MotionModelError("트리 leaf 확률 크기가 classes와 다릅니다.")
        total += leaf
    total /= float(len(trees))
    denominator = float(np.sum(total))
    if denominator <= 0.0 or not np.isfinite(total).all():
        raise H10MotionModelError("트리 분류기 확률이 유효하지 않습니다.")
    return total / denominator


class FullCrouchGate:
    """양쪽 고관절이 학습된 최대 굴곡 plateau일 때만 true를 낸다."""

    def __init__(self, config: Mapping[str, Any]):
        self.config = dict(config)
        self.reset()

    def reset(self) -> None:
        self.active = False
        self.enter_counter = 0

    def update(self, history: np.ndarray, *, disabled: bool = False) -> dict[str, Any]:
        if disabled:
            self.reset()
        current = np.mean(history[-50:, [2, 3]], axis=0)
        prior = np.mean(history[-150:-100, [2, 3]], axis=0)
        velocity = (current - prior) / 0.1
        direction = np.asarray(self.config["direction"], dtype=np.float64)
        full_span = np.asarray(self.config["full_flexion_span_deg"], dtype=np.float64)
        progress = direction * current / full_span
        enter_fraction = float(self.config["enter_fraction"])
        max_velocity = float(self.config["plateau_max_velocity_dps"])
        enter_condition = bool(
            np.all(progress >= enter_fraction)
            and np.all(np.abs(velocity) <= max_velocity)
        )
        released = False
        if self.active:
            # 진입 조건을 벗어난 첫 tick에 바로 해제한다. 사용자가 요구한 계약은
            # hysteresis crouch latch가 아니라 "완전 굴곡 plateau일 때만 Shift"다.
            if not enter_condition:
                self.active = False
                self.enter_counter = 0
                released = True
        elif not disabled:
            self.enter_counter = self.enter_counter + 1 if enter_condition else 0
            if self.enter_counter >= int(self.config["enter_confirm_ticks"]):
                self.active = True
        return {
            "active": bool(self.active),
            "released": released,
            "enter_condition": enter_condition,
            "left_progress": float(progress[0]),
            "right_progress": float(progress[1]),
            "left_velocity_dps": float(velocity[0]),
            "right_velocity_dps": float(velocity[1]),
        }


class JumpEventGate:
    """점프 사건 확률을 한 jump당 한 번의 rising edge로 바꾼다."""

    def __init__(self, config: Mapping[str, Any]):
        self.config = dict(config)
        enter_threshold = float(self.config["enter_threshold"])
        rearm_threshold = float(self.config["rearm_threshold"])
        if not 0.0 <= rearm_threshold < enter_threshold <= 1.0:
            raise H10MotionModelError(
                "jump enter threshold는 rearm threshold보다 커야 합니다: "
                f"enter={enter_threshold}, rearm={rearm_threshold}"
            )
        self.reset()

    def reset(self) -> None:
        self.armed = True
        self.low_ticks = 0
        self.refractory_ticks = 0

    def update(self, probability: float, *, blocked: bool = False) -> bool:
        probability = float(probability)
        if probability < float(self.config["rearm_threshold"]):
            self.low_ticks += 1
        else:
            self.low_ticks = 0
        if self.refractory_ticks > 0:
            self.refractory_ticks -= 1
        if (
            not self.armed
            and self.refractory_ticks == 0
            and self.low_ticks >= int(self.config["rearm_confirm_ticks"])
        ):
            self.armed = True
        if blocked or not self.armed:
            return False
        if probability < float(self.config["enter_threshold"]):
            return False
        self.armed = False
        self.low_ticks = 0
        self.refractory_ticks = max(
            1,
            int(math.ceil(float(self.config["refractory_ms"]) / HOP_MS)),
        )
        return True


class ContinuousMotionDecisionStateMachine:
    """full-crouch와 jump event가 분리된 연속 50 ms 상태기계."""

    def __init__(self, config: Mapping[str, Any]):
        self.config = dict(config)
        self.reset()

    def reset(self) -> None:
        self.state = "stand"
        self.tick = 0
        self.state_ticks = 0
        self.walk_counter = 0
        self.low_evidence_ticks = 0
        self.was_crouched = False
        self.post_crouch_block_ticks = 0
        self.walk_intensity = 0.0
        self.jump_intensity = 0.0

    def _change(self, state: str) -> bool:
        if state == self.state:
            return False
        self.state = state
        self.state_ticks = 0
        self.low_evidence_ticks = 0
        return True

    def update(
        self,
        probabilities: Mapping[str, float],
        *,
        jump_event: bool,
        crouch_active: bool,
        crouch_released: bool,
        walk_intensity: float,
        jump_intensity: float,
        turn_rate_dps: float,
        walk_entry_blocked: bool,
        jump_event_probability: float,
    ) -> dict[str, Any]:
        self.tick += 1
        self.state_ticks += 1
        p_walk = float(probabilities.get("walk", 0.0))
        p_jump = float(probabilities.get("jump", 0.0))
        raw_state = max(probabilities, key=probabilities.__getitem__)
        changed = False

        if crouch_active:
            self.was_crouched = True
            self.post_crouch_block_ticks = int(
                self.config.get("post_crouch_jump_block_ticks", 0)
            )
            self.walk_counter = 0
            changed = self._change("sit")
            jump_event = False
        else:
            if self.was_crouched or crouch_released:
                self.was_crouched = False
                self.post_crouch_block_ticks = int(
                    self.config.get("post_crouch_jump_block_ticks", 0)
                )
                changed = self._change("stand")
            elif self.post_crouch_block_ticks > 0:
                self.post_crouch_block_ticks -= 1

            jump_started = bool(jump_event and self.post_crouch_block_ticks == 0)
            if jump_started:
                self.jump_intensity = float(np.clip(jump_intensity, 0.0, 1.0))
                self.low_evidence_ticks = 0
                changed = self._change("jump") or changed
            elif self.state == "jump":
                self.jump_intensity = max(
                    float(np.clip(jump_intensity, 0.0, 1.0)),
                    0.8 * self.jump_intensity,
                )
                self.low_evidence_ticks = (
                    self.low_evidence_ticks + 1
                    if p_jump < float(self.config["jump_exit_threshold"])
                    else 0
                )
                if (
                    self.state_ticks >= int(self.config["jump_min_hold_ticks"])
                    and self.low_evidence_ticks
                    >= int(self.config["jump_exit_confirm_ticks"])
                ):
                    next_state = (
                        "walk"
                        if p_walk >= float(self.config["walk_enter_threshold"])
                        and not walk_entry_blocked
                        else "stand"
                    )
                    changed = self._change(next_state) or changed
            elif self.post_crouch_block_ticks > 0:
                changed = self._change("stand") or changed
            elif self.state == "walk":
                self.walk_intensity = 0.8 * self.walk_intensity + 0.2 * float(
                    np.clip(walk_intensity, 0.0, 1.0)
                )
                self.low_evidence_ticks = (
                    self.low_evidence_ticks + 1
                    if p_walk < float(self.config["walk_exit_threshold"])
                    else 0
                )
                if self.low_evidence_ticks >= int(
                    self.config["walk_exit_confirm_ticks"]
                ):
                    changed = self._change("stand") or changed
            else:
                self.walk_counter = (
                    self.walk_counter + 1
                    if not walk_entry_blocked
                    and p_walk >= float(self.config["walk_enter_threshold"])
                    else 0
                )
                if self.walk_counter >= int(self.config["walk_confirm_ticks"]):
                    self.walk_intensity = float(np.clip(walk_intensity, 0.0, 1.0))
                    changed = self._change("walk") or changed
                else:
                    changed = self._change("stand") or changed

        jump_started = bool(
            jump_event
            and not crouch_active
            and self.post_crouch_block_ticks == 0
        )
        key_command = (
            "SHIFT"
            if self.state == "sit"
            else ("W" if self.state == "walk" else ("SPACE" if jump_started else "NONE"))
        )
        return {
            "tick": self.tick,
            "time_ms": self.tick * HOP_MS,
            "state": self.state,
            "raw_state": raw_state,
            "state_changed": changed,
            "jump_started": jump_started,
            "jump_onset_probability": float(jump_event_probability),
            "jump_trigger_source": "continuous_event" if jump_started else "none",
            "walk_entry_blocked": bool(walk_entry_blocked),
            "key_command": key_command,
            "confidence": float(max(probabilities.values())),
            "walk_intensity": self.walk_intensity if self.state == "walk" else None,
            "jump_intensity": self.jump_intensity if self.state == "jump" else None,
            "turn_rate_dps": float(turn_rate_dps),
            "turn_direction": (
                "cw" if turn_rate_dps < 0.0 else ("ccw" if turn_rate_dps > 0.0 else "none")
            ),
            "crouch_active": bool(crouch_active),
        }


class MotionDecisionStateMachine:
    """조기 판정을 동작 지속 상태로 바꾸는 50 ms 래치."""

    def __init__(self, config: Mapping[str, Any]):
        self.config = dict(config)
        self.reset()

    def reset(self) -> None:
        self.state = "stand"
        self.tick = 0
        self.state_ticks = 0
        self.low_evidence_ticks = 0
        self.jump_counter = 0
        self.walk_counter = 0
        self.sit_counter = 0
        self.up_counter = 0
        self.crouched = False
        self.jump_hold_ticks = 0
        self.walk_intensity = 0.0
        self.jump_intensity = 0.0

    def _change(self, state: str) -> bool:
        if state == self.state:
            return False
        self.state = state
        self.state_ticks = 0
        self.low_evidence_ticks = 0
        return True

    def update(
        self,
        probabilities: Mapping[str, float],
        *,
        walk_intensity: float,
        jump_intensity: float,
        turn_rate_dps: float,
        jump_onset_probability: float | None = None,
        walk_entry_blocked: bool = False,
    ) -> dict[str, Any]:
        cfg = self.config
        self.tick += 1
        self.state_ticks += 1
        p_jump = float(probabilities["jump"])
        p_jump_onset = (
            p_jump
            if jump_onset_probability is None
            else float(jump_onset_probability)
        )
        p_walk = float(probabilities["walk"])
        p_sit = float(probabilities["sit"])
        p_up = float(probabilities["stand_up"])
        jump_posture_block = float(
            cfg.get("jump_posture_block_threshold", 1.1)
        )
        jump_allowed = max(p_sit, p_up) < jump_posture_block
        raw_state = max(probabilities, key=probabilities.__getitem__)
        raw_jump_fallback = bool(cfg.get("jump_raw_state_fallback", False)) and (
            raw_state == "jump"
        )

        self.jump_counter = (
            self.jump_counter + 1
            if jump_allowed
            and (
                p_jump_onset >= float(cfg["jump_enter_threshold"])
                or raw_jump_fallback
            )
            else 0
        )
        self.walk_counter = (
            self.walk_counter + 1
            if not walk_entry_blocked
            and p_walk >= float(cfg["walk_enter_threshold"])
            else 0
        )
        self.sit_counter = (
            self.sit_counter + 1
            if p_sit >= float(cfg["sit_enter_threshold"])
            else 0
        )
        self.up_counter = (
            self.up_counter + 1
            if p_up >= float(cfg["stand_up_enter_threshold"])
            else 0
        )

        changed = False
        jump_started = False
        if self.state == "jump":
            self.jump_intensity = max(
                self.jump_intensity,
                0.8 * self.jump_intensity + 0.2 * float(jump_intensity),
            )
            self.low_evidence_ticks = (
                self.low_evidence_ticks + 1
                if p_jump < float(cfg["jump_exit_threshold"])
                else 0
            )
            release = (
                self.state_ticks >= self.jump_hold_ticks
                and self.low_evidence_ticks >= int(cfg["jump_exit_confirm_ticks"])
            ) or self.state_ticks >= int(cfg["jump_max_hold_ticks"])
            if release:
                next_state = (
                    "walk"
                    if p_walk >= float(cfg["walk_enter_threshold"])
                    else ("sit" if self.crouched else "stand")
                )
                changed = self._change(next_state)
        elif self.jump_counter >= int(cfg["jump_confirm_ticks"]):
            self.jump_intensity = float(np.clip(jump_intensity, 0.0, 1.0))
            hold_ms = float(cfg["jump_hold_base_ms"]) + float(
                cfg["jump_hold_intensity_ms"]
            ) * self.jump_intensity
            self.jump_hold_ticks = max(1, int(math.ceil(hold_ms / HOP_MS)))
            changed = self._change("jump")
            jump_started = True
        elif self.crouched:
            if self.up_counter >= int(cfg["stand_up_confirm_ticks"]):
                self.crouched = False
                changed = self._change("stand_up")
            else:
                changed = self._change("sit")
        elif self.state == "stand_up":
            self.low_evidence_ticks = (
                self.low_evidence_ticks + 1
                if p_up < float(cfg["stand_up_exit_threshold"])
                else 0
            )
            if (
                self.state_ticks >= int(cfg["stand_up_min_hold_ticks"])
                and self.low_evidence_ticks >= int(cfg["stand_up_exit_confirm_ticks"])
            ):
                changed = self._change("stand")
        elif self.sit_counter >= int(cfg["sit_confirm_ticks"]):
            self.crouched = True
            changed = self._change("sit")
        elif self.up_counter >= int(cfg["stand_up_confirm_ticks"]):
            changed = self._change("stand_up")
        elif self.state == "walk":
            self.walk_intensity = 0.8 * self.walk_intensity + 0.2 * float(
                walk_intensity
            )
            self.low_evidence_ticks = (
                self.low_evidence_ticks + 1
                if p_walk < float(cfg["walk_exit_threshold"])
                else 0
            )
            if (
                self.state_ticks >= int(cfg["walk_min_hold_ticks"])
                and self.low_evidence_ticks >= int(cfg["walk_exit_confirm_ticks"])
            ):
                changed = self._change("stand")
        elif self.walk_counter >= int(cfg["walk_confirm_ticks"]):
            self.walk_intensity = float(np.clip(walk_intensity, 0.0, 1.0))
            changed = self._change("walk")
        else:
            changed = self._change("stand")

        if self.state == "walk":
            key_command = "W"
        elif self.state == "jump":
            key_command = "SPACE"
        elif self.state == "sit":
            key_command = "SHIFT"
        else:
            key_command = "NONE"

        return {
            "tick": self.tick,
            "time_ms": self.tick * HOP_MS,
            "state": self.state,
            "raw_state": raw_state,
            "state_changed": changed,
            "jump_started": jump_started,
            "jump_onset_probability": p_jump_onset,
            "jump_trigger_source": (
                "onset"
                if p_jump_onset >= float(cfg["jump_enter_threshold"])
                else ("raw_state" if raw_jump_fallback else "none")
            ),
            "walk_entry_blocked": bool(walk_entry_blocked),
            "key_command": key_command,
            "confidence": float(max(probabilities.values())),
            "walk_intensity": self.walk_intensity if self.state == "walk" else None,
            "jump_intensity": self.jump_intensity if self.state == "jump" else None,
            "turn_rate_dps": float(turn_rate_dps),
            "turn_direction": (
                "cw" if turn_rate_dps < 0.0 else ("ccw" if turn_rate_dps > 0.0 else "none")
            ),
        }


_STATE_MACHINE_TICK_FIELDS = (
    "jump_confirm_ticks",
    "jump_exit_confirm_ticks",
    "jump_max_hold_ticks",
    "walk_confirm_ticks",
    "walk_exit_confirm_ticks",
    "walk_min_hold_ticks",
    "sit_confirm_ticks",
    "stand_up_confirm_ticks",
    "stand_up_exit_confirm_ticks",
    "stand_up_min_hold_ticks",
)


def _retime_tick_config(
    config: Mapping[str, Any], source_period_ms: int, target_period_ms: int
) -> dict[str, Any]:
    """20 ms 모델의 tick 기반 유지시간을 50 ms 런타임에 맞춰 보존한다."""
    result = dict(config)
    if source_period_ms <= 0 or target_period_ms <= 0:
        raise H10MotionModelError("판정 주기는 양수여야 합니다.")
    for field in _STATE_MACHINE_TICK_FIELDS:
        if field in result:
            duration_ms = int(result[field]) * source_period_ms
            result[field] = max(1, int(math.ceil(duration_ms / target_period_ms)))
    return result


class TurnRateGate:
    """지속되는 signed yaw만 회전속도로 통과시키는 짧은 확인 게이트."""

    def __init__(self, config: Mapping[str, Any]):
        self.config = dict(config)
        self.reset()

    def reset(self) -> None:
        self.candidate_sign = 0
        self.candidate_ticks = 0

    def update(self, raw_rate_dps: float) -> float:
        threshold = float(self.config["deadzone_dps"])
        sign = -1 if raw_rate_dps < 0.0 else (1 if raw_rate_dps > 0.0 else 0)
        if abs(raw_rate_dps) < threshold:
            self.candidate_sign = 0
            self.candidate_ticks = 0
            return 0.0
        if sign == self.candidate_sign:
            self.candidate_ticks += 1
        else:
            self.candidate_sign = sign
            self.candidate_ticks = 1
        if self.candidate_ticks < int(self.config["confirm_ticks"]):
            return 0.0
        return float(raw_rate_dps)


def load_stream_model(path: Path) -> dict[str, Any]:
    payload = json.loads(path.read_text(encoding="utf-8"))
    schema = payload.get("schema_version")
    if schema not in {MODEL_SCHEMA_VERSION, LEGACY_MODEL_SCHEMA_VERSION}:
        raise H10MotionModelError("지원하지 않는 스트리밍 모델 schema입니다.")
    if payload.get("decision_mode") == "continuous_tree_v1":
        if tuple(payload.get("feature_names", ())) != CONTINUOUS_FEATURE_NAMES:
            raise H10MotionModelError("연속 트리 모델 특징 계약이 다릅니다.")
        if tuple(payload.get("decision_feature_names", ())) != CONTINUOUS_FEATURE_NAMES:
            raise H10MotionModelError("연속 트리 모델 판정 특징 계약이 다릅니다.")
    else:
        if tuple(payload.get("feature_names", ())) != FEATURE_NAMES:
            raise H10MotionModelError("스트리밍 모델 특징 계약이 다릅니다.")
        if tuple(payload.get("decision_feature_names", ())) != DECISION_FEATURE_NAMES:
            raise H10MotionModelError("스트리밍 모델 판정 특징 계약이 다릅니다.")
    return payload


class CausalMotionRuntime:
    """50개 센서 행을 받을 때마다 즉시 한 판정 결과를 반환한다."""

    def __init__(
        self,
        model: Mapping[str, Any],
        *,
        disable_crouch: bool = False,
        jump_enter_threshold: float | None = None,
    ):
        self.model = dict(model)
        self.disable_crouch = bool(disable_crouch)
        self.continuous_mode = model.get("decision_mode") == "continuous_tree_v1"
        source_period_ms = int(model.get("output_period_ms", HOP_MS))
        if self.continuous_mode:
            state_machine_config = _retime_tick_config(
                model["state_machine"], source_period_ms, HOP_MS
            )
            jump_event_config = dict(model["jump_event_gate"])
            if jump_enter_threshold is not None:
                threshold = float(jump_enter_threshold)
                if not 0.0 <= threshold <= 1.0:
                    raise H10MotionModelError("jump_enter_threshold는 0~1이어야 합니다.")
                jump_event_config["enter_threshold"] = threshold
            enter_threshold = float(jump_event_config["enter_threshold"])
            rearm_threshold = float(jump_event_config["rearm_threshold"])
            if not rearm_threshold < enter_threshold <= 1.0:
                raise H10MotionModelError(
                    "jump enter threshold는 rearm threshold보다 커야 합니다: "
                    f"enter={enter_threshold}, rearm={rearm_threshold}"
                )
            self.state_machine = ContinuousMotionDecisionStateMachine(
                state_machine_config
            )
            self.jump_event_gate = JumpEventGate(jump_event_config)
            self.crouch_gate = FullCrouchGate(model["crouch_model"])
        else:
            state_machine_config = _retime_tick_config(
                model["state_machine"], source_period_ms, HOP_MS
            )
            if jump_enter_threshold is not None:
                threshold = float(jump_enter_threshold)
                if not 0.0 <= threshold <= 1.0:
                    raise H10MotionModelError("jump_enter_threshold는 0~1이어야 합니다.")
                state_machine_config["jump_enter_threshold"] = threshold
            if self.disable_crouch:
                # 구형 모델에서만 raw winner fallback을 유지한다.
                state_machine_config["jump_raw_state_fallback"] = True
            self.state_machine = MotionDecisionStateMachine(state_machine_config)
        turn_config = dict(model["turn_model"])
        if "confirm_ticks" in turn_config:
            confirm_duration_ms = int(turn_config["confirm_ticks"]) * source_period_ms
            turn_config["confirm_ticks"] = max(
                1, int(math.ceil(confirm_duration_ms / HOP_MS))
            )
        self.turn_gate = TurnRateGate(turn_config)
        self.reset()

    def reset(self) -> None:
        self.history = np.zeros(
            (MAX_HISTORY_SAMPLES, len(SIGNAL_NAMES)), dtype=np.float64
        )
        self.state_machine.reset()
        if self.continuous_mode:
            self.jump_event_gate.reset()
            self.crouch_gate.reset()
        self.turn_gate.reset()

    def update_50ms(self, frame: pd.DataFrame) -> dict[str, Any]:
        if len(frame) != HOP_SAMPLES:
            raise H10MotionModelError(
                f"{HOP_MS} ms 판정에는 정확히 {HOP_SAMPLES}개 행이 필요합니다: {len(frame)}"
            )
        signals = transform_sensor_frame(frame, self.model["calibration"])
        self.history = np.vstack((self.history[HOP_SAMPLES:], signals))
        if self.continuous_mode:
            return self._update_continuous()
        features = causal_feature_vector(self.history)[DECISION_FEATURE_INDICES]
        mean = np.asarray(self.model["scaler"]["mean"], dtype=np.float64)
        scale = np.asarray(self.model["scaler"]["scale"], dtype=np.float64)
        standardized = (features - mean) / scale
        state_probability = linear_probabilities(
            standardized, self.model["state_model"]
        )
        jump_onset_probability = float(
            linear_probabilities(standardized, self.model["jump_onset_model"])[1]
        )
        probabilities = {
            state: float(state_probability[index])
            for index, state in enumerate(STATE_CLASSES)
        }
        if self.disable_crouch:
            # Crouch를 쓰지 않는 게임에서 sit/stand_up이 점프 예비
            # 동작의 확률과 jump posture gate를 동시에 누르지 않게 한다.
            probabilities = suppress_crouch_probabilities(probabilities)
        intensities: dict[str, float] = {}
        for family in ("walk", "jump"):
            contract = self.model["intensity_models"][family]
            probability = linear_probabilities(standardized, contract)
            intensities[family] = float(
                probability @ np.asarray(contract["level_values"], dtype=np.float64)
            )
        turn_window = self.history[-int(self.model["turn_model"]["window_ms"]):, 11]
        turn_rate = self.turn_gate.update(float(np.mean(turn_window)))
        guard_window_ms = int(
            self.model["turn_model"].get("walk_entry_guard_window_ms", 80)
        )
        guard_rate = float(np.mean(self.history[-guard_window_ms:, 11]))
        walk_entry_blocked = abs(guard_rate) >= float(
            self.model["turn_model"].get("walk_entry_guard_dps", 30.0)
        )
        result = self.state_machine.update(
            probabilities,
            walk_intensity=intensities["walk"],
            jump_intensity=intensities["jump"],
            turn_rate_dps=turn_rate,
            jump_onset_probability=jump_onset_probability,
            walk_entry_blocked=walk_entry_blocked,
        )
        result["probabilities"] = probabilities
        return result

    def _update_continuous(self) -> dict[str, Any]:
        features = continuous_feature_vector(self.history)
        state_probability = tree_ensemble_probabilities(
            features, self.model["state_model"]
        )
        state_classes = tuple(self.model["state_model"]["classes"])
        probabilities = {
            str(state): float(state_probability[index])
            for index, state in enumerate(state_classes)
        }
        for state in ("stand", "walk", "jump"):
            probabilities.setdefault(state, 0.0)

        jump_probability_vector = tree_ensemble_probabilities(
            features, self.model["jump_event_model"]
        )
        jump_classes = tuple(self.model["jump_event_model"]["classes"])
        positive_class = str(
            self.model["jump_event_model"].get("positive_class", "jump_event")
        )
        try:
            jump_index = [str(value) for value in jump_classes].index(positive_class)
        except ValueError as exc:
            raise H10MotionModelError("jump_event positive_class가 classes에 없습니다.") from exc
        jump_event_probability = float(jump_probability_vector[jump_index])

        crouch = self.crouch_gate.update(
            self.history, disabled=self.disable_crouch
        )
        posture_blocked = bool(
            crouch["active"]
            or min(crouch["left_progress"], crouch["right_progress"])
            >= float(self.model["crouch_model"]["release_fraction"])
        )
        jump_event = self.jump_event_gate.update(
            jump_event_probability,
            blocked=posture_blocked,
        )

        intensities: dict[str, float] = {}
        for family in ("walk", "jump"):
            contract = self.model["intensity_models"][family]
            level_probability = tree_ensemble_probabilities(features, contract)
            intensities[family] = float(
                level_probability
                @ np.asarray(contract["level_values"], dtype=np.float64)
            )

        turn_window = self.history[-int(self.model["turn_model"]["window_ms"]):, 11]
        turn_rate = self.turn_gate.update(float(np.mean(turn_window)))
        guard_window_ms = int(
            self.model["turn_model"].get("walk_entry_guard_window_ms", 80)
        )
        guard_rate = float(np.mean(self.history[-guard_window_ms:, 11]))
        walk_entry_blocked = abs(guard_rate) >= float(
            self.model["turn_model"].get("walk_entry_guard_dps", 30.0)
        )
        result = self.state_machine.update(
            probabilities,
            jump_event=jump_event,
            crouch_active=bool(crouch["active"]),
            crouch_released=bool(crouch["released"]),
            walk_intensity=intensities["walk"],
            jump_intensity=intensities["jump"],
            turn_rate_dps=turn_rate,
            walk_entry_blocked=walk_entry_blocked,
            jump_event_probability=jump_event_probability,
        )
        result["probabilities"] = probabilities
        result["crouch_progress_left"] = crouch["left_progress"]
        result["crouch_progress_right"] = crouch["right_progress"]
        result["crouch_velocity_left_dps"] = crouch["left_velocity_dps"]
        result["crouch_velocity_right_dps"] = crouch["right_velocity_dps"]
        return result

    def update_20ms(self, frame: pd.DataFrame) -> dict[str, Any]:
        """이전 호출명 호환용 별칭. 현재 계약은 50행/50 ms다."""
        return self.update_50ms(frame)


def suppress_crouch_probabilities(
    probabilities: Mapping[str, float],
) -> dict[str, float]:
    """sit/stand_up을 제외하고 stand/walk/jump 확률을 다시 정규화한다."""
    enabled_states = ("stand", "walk", "jump")
    total = sum(max(float(probabilities.get(state, 0.0)), 0.0) for state in enabled_states)
    if total <= 0.0:
        return {
            "stand": 1.0,
            "walk": 0.0,
            "jump": 0.0,
            "sit": 0.0,
            "stand_up": 0.0,
        }
    result = {
        state: max(float(probabilities.get(state, 0.0)), 0.0) / total
        for state in enabled_states
    }
    result["sit"] = 0.0
    result["stand_up"] = 0.0
    return result


def neutral_frame(calibration: Mapping[str, Any], samples: int) -> pd.DataFrame:
    """상태 해제 시험용 모델 중립 센서 프레임을 만든다."""
    neutral = calibration["neutral"]
    emg = calibration["emg"]["channels"]
    data = {
        "emg_thigh": np.full(samples, emg["emg_thigh"]["rest_median"]),
        "emg_calf": np.full(samples, emg["emg_calf"]["rest_median"]),
        "l_angle": np.full(samples, neutral["l_angle"]),
        "r_angle": np.full(samples, neutral["r_angle"]),
        "imu_x": np.full(samples, neutral["imu_x"]),
        "imu_y": np.full(samples, neutral["imu_y"]),
        "imu_z": np.full(samples, neutral["imu_z"]),
        "gyro_x": np.full(samples, neutral["gyro_x"]),
        "gyro_y": np.full(samples, neutral["gyro_y"]),
        "gyro_z": np.full(samples, neutral["gyro_z"]),
    }
    return pd.DataFrame(data, columns=REQUIRED_SENSOR_COLUMNS)
