#!/usr/bin/env python3
"""H10 한 동작 CSV용 5상태 + 강도 + 독립 회전 판정 공통 코드.

입력은 이미 한 동작으로 잘린 약 1 kHz CSV다. EMG는 학습 데이터에서 구한 정지
기준선과 활성 P95로 세션 상대 정규화해 판정에 넣고, 토크는 특징에서 제외한다.
"""
from __future__ import annotations

import json
import math
from pathlib import Path
from typing import Any, Mapping

import numpy as np
import pandas as pd


MODEL_SCHEMA_VERSION = "h10-motion-composite-emg-softmax-3.0"
SAMPLE_RATE_HZ = 1000.0
STATE_CLASSES = ("stand", "walk", "jump", "sit", "stand_up")
REQUIRED_SENSOR_COLUMNS = (
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
)


class H10MotionModelError(RuntimeError):
    """입력 또는 모델 계약이 맞지 않을 때 발생한다."""


def _feature_names() -> tuple[str, ...]:
    names = ["duration_s", "cadence_hz"]
    for channel in ("l_angle", "r_angle", "mean_angle", "diff_angle"):
        names.extend(
            (
                f"{channel}_mean",
                f"{channel}_std",
                f"{channel}_range",
                f"{channel}_net",
                f"{channel}_abs_velocity_mean",
            )
        )
    names.append("angle_corr")
    for channel in ("emg_thigh", "emg_calf"):
        names.extend(
            (
                f"{channel}_scaled_mean",
                f"{channel}_scaled_std",
                f"{channel}_scaled_p50",
                f"{channel}_scaled_p95",
                f"{channel}_scaled_max",
                f"{channel}_scaled_active_fraction",
            )
        )
    names.extend(("emg_scaled_total_mean", "emg_scaled_balance_mean"))
    for channel in ("acc_norm", "gyro_norm"):
        names.extend(
            (
                f"{channel}_mean",
                f"{channel}_std",
                f"{channel}_range",
                f"{channel}_p95",
            )
        )
    for channel in ("imu_x", "imu_y", "imu_z", "gyro_x", "gyro_y", "gyro_z"):
        names.extend(
            (f"{channel}_mean", f"{channel}_std", f"{channel}_range")
        )
    names.append("gyro_y_integral")
    return tuple(names)


FEATURE_NAMES = _feature_names()
# 회전은 독립 출력이므로 기본 동작·강도 판정에서 yaw 축과 yaw가 지배하는 gyro norm을 뺀다.
DECISION_FEATURE_NAMES = tuple(
    name
    for name in FEATURE_NAMES
    if not name.startswith("gyro_y_") and not name.startswith("gyro_norm_")
)


def validate_sensor_frame(frame: pd.DataFrame) -> None:
    missing = sorted(set(REQUIRED_SENSOR_COLUMNS) - set(frame.columns))
    if missing:
        raise H10MotionModelError(f"필수 센서 열 누락: {missing}")
    if len(frame) < 50:
        raise H10MotionModelError("동작 조각이 50 ms보다 짧습니다.")
    values = frame.loc[:, REQUIRED_SENSOR_COLUMNS].to_numpy(dtype=np.float64)
    if not np.isfinite(values).all():
        raise H10MotionModelError("센서 열에 NaN 또는 무한대가 있습니다.")


def fit_emg_scaling(
    frames: list[pd.DataFrame], rest_frames: list[pd.DataFrame]
) -> dict[str, Any]:
    """학습 행만으로 채널별 정지 기준선과 활성 P95를 계산한다."""
    if not frames or not rest_frames:
        raise H10MotionModelError("EMG scaling에 전체 학습 frame과 정지 frame이 필요합니다.")
    channels: dict[str, dict[str, float]] = {}
    for channel in ("emg_thigh", "emg_calf"):
        rest = np.concatenate(
            [frame[channel].to_numpy(dtype=np.float64) for frame in rest_frames]
        )
        baseline = float(np.median(rest))
        activation = np.concatenate(
            [
                np.maximum(frame[channel].to_numpy(dtype=np.float64) - baseline, 0.0)
                for frame in frames
            ]
        )
        scale = float(np.quantile(activation, 0.95))
        if not math.isfinite(scale) or scale < 1.0e-6:
            raise H10MotionModelError(f"EMG 활성 P95가 너무 작습니다: {channel}={scale}")
        channels[channel] = {
            "rest_median": baseline,
            "activation_p95": scale,
        }
    return {
        "method": "rest_median_subtract_train_activation_p95_clip",
        "clip_min": 0.0,
        "clip_max": 1.5,
        "channels": channels,
        "authority": "session_relative_not_percent_mvc",
    }


def _scaled_emg(frame: pd.DataFrame, scaling: Mapping[str, Any]) -> dict[str, np.ndarray]:
    if scaling.get("method") != "rest_median_subtract_train_activation_p95_clip":
        raise H10MotionModelError("지원하지 않는 EMG scaling 계약입니다.")
    result: dict[str, np.ndarray] = {}
    for channel in ("emg_thigh", "emg_calf"):
        try:
            contract = scaling["channels"][channel]
            baseline = float(contract["rest_median"])
            scale = float(contract["activation_p95"])
            clip_min = float(scaling["clip_min"])
            clip_max = float(scaling["clip_max"])
        except (KeyError, TypeError, ValueError) as exc:
            raise H10MotionModelError(f"EMG scaling 필드 오류: {channel}") from exc
        if not all(math.isfinite(value) for value in (baseline, scale, clip_min, clip_max)):
            raise H10MotionModelError(f"EMG scaling 값이 유한하지 않습니다: {channel}")
        if scale <= 0.0 or clip_max <= clip_min:
            raise H10MotionModelError(f"EMG scaling 범위가 잘못되었습니다: {channel}")
        raw = frame[channel].to_numpy(dtype=np.float64)
        result[channel] = np.clip((raw - baseline) / scale, clip_min, clip_max)
    return result


def split_sit_stand_phases(frame: pd.DataFrame) -> dict[str, tuple[pd.DataFrame, dict[str, int | float]]]:
    """서기→앉기→서기 조각을 sit과 stand_up의 동적 구간으로 분리한다."""
    validate_sensor_frame(frame)
    mean_angle = 0.5 * (
        frame["l_angle"].to_numpy(dtype=np.float64)
        + frame["r_angle"].to_numpy(dtype=np.float64)
    )
    smooth = (
        pd.Series(mean_angle)
        .rolling(window=101, center=True, min_periods=1)
        .mean()
        .to_numpy(dtype=np.float64)
    )
    low, high = np.quantile(smooth, [0.05, 0.95])
    span = float(high - low)
    if span < 20.0:
        raise H10MotionModelError(f"앉기-일어서기 각도 범위가 너무 작습니다: {span:.3f} deg")
    low_cross = float(low + 0.20 * span)
    high_cross = float(low + 0.80 * span)
    peak = int(np.argmax(smooth))

    sit_start_candidates = np.flatnonzero(smooth[: peak + 1] >= low_cross)
    sit_end_candidates = np.flatnonzero(smooth[: peak + 1] >= high_cross)
    up_start_candidates = np.flatnonzero(smooth[peak:] <= high_cross)
    up_end_candidates = np.flatnonzero(smooth[peak:] <= low_cross)
    if any(
        len(values) == 0
        for values in (
            sit_start_candidates,
            sit_end_candidates,
            up_start_candidates,
            up_end_candidates,
        )
    ):
        raise H10MotionModelError("앉기 또는 일어서기 임계값 교차를 찾지 못했습니다.")

    sit_start = int(sit_start_candidates[0])
    sit_end = int(sit_end_candidates[0]) + 1
    up_start = peak + int(up_start_candidates[0])
    up_end = peak + int(up_end_candidates[0]) + 1
    if sit_end - sit_start < 100 or up_end - up_start < 100:
        raise H10MotionModelError("앉기 또는 일어서기 동적 구간이 100 ms보다 짧습니다.")

    return {
        "sit": (
            frame.iloc[sit_start:sit_end].copy(),
            {
                "start_index": sit_start,
                "end_index_exclusive": sit_end,
                "low_cross_deg": low_cross,
                "high_cross_deg": high_cross,
            },
        ),
        "stand_up": (
            frame.iloc[up_start:up_end].copy(),
            {
                "start_index": up_start,
                "end_index_exclusive": up_end,
                "low_cross_deg": low_cross,
                "high_cross_deg": high_cross,
            },
        ),
    }


def _add_angle_features(result: dict[str, float], name: str, values: np.ndarray) -> None:
    result[f"{name}_mean"] = float(np.mean(values))
    result[f"{name}_std"] = float(np.std(values, ddof=0))
    result[f"{name}_range"] = float(np.ptp(values))
    result[f"{name}_net"] = float(values[-1] - values[0])
    result[f"{name}_abs_velocity_mean"] = float(
        np.mean(np.abs(np.diff(values))) * SAMPLE_RATE_HZ
    )


def extract_feature_mapping(
    frame: pd.DataFrame, *, emg_scaling: Mapping[str, Any]
) -> dict[str, float]:
    """가변 길이 한 동작을 EMG 포함 64개 요약 특징으로 변환한다."""
    validate_sensor_frame(frame)
    result: dict[str, float] = {
        "duration_s": len(frame) / SAMPLE_RATE_HZ,
        "cadence_hz": SAMPLE_RATE_HZ / len(frame),
    }
    left = frame["l_angle"].to_numpy(dtype=np.float64)
    right = frame["r_angle"].to_numpy(dtype=np.float64)
    for name, values in (
        ("l_angle", left),
        ("r_angle", right),
        ("mean_angle", 0.5 * (left + right)),
        ("diff_angle", left - right),
    ):
        _add_angle_features(result, name, values)
    if np.std(left) > 1.0e-6 and np.std(right) > 1.0e-6:
        result["angle_corr"] = float(np.corrcoef(left, right)[0, 1])
    else:
        result["angle_corr"] = 0.0

    scaled_emg = _scaled_emg(frame, emg_scaling)
    for channel in ("emg_thigh", "emg_calf"):
        values = scaled_emg[channel]
        result[f"{channel}_scaled_mean"] = float(np.mean(values))
        result[f"{channel}_scaled_std"] = float(np.std(values, ddof=0))
        result[f"{channel}_scaled_p50"] = float(np.quantile(values, 0.50))
        result[f"{channel}_scaled_p95"] = float(np.quantile(values, 0.95))
        result[f"{channel}_scaled_max"] = float(np.max(values))
        result[f"{channel}_scaled_active_fraction"] = float(np.mean(values > 0.10))
    thigh = scaled_emg["emg_thigh"]
    calf = scaled_emg["emg_calf"]
    result["emg_scaled_total_mean"] = float(np.mean(thigh + calf))
    result["emg_scaled_balance_mean"] = float(
        np.mean((thigh - calf) / (thigh + calf + 1.0e-6))
    )

    acceleration = frame[["imu_x", "imu_y", "imu_z"]].to_numpy(dtype=np.float64)
    gyroscope = frame[["gyro_x", "gyro_y", "gyro_z"]].to_numpy(dtype=np.float64)
    for name, values in (
        ("acc_norm", np.linalg.norm(acceleration, axis=1)),
        ("gyro_norm", np.linalg.norm(gyroscope, axis=1)),
    ):
        result[f"{name}_mean"] = float(np.mean(values))
        result[f"{name}_std"] = float(np.std(values, ddof=0))
        result[f"{name}_range"] = float(np.ptp(values))
        result[f"{name}_p95"] = float(np.quantile(values, 0.95))

    for channel in ("imu_x", "imu_y", "imu_z", "gyro_x", "gyro_y", "gyro_z"):
        values = frame[channel].to_numpy(dtype=np.float64)
        result[f"{channel}_mean"] = float(np.mean(values))
        result[f"{channel}_std"] = float(np.std(values, ddof=0))
        result[f"{channel}_range"] = float(np.ptp(values))
    result["gyro_y_integral"] = float(frame["gyro_y"].sum() / SAMPLE_RATE_HZ)

    if tuple(result) != FEATURE_NAMES:
        raise H10MotionModelError("특징 순서가 동결 계약과 다릅니다.")
    if not all(math.isfinite(value) for value in result.values()):
        raise H10MotionModelError("생성된 특징에 NaN 또는 무한대가 있습니다.")
    return result


def extract_features(
    frame: pd.DataFrame, *, emg_scaling: Mapping[str, Any]
) -> np.ndarray:
    mapping = extract_feature_mapping(frame, emg_scaling=emg_scaling)
    return np.asarray([mapping[name] for name in FEATURE_NAMES], dtype=np.float64)


def softmax(logits: np.ndarray) -> np.ndarray:
    values = np.asarray(logits, dtype=np.float64)
    values = values - np.max(values, axis=-1, keepdims=True)
    exponent = np.exp(values)
    return exponent / np.sum(exponent, axis=-1, keepdims=True)


def fit_balanced_softmax(
    features: np.ndarray,
    labels: np.ndarray,
    class_count: int,
    *,
    l2: float = 0.05,
    iterations: int = 5000,
    learning_rate: float = 0.02,
) -> tuple[np.ndarray, np.ndarray]:
    """외부 ML 패키지 없이 결정적인 class-balanced softmax를 학습한다."""
    x = np.asarray(features, dtype=np.float64)
    y = np.asarray(labels, dtype=np.int64)
    if x.ndim != 2 or y.shape != (x.shape[0],):
        raise H10MotionModelError("softmax 학습 배열 크기가 맞지 않습니다.")
    counts = np.bincount(y, minlength=class_count)
    if np.any(counts == 0):
        raise H10MotionModelError(f"softmax 학습 클래스 누락: {counts.tolist()}")

    weights = np.zeros((x.shape[1], class_count), dtype=np.float64)
    bias = np.zeros(class_count, dtype=np.float64)
    moment_w = np.zeros_like(weights)
    variance_w = np.zeros_like(weights)
    moment_b = np.zeros_like(bias)
    variance_b = np.zeros_like(bias)
    targets = np.eye(class_count, dtype=np.float64)[y]
    class_weights = len(y) / (class_count * counts)
    sample_weights = class_weights[y]

    for step in range(1, iterations + 1):
        probabilities = softmax(x @ weights + bias)
        derivative = (
            (probabilities - targets)
            * sample_weights[:, np.newaxis]
            / np.sum(sample_weights)
        )
        gradient_w = x.T @ derivative + l2 * weights
        gradient_b = np.sum(derivative, axis=0)

        moment_w = 0.9 * moment_w + 0.1 * gradient_w
        variance_w = 0.999 * variance_w + 0.001 * gradient_w * gradient_w
        moment_b = 0.9 * moment_b + 0.1 * gradient_b
        variance_b = 0.999 * variance_b + 0.001 * gradient_b * gradient_b
        corrected_moment_w = moment_w / (1.0 - 0.9**step)
        corrected_variance_w = variance_w / (1.0 - 0.999**step)
        corrected_moment_b = moment_b / (1.0 - 0.9**step)
        corrected_variance_b = variance_b / (1.0 - 0.999**step)
        weights -= learning_rate * corrected_moment_w / (
            np.sqrt(corrected_variance_w) + 1.0e-8
        )
        bias -= learning_rate * corrected_moment_b / (
            np.sqrt(corrected_variance_b) + 1.0e-8
        )
    return weights, bias


def load_model(path: Path) -> dict[str, Any]:
    try:
        model = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as exc:
        raise H10MotionModelError(f"모델을 읽을 수 없습니다: {path}") from exc
    if (
        model.get("schema_version") != MODEL_SCHEMA_VERSION
        or tuple(model.get("state_classes", [])) != STATE_CLASSES
        or tuple(model.get("feature_names", [])) != FEATURE_NAMES
        or tuple(model.get("decision_feature_names", [])) != DECISION_FEATURE_NAMES
        or model.get("sample_rate_hz") != SAMPLE_RATE_HZ
        or not isinstance(model.get("emg_scaling"), dict)
    ):
        raise H10MotionModelError("모델 스키마 또는 특징 계약이 맞지 않습니다.")
    return model


def _model_probabilities(
    standardized: np.ndarray, model: Mapping[str, Any], key: str
) -> np.ndarray:
    block = model[key]
    weights = np.asarray(block["weights"], dtype=np.float64)
    bias = np.asarray(block["bias"], dtype=np.float64)
    if weights.shape[0] != standardized.size or bias.shape != (weights.shape[1],):
        raise H10MotionModelError(f"모델 행렬 크기 오류: {key}")
    return softmax(standardized @ weights + bias)


def estimate_turn_output(
    frame: pd.DataFrame,
    *,
    bias_dps: float,
    window_ms: int = 500,
    deadzone_dps: float = 30.0,
) -> dict[str, float | str | bool]:
    """yaw gyro를 signed 회전속도와 누적 회전각으로 바꾼다.

    이 H10 기록 좌표에서는 음수=CW, 양수=CCW다. 500 ms 평균과 30 deg/s
    dead-zone은 현재 11개 실측 파일에서 정지/보행/점프 흔들림을 거른 값이다.
    """
    validate_sensor_frame(frame)
    window_samples = max(1, int(round(window_ms * SAMPLE_RATE_HZ / 1000.0)))
    gyro_y = frame["gyro_y"].to_numpy(dtype=np.float64) - float(bias_dps)
    filtered_rate = float(np.mean(gyro_y[-min(len(gyro_y), window_samples) :]))
    active = abs(filtered_rate) >= float(deadzone_dps)
    rate = filtered_rate if active else 0.0
    direction = "cw" if rate < 0.0 else "ccw" if rate > 0.0 else "none"
    angle = float(np.sum(gyro_y) / SAMPLE_RATE_HZ) if active else 0.0
    return {
        "turn_active": active,
        "turn_direction": direction,
        "turn_rate_dps": rate,
        "turn_angle_deg": angle,
        "turn_raw_filtered_dps": filtered_rate,
    }


def predict_frame(frame: pd.DataFrame, model: Mapping[str, Any]) -> dict[str, Any]:
    feature_mapping = extract_feature_mapping(
        frame, emg_scaling=model["emg_scaling"]
    )
    features = np.asarray(
        [feature_mapping[name] for name in DECISION_FEATURE_NAMES], dtype=np.float64
    )
    mean = np.asarray(model["scaler"]["mean"], dtype=np.float64)
    scale = np.asarray(model["scaler"]["scale"], dtype=np.float64)
    if mean.shape != features.shape or scale.shape != features.shape or np.any(scale <= 0.0):
        raise H10MotionModelError("모델 scaler 크기 또는 값이 잘못되었습니다.")
    standardized = (features - mean) / scale
    state_probabilities = _model_probabilities(standardized, model, "state_model")
    state_index = int(np.argmax(state_probabilities))
    state = STATE_CLASSES[state_index]

    result: dict[str, Any] = {
        "state": state,
        "confidence": float(state_probabilities[state_index]),
        "walk_intensity": None,
        "jump_intensity": None,
        "probabilities": {
            name: float(value) for name, value in zip(STATE_CLASSES, state_probabilities)
        },
        "duration_s": float(len(frame) / SAMPLE_RATE_HZ),
    }
    turn_model = model["turn_model"]
    result.update(
        estimate_turn_output(
            frame,
            bias_dps=float(turn_model["bias_dps"]),
            window_ms=int(turn_model["window_ms"]),
            deadzone_dps=float(turn_model["deadzone_dps"]),
        )
    )
    if state in ("walk", "jump"):
        intensity_model = model["intensity_models"][state]
        probabilities = _model_probabilities(
            standardized, {state: intensity_model}, state
        )
        level_values = np.asarray(intensity_model["level_values"], dtype=np.float64)
        intensity = float(np.clip(probabilities @ level_values, 0.0, 1.0))
        result[f"{state}_intensity"] = intensity
        result[f"{state}_level_probabilities"] = {
            name: float(value)
            for name, value in zip(intensity_model["level_names"], probabilities)
        }
    return result
