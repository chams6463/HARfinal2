#!/usr/bin/env python3
"""11개 H10 원본 로그로 50 ms 연속 Minecraft 동작 판정기를 학습한다.

이 학습기는 자동 절단 조각을 다시 섞지 않는다. 각 원본 파일의 시간축을 그대로
유지한 채 앞 60%만 학습하고, 60~78%는 선택용, 마지막 22%는 잠금 시험용으로
사용한다. 320 ms 인과 특징이 분할 경계를 넘지 않도록 각 경계 양쪽 400 ms는
평가에서 제외한다.

점프는 두 문제로 분리한다.

* ``state_model``: stand / walk / jump의 지속 상태
* ``jump_event_model``: 한 번의 점프 시작에 SPACE를 한 번만 내는 물리 사건

웅크리기는 분류 클래스가 아니다. stand 중립각과 sit_stand의 최대 굴곡 P95로
결정적인 full-crouch gate를 만들며, 최대 굴곡 영역을 벗어나거나 일어서기 속도가
관측되면 다음 50 ms 판정에서 해제한다.
"""
from __future__ import annotations

import argparse
import hashlib
import json
import math
import shutil
from dataclasses import dataclass
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Iterable, Mapping, Sequence

import numpy as np
import pandas as pd
from scipy.signal import find_peaks, medfilt
from sklearn.ensemble import ExtraTreesClassifier
from sklearn.metrics import (
    accuracy_score,
    average_precision_score,
    confusion_matrix,
    f1_score,
    mean_absolute_error,
    mean_squared_error,
    precision_score,
    recall_score,
)

try:
    from tools.h10_motion_segment_model import (
        H10MotionModelError,
        REQUIRED_SENSOR_COLUMNS,
    )
    from tools.h10_stream_motion_model import (
        CONTINUOUS_FEATURE_NAMES,
        FULL_CROUCH_FRACTION,
        HOP_MS,
        HOP_SAMPLES,
        MODEL_SCHEMA_VERSION,
        continuous_feature_rows,
        fit_stream_calibration,
    )
except ModuleNotFoundError:
    from h10_motion_segment_model import (  # type: ignore[no-redef]
        H10MotionModelError,
        REQUIRED_SENSOR_COLUMNS,
    )
    from h10_stream_motion_model import (  # type: ignore[no-redef]
        CONTINUOUS_FEATURE_NAMES,
        FULL_CROUCH_FRACTION,
        HOP_MS,
        HOP_SAMPLES,
        MODEL_SCHEMA_VERSION,
        continuous_feature_rows,
        fit_stream_calibration,
    )


SAMPLE_RATE_HZ = 1000
STATE_CLASSES = ("stand", "walk", "jump")
STATE_TO_INDEX = {name: index for index, name in enumerate(STATE_CLASSES)}
EMBARGO_SAMPLES = 400
TRAIN_FRACTION = 0.60
DEV_FRACTION_END = 0.78

# 21 ms는 1 kHz 원본에서 짧은 비행 저가속 구간을 보존한다. 51 ms는
# jump_1/2에서 실제 반복을 합쳐 버려 물리 사건 정답을 누락시켰다.
FLIGHT_MEDIAN_KERNEL = 21
FLIGHT_MAX_ACC_NORM = 1.5
FLIGHT_MIN_DISTANCE_SAMPLES = 700
FLIGHT_MIN_PROMINENCE = 1.0
FORCE_SEARCH_BACK_SAMPLES = 600
JUMP_EVENT_POSITIVE_RADIUS_SAMPLES = 100
EVENT_MATCH_EARLY_SAMPLES = 400
EVENT_MATCH_LATE_SAMPLES = 150
EVENT_REFRACTORY_MS = 500
EVENT_REARM_THRESHOLD = 0.25
EVENT_REARM_CONFIRM_TICKS = 2


@dataclass(frozen=True)
class SourceSpec:
    key: str
    filename: str
    state: str
    intensity_family: str | None = None
    intensity_class: int | None = None
    intensity_value: float | None = None


SOURCE_SPECS = (
    SourceSpec("stand", "stand_1m.csv", "stand"),
    SourceSpec("walk_1", "walk_1.csv", "walk", "walk", 0, 0.0),
    SourceSpec("walk_2", "walk_2.csv", "walk", "walk", 1, 1.0 / 3.0),
    SourceSpec("walk_3", "walk_3.csv", "walk", "walk", 2, 2.0 / 3.0),
    SourceSpec("run", "run.csv", "walk", "walk", 3, 1.0),
    SourceSpec("jump_1", "jump_1.csv", "jump", "jump", 0, 0.0),
    SourceSpec("jump_2", "jump_2.csv", "jump", "jump", 1, 0.5),
    SourceSpec("jump_3", "jump_3.csv", "jump", "jump", 2, 1.0),
    # sit/stand의 하강·상승 구간은 jump hard-negative인 stand로 학습한다.
    SourceSpec("sit_stand", "sit_stand.csv", "stand"),
    # 회전은 별도 signed yaw 출력이다. 이동 상태 모델에는 stand hard-negative다.
    SourceSpec("turn_clw", "turn_clw.csv", "stand"),
    SourceSpec("turn_ccw", "turn_ccw.csv", "stand"),
)


@dataclass
class Recording:
    spec: SourceSpec
    path: Path
    frame: pd.DataFrame
    sha256: str
    candidate_events: list[dict[str, Any]]
    jump_events: list[dict[str, Any]]


@dataclass
class FeatureRows:
    features: np.ndarray
    end_samples: np.ndarray
    recording_index: np.ndarray
    source_key: np.ndarray
    split: np.ndarray
    state_label: np.ndarray
    jump_event_label: np.ndarray
    walk_intensity_class: np.ndarray
    jump_intensity_class: np.ndarray
    row_slices: tuple[slice, ...]


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def _load_recordings(source_dir: Path) -> list[Recording]:
    recordings: list[Recording] = []
    for spec in SOURCE_SPECS:
        path = source_dir / spec.filename
        if not path.is_file():
            raise H10MotionModelError(f"필수 원본 파일이 없습니다: {path}")
        frame = pd.read_csv(path)
        missing = sorted(set(REQUIRED_SENSOR_COLUMNS) - set(frame.columns))
        if missing:
            raise H10MotionModelError(f"{spec.filename} 센서 열 누락: {missing}")
        if len(frame) < 5000:
            raise H10MotionModelError(
                f"{spec.filename}은 연속 학습에 너무 짧습니다: {len(frame)} rows"
            )
        values = frame.loc[:, REQUIRED_SENSOR_COLUMNS].to_numpy(dtype=np.float64)
        if not np.isfinite(values).all():
            raise H10MotionModelError(f"{spec.filename}에 NaN 또는 무한대가 있습니다.")
        candidates = _derive_physical_jump_candidates(frame)
        recordings.append(
            Recording(
                spec=spec,
                path=path,
                frame=frame,
                sha256=_sha256(path),
                candidate_events=candidates,
                jump_events=candidates if spec.state == "jump" else [],
            )
        )
    return recordings


def _derive_physical_jump_candidates(frame: pd.DataFrame) -> list[dict[str, Any]]:
    """저가속 비행점과 그 직전 600 ms의 최대 push-off를 점프 사건으로 만든다."""
    acceleration = frame.loc[:, ["imu_x", "imu_y", "imu_z"]].to_numpy(
        dtype=np.float64
    )
    acc_norm = np.linalg.norm(acceleration, axis=1)
    filtered = medfilt(acc_norm, kernel_size=FLIGHT_MEDIAN_KERNEL)
    flight_indices, properties = find_peaks(
        -filtered,
        height=-FLIGHT_MAX_ACC_NORM,
        distance=FLIGHT_MIN_DISTANCE_SAMPLES,
        prominence=FLIGHT_MIN_PROMINENCE,
    )
    events: list[dict[str, Any]] = []
    for ordinal, flight_index in enumerate(flight_indices):
        search_start = max(0, int(flight_index) - FORCE_SEARCH_BACK_SAMPLES)
        # endpoint가 최대인 경우도 놓치지 않기 위해 local peak가 없으면 argmax를 쓴다.
        search = filtered[search_start : int(flight_index) + 1]
        local_peaks, _ = find_peaks(search, prominence=0.1)
        if len(local_peaks):
            local_force = int(
                local_peaks[int(np.argmax(search[local_peaks]))]
            )
            method = "highest_local_peak_previous_600ms"
        else:
            local_force = int(np.argmax(search))
            method = "argmax_previous_600ms"
        force_index = search_start + local_force
        events.append(
            {
                "event_ordinal": ordinal + 1,
                "force_peak_sample": int(force_index),
                "flight_minimum_sample": int(flight_index),
                "force_to_flight_ms": int(flight_index) - int(force_index),
                "force_acc_norm": float(filtered[force_index]),
                "flight_acc_norm": float(filtered[int(flight_index)]),
                "flight_prominence": float(properties["prominences"][ordinal]),
                "force_peak_method": method,
            }
        )
    return events


def _split_boundaries(row_count: int) -> tuple[int, int]:
    return int(math.floor(row_count * TRAIN_FRACTION)), int(
        math.floor(row_count * DEV_FRACTION_END)
    )


def _split_end_samples(end_samples: np.ndarray, row_count: int) -> np.ndarray:
    train_end, dev_end = _split_boundaries(row_count)
    result = np.full(len(end_samples), "embargo", dtype=object)
    result[end_samples < train_end - EMBARGO_SAMPLES] = "train"
    result[
        (end_samples >= train_end + EMBARGO_SAMPLES)
        & (end_samples < dev_end - EMBARGO_SAMPLES)
    ] = "dev"
    result[end_samples >= dev_end + EMBARGO_SAMPLES] = "test"
    return result


def _event_split(sample: int, row_count: int) -> str:
    return str(_split_end_samples(np.asarray([sample + 1]), row_count)[0])


def _evaluation_calibration(recordings: Sequence[Recording]) -> dict[str, Any]:
    train_frames: list[pd.DataFrame] = []
    rest_frames: list[pd.DataFrame] = []
    for recording in recordings:
        train_end, _ = _split_boundaries(len(recording.frame))
        usable_end = train_end - EMBARGO_SAMPLES
        if usable_end <= 0:
            raise H10MotionModelError(f"학습 보정 구간이 비었습니다: {recording.spec.key}")
        portion = recording.frame.iloc[:usable_end].copy()
        train_frames.append(portion)
        if recording.spec.key == "stand":
            rest_frames.append(portion)
    calibration = fit_stream_calibration(train_frames, rest_frames)
    calibration["authority"] = "first_60_percent_minus_400ms_embargo_training_only"
    calibration["source_key"] = "stand"
    return calibration


def _final_calibration(recordings: Sequence[Recording]) -> dict[str, Any]:
    calibration = fit_stream_calibration(
        [recording.frame for recording in recordings],
        [recording.frame for recording in recordings if recording.spec.key == "stand"],
    )
    calibration["authority"] = "all_available_rows_after_locked_evaluation"
    calibration["source_key"] = "stand"
    return calibration


def _jump1_prep_start(recording: Recording) -> int | None:
    if recording.spec.key != "jump_1" or not recording.jump_events:
        return None
    # 시험 허용창의 시작과 같은 정의를 써서 초기 장비 안정화 구간을 stand로 둔다.
    return max(
        0,
        int(recording.jump_events[0]["force_peak_sample"])
        - EVENT_MATCH_EARLY_SAMPLES,
    )


def _build_feature_rows(
    recordings: Sequence[Recording], calibration: Mapping[str, Any]
) -> FeatureRows:
    feature_blocks: list[np.ndarray] = []
    end_blocks: list[np.ndarray] = []
    recording_blocks: list[np.ndarray] = []
    source_blocks: list[np.ndarray] = []
    split_blocks: list[np.ndarray] = []
    state_blocks: list[np.ndarray] = []
    event_blocks: list[np.ndarray] = []
    walk_blocks: list[np.ndarray] = []
    jump_blocks: list[np.ndarray] = []
    row_slices: list[slice] = []
    offset = 0

    for recording_index, recording in enumerate(recordings):
        end_samples, features = continuous_feature_rows(recording.frame, calibration)
        if features.shape != (len(end_samples), len(CONTINUOUS_FEATURE_NAMES)):
            raise H10MotionModelError(
                f"{recording.spec.key} 연속 특징 크기 오류: {features.shape}"
            )
        state = np.full(
            len(end_samples), STATE_TO_INDEX[recording.spec.state], dtype=np.int64
        )
        prep_start = _jump1_prep_start(recording)
        if prep_start is not None:
            state[end_samples <= prep_start] = STATE_TO_INDEX["stand"]

        event_label = np.zeros(len(end_samples), dtype=np.int64)
        for event in recording.jump_events:
            center = int(event["force_peak_sample"]) + 1
            event_label[np.abs(end_samples - center) <= JUMP_EVENT_POSITIVE_RADIUS_SAMPLES] = 1

        walk_class = np.full(len(end_samples), -1, dtype=np.int64)
        jump_class = np.full(len(end_samples), -1, dtype=np.int64)
        if recording.spec.intensity_family == "walk":
            walk_class[:] = int(recording.spec.intensity_class)
        elif recording.spec.intensity_family == "jump":
            jump_class[state == STATE_TO_INDEX["jump"]] = int(
                recording.spec.intensity_class
            )

        feature_blocks.append(features)
        end_blocks.append(end_samples.astype(np.int64))
        recording_blocks.append(
            np.full(len(end_samples), recording_index, dtype=np.int64)
        )
        source_blocks.append(
            np.full(len(end_samples), recording.spec.key, dtype=object)
        )
        split_blocks.append(_split_end_samples(end_samples, len(recording.frame)))
        state_blocks.append(state)
        event_blocks.append(event_label)
        walk_blocks.append(walk_class)
        jump_blocks.append(jump_class)
        row_slices.append(slice(offset, offset + len(end_samples)))
        offset += len(end_samples)

    return FeatureRows(
        features=np.vstack(feature_blocks),
        end_samples=np.concatenate(end_blocks),
        recording_index=np.concatenate(recording_blocks),
        source_key=np.concatenate(source_blocks),
        split=np.concatenate(split_blocks),
        state_label=np.concatenate(state_blocks),
        jump_event_label=np.concatenate(event_blocks),
        walk_intensity_class=np.concatenate(walk_blocks),
        jump_intensity_class=np.concatenate(jump_blocks),
        row_slices=tuple(row_slices),
    )


def _source_balanced_weights(labels: np.ndarray, sources: np.ndarray) -> np.ndarray:
    """클래스 총량과 그 안의 원본 파일 총량을 동일하게 만든다."""
    result = np.zeros(len(labels), dtype=np.float64)
    classes = np.unique(labels)
    for class_value in classes:
        class_mask = labels == class_value
        class_sources = np.unique(sources[class_mask])
        for source in class_sources:
            mask = class_mask & (sources == source)
            result[mask] = 1.0 / (
                len(classes) * len(class_sources) * int(np.sum(mask))
            )
    result *= len(result) / float(np.sum(result))
    return result


STATE_CANDIDATES = (
    {"max_depth": 16, "min_samples_leaf": 2, "max_features": "sqrt"},
    {"max_depth": 22, "min_samples_leaf": 2, "max_features": 0.60},
    {"max_depth": None, "min_samples_leaf": 3, "max_features": 0.50},
)


def _new_forest(
    *,
    n_estimators: int,
    random_state: int,
    max_depth: int | None,
    min_samples_leaf: int,
    max_features: str | float,
) -> ExtraTreesClassifier:
    return ExtraTreesClassifier(
        n_estimators=n_estimators,
        criterion="entropy",
        max_depth=max_depth,
        min_samples_leaf=min_samples_leaf,
        max_features=max_features,
        bootstrap=False,
        class_weight=None,
        random_state=random_state,
        n_jobs=-1,
    )


def _state_metrics(truth: np.ndarray, prediction: np.ndarray) -> dict[str, Any]:
    return {
        "support_50ms_windows": int(len(truth)),
        "accuracy": float(accuracy_score(truth, prediction)),
        "macro_f1": float(f1_score(truth, prediction, average="macro")),
        "confusion_matrix": confusion_matrix(
            truth, prediction, labels=np.arange(len(STATE_CLASSES))
        ).tolist(),
        "labels": list(STATE_CLASSES),
    }


def _select_state_model(
    rows: FeatureRows,
) -> tuple[ExtraTreesClassifier, dict[str, Any], list[dict[str, Any]]]:
    train = rows.split == "train"
    dev = rows.split == "dev"
    weights = _source_balanced_weights(
        rows.state_label[train], rows.source_key[train]
    )
    records: list[dict[str, Any]] = []
    fitted: list[ExtraTreesClassifier] = []
    for candidate_index, candidate in enumerate(STATE_CANDIDATES):
        model = _new_forest(
            n_estimators=96,
            random_state=1100 + candidate_index,
            **candidate,
        )
        model.fit(
            rows.features[train],
            rows.state_label[train],
            sample_weight=weights,
        )
        prediction = model.predict(rows.features[dev]).astype(np.int64)
        metrics = _state_metrics(rows.state_label[dev], prediction)
        records.append({"parameters": candidate, **metrics})
        fitted.append(model)
    chosen_index = min(
        range(len(records)),
        key=lambda index: (
            -float(records[index]["macro_f1"]),
            -float(records[index]["accuracy"]),
            index,
        ),
    )
    chosen = dict(STATE_CANDIDATES[chosen_index])
    final = _new_forest(
        n_estimators=160,
        random_state=1200 + chosen_index,
        **chosen,
    )
    final.fit(
        rows.features[train],
        rows.state_label[train],
        sample_weight=weights,
    )
    return final, chosen, records


def _fit_jump_event_model(
    rows: FeatureRows, mask: np.ndarray, *, random_state: int
) -> ExtraTreesClassifier:
    labels = rows.jump_event_label[mask]
    if set(np.unique(labels).tolist()) != {0, 1}:
        raise H10MotionModelError("점프 사건 학습 구간에 positive/negative가 모두 필요합니다.")
    # 양성/음성 총량을 동일하게 하되 음성은 원본 파일별 총량도 같게 둔다.
    weights = _source_balanced_weights(labels, rows.source_key[mask])
    model = _new_forest(
        n_estimators=192,
        random_state=random_state,
        max_depth=20,
        min_samples_leaf=1,
        max_features=0.70,
    )
    model.fit(rows.features[mask], labels, sample_weight=weights)
    return model


def _positive_probability(model: ExtraTreesClassifier, features: np.ndarray) -> np.ndarray:
    classes = model.classes_.astype(np.int64)
    positions = np.flatnonzero(classes == 1)
    if len(positions) != 1:
        raise H10MotionModelError(f"positive class=1이 없습니다: {classes.tolist()}")
    return model.predict_proba(features)[:, int(positions[0])]


def _gated_triggers(
    probability: np.ndarray,
    end_samples: np.ndarray,
    threshold: float,
) -> list[int]:
    armed = True
    low_ticks = 0
    last_trigger = -10**12
    triggers: list[int] = []
    for value, end_sample in zip(probability, end_samples, strict=True):
        if float(value) < EVENT_REARM_THRESHOLD:
            low_ticks += 1
        else:
            low_ticks = 0
        if (
            not armed
            and int(end_sample) - last_trigger >= EVENT_REFRACTORY_MS
            and low_ticks >= EVENT_REARM_CONFIRM_TICKS
        ):
            armed = True
        if not armed or float(value) < threshold:
            continue
        triggers.append(int(end_sample))
        last_trigger = int(end_sample)
        armed = False
        low_ticks = 0
    return triggers


def _match_events(triggers: Sequence[int], events: Sequence[int]) -> dict[str, Any]:
    unmatched = set(range(len(events)))
    matched: list[tuple[int, int]] = []
    false_triggers: list[int] = []
    for trigger in triggers:
        candidates = [
            index
            for index in unmatched
            if -EVENT_MATCH_EARLY_SAMPLES
            <= int(trigger) - int(events[index])
            <= EVENT_MATCH_LATE_SAMPLES
        ]
        if not candidates:
            false_triggers.append(int(trigger))
            continue
        chosen = min(candidates, key=lambda index: abs(int(trigger) - int(events[index])))
        unmatched.remove(chosen)
        matched.append((int(trigger), int(events[chosen])))
    return {
        "matched": matched,
        "false_triggers": false_triggers,
        "missed_events": [int(events[index]) for index in sorted(unmatched)],
    }


def _event_detection_metrics(
    recordings: Sequence[Recording],
    rows: FeatureRows,
    probabilities: np.ndarray,
    split_name: str,
    threshold: float,
) -> dict[str, Any]:
    true_positive = 0
    false_positive = 0
    false_negative = 0
    latency_ms: list[int] = []
    per_source: list[dict[str, Any]] = []
    evaluated_ticks = 0
    for recording_index, recording in enumerate(recordings):
        row_slice = rows.row_slices[recording_index]
        local_split = rows.split[row_slice]
        selected = local_split == split_name
        local_probability = probabilities[row_slice][selected]
        local_end = rows.end_samples[row_slice][selected]
        triggers = _gated_triggers(local_probability, local_end, threshold)
        events = [
            int(event["force_peak_sample"]) + 1
            for event in recording.jump_events
            if _event_split(
                int(event["force_peak_sample"]), len(recording.frame)
            )
            == split_name
        ]
        match = _match_events(triggers, events)
        matched = match["matched"]
        true_positive += len(matched)
        false_positive += len(match["false_triggers"])
        false_negative += len(match["missed_events"])
        latency_ms.extend(trigger - event for trigger, event in matched)
        evaluated_ticks += len(local_end)
        per_source.append(
            {
                "source_key": recording.spec.key,
                "truth_events": len(events),
                "triggers": len(triggers),
                "matched": len(matched),
                "false_triggers": len(match["false_triggers"]),
                "missed": len(match["missed_events"]),
                "latency_ms": [trigger - event for trigger, event in matched],
            }
        )
    precision = true_positive / max(1, true_positive + false_positive)
    recall = true_positive / max(1, true_positive + false_negative)
    f1 = 2.0 * precision * recall / max(1.0e-12, precision + recall)
    minutes = evaluated_ticks * HOP_MS / 60000.0
    return {
        "threshold": float(threshold),
        "match_window_ms": [-EVENT_MATCH_EARLY_SAMPLES, EVENT_MATCH_LATE_SAMPLES],
        "truth_events": int(true_positive + false_negative),
        "triggers": int(true_positive + false_positive),
        "true_positive": int(true_positive),
        "false_positive": int(false_positive),
        "false_negative": int(false_negative),
        "precision": float(precision),
        "recall": float(recall),
        "f1": float(f1),
        "false_triggers_per_minute": float(false_positive / max(minutes, 1.0e-12)),
        "latency_ms": {
            "values": latency_ms,
            "median": float(np.median(latency_ms)) if latency_ms else None,
            "p90": float(np.percentile(latency_ms, 90)) if latency_ms else None,
            "minimum": int(min(latency_ms)) if latency_ms else None,
            "maximum": int(max(latency_ms)) if latency_ms else None,
        },
        "per_source": per_source,
    }


def _select_jump_threshold(
    recordings: Sequence[Recording],
    rows: FeatureRows,
    probabilities: np.ndarray,
) -> tuple[float, list[dict[str, Any]]]:
    candidates = sorted(
        set(np.round(np.arange(0.05, 0.951, 0.025), 3).tolist() + [0.5])
    )
    records = [
        _event_detection_metrics(recordings, rows, probabilities, "dev", threshold)
        for threshold in candidates
    ]

    def rank(metrics: Mapping[str, Any]) -> tuple[float, ...]:
        latency = metrics["latency_ms"]["median"]
        latency_cost = abs(float(latency)) if latency is not None else 10**9
        return (
            -float(metrics["f1"]),
            -float(metrics["recall"]),
            -float(metrics["precision"]),
            float(metrics["false_triggers_per_minute"]),
            # dev의 사건 성능과 오검출이 완전히 같은 plateau에서는 점프 누락을
            # 줄이는 목적에 맞춰 가장 낮은 문턱을 사전 정의 tie-break로 택한다.
            float(metrics["threshold"]),
            latency_cost,
        )

    chosen = min(records, key=rank)
    return float(chosen["threshold"]), records


def _fit_intensity_model(
    rows: FeatureRows,
    family: str,
    mask: np.ndarray,
    *,
    random_state: int,
) -> ExtraTreesClassifier:
    labels = (
        rows.walk_intensity_class if family == "walk" else rows.jump_intensity_class
    )
    family_mask = mask & (labels >= 0)
    family_labels = labels[family_mask]
    expected = {0, 1, 2, 3} if family == "walk" else {0, 1, 2}
    if set(np.unique(family_labels).tolist()) != expected:
        raise H10MotionModelError(
            f"{family} 강도 클래스 부족: {np.unique(family_labels).tolist()}"
        )
    weights = _source_balanced_weights(
        family_labels, rows.source_key[family_mask]
    )
    model = _new_forest(
        n_estimators=128,
        random_state=random_state,
        max_depth=18,
        min_samples_leaf=2,
        max_features=0.60,
    )
    model.fit(rows.features[family_mask], family_labels, sample_weight=weights)
    return model


INTENSITY_LEVELS = {
    "walk": np.asarray([0.0, 1.0 / 3.0, 2.0 / 3.0, 1.0]),
    "jump": np.asarray([0.0, 0.5, 1.0]),
}


def _intensity_metrics(
    rows: FeatureRows,
    model: ExtraTreesClassifier,
    family: str,
    split_name: str,
) -> dict[str, Any]:
    labels = (
        rows.walk_intensity_class if family == "walk" else rows.jump_intensity_class
    )
    selected = (rows.split == split_name) & (labels >= 0)
    probability = model.predict_proba(rows.features[selected])
    classes = model.classes_.astype(np.int64)
    levels = INTENSITY_LEVELS[family][classes]
    prediction = probability @ levels
    truth = INTENSITY_LEVELS[family][labels[selected]]
    predicted_class = classes[np.argmax(probability, axis=1)]
    return {
        "support_50ms_windows": int(np.sum(selected)),
        "mae_0_to_1": float(mean_absolute_error(truth, prediction)),
        "rmse_0_to_1": float(math.sqrt(mean_squared_error(truth, prediction))),
        "nearest_level_accuracy": float(accuracy_score(labels[selected], predicted_class)),
    }


def _serialize_classifier(
    model: ExtraTreesClassifier,
    *,
    classes: Sequence[str | int],
) -> dict[str, Any]:
    """sklearn ExtraTrees를 런타임의 순수 NumPy 배열 계약으로 저장한다."""
    if len(model.classes_) != len(classes):
        raise H10MotionModelError(
            f"직렬화 클래스 수 불일치: {len(model.classes_)} != {len(classes)}"
        )
    trees: list[dict[str, Any]] = []
    for estimator in model.estimators_:
        tree = estimator.tree_
        leaf_probability: list[list[float]] = []
        for node in range(tree.node_count):
            if int(tree.children_left[node]) >= 0:
                leaf_probability.append([0.0] * len(classes))
                continue
            value = np.asarray(tree.value[node, 0], dtype=np.float64)
            total = float(np.sum(value))
            if total <= 0.0:
                raise H10MotionModelError("ExtraTrees leaf의 class mass가 0입니다.")
            leaf_probability.append((value / total).tolist())
        trees.append(
            {
                "children_left": tree.children_left.astype(np.int64).tolist(),
                "children_right": tree.children_right.astype(np.int64).tolist(),
                "feature": tree.feature.astype(np.int64).tolist(),
                "threshold": tree.threshold.astype(np.float64).tolist(),
                "leaf_probability": leaf_probability,
            }
        )
    return {
        "family": "sklearn_extra_trees_classifier_json_v1",
        "classes": list(classes),
        "n_features": int(model.n_features_in_),
        "n_estimators": len(trees),
        "trees": trees,
    }


def _fit_crouch_model(
    recordings: Sequence[Recording], *, all_rows: bool
) -> dict[str, Any]:
    stand = next(item for item in recordings if item.spec.key == "stand")
    sit = next(item for item in recordings if item.spec.key == "sit_stand")
    if all_rows:
        stand_frame = stand.frame
        sit_frame = sit.frame
        authority = "all_rows_after_locked_evaluation"
    else:
        stand_end, _ = _split_boundaries(len(stand.frame))
        sit_end, _ = _split_boundaries(len(sit.frame))
        stand_frame = stand.frame.iloc[: stand_end - EMBARGO_SAMPLES]
        sit_frame = sit.frame.iloc[: sit_end - EMBARGO_SAMPLES]
        authority = "training_rows_only"

    neutral = np.asarray(
        [np.median(stand_frame["l_angle"]), np.median(stand_frame["r_angle"])],
        dtype=np.float64,
    )
    sit_angles = sit_frame.loc[:, ["l_angle", "r_angle"]].to_numpy(dtype=np.float64)
    relative = sit_angles - neutral
    positive = np.percentile(relative, 95, axis=0)
    negative = np.percentile(relative, 5, axis=0)
    direction = np.where(np.abs(positive) >= np.abs(negative), 1.0, -1.0)
    directed = relative * direction
    full_span = np.percentile(directed, 95, axis=0)
    if np.any(full_span < 10.0):
        raise H10MotionModelError(
            f"full-crouch 굴곡 span이 너무 작습니다: {full_span.tolist()}"
        )
    return {
        "family": "bilateral_full_flexion_plateau_v1",
        "authority": authority,
        "neutral_angle_deg": neutral.tolist(),
        "direction": direction.astype(np.int64).tolist(),
        "full_flexion_statistic": "sit_stand directed angle q95 relative to stand median",
        "full_flexion_span_deg": full_span.tolist(),
        "enter_fraction": FULL_CROUCH_FRACTION,
        "enter_confirm_ticks": 2,
        "release_fraction": FULL_CROUCH_FRACTION,
        "plateau_max_velocity_dps": 30.0,
        "release_ascent_velocity_dps": 30.0,
        "semantic": (
            "SHIFT is held only after both hips reach at least 95 percent of their "
            "training maximum for two ticks; otherwise SHIFT is released"
        ),
    }


def _model_payload(
    *,
    calibration: Mapping[str, Any],
    state_model: ExtraTreesClassifier,
    jump_event_model: ExtraTreesClassifier,
    walk_intensity_model: ExtraTreesClassifier,
    jump_intensity_model: ExtraTreesClassifier,
    crouch_model: Mapping[str, Any],
    jump_threshold: float,
    training_rows: int,
    training_scope: str,
    selected_state_parameters: Mapping[str, Any],
) -> dict[str, Any]:
    walk_contract = _serialize_classifier(
        walk_intensity_model, classes=[0, 1, 2, 3]
    )
    walk_contract["level_values"] = INTENSITY_LEVELS["walk"].tolist()
    jump_contract = _serialize_classifier(
        jump_intensity_model, classes=[0, 1, 2]
    )
    jump_contract["level_values"] = INTENSITY_LEVELS["jump"].tolist()
    return {
        "schema_version": MODEL_SCHEMA_VERSION,
        "created_utc": datetime.now(timezone.utc).isoformat(),
        "decision_mode": "continuous_tree_v1",
        "status": "single_wearer_single_session_time_holdout",
        "input_rate_hz": SAMPLE_RATE_HZ,
        "output_period_ms": HOP_MS,
        "causal": True,
        "feature_names": list(CONTINUOUS_FEATURE_NAMES),
        "decision_feature_names": list(CONTINUOUS_FEATURE_NAMES),
        "calibration": dict(calibration),
        "state_classes": list(STATE_CLASSES),
        "state_model": _serialize_classifier(
            state_model, classes=list(STATE_CLASSES)
        ),
        "jump_event_model": {
            **_serialize_classifier(
                jump_event_model,
                classes=["not_jump_event", "jump_event"],
            ),
            "positive_class": "jump_event",
            "truth_definition": "push-off force peak plus or minus 100 ms",
        },
        "intensity_models": {
            "walk": walk_contract,
            "jump": jump_contract,
        },
        "crouch_model": dict(crouch_model),
        "jump_event_gate": {
            "enter_threshold": float(jump_threshold),
            "refractory_ms": EVENT_REFRACTORY_MS,
            "rearm_threshold": EVENT_REARM_THRESHOLD,
            "rearm_confirm_ticks": EVENT_REARM_CONFIRM_TICKS,
        },
        "state_machine": {
            "walk_enter_threshold": 0.55,
            "walk_confirm_ticks": 1,
            "walk_exit_threshold": 0.35,
            "walk_exit_confirm_ticks": 3,
            "jump_exit_threshold": 0.35,
            "jump_exit_confirm_ticks": 3,
            "jump_min_hold_ticks": 6,
            # sit_stand 전체를 jump hard-negative로 학습했으므로 release 뒤의
            # 실제 점프 event를 버리는 추가 400 ms block은 두지 않는다.
            "post_crouch_jump_block_ticks": 0,
        },
        "turn_model": {
            "window_ms": 320,
            "deadzone_dps": 40.0,
            "confirm_ticks": 1,
            "walk_entry_guard_window_ms": 80,
            "walk_entry_guard_dps": 30.0,
            "sign_contract": "negative=cw, positive=ccw",
        },
        "training": {
            "scope": training_scope,
            "windows_50ms": int(training_rows),
            "selected_state_parameters": dict(selected_state_parameters),
        },
        "output_contract": {
            "state": "stand|walk|jump|sit",
            "key_command": "NONE|W|SPACE|SHIFT",
            "jump_started": "one 50 ms rising edge per detected physical jump",
            "walk_intensity": "0..1 while state=walk",
            "jump_intensity": "0..1 while state=jump",
            "crouch": "SHIFT held only at bilateral maximum-flexion plateau",
            "not_crouch": "SHIFT released",
            "turn_rate_dps": "negative=cw, positive=ccw, zero=deadzone",
        },
        "limitations": [
            "한 착용자와 한 수집 세션 내부 시간 홀드아웃이다.",
            "동작별 원본 파일이므로 실제 자유 전환 로그의 독립 시험은 별도로 필요하다.",
            "점프 정답은 몸통 IMU 저가속 비행점과 직전 push-off peak로 만든 내부 물리 프록시다.",
            "다른 날, 전극 재부착, 다른 착용자 일반화는 검증하지 않았다.",
            "이 학습 실행은 Windows CDC, 키 입력, 실제 Minecraft 연결을 검증하지 않는다.",
        ],
    }


def _source_provenance(recordings: Sequence[Recording]) -> list[dict[str, Any]]:
    result: list[dict[str, Any]] = []
    for recording in recordings:
        train_end, dev_end = _split_boundaries(len(recording.frame))
        time_values = (
            recording.frame["time_s"].to_numpy(dtype=np.float64)
            if "time_s" in recording.frame
            else np.asarray([], dtype=np.float64)
        )
        result.append(
            {
                "source_key": recording.spec.key,
                "filename": recording.spec.filename,
                "path": str(recording.path),
                "sha256": recording.sha256,
                "rows": len(recording.frame),
                "duration_s_from_1khz_row_order": len(recording.frame) / SAMPLE_RATE_HZ,
                "state_label": recording.spec.state,
                "intensity_family": recording.spec.intensity_family,
                "intensity_value": recording.spec.intensity_value,
                "split_boundaries_samples": {
                    "train_fraction_end": train_end,
                    "dev_fraction_end": dev_end,
                    "embargo_each_side": EMBARGO_SAMPLES,
                },
                "time_s_strict_increase_fraction": (
                    float(np.mean(np.diff(time_values) > 0.0))
                    if len(time_values) > 1
                    else None
                ),
                "low_acc_candidate_count": len(recording.candidate_events),
                "accepted_jump_event_count": len(recording.jump_events),
            }
        )
    return result


def _event_manifest(recordings: Sequence[Recording]) -> list[dict[str, Any]]:
    output: list[dict[str, Any]] = []
    for recording in recordings:
        for event in recording.candidate_events:
            output.append(
                {
                    "source_key": recording.spec.key,
                    "accepted_as_jump_truth": recording.spec.state == "jump",
                    "split": _event_split(
                        int(event["force_peak_sample"]), len(recording.frame)
                    ),
                    **event,
                }
            )
    return output


def _write_test_predictions(
    path: Path,
    recordings: Sequence[Recording],
    rows: FeatureRows,
    state_model: ExtraTreesClassifier,
    jump_model: ExtraTreesClassifier,
    walk_model: ExtraTreesClassifier,
    jump_intensity_model: ExtraTreesClassifier,
) -> None:
    selected = rows.split == "test"
    state_probability = state_model.predict_proba(rows.features[selected])
    state_prediction = state_model.predict(rows.features[selected]).astype(np.int64)
    jump_probability = _positive_probability(jump_model, rows.features[selected])
    selected_indices = np.flatnonzero(selected)
    output: list[dict[str, Any]] = []
    for local, global_index in enumerate(selected_indices):
        recording = recordings[int(rows.recording_index[global_index])]
        record: dict[str, Any] = {
            "source_key": recording.spec.key,
            "source_end_sample": int(rows.end_samples[global_index]),
            "source_time_s_from_row_order": rows.end_samples[global_index]
            / SAMPLE_RATE_HZ,
            "true_state": STATE_CLASSES[int(rows.state_label[global_index])],
            "predicted_state": STATE_CLASSES[int(state_prediction[local])],
            "p_stand": float(state_probability[local, 0]),
            "p_walk": float(state_probability[local, 1]),
            "p_jump": float(state_probability[local, 2]),
            "true_jump_event_window": int(rows.jump_event_label[global_index]),
            "p_jump_event": float(jump_probability[local]),
        }
        if rows.walk_intensity_class[global_index] >= 0:
            probability = walk_model.predict_proba(
                rows.features[global_index : global_index + 1]
            )[0]
            record["predicted_intensity"] = float(
                probability @ INTENSITY_LEVELS["walk"][walk_model.classes_.astype(int)]
            )
        elif rows.jump_intensity_class[global_index] >= 0:
            probability = jump_intensity_model.predict_proba(
                rows.features[global_index : global_index + 1]
            )[0]
            record["predicted_intensity"] = float(
                probability
                @ INTENSITY_LEVELS["jump"][jump_intensity_model.classes_.astype(int)]
            )
        else:
            record["predicted_intensity"] = None
        output.append(record)
    pd.DataFrame(output).to_csv(path, index=False, float_format="%.8f")


def _prepare_output(output_dir: Path, source_dir: Path, replace: bool) -> None:
    resolved_output = output_dir.resolve()
    resolved_source = source_dir.resolve()
    if resolved_output == resolved_source or resolved_output == Path.home().resolve():
        raise SystemExit(f"안전하지 않은 출력 폴더입니다: {resolved_output}")
    if output_dir.exists():
        if not replace:
            raise SystemExit(f"출력 폴더가 이미 있습니다: {output_dir}")
        shutil.rmtree(output_dir)
    output_dir.mkdir(parents=True)


def main() -> None:
    parser = argparse.ArgumentParser(
        description="11개 H10 원본 CSV로 50 ms 연속 tree 판정기를 학습합니다."
    )
    parser.add_argument("--source-dir", type=Path, required=True)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--replace", action="store_true")
    args = parser.parse_args()

    recordings = _load_recordings(args.source_dir)
    _prepare_output(args.output_dir, args.source_dir, args.replace)

    # 1) 잠금 평가: 보정과 모델 fitting은 train에만, 선택은 dev에만 사용한다.
    evaluation_calibration = _evaluation_calibration(recordings)
    evaluation_rows = _build_feature_rows(recordings, evaluation_calibration)
    train_mask = evaluation_rows.split == "train"
    dev_mask = evaluation_rows.split == "dev"
    test_mask = evaluation_rows.split == "test"

    state_model, state_parameters, state_selection = _select_state_model(
        evaluation_rows
    )
    jump_event_model = _fit_jump_event_model(
        evaluation_rows, train_mask, random_state=2200
    )
    walk_intensity_model = _fit_intensity_model(
        evaluation_rows, "walk", train_mask, random_state=3200
    )
    jump_intensity_model = _fit_intensity_model(
        evaluation_rows, "jump", train_mask, random_state=3300
    )

    all_jump_probability = _positive_probability(
        jump_event_model, evaluation_rows.features
    )
    jump_threshold, threshold_selection = _select_jump_threshold(
        recordings, evaluation_rows, all_jump_probability
    )
    test_state_prediction = state_model.predict(
        evaluation_rows.features[test_mask]
    ).astype(np.int64)
    test_state_metrics = _state_metrics(
        evaluation_rows.state_label[test_mask], test_state_prediction
    )
    test_event_window = evaluation_rows.jump_event_label[test_mask]
    test_event_probability = all_jump_probability[test_mask]
    test_event_detection = _event_detection_metrics(
        recordings,
        evaluation_rows,
        all_jump_probability,
        "test",
        jump_threshold,
    )
    test_event_detection["window_average_precision"] = float(
        average_precision_score(test_event_window, test_event_probability)
    )
    test_event_detection["window_threshold_f1"] = float(
        f1_score(test_event_window, test_event_probability >= jump_threshold)
    )
    test_event_detection["window_threshold_precision"] = float(
        precision_score(
            test_event_window,
            test_event_probability >= jump_threshold,
            zero_division=0,
        )
    )
    test_event_detection["window_threshold_recall"] = float(
        recall_score(
            test_event_window,
            test_event_probability >= jump_threshold,
            zero_division=0,
        )
    )

    evaluation_crouch_model = _fit_crouch_model(recordings, all_rows=False)
    evaluation_model = _model_payload(
        calibration=evaluation_calibration,
        state_model=state_model,
        jump_event_model=jump_event_model,
        walk_intensity_model=walk_intensity_model,
        jump_intensity_model=jump_intensity_model,
        crouch_model=evaluation_crouch_model,
        jump_threshold=jump_threshold,
        training_rows=int(np.sum(train_mask)),
        training_scope="train_before_60_percent_only",
        selected_state_parameters=state_parameters,
    )

    evaluation = {
        "status": "locked_time_holdout_single_wearer_single_session",
        "split_policy": {
            "per_source_file": True,
            "train": "time < 60 percent",
            "dev": "60 percent <= time < 78 percent",
            "test": "time >= 78 percent",
            "embargo": "400 ms removed on both sides of the 60 and 78 percent boundaries",
            "clock": "1 kHz row order; time_s is not used because USB chunks repeat timestamps",
        },
        "window_counts": {
            "all": len(evaluation_rows.features),
            "train": int(np.sum(train_mask)),
            "dev": int(np.sum(dev_mask)),
            "test": int(np.sum(test_mask)),
            "embargo": int(np.sum(evaluation_rows.split == "embargo")),
        },
        "selection": {
            "state_model": {
                "chosen_parameters": state_parameters,
                "criterion": "highest dev macro F1, then accuracy",
                "candidates": state_selection,
            },
            "jump_event_threshold": {
                "chosen": jump_threshold,
                "criterion": (
                    "dev event F1, recall, precision, false triggers/min, latency; "
                    "one-to-one match [-400,+150] ms"
                ),
                "candidates": threshold_selection,
            },
        },
        "locked_test": {
            "state": test_state_metrics,
            "jump_event": test_event_detection,
            "walk_intensity": _intensity_metrics(
                evaluation_rows, walk_intensity_model, "walk", "test"
            ),
            "jump_intensity": _intensity_metrics(
                evaluation_rows, jump_intensity_model, "jump", "test"
            ),
        },
        "crouch_contract": evaluation_crouch_model,
        "claim_boundary": (
            "실제 센서 CSV의 파일 내부 시간 홀드아웃 결과다. 자유 전환을 한 번에 "
            "수집한 독립 로그, Windows CDC, Minecraft 키 출력 검증은 아니다."
        ),
    }

    (args.output_dir / "evaluation_model.json").write_text(
        json.dumps(evaluation_model, ensure_ascii=False, indent=2), encoding="utf-8"
    )
    (args.output_dir / "evaluation.json").write_text(
        json.dumps(evaluation, ensure_ascii=False, indent=2), encoding="utf-8"
    )
    pd.DataFrame(_event_manifest(recordings)).to_csv(
        args.output_dir / "jump_event_manifest.csv", index=False, float_format="%.8f"
    )
    _write_test_predictions(
        args.output_dir / "test_50ms_predictions.csv",
        recordings,
        evaluation_rows,
        state_model,
        jump_event_model,
        walk_intensity_model,
        jump_intensity_model,
    )

    # 2) 잠금 시험이 끝난 뒤 같은 선택값으로 모든 원본 행을 다시 fitting한다.
    final_calibration = _final_calibration(recordings)
    final_rows = _build_feature_rows(recordings, final_calibration)
    all_mask = np.ones(len(final_rows.features), dtype=bool)
    final_state_model = _new_forest(
        n_estimators=160,
        random_state=4200,
        **state_parameters,
    )
    final_state_model.fit(
        final_rows.features,
        final_rows.state_label,
        sample_weight=_source_balanced_weights(
            final_rows.state_label, final_rows.source_key
        ),
    )
    final_jump_event_model = _fit_jump_event_model(
        final_rows, all_mask, random_state=5200
    )
    final_walk_intensity_model = _fit_intensity_model(
        final_rows, "walk", all_mask, random_state=6200
    )
    final_jump_intensity_model = _fit_intensity_model(
        final_rows, "jump", all_mask, random_state=6300
    )
    final_model = _model_payload(
        calibration=final_calibration,
        state_model=final_state_model,
        jump_event_model=final_jump_event_model,
        walk_intensity_model=final_walk_intensity_model,
        jump_intensity_model=final_jump_intensity_model,
        crouch_model=_fit_crouch_model(recordings, all_rows=True),
        jump_threshold=jump_threshold,
        training_rows=len(final_rows.features),
        training_scope="all_rows_refit_after_locked_test",
        selected_state_parameters=state_parameters,
    )
    final_model["evaluation_artifact"] = "evaluation.json"
    final_model["provenance_artifact"] = "provenance.json"
    (args.output_dir / "h10_stream_motion_model.json").write_text(
        json.dumps(final_model, ensure_ascii=False, indent=2), encoding="utf-8"
    )

    provenance = {
        "created_utc": datetime.now(timezone.utc).isoformat(),
        "trainer": str(Path(__file__).resolve()),
        "schema_version": MODEL_SCHEMA_VERSION,
        "source_dir": str(args.source_dir.resolve()),
        "source_files": _source_provenance(recordings),
        "physical_jump_truth": {
            "acc_norm_filter": f"scipy.signal.medfilt kernel={FLIGHT_MEDIAN_KERNEL}",
            "flight_detector": {
                "call": "find_peaks(-acc_norm)",
                "height": -FLIGHT_MAX_ACC_NORM,
                "distance_samples": FLIGHT_MIN_DISTANCE_SAMPLES,
                "prominence": FLIGHT_MIN_PROMINENCE,
            },
            "event": "highest force peak in previous 600 ms before flight minimum",
            "positive_training_window_ms": [
                -JUMP_EVENT_POSITIVE_RADIUS_SAMPLES,
                JUMP_EVENT_POSITIVE_RADIUS_SAMPLES,
            ],
            "evaluation_match_window_ms": [
                -EVENT_MATCH_EARLY_SAMPLES,
                EVENT_MATCH_LATE_SAMPLES,
            ],
            "non_jump_low_acc_candidates": (
                "retained as negative training windows, including run hard negatives"
            ),
        },
        "files_written": [
            "evaluation_model.json",
            "h10_stream_motion_model.json",
            "evaluation.json",
            "provenance.json",
            "jump_event_manifest.csv",
            "test_50ms_predictions.csv",
        ],
    }
    (args.output_dir / "provenance.json").write_text(
        json.dumps(provenance, ensure_ascii=False, indent=2), encoding="utf-8"
    )

    summary = evaluation["locked_test"]
    print(
        json.dumps(
            {
                "model": str(args.output_dir / "h10_stream_motion_model.json"),
                "state_accuracy": summary["state"]["accuracy"],
                "state_macro_f1": summary["state"]["macro_f1"],
                "jump_event_f1": summary["jump_event"]["f1"],
                "jump_event_recall": summary["jump_event"]["recall"],
                "jump_event_precision": summary["jump_event"]["precision"],
                "jump_event_median_latency_ms": summary["jump_event"]["latency_ms"][
                    "median"
                ],
                "jump_threshold": jump_threshold,
            },
            ensure_ascii=False,
            indent=2,
        )
    )


if __name__ == "__main__":
    main()
