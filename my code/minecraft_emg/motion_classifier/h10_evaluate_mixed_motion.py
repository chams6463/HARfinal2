#!/usr/bin/env python3
"""학습에 쓰지 않은 원본 뒤쪽 블록을 섞어 50 ms 연속 판정을 검증한다.

서로 다른 원본을 붙인 경계 뒤 320 ms는 state 정확도에서 제외한다. 센서 행은
그대로 복사하고 time_s/seq_id만 혼합 파일의 연속 축으로 다시 기록한다.
"""
from __future__ import annotations

import argparse
import json
import math
import shutil
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import numpy as np
import pandas as pd
from scipy.signal import find_peaks, medfilt
from sklearn.metrics import confusion_matrix, f1_score, precision_recall_fscore_support

try:
    from tools.h10_stream_motion_model import (
        HOP_SAMPLES,
        MAX_HISTORY_SAMPLES,
        CausalMotionRuntime,
        FULL_CROUCH_FRACTION,
        load_stream_model,
    )
except ModuleNotFoundError:
    from h10_stream_motion_model import (  # type: ignore[no-redef]
        HOP_SAMPLES,
        MAX_HISTORY_SAMPLES,
        CausalMotionRuntime,
        FULL_CROUCH_FRACTION,
        load_stream_model,
    )


SOURCE_FILES = (
    "stand_1m.csv",
    "walk_1.csv",
    "walk_2.csv",
    "walk_3.csv",
    "run.csv",
    "jump_1.csv",
    "jump_2.csv",
    "jump_3.csv",
    "sit_stand.csv",
    "turn_clw.csv",
    "turn_ccw.csv",
)
STATE_LABELS = ("stand", "walk", "jump", "sit")


@dataclass(frozen=True)
class Block:
    block_id: str
    source_file: str
    start: int
    end: int
    nominal_state: str


def _aligned(value: float) -> int:
    return int(math.ceil(value / HOP_SAMPLES) * HOP_SAMPLES)


def _acc_norm(frame: pd.DataFrame) -> np.ndarray:
    return np.linalg.norm(
        frame[["imu_x", "imu_y", "imu_z"]].to_numpy(dtype=np.float64), axis=1
    )


def _jump_events(frame: pd.DataFrame) -> list[dict[str, int]]:
    """저중력 비행과 그 직전 600 ms의 이륙 힘 피크를 찾는다."""
    smoothed = medfilt(_acc_norm(frame), kernel_size=21)
    flights, _ = find_peaks(
        -smoothed,
        height=-1.5,
        distance=700,
        prominence=1.0,
    )
    events: list[dict[str, int]] = []
    for flight in flights:
        start = max(0, int(flight) - 600)
        search = smoothed[start : int(flight) + 1]
        local_peaks, _ = find_peaks(search, prominence=0.1)
        if len(local_peaks):
            local_force = int(local_peaks[int(np.argmax(search[local_peaks]))])
        else:
            local_force = int(np.argmax(search))
        force_peak = start + local_force
        events.append({"force_peak": force_peak, "flight_min": int(flight)})
    return events


def _runs(mask: np.ndarray) -> list[tuple[int, int]]:
    transitions = np.diff(np.r_[False, mask.astype(bool), False].astype(np.int8))
    return [
        (int(start), int(end))
        for start, end in zip(
            np.flatnonzero(transitions == 1), np.flatnonzero(transitions == -1)
        )
    ]


def _offline_full_crouch_truth(frame: pd.DataFrame) -> np.ndarray:
    """각 반복의 실제 정점을 사용한 비인과 offline full-crouch 정답."""
    left = frame["l_angle"].to_numpy(dtype=np.float64)
    right = frame["r_angle"].to_numpy(dtype=np.float64)
    mean_angle = 0.5 * (left + right)
    smooth = (
        pd.Series(mean_angle)
        .rolling(101, center=True, min_periods=1)
        .mean()
        .to_numpy(dtype=np.float64)
    )
    low, high = np.quantile(smooth, [0.10, 0.90])
    plateaus = [
        pair for pair in _runs(smooth >= 0.5 * (low + high)) if pair[1] - pair[0] >= 500
    ]
    left_mean = (
        pd.Series(left).rolling(50, min_periods=50).mean().to_numpy(dtype=np.float64)
    )
    right_mean = (
        pd.Series(right).rolling(50, min_periods=50).mean().to_numpy(dtype=np.float64)
    )
    left_velocity = (left_mean - np.roll(left_mean, 100)) / 0.1
    right_velocity = (right_mean - np.roll(right_mean, 100)) / 0.1
    left_velocity[:150] = np.nan
    right_velocity[:150] = np.nan
    truth = np.zeros(len(frame), dtype=bool)
    for start, end in plateaus:
        baseline_start = max(0, start - 700)
        left_low = float(np.quantile(left[baseline_start:start], 0.10))
        right_low = float(np.quantile(right[baseline_start:start], 0.10))
        left_high = float(np.quantile(left[start:end], 0.99))
        right_high = float(np.quantile(right[start:end], 0.99))
        left_threshold = left_low + FULL_CROUCH_FRACTION * (left_high - left_low)
        right_threshold = right_low + FULL_CROUCH_FRACTION * (right_high - right_low)
        local = (
            (left_mean[start:end] >= left_threshold)
            & (right_mean[start:end] >= right_threshold)
            & (np.abs(left_velocity[start:end]) <= 30.0)
            & (np.abs(right_velocity[start:end]) <= 30.0)
        )
        truth[start:end] = local
    return truth


def _generic_block(
    frames: dict[str, pd.DataFrame],
    source_file: str,
    block_id: str,
    *,
    offset_s: float,
    duration_s: float,
    state: str,
) -> Block:
    count = len(frames[source_file])
    start = _aligned(0.80 * count + offset_s * 1000.0)
    end = min(count - count % HOP_SAMPLES, start + int(duration_s * 1000.0))
    if end - start < 1000:
        raise RuntimeError(f"혼합 블록이 너무 짧습니다: {block_id}")
    return Block(block_id, source_file, start, end, state)


def _jump_block(
    frames: dict[str, pd.DataFrame], source_file: str, block_id: str
) -> Block:
    count = len(frames[source_file])
    candidates = [
        event
        for event in _jump_events(frames[source_file])
        if event["force_peak"] >= 0.80 * count
    ]
    if len(candidates) < 2:
        raise RuntimeError(f"시험용 점프 사건이 부족합니다: {source_file}")
    start = max(_aligned(0.78 * count), _aligned(candidates[0]["force_peak"] - 500))
    last = candidates[min(len(candidates) - 1, 3)]["flight_min"] + 400
    end = min(count - count % HOP_SAMPLES, _aligned(last))
    return Block(block_id, source_file, start, end, "jump")


def _sit_block(frames: dict[str, pd.DataFrame]) -> Block:
    frame = frames["sit_stand.csv"]
    truth = _offline_full_crouch_truth(frame)
    runs = [pair for pair in _runs(truth) if pair[0] >= 0.78 * len(frame)]
    if not runs:
        raise RuntimeError("시험용 full-crouch plateau가 없습니다.")
    start = max(_aligned(0.78 * len(frame)), _aligned(runs[0][0] - 1200))
    end = min(len(frame) - len(frame) % HOP_SAMPLES, _aligned(runs[-1][1] + 1200))
    return Block("crouch", "sit_stand.csv", start, end, "stand")


def _build_blocks(frames: dict[str, pd.DataFrame]) -> list[Block]:
    return [
        _generic_block(frames, "stand_1m.csv", "stand_a", offset_s=0.0, duration_s=2.5, state="stand"),
        _generic_block(frames, "walk_1.csv", "walk_low", offset_s=0.0, duration_s=4.0, state="walk"),
        _jump_block(frames, "jump_1.csv", "jump_low"),
        _generic_block(frames, "stand_1m.csv", "stand_b", offset_s=3.0, duration_s=2.5, state="stand"),
        _generic_block(frames, "walk_2.csv", "walk_mid", offset_s=0.0, duration_s=4.0, state="walk"),
        _jump_block(frames, "jump_2.csv", "jump_mid"),
        _sit_block(frames),
        _generic_block(frames, "stand_1m.csv", "stand_c", offset_s=6.0, duration_s=2.5, state="stand"),
        _generic_block(frames, "walk_3.csv", "walk_high", offset_s=0.0, duration_s=4.0, state="walk"),
        _jump_block(frames, "jump_3.csv", "jump_high"),
        _generic_block(frames, "run.csv", "run", offset_s=0.0, duration_s=4.0, state="walk"),
        _generic_block(frames, "turn_clw.csv", "turn_cw", offset_s=0.0, duration_s=3.0, state="stand"),
        _generic_block(frames, "turn_ccw.csv", "turn_ccw", offset_s=0.0, duration_s=3.0, state="stand"),
        _generic_block(frames, "stand_1m.csv", "stand_d", offset_s=9.0, duration_s=2.0, state="stand"),
    ]


def _mixed_frame(
    frames: dict[str, pd.DataFrame], blocks: list[Block]
) -> tuple[pd.DataFrame, list[dict[str, Any]]]:
    parts: list[pd.DataFrame] = []
    event_records: list[dict[str, Any]] = []
    mixed_offset = 0
    crouch_truth = _offline_full_crouch_truth(frames["sit_stand.csv"])
    for block in blocks:
        source = frames[block.source_file]
        part = source.iloc[block.start : block.end].copy().reset_index(drop=True)
        part.insert(0, "source_seq_id", part["seq_id"].to_numpy(copy=True))
        part.insert(0, "source_time_s", part["time_s"].to_numpy(copy=True))
        part["source_file"] = block.source_file
        part["source_row"] = np.arange(block.start, block.end, dtype=np.int64) + 1
        part["block_id"] = block.block_id
        part["true_state"] = block.nominal_state
        part["true_crouch_active"] = False
        part["true_jump_event"] = False
        part["state_eval_valid"] = True
        # 50 ms grid에서 300 ms 결과까지는 이전 블록 표본이 320 ms history에
        # 남고, 350 ms 결과부터 history 전체가 현재 블록이다.
        warmup_last_row = MAX_HISTORY_SAMPLES - HOP_SAMPLES - 1
        part.loc[:warmup_last_row, "state_eval_valid"] = False

        if block.source_file == "sit_stand.csv":
            local_crouch = crouch_truth[block.start : block.end]
            part["true_crouch_active"] = local_crouch
            part.loc[local_crouch, "true_state"] = "sit"
        if block.source_file.startswith("jump_"):
            events = [
                event
                for event in _jump_events(source)
                if block.start <= event["force_peak"] < block.end
            ]
            if events:
                state_start = max(block.start, events[0]["force_peak"] - 400)
                part.loc[: state_start - block.start - 1, "true_state"] = "stand"
            for event_index, event in enumerate(events, start=1):
                local = event["force_peak"] - block.start
                part.loc[local, "true_jump_event"] = True
                lo = max(0, local - 400)
                hi = min(len(part), local + 151)
                part.loc[lo : hi - 1, "state_eval_valid"] = False
                event_records.append(
                    {
                        "event_id": f"{block.block_id}_{event_index:02d}",
                        "block_id": block.block_id,
                        "source_file": block.source_file,
                        "source_force_peak_row": event["force_peak"] + 1,
                        "mixed_force_peak_sample": mixed_offset + local,
                        # trainer의 exclusive end-sample 좌표와 동일하게 +1 한다.
                        "mixed_force_peak_ms": mixed_offset + local + 1,
                    }
                )

        parts.append(part)
        mixed_offset += len(part)

    mixed = pd.concat(parts, ignore_index=True)
    mixed["time_s"] = np.arange(len(mixed), dtype=np.float64) / 1000.0
    mixed["seq_id"] = np.arange(len(mixed), dtype=np.int64) % 65536
    return mixed, event_records


def _match_events(
    predictions: list[int], truth: list[int]
) -> tuple[list[dict[str, int]], int, int]:
    used: set[int] = set()
    matches: list[dict[str, int]] = []
    for prediction in predictions:
        candidates = [
            (abs(prediction - target), index, target)
            for index, target in enumerate(truth)
            if index not in used and target - 400 <= prediction <= target + 150
        ]
        if not candidates:
            continue
        _, index, target = min(candidates)
        used.add(index)
        matches.append(
            {
                "prediction_ms": prediction,
                "truth_force_peak_ms": target,
                "latency_ms": prediction - target,
            }
        )
    return matches, len(predictions) - len(matches), len(truth) - len(matches)


def _evaluate(
    mixed: pd.DataFrame,
    event_records: list[dict[str, Any]],
    model: dict[str, Any],
) -> tuple[dict[str, Any], pd.DataFrame]:
    runtime = CausalMotionRuntime(model)
    rows: list[dict[str, Any]] = []
    usable = len(mixed) - len(mixed) % HOP_SAMPLES
    for start in range(0, usable, HOP_SAMPLES):
        chunk = mixed.iloc[start : start + HOP_SAMPLES]
        result = runtime.update_50ms(chunk)
        rows.append(
            {
                "tick": start // HOP_SAMPLES + 1,
                "time_ms": start + HOP_SAMPLES,
                "block_id": str(chunk["block_id"].iloc[-1]),
                "source_file": str(chunk["source_file"].iloc[-1]),
                "true_state": str(chunk["true_state"].iloc[-1]),
                "true_crouch_active": bool(chunk["true_crouch_active"].iloc[-1]),
                "state_eval_valid": bool(chunk["state_eval_valid"].all()),
                **result,
            }
        )
    prediction = pd.DataFrame(rows)
    state_mask = prediction["state_eval_valid"].to_numpy(dtype=bool)
    truth_state = prediction.loc[state_mask, "true_state"].to_numpy(dtype=object)
    pred_state = prediction.loc[state_mask, "state"].to_numpy(dtype=object)
    labels = [label for label in STATE_LABELS if np.any(truth_state == label)]
    per_precision, per_recall, per_f1, per_support = precision_recall_fscore_support(
        truth_state, pred_state, labels=labels, zero_division=0
    )
    state_metrics = {
        "valid_50ms_ticks": int(np.sum(state_mask)),
        "accuracy": float(np.mean(truth_state == pred_state)),
        "macro_f1": float(f1_score(truth_state, pred_state, labels=labels, average="macro")),
        "labels": labels,
        "confusion_matrix": confusion_matrix(truth_state, pred_state, labels=labels).tolist(),
        "per_state": {
            label: {
                "precision": float(per_precision[index]),
                "recall": float(per_recall[index]),
                "f1": float(per_f1[index]),
                "support": int(per_support[index]),
            }
            for index, label in enumerate(labels)
        },
    }

    truth_crouch = prediction["true_crouch_active"].to_numpy(dtype=bool)
    pred_crouch = prediction["crouch_active"].to_numpy(dtype=bool)
    crouch_tp = int(np.sum(truth_crouch & pred_crouch))
    crouch_fp = int(np.sum(~truth_crouch & pred_crouch))
    crouch_fn = int(np.sum(truth_crouch & ~pred_crouch))
    truth_crouch_runs = _runs(truth_crouch)
    predicted_crouch_runs = _runs(pred_crouch)
    detected_crouch_runs = sum(
        bool(np.any(pred_crouch[start:end])) for start, end in truth_crouch_runs
    )
    false_crouch_runs = sum(
        not bool(np.any(truth_crouch[start:end]))
        for start, end in predicted_crouch_runs
    )
    non_sit_source_false_ticks = int(
        np.sum(
            pred_crouch
            & prediction["source_file"].ne("sit_stand.csv").to_numpy(dtype=bool)
        )
    )
    crouch_metrics = {
        "true_ticks": int(np.sum(truth_crouch)),
        "predicted_ticks": int(np.sum(pred_crouch)),
        "precision": float(crouch_tp / (crouch_tp + crouch_fp)) if crouch_tp + crouch_fp else 0.0,
        "recall": float(crouch_tp / (crouch_tp + crouch_fn)) if crouch_tp + crouch_fn else 0.0,
        "false_positive_ticks": crouch_fp,
        "false_negative_ticks": crouch_fn,
        "truth_plateau_events": len(truth_crouch_runs),
        "detected_plateau_events": detected_crouch_runs,
        "missed_plateau_events": len(truth_crouch_runs) - detected_crouch_runs,
        "false_plateau_events": false_crouch_runs,
        "plateau_event_recall": float(
            detected_crouch_runs / len(truth_crouch_runs)
        )
        if truth_crouch_runs
        else 0.0,
        "non_sit_source_false_positive_ticks": non_sit_source_false_ticks,
    }

    predicted_events = prediction.loc[
        prediction["jump_started"].astype(bool), "time_ms"
    ].astype(int).tolist()
    true_events = [int(row["mixed_force_peak_ms"]) for row in event_records]
    matches, false_events, missed_events = _match_events(predicted_events, true_events)
    precision = len(matches) / len(predicted_events) if predicted_events else 0.0
    recall = len(matches) / len(true_events) if true_events else 0.0
    latencies = [row["latency_ms"] for row in matches]
    negative_false = int(
        np.sum(
            prediction["jump_started"].astype(bool)
            & ~prediction["source_file"].str.startswith("jump_")
        )
    )
    jump_metrics = {
        "truth_events": len(true_events),
        "predicted_events": len(predicted_events),
        "matched_events": len(matches),
        "false_events": false_events,
        "missed_events": missed_events,
        "precision": float(precision),
        "recall": float(recall),
        "f1": float(2.0 * precision * recall / (precision + recall))
        if precision + recall
        else 0.0,
        "latency_ms_median_vs_takeoff_force_peak": float(np.median(latencies))
        if latencies
        else None,
        "latency_ms_p95_vs_takeoff_force_peak": float(np.quantile(latencies, 0.95))
        if latencies
        else None,
        "nonjump_false_events": negative_false,
        "matching_window_ms": [-400, 150],
        "matches": matches,
    }
    metrics = {
        "status": "same_wearer_same_session_test_tail_synthetic_splice_proxy",
        "runtime_reset_count": 0,
        "output_period_ms": 50,
        "causal_history_ms": MAX_HISTORY_SAMPLES,
        "first_valid_result_after_splice_ms": int(
            math.ceil(MAX_HISTORY_SAMPLES / HOP_SAMPLES) * HOP_SAMPLES
        ),
        "state": state_metrics,
        "jump_events": jump_metrics,
        "full_crouch": crouch_metrics,
        "limits": [
            "동작별 원본의 시험용 뒤쪽 블록을 이어 붙인 혼합 프록시다.",
            "원본 사이 splice와 그 뒤 320 ms는 state 지표에서 제외했다.",
            "점프 시점 정답은 저중력 비행 직전 acc_norm 힘 피크 자동 프록시다.",
            "새로운 자유 전환 실측 로그와 다른 착용일 일반화는 검증하지 않았다.",
        ],
    }
    return metrics, prediction


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--source-dir", type=Path, required=True)
    parser.add_argument("--model", type=Path, required=True)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--replace", action="store_true")
    args = parser.parse_args()
    if args.output_dir.exists():
        if not args.replace:
            raise SystemExit(f"출력 폴더가 이미 있습니다: {args.output_dir}")
        shutil.rmtree(args.output_dir)
    args.output_dir.mkdir(parents=True)
    frames = {
        filename: pd.read_csv(args.source_dir / filename) for filename in SOURCE_FILES
    }
    blocks = _build_blocks(frames)
    mixed, events = _mixed_frame(frames, blocks)
    model = load_stream_model(args.model)
    scope = str(model.get("training", {}).get("scope", ""))
    if scope != "train_before_60_percent_only":
        raise SystemExit(
            "혼합 holdout에는 evaluation_model.json만 사용할 수 있습니다: "
            f"training.scope={scope or 'missing'}"
        )
    metrics, predictions = _evaluate(mixed, events, model)
    mixed.to_csv(args.output_dir / "mixed_holdout.csv", index=False, float_format="%.8f")
    predictions.to_csv(
        args.output_dir / "mixed_50ms_predictions.csv", index=False, float_format="%.8f"
    )
    pd.DataFrame(events).to_csv(args.output_dir / "mixed_jump_events.csv", index=False)
    (args.output_dir / "mixed_evaluation.json").write_text(
        json.dumps(metrics, ensure_ascii=False, indent=2), encoding="utf-8"
    )
    (args.output_dir / "mixed_provenance.json").write_text(
        json.dumps(
            {
                "source_dir": str(args.source_dir),
                "model": str(args.model),
                "blocks": [block.__dict__ for block in blocks],
                "sensor_rows_are_original": True,
                "rewritten_columns": ["time_s", "seq_id"],
            },
            ensure_ascii=False,
            indent=2,
        ),
        encoding="utf-8",
    )
    print(json.dumps(metrics, ensure_ascii=False))


if __name__ == "__main__":
    main()
