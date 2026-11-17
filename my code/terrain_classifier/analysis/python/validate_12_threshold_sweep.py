#!/usr/bin/env python3
"""margin 임계값 스윕 — 투표 게이트가 지형 전환을 억제하는 문제 완화.

배경
----
validate_11 결과에서 steady_event_vote_fraction = 52.5% 였다. 즉 완성된 안정
stride 의 절반가량이 margin 게이트에서 걸러져 3/2 투표 버퍼에 아예 들어가지
못한다. 3/2 다수결은 "확신 있는 후보가 2번 연속" 이어야 전환되는데, 그 앞단
게이트가 후보를 계속 버리면 전환이 구조적으로 억제된다.

이 스크립트가 바꾸는 것 / 안 바꾸는 것
------------------------------------
바꾼다:  MODE_CONFIRM_TH (안정 stride 게이트, 기본 0.35)
         MODE_TRANSITION_CONFIRM_TH (IDLE 전환증거 게이트, 기본 0.15)
안 바꾼다: VOTE_WINDOW=3, VOTE_MAJORITY=2, _PushVote/_UpdateModeFromVotes 로직
         (validate_9.simulate_revised 의 투표 버퍼 코드를 그대로 재사용한다)

과적합 방지
-----------
threshold 를 평가셋 전체에서 고르면 그 수치는 그 평가셋에 과적합된다.
따라서 clean8 을 둘로 나눈다:
    FIT     = AB06, AB07, AB08, AB09   <- 여기서만 스윕하고 고른다
    HOLDOUT = AB10, AB12, AB13, AB14   <- 고른 뒤 딱 한 번만 확인한다
holdout 결과를 보고 threshold 를 다시 고르지 않는다.

경계 사건은 validate_11 의 최종 채택안과 동일하게 heel-strike(trial.oracle_events,
H10 footContact 상한 proxy)를 쓴다. 전환 라벨은 scorer 만 사용한다.

실행:
    CAMARGO_RAW_ROOT=... python validate_12_threshold_sweep.py --calib-reps 3
"""
from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path
from typing import Any

import numpy as np
from scipy.io import loadmat

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))

import validate_6_continuous_online as core  # noqa: E402
import validate_9_improved_runtime as revised  # noqa: E402
from camargo_common import BANK_MODES, build_bank_from_subjects  # noqa: E402

FIT_SUBJECTS = ["AB06", "AB07", "AB08", "AB09"]
HOLDOUT_SUBJECTS = ["AB10", "AB12", "AB13", "AB14"]

BASELINE = (0.35, 0.15)

# (stable_margin, transition_margin) — 기본값에서 점진적으로 낮춘다.
GRID: list[tuple[float, float]] = [
    (0.35, 0.15),   # baseline (현재 채택본)
    (0.30, 0.15),
    (0.25, 0.12),
    (0.20, 0.10),
    (0.15, 0.08),
    (0.12, 0.05),
    (0.08, 0.03),
    (0.05, 0.00),
    (0.00, 0.00),   # 게이트 완전 개방 (극단 참조점)
]

METRIC_KEYS = (
    "steady_event_argmin_accuracy",
    "steady_event_vote_fraction",
    "correct_commit_fraction",
    "within_two_fraction",
    "wrong_mode_fraction",
    "early_target_fraction",
    "steady_movement_time_accuracy",
    "origin_correct_fraction",
    "median_delay_s",
)


def run_lane(trials, bank, idle, shift, calib,
             stable_th: float, transition_th: float) -> list[dict[str, Any]]:
    """threshold 두 개만 바꿔치기하고 validate_9 의 시뮬레이터를 그대로 돌린다."""
    saved_stable = core.CONFIRM_TH
    saved_transition = revised.TRANSITION_CONFIRM_TH
    core.CONFIRM_TH = stable_th
    revised.TRANSITION_CONFIRM_TH = transition_th
    try:
        scores = []
        for trial in trials:
            original = trial.events
            trial.events = trial.oracle_events   # heel-strike 경계(최종 채택안)
            try:
                outputs, logs = revised.simulate_revised(trial, bank, idle, shift, calib)
            finally:
                trial.events = original
            scores.append(core.score_trial(trial, outputs, logs))
        return scores
    finally:
        core.CONFIRM_TH = saved_stable
        revised.TRANSITION_CONFIRM_TH = saved_transition


def summarize(rows: list[dict[str, Any]]) -> dict[str, float | None]:
    out = {}
    for key in METRIC_KEYS:
        values = np.asarray([r[key] for r in rows if r[key] is not None], dtype=float)
        out[key] = float(np.mean(values)) if values.size else None
    return out


def evaluate_cohort(subjects: list[str], grid: list[tuple[float, float]],
                    calib_reps: int, seed: int) -> dict[str, dict[str, Any]]:
    """subject 별 데이터를 한 번만 로드하고 grid 전체를 그 위에서 돌린다."""
    population = loadmat(core.POPULATION, squeeze_me=True, struct_as_record=False)
    subject_curves = population["h_d_subject"]
    population_subjects = list(subject_curves._fieldnames)
    rng = np.random.default_rng(seed)

    # ledger[(stable, transition)][repeat] = [trial score, ...]
    ledger: dict[tuple[float, float], list[list[dict[str, Any]]]] = {
        combo: [[] for _ in range(calib_reps)] for combo in grid
    }

    for subject in subjects:
        train = [s for s in population_subjects if s != subject]
        bank_with_idle = build_bank_from_subjects(
            subject_curves, train, modes=BANK_MODES + ["STAND"]
        )
        bank = {mode: bank_with_idle[mode] for mode in BANK_MODES}
        idle = core.build_idle(bank_with_idle)
        _mhe_pool, heel_pool = core.load_calibration_pool(subject)
        trials = core.load_evaluation_trials(subject, None)

        for repeat in range(calib_reps):
            indices = rng.choice(len(heel_pool), size=8, replace=False)
            shift, calib = revised.revised_calibration(heel_pool, bank, indices)
            for combo in grid:
                scores = run_lane(trials, bank, idle, shift, calib, combo[0], combo[1])
                ledger[combo][repeat].extend(scores)
        print(f"  [{subject}] done", flush=True)

    result = {}
    for combo in grid:
        per_repeat = [core.summarize_scores(scores) for scores in ledger[combo]]
        result[f"{combo[0]:.2f}/{combo[1]:.2f}"] = summarize(per_repeat)
    return result


def print_table(title: str, table: dict[str, dict[str, Any]]) -> None:
    print(f"\n{title}")
    print("%-12s %9s %9s %9s %9s %9s %9s" % (
        "stab/trans", "argmin", "vote%", "commit", "within2", "WRONG", "steady"))
    print("-" * 74)
    for name, row in table.items():
        print("%-12s %9.4f %9.4f %9.4f %9.4f %9.4f %9.4f" % (
            name,
            row["steady_event_argmin_accuracy"],
            row["steady_event_vote_fraction"],
            row["correct_commit_fraction"],
            row["within_two_fraction"],
            row["wrong_mode_fraction"],
            row["steady_movement_time_accuracy"],
        ))


def print_tradeoff(table: dict[str, dict[str, Any]], base_key: str) -> None:
    """baseline 대비 얻은 것(commit)과 잃은 것(wrong_mode)의 교환비.

    ratio = Δwrong / Δcommit. 1.0 이면 정답전환 1%p 얻는 대가로 제3모드
    오출력 1%p 를 지불한다는 뜻이다. 낮을수록 유리한 거래다.
    """
    base = table[base_key]
    print("\n트레이드오프 (baseline 대비)")
    print("%-12s %11s %11s %11s" % ("stab/trans", "Δcommit", "Δwrong", "Δwrong/Δcommit"))
    print("-" * 74)
    for name, row in table.items():
        d_commit = row["correct_commit_fraction"] - base["correct_commit_fraction"]
        d_wrong = row["wrong_mode_fraction"] - base["wrong_mode_fraction"]
        if abs(d_commit) < 1e-9:
            ratio = "—"
        else:
            ratio = "%.2f" % (d_wrong / d_commit)
        print("%-12s %+11.4f %+11.4f %11s" % (name, d_commit, d_wrong, ratio))


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--calib-reps", type=int, default=3)
    parser.add_argument("--seed", type=int, default=20260825)
    parser.add_argument("--output", type=Path,
                        default=HERE / "threshold_sweep.json")
    args = parser.parse_args()

    print("=" * 74)
    print(" margin 임계값 스윕 — 3/2 다수결은 변경하지 않음")
    print(f" FIT     = {FIT_SUBJECTS}")
    print(f" HOLDOUT = {HOLDOUT_SUBJECTS}  (선택 후 1회만 확인)")
    print("=" * 74)

    print("\n[1] FIT cohort 스윕")
    fit_table = evaluate_cohort(FIT_SUBJECTS, GRID, args.calib_reps, args.seed)
    print_table("FIT 결과 (AB06,07,08,09)", fit_table)
    print_tradeoff(fit_table, f"{BASELINE[0]:.2f}/{BASELINE[1]:.2f}")

    # ---- 선택 규칙(결과 보기 전에 고정) ----
    # wrong_mode 를 baseline 이상으로 올리지 않는 선에서 correct_commit 최대화.
    base_key = f"{BASELINE[0]:.2f}/{BASELINE[1]:.2f}"
    base_wrong = fit_table[base_key]["wrong_mode_fraction"]
    base_commit = fit_table[base_key]["correct_commit_fraction"]

    print(f"\n[2] 선택 규칙: wrong_mode <= baseline({base_wrong:.4f}) 이면서 commit 최대")
    eligible = {
        name: row for name, row in fit_table.items()
        if row["wrong_mode_fraction"] <= base_wrong + 1e-9
    }
    if eligible:
        chosen_name = max(eligible, key=lambda n: eligible[n]["correct_commit_fraction"])
        rule = "wrong_mode<=baseline 제약 하 commit 최대"
    else:
        chosen_name = base_key
        rule = "제약을 만족하는 후보 없음 -> baseline 유지"
    chosen = tuple(float(x) for x in chosen_name.split("/"))
    print(f"    선택: {chosen_name}  ({rule})")
    print(f"    fit commit {base_commit:.4f} -> {fit_table[chosen_name]['correct_commit_fraction']:.4f}")

    # ---- holdout 확인 (baseline 과 선택안, 단 두 조합만) ----
    print("\n[3] HOLDOUT 확인 (baseline vs 선택안, 1회)")
    holdout_grid = [BASELINE] if chosen == BASELINE else [BASELINE, chosen]
    holdout_table = evaluate_cohort(HOLDOUT_SUBJECTS, holdout_grid,
                                    args.calib_reps, args.seed)
    print_table("HOLDOUT 결과 (AB10,12,13,14)", holdout_table)

    payload = {
        "schema": "minseo-threshold-sweep-v1",
        "contract": {
            "majority": "unchanged window=3 majority=2",
            "boundary": "heel-strike (H10 footContact upper-bound proxy)",
            "runtime_transition_label_access": False,
            "fit_subjects": FIT_SUBJECTS,
            "holdout_subjects": HOLDOUT_SUBJECTS,
            "baseline": {"stable": BASELINE[0], "transition": BASELINE[1]},
            "selection_rule": rule,
            "calibration_repeats": args.calib_reps,
            "seed": args.seed,
        },
        "chosen": {"stable": chosen[0], "transition": chosen[1]},
        "fit": fit_table,
        "holdout": holdout_table,
    }
    args.output.write_text(json.dumps(payload, ensure_ascii=False, indent=2),
                           encoding="utf-8")
    print(f"\n{args.output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
