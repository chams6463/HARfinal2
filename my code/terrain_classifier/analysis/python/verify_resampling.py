"""
verify_resampling.py
===============================================================================
_ResamplePhaseCurve() 의 리샘플링 수식을 Python 으로 그대로 재현해서, C 코드를
빌드하지 않고도 로직이 의도대로 동작하는지 수치로 검증한다.

검증 항목:
 (1) 정상 케이스: 알려진 해석함수(sine)를 원시 버퍼처럼 10ms 간격으로 샘플링한
     뒤 리샘플링하면, 실제 함수값과 거의 일치해야 한다(선형보간 오차 수준).
 (2) 전환 케이스: "직전 stride 길이로 causal 추정"과 비교해서, stride 길이가
     달라졌을 때(예: WALK->RAMP_UP) 새 방식이 실제로 위상 왜곡을 없애는지
     정량적으로 보여준다.
 (3) 경계 케이스: stride_ms 가 raw_count*RAW_SAMPLE_PERIOD_MS 근처/이상일 때
     인덱스가 범위를 벗어나지 않는지(clamp 동작) 확인한다.
===============================================================================
"""

import numpy as np

RAW_SAMPLE_PERIOD_MS = 10
RC_THETA_POINTS = 101


def build_raw_buffer(signal_fn, stride_ms, period_ms=RAW_SAMPLE_PERIOD_MS):
    """C의 _AccumulateRawSample 을 흉내: [j*period,(j+1)*period) 창 평균."""
    n = int(stride_ms // period_ms)
    buf = []
    for j in range(n):
        t0, t1 = j * period_ms, (j + 1) * period_ms
        ts = np.linspace(t0, t1, 5, endpoint=False)
        buf.append(np.mean(signal_fn(ts)))
    return np.array(buf)


def resample_phase_curve(raw_buf, stride_ms, period_ms=RAW_SAMPLE_PERIOD_MS,
                          n_theta=RC_THETA_POINTS):
    """C 의 _ResamplePhaseCurve() 를 그대로 재현."""
    raw_count = len(raw_buf)
    out = np.zeros(n_theta)
    if raw_count < 2:
        return None
    last_j = raw_count - 2
    for i in range(n_theta):
        t_target = (i / (n_theta - 1)) * stride_ms
        pos = (t_target / period_ms) - 0.5
        pos = max(pos, 0.0)
        j0 = int(pos)
        j0 = min(j0, last_j)
        frac = pos - j0
        frac = min(max(frac, 0.0), 1.0)
        a, b = raw_buf[j0], raw_buf[j0 + 1]
        out[i] = a + (b - a) * frac
    return out


def causal_bin_old_way(signal_fn, stride_ms, prev_period_ms, n_theta=RC_THETA_POINTS,
                        dt_ms=1):
    """구버전(직전 stride 길이로 causal 추정) 방식 재현 — 비교용."""
    out = np.full(n_theta, np.nan)
    t = 0
    while t <= stride_ms:
        theta = t / prev_period_ms
        if theta <= 1.0:
            bin_idx = int(round(theta * (n_theta - 1)))
            bin_idx = min(bin_idx, n_theta - 1)
            out[bin_idx] = float(signal_fn(np.array([t]))[0])
        t += dt_ms
    return out


def main():
    print("=" * 78)
    print(" _ResamplePhaseCurve 수치 검증 (Python 재현)")
    print("=" * 78)

    # ---------------- (1) 정상 케이스: sine 함수 복원 ----------------
    print("\n###### (1) 해석함수 복원 정확도 (stride=1100ms, 그 stride 자체 길이로 리샘플) ######")
    stride_ms = 1100
    true_fn = lambda t: np.sin(2 * np.pi * t / stride_ms) + 0.3 * np.sin(4 * np.pi * t / stride_ms)

    raw = build_raw_buffer(true_fn, stride_ms)
    curve = resample_phase_curve(raw, stride_ms)

    theta_grid = np.linspace(0, 1, RC_THETA_POINTS)
    true_vals = true_fn(theta_grid * stride_ms)
    err = np.abs(curve - true_vals)
    print("  raw_count=%d, max|err|=%.4f, rms|err|=%.4f  (신호 진폭~1.3 대비)"
          % (len(raw), err.max(), np.sqrt(np.mean(err ** 2))))
    print("  판정:", "OK — 선형보간 오차 수준" if err.max() < 0.05 else "FAIL — 오차 과대")

    # ---------------- (2) 전환 케이스: causal 추정 대비 왜곡 제거 확인 ----------------
    print("\n###### (2) 전환 stride — 구버전(causal) vs 신버전(사후 리샘플링) ######")
    print("WALK(prev=1100ms) 다음에 RAMP_UP(현재=1350ms) 로 늘어나는 stride를 가정.")
    print("thigh_GYROMAG 스타일: theta=0.65 부근에 물리적 피크가 있는 신호로 모사.")

    prev_period = 1100
    cur_stride = 1350
    peak_theta = 0.65

    def gyromag_like(t):
        th = np.asarray(t, dtype=float) / cur_stride
        return np.exp(-((th - peak_theta) ** 2) / (2 * 0.05 ** 2))

    true_peak_theta = peak_theta  # 정의상 실제 피크는 theta=0.65 (현재 stride 기준)

    # 신버전
    raw2 = build_raw_buffer(gyromag_like, cur_stride)
    new_curve = resample_phase_curve(raw2, cur_stride)
    new_peak_theta = theta_grid[np.argmax(new_curve)]

    # 구버전 (causal: 직전 stride 길이 1100ms 로 나눔 -> theta>1 구간은 버려짐)
    old_curve = causal_bin_old_way(gyromag_like, cur_stride, prev_period)
    valid_mask = ~np.isnan(old_curve)
    if valid_mask.any():
        old_peak_theta = theta_grid[valid_mask][np.nanargmax(old_curve[valid_mask])]
    else:
        old_peak_theta = float('nan')
    n_missing = int(np.sum(~valid_mask))

    print("  실제 피크(진짜 theta 기준)     : %.3f" % true_peak_theta)
    print("  신버전(사후 리샘플링) 피크위치 : %.3f  (오차 %.3f)"
          % (new_peak_theta, abs(new_peak_theta - true_peak_theta)))
    print("  구버전(causal 추정) 피크위치   : %.3f  (오차 %.3f)"
          % (old_peak_theta, abs(old_peak_theta - true_peak_theta)))
    print("  구버전에서 유실된 bin 수       : %d / %d (theta>1 로 클리핑됨)"
          % (n_missing, RC_THETA_POINTS))
    print("  판정:", "OK — 신버전이 훨씬 정확하고 유실 없음"
          if abs(new_peak_theta - true_peak_theta) < abs(old_peak_theta - true_peak_theta)
          else "FAIL")

    # ---------------- (3) 경계 케이스 ----------------
    print("\n###### (3) 경계 케이스 — 인덱스 범위 안전성 ######")
    for stride_ms_case, label in [(500, "최소 stride"), (3000, "최대 stride"),
                                   (505, "raw_count 최소치 근처")]:
        raw3 = build_raw_buffer(lambda t: np.ones_like(t, dtype=float), stride_ms_case)
        curve3 = resample_phase_curve(raw3, stride_ms_case)
        ok = curve3 is not None and np.all(np.isfinite(curve3))
        print("  stride=%dms (%s): raw_count=%d  %s"
              % (stride_ms_case, label, len(raw3), "OK" if ok else "FAIL"))

    # raw_count 부족 (표본 < 2)
    curve4 = resample_phase_curve(np.array([1.0]), 500)
    print("  raw_count=1 (표본 부족): %s" % ("OK — None 반환(invalid 처리)" if curve4 is None else "FAIL"))


if __name__ == "__main__":
    main()
