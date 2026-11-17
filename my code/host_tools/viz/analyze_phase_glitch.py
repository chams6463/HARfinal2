"""
16E rep 1 phase glitch + ASCEND torque smoothness 분석.
"""
import csv
import numpy as np

BASE = r"D:\HARfinal2\HARfinal\PythonDecoder"

def load(name):
    rows = []
    with open(f"{BASE}\\{name}") as f:
        for r in csv.DictReader(f):
            rows.append({
                't': float(r['time_s']),
                'lh': float(r['EMG_LH_Env']),
                'rh': float(r['EMG_RH_Env']),
                'la': float(r['L_Angle']),
                'ra': float(r['R_Angle']),
                'lt': float(r['L_Torque']),
                'rt': float(r['R_Torque']),
                'ph': int(float(r['Squat_Ph'])),
                'in': int(float(r['User_intent'])),
            })
    return rows

for fname in ['notorque.csv', '16B.csv', '16E.csv']:
    rows = load(fname)
    print(f"\n=== {fname} ===")
    # phase transitions
    prev_ph = -1
    trans = []
    for i, r in enumerate(rows):
        if r['ph'] != prev_ph:
            trans.append((i, r['t'], prev_ph, r['ph']))
            prev_ph = r['ph']
    print(f"Total phase transitions: {len(trans)-1}")
    # count occurrences of phase
    from collections import Counter
    ph_count = Counter([r['ph'] for r in rows])
    print(f"Phase histogram: {dict(sorted(ph_count.items()))}")
    # detect short phase residencies (potential chattering)
    short_phases = []
    for k in range(1, len(trans)):
        i_now, t_now, prev, cur = trans[k]
        if k < len(trans)-1:
            i_next, t_next, _, _ = trans[k+1]
            duration_ms = (t_next - t_now) * 1000
        else:
            duration_ms = (rows[-1]['t'] - t_now) * 1000
        if duration_ms < 200:
            short_phases.append((t_now, cur, duration_ms, prev))
    print(f"Phases lasting < 200ms (potential chattering): {len(short_phases)}")
    for t, ph, dur, prv in short_phases[:20]:
        print(f"  t={t:.2f}s  phase={prv}→{ph}  duration={dur:.0f}ms")

    # ASCEND torque slope analysis
    asc_segments = []
    cur_seg = []
    for r in rows:
        if r['ph'] == 3:
            cur_seg.append(r)
        elif cur_seg:
            asc_segments.append(cur_seg)
            cur_seg = []
    if cur_seg:
        asc_segments.append(cur_seg)

    print(f"\nASCEND segments: {len(asc_segments)}")
    for j, seg in enumerate(asc_segments):
        if len(seg) < 3: continue
        torques = [(r['lt']+r['rt'])/2 for r in seg]
        # detect non-monotonic jumps (smoothness check)
        dt_torque = np.diff(torques)
        max_jump = np.max(np.abs(dt_torque))
        std_jump = np.std(dt_torque)
        # count sign changes (oscillations)
        sign_changes = np.sum(np.diff(np.sign(dt_torque)) != 0)
        print(f"  rep {j+1}: t={seg[0]['t']:.1f}~{seg[-1]['t']:.1f}s  "
              f"torque max_step={max_jump:.3f} Nm  std={std_jump:.3f}  "
              f"sign_changes={sign_changes}/{len(dt_torque)}")
