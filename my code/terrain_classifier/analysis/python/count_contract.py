import os
import sys
sys.path.insert(0, r"D:\ifac_wearable_contest\verify\terrian_detection_minseo_h10_selected_clean8\terrian_detection_minseo\Camargo_matlab")
os.environ["CAMARGO_RAW_ROOT"] = r"D:\ifac_wearable_contest\verify\camargo_data_clean8"

import validate_6_continuous_online as core

subjects = sorted(p.name for p in core.RAW_ROOT.glob("AB*") if p.is_dir())
print("subjects found:", subjects)

total_trials = 0
total_transitions = 0
clean_trials = 0
clean_transitions = 0
per_subject = {}

for s in subjects:
    trials = core.load_evaluation_trials(s, None)
    n_trials = len(trials)
    n_trans = sum(len(core.transition_runs(t.labels)) for t in trials)
    per_subject[s] = (n_trials, n_trans)
    total_trials += n_trials
    total_transitions += n_trans
    if s != "AB11":
        clean_trials += n_trials
        clean_transitions += n_trans
    print(f"  {s}: trials={n_trials} transitions={n_trans}")

print()
print(f"ALL 9 (incl AB11):   trials={total_trials}  transitions={total_transitions}   (프롬프트 주장: 947 / 3788)")
print(f"CLEAN 8 (AB11 제외): trials={clean_trials}  transitions={clean_transitions}   (프롬프트 주장: 839 / 3356)")
