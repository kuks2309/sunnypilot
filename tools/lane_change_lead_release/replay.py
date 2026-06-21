# tools/lane_change_lead_release/replay.py
"""baseline(항상 lead 유지) vs patched(LeadReleaseDecider) 비교 리포트."""
import argparse
import numpy as np
from openpilot.common.constants import CV
from tools.lane_change_lead_release.extractor import frames
from tools.lane_change_lead_release.decider import LeadReleaseDecider

# openpilot 가속 곡선 (longitudinal_planner get_max_accel)
A_CRUISE_MAX_BP = [0., 10.0, 25., 40.]
A_CRUISE_MAX_VALS = [1.6, 1.2, 0.8, 0.6]


def get_max_accel(v_ego):
    return float(np.interp(v_ego, A_CRUISE_MAX_BP, A_CRUISE_MAX_VALS))


def run(path):
    dec = LeadReleaseDecider()
    rows = []
    t_release = None
    t_off = None
    prev_state = None
    for t, f in frames(path):
        out = dec.update(f)
        if out.release and t_release is None:
            t_release = t
        if prev_state == "laneChangeStarting" and f.lane_change_state != "laneChangeStarting":
            if t_off is None:
                t_off = t  # starting 종료 ≈ baseline 해제 시점
        prev_state = f.lane_change_state
        rows.append((t, f, out))
    return rows, t_release, t_off


def estimate_speed_gain(rows, t_release, v_cruise_ms):
    """t_release 부터 get_max_accel 적분(크루즈 캡)으로 patched v 근사 vs baseline planV."""
    if t_release is None:
        return None
    v = None
    gain = []
    for t, f, out in rows:
        if t < t_release:
            continue
        if v is None:
            v = f.v_ego
        v = min(v + get_max_accel(v) * f.dt, v_cruise_ms)
        gain.append((t, f.v_ego * CV.MS_TO_KPH, v * CV.MS_TO_KPH))  # (t, baseline≈vEgo, patched추정)
    return gain


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("rlog")
    args = ap.parse_args()
    rows, t_release, t_off = run(args.rlog)
    print(f"=== {args.rlog} ===")
    print(f"patched lead 해제 시점 t_release = {t_release}")
    print(f"baseline 해제(≈starting 종료) t_off = {t_off}")
    if t_release is not None and t_off is not None:
        print(f"Δt(앞당김) = {round(t_off - t_release, 2)} s")
    if rows:
        vc = rows[-1][1].v_cruise
        gain = estimate_speed_gain(rows, t_release, vc)
        if gain:
            t_end, b_end, p_end = gain[-1]
            print(f"추정 속도(@{t_end}s): baseline≈{b_end:.0f} vs patched≈{p_end:.0f} km/h "
                  f"(추정 — 최종은 B단계 process_replay)")


if __name__ == "__main__":
    main()
