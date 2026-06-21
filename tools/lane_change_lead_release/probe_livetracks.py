# tools/lane_change_lead_release/probe_livetracks.py
"""seg 로그에서 liveTracks 존재/스키마/샘플을 출력해 extractor 설계를 확정한다."""
import sys
from openpilot.tools.lib.logreader import LogReader

def main(path):
    counts = {}
    sample = None
    for m in LogReader(path, sort_by_time=True):
        w = m.which()
        counts[w] = counts.get(w, 0) + 1
        if w == "liveTracks" and sample is None:
            sample = m.liveTracks
            print("liveTracks 타입:", type(sample))
            try:
                print("길이:", len(sample))
                if len(sample):
                    t0 = sample[0]
                    print("첫 트랙 필드:", [f for f in dir(t0) if not f.startswith("_")][:20])
                    print("dRel/yRel 샘플:", getattr(t0, "dRel", "NA"), getattr(t0, "yRel", "NA"))
            except TypeError:
                print("단일 메시지 필드:", [f for f in dir(sample) if not f.startswith("_")][:25])
    print("=== 메시지 카운트 ===")
    for k in ("carState", "modelV2", "radarState", "liveTracks", "longitudinalPlan"):
        print(f"  {k}: {counts.get(k, 0)}")

if __name__ == "__main__":
    main(sys.argv[1])
