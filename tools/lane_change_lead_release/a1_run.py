#!/usr/bin/env python3
"""A1' 백그라운드 러너 (기기에서 실행).

1) qlog(작음)로 깜빡이 있는 세그먼트 사전필터 → 후보
2) 후보 rlog만 슬롯분석: laneChangeStarting 구간에서 slot0(현재) vs slot1/2(+2s/+4s)
   의 거리/속도 차(Δx, Δv) 집계
3) go(목표차선 더 열림: Δ>0) / stop(목표차선 앞차: Δ<0) 분류 출력

실행: PYTHONPATH=/data/openpilot /usr/local/venv/bin/python a1_run.py [base] [extra rlog...]
"""
import glob
import os
import statistics as st
import sys

from openpilot.tools.lib.logreader import LogReader

BASE = sys.argv[1] if len(sys.argv) > 1 else "/data/media/0/realdata"
EXTRA = sys.argv[2:]
MINF = 5


def has_blinker(qlog):
    try:
        for m in LogReader(qlog):
            if m.which() == "carState" and (m.carState.leftBlinker or m.carState.rightBlinker):
                return True
    except Exception:
        return False
    return False


def analyze(rlog):
    out = []
    cur = None
    try:
        for m in LogReader(rlog, sort_by_time=True):
            if m.which() != "modelV2":
                continue
            md = m.modelV2
            lcs = str(md.meta.laneChangeState).split(".")[-1]
            if lcs == "laneChangeStarting":
                if cur is None:
                    cur = {"dir": str(md.meta.laneChangeDirection).split(".")[-1], "rows": []}
                s = md.leadsV3
                if len(s) >= 3 and len(s[0].x) and s[0].prob > 0.5:
                    cur["rows"].append((s[0].x[0], s[1].x[0], s[2].x[0],
                                        s[0].v[0], s[1].v[0], s[2].v[0]))
            else:
                if cur and len(cur["rows"]) >= MINF:
                    out.append(cur)
                cur = None
        if cur and len(cur["rows"]) >= MINF:
            out.append(cur)
    except Exception:
        pass
    return out


print("=== A1' 시작 ===", flush=True)
segdirs = sorted(glob.glob(os.path.join(BASE, "*/")))
print("세그먼트 dir:", len(segdirs), flush=True)
cands = []
for d in segdirs:
    q = os.path.join(d, "qlog.zst")
    if os.path.exists(q) and has_blinker(q):
        r = os.path.join(d, "rlog.zst")
        if os.path.exists(r):
            cands.append(r)
cands += [e for e in EXTRA if os.path.exists(e)]
print("깜빡이 후보 세그먼트:", len(cands), flush=True)

found = []
for r in cands:
    for cur in analyze(r):
        found.append((r, cur))
print("차선변경(lead 있는):", len(found), flush=True)
print("seg          dir   | n  |  dx1  dx2 |  dv1   dv2 (km/h) | 해석", flush=True)
go = stop = neutral = 0
for r, cur in found:
    rows = cur["rows"]
    dx1 = st.mean(x[1] - x[0] for x in rows)
    dx2 = st.mean(x[2] - x[0] for x in rows)
    dv1 = st.mean((x[4] - x[3]) * 3.6 for x in rows)
    dv2 = st.mean((x[5] - x[3]) * 3.6 for x in rows)
    seg = os.path.basename(os.path.dirname(r))[-10:]
    if dx1 > 1 or dv1 > 2:
        tag = "열림(go)"; go += 1
    elif dx1 < -1 or dv1 < -2:
        tag = "앞차(stop)"; stop += 1
    else:
        tag = "중립"; neutral += 1
    print("%-11s %-5s | %2d | %+5.0f %+5.0f | %+5.1f %+5.1f | %s"
          % (seg, cur["dir"], len(rows), dx1, dx2, dv1, dv2, tag), flush=True)
print("=== 요약: 총 %d | 열림(go) %d | 앞차(stop) %d | 중립 %d ==="
      % (len(found), go, stop, neutral), flush=True)
print("=== A1' 완료 ===", flush=True)
