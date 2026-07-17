#!/usr/bin/env python3
"""Adjacent Lane Bias §7.1 선결 검증 프로브 (기기 offroad 실행).

설계 스펙: docs/superpowers/specs/2026-07-13-adjacent-lane-bias-design.md

두 가지를 확정한다:

 A) BSM(Blind Spot Monitoring) 신호 실재 — qlog 전수 스캔
    carState.leftBlindspot/rightBlindspot 가 실주행에서 실제로 토글되는가?
    (Tesla DAS_status.DAS_blindSpotRearLeft/Right → carstate.py 파싱 경로가 살아있는가)
    토글이 0이면 이 기능은 성립 불가 → 감지 소스 재설계 필요.
    덤으로 XOR(정확히 한쪽) 조건이 60km/h 이상에서 얼마나 자주 성립하는지 =
    기능이 실제로 작동했을 시간 비율.

 B) 차선 y 부호 & 기준값 — rlog 정밀 분석
    openpilot ego frame 은 y+ = 좌. 따라서 기대값:
      laneLines[1].y[0] (좌측 마커) ≈ +1.8m
      laneLines[2].y[0] (우측 마커) ≈ -1.8m
      measured = -(y1 + y2) / 2 ≈ 0  (양수 = 중앙보다 왼쪽)
      차선폭 = y1 - y2 ≈ 3.5m  (sanity check)

실행 (기기, 반드시 주차/offroad 상태에서):
    PYTHONPATH=/data/openpilot nice -n 10 /usr/local/venv/bin/python \
      tools/adjacent_lane_bias/probe_bsm_lane.py [base] [extra rlog...]
"""
import glob
import os
import statistics as st
import sys

from openpilot.tools.lib.logreader import LogReader

BASE = sys.argv[1] if len(sys.argv) > 1 else "/data/media/0/realdata"
EXTRA = sys.argv[2:]

SPEED_GATE_MS = 60 / 3.6  # 스펙 §2.2 속도 게이트
PROB_GATE = 0.5           # 스펙 §5 저확률 차선 게이트
MIN_FRAMES = 20           # 통계 최소 프레임
MAX_RLOG = 8              # rlog 정밀분석 상한 (기기 부하 방지)


# ─────────────────────────── A) BSM 전수 스캔 (qlog) ───────────────────────────

def scan_bsm(qlog):
    """qlog 1개 → BSM 통계. carState 만 읽으므로 가볍다."""
    s = {"frames": 0, "left": 0, "right": 0, "xor": 0, "both": 0,
         "xor_fast": 0, "fast": 0, "trans_l": 0, "trans_r": 0, "vmax": 0.0}
    pl = pr = False
    try:
        for m in LogReader(qlog):
            if m.which() != "carState":
                continue
            cs = m.carState
            l, r = bool(cs.leftBlindspot), bool(cs.rightBlindspot)
            v = float(cs.vEgo)
            s["frames"] += 1
            s["left"] += l
            s["right"] += r
            s["both"] += (l and r)
            s["xor"] += (l != r)
            if v >= SPEED_GATE_MS:
                s["fast"] += 1
                s["xor_fast"] += (l != r)
            s["trans_l"] += (l != pl)
            s["trans_r"] += (r != pr)
            if (l or r) and v > s["vmax"]:
                s["vmax"] = v
            pl, pr = l, r
    except Exception as e:
        print(f"  ! qlog 실패 {os.path.basename(os.path.dirname(qlog))}: {e}", flush=True)
    return s


# ─────────────────────── B) 차선 부호/기준값 분석 (rlog) ───────────────────────

def analyze_lanes(rlog):
    """rlog 1개 → 차선 y 통계. 직선·고속·고확률·차선변경 아님 구간만."""
    y1s, y2s, meas, widths = [], [], [], []
    lc_trend = []  # (direction, measured 변화량) — 부호 2차 확인용
    v_ego = 0.0
    lc_prev, lc_start_meas = "off", None
    try:
        for m in LogReader(rlog, sort_by_time=True):
            w = m.which()
            if w == "carState":
                v_ego = float(m.carState.vEgo)
                continue
            if w != "modelV2":
                continue
            md = m.modelV2
            if len(md.laneLines) < 3 or len(md.laneLineProbs) < 3:
                continue
            if not len(md.laneLines[1].y) or not len(md.laneLines[2].y):
                continue

            y1 = float(md.laneLines[1].y[0])
            y2 = float(md.laneLines[2].y[0])
            p1 = float(md.laneLineProbs[1])
            p2 = float(md.laneLineProbs[2])
            measured = -(y1 + y2) / 2.0
            lcs = str(md.meta.laneChangeState).split(".")[-1]

            # 부호 2차 확인: 차선변경 구간의 measured 변화 방향
            if lcs == "laneChangeStarting":
                if lc_prev != "laneChangeStarting":
                    lc_start_meas = measured
                    lc_dir = str(md.meta.laneChangeDirection).split(".")[-1]
                    lc_trend.append([lc_dir, measured, measured])
                elif lc_trend:
                    lc_trend[-1][2] = measured
            lc_prev = lcs

            # 기준값 통계: 정상 직진 주행 구간만
            if lcs != "laneChangeOff" or v_ego < SPEED_GATE_MS:
                continue
            if p1 < PROB_GATE or p2 < PROB_GATE:
                continue
            y1s.append(y1)
            y2s.append(y2)
            meas.append(measured)
            widths.append(y1 - y2)
    except Exception as e:
        print(f"  ! rlog 실패 {os.path.basename(os.path.dirname(rlog))}: {e}", flush=True)
    return y1s, y2s, meas, widths, lc_trend


def stats(v):
    if len(v) < 2:
        return None
    return st.mean(v), st.pstdev(v), min(v), max(v)


# ─────────────────────────────────── main ───────────────────────────────────

print("=== Adjacent Lane Bias §7.1 프로브 시작 ===", flush=True)
segdirs = sorted(glob.glob(os.path.join(BASE, "*/")))
print(f"세그먼트 dir: {len(segdirs)}", flush=True)

# ---- A) BSM 전수 스캔 ----
print("\n--- A) BSM 신호 실재 확인 (qlog 전수) ---", flush=True)
tot = {k: 0 for k in ("frames", "left", "right", "xor", "both", "xor_fast", "fast", "trans_l", "trans_r")}
vmax = 0.0
bsm_segs = []
for d in segdirs:
    q = os.path.join(d, "qlog.zst")
    if not os.path.exists(q):
        continue
    s = scan_bsm(q)
    for k in tot:
        tot[k] += s[k]
    vmax = max(vmax, s["vmax"])
    if s["left"] or s["right"]:
        r = os.path.join(d, "rlog.zst")
        bsm_segs.append((s["left"] + s["right"], r if os.path.exists(r) else None))

f = tot["frames"]
print(f"carState 프레임: {f}", flush=True)
if f == 0:
    print("!! carState 0 — 로그 경로/형식 확인 필요", flush=True)
else:
    pct = lambda n: 100.0 * n / f
    print(f"  leftBlindspot  ON: {tot['left']:6d} ({pct(tot['left']):5.2f}%)  전환 {tot['trans_l']:4d}회", flush=True)
    print(f"  rightBlindspot ON: {tot['right']:6d} ({pct(tot['right']):5.2f}%)  전환 {tot['trans_r']:4d}회", flush=True)
    print(f"  양쪽 동시(기능 미작동): {tot['both']:6d} ({pct(tot['both']):5.2f}%)", flush=True)
    print(f"  XOR 한쪽만(기능 작동조건): {tot['xor']:6d} ({pct(tot['xor']):5.2f}%)", flush=True)
    print(f"  BSM 감지 중 최고속도: {vmax * 3.6:.1f} km/h", flush=True)
    if tot["fast"]:
        print(f"  60km/h↑ 구간 중 XOR 성립: {tot['xor_fast']} / {tot['fast']} "
              f"({100.0 * tot['xor_fast'] / tot['fast']:.2f}%) ← 기능 실작동 예상 비율", flush=True)
    if tot["trans_l"] + tot["trans_r"] == 0:
        print("  !! 판정: BSM 전환 0회 — 신호 미채워짐. 기능 성립 불가 → 감지 소스 재설계 필요", flush=True)
    else:
        print("  ✅ 판정: BSM 토글 확인 — 트리거 소스 사용 가능", flush=True)

# ---- B) 차선 부호/기준값 ----
print("\n--- B) 차선 y 부호 & 기준값 확인 (rlog 정밀) ---", flush=True)
cands = [r for _, r in sorted(bsm_segs, reverse=True) if r][:MAX_RLOG]
cands += [e for e in EXTRA if os.path.exists(e)]
if not cands:
    cands = [os.path.join(d, "rlog.zst") for d in segdirs
             if os.path.exists(os.path.join(d, "rlog.zst"))][:MAX_RLOG]
print(f"분석 rlog: {len(cands)}개", flush=True)

Y1, Y2, M, W, LC = [], [], [], [], []
for r in cands:
    y1s, y2s, meas, widths, lct = analyze_lanes(r)
    Y1 += y1s; Y2 += y2s; M += meas; W += widths; LC += lct

print(f"유효 프레임(60km/h↑, prob≥{PROB_GATE}, 차선변경 아님): {len(M)}", flush=True)
if len(M) < MIN_FRAMES:
    print(f"!! 프레임 부족(<{MIN_FRAMES}) — 고속 주행 로그 필요", flush=True)
else:
    for name, v, exp in (("laneLines[1].y[0] 좌측", Y1, "≈ +1.8"),
                         ("laneLines[2].y[0] 우측", Y2, "≈ -1.8"),
                         ("measured=-(y1+y2)/2  ", M, "≈  0.0"),
                         ("차선폭 y1-y2        ", W, "≈  3.5")):
        s = stats(v)
        print(f"  {name}: mean {s[0]:+6.2f}  sd {s[1]:4.2f}  "
              f"[{s[2]:+6.2f}, {s[3]:+6.2f}]  기대 {exp}", flush=True)

    ok_sign = st.mean(Y1) > 0 and st.mean(Y2) < 0
    ok_center = abs(st.mean(M)) < 0.25
    ok_width = 2.5 < st.mean(W) < 4.5
    print(f"  부호 규약(y+=좌): {'✅ 확인' if ok_sign else '!! 반대 — 스펙 §4.1 부호 뒤집을 것'}", flush=True)
    print(f"  중앙 기준값 ≈0  : {'✅ 확인' if ok_center else '!! 편중 — 오프셋 기준 재검토'}", flush=True)
    print(f"  차선폭 타당     : {'✅ 확인' if ok_width else '!! 이상 — 차선 인덱스 확인'}", flush=True)
    if ok_sign:
        print("  → measured > 0 = 차가 중앙보다 왼쪽. 스펙 §2.1 target 부호 그대로 사용 가능", flush=True)

if LC:
    print(f"\n  [부호 2차 확인] 차선변경 {len(LC)}건의 measured 변화:", flush=True)
    for d, m0, m1 in LC[:10]:
        print(f"    {d:<5} measured {m0:+.2f} → {m1:+.2f} (Δ{m1 - m0:+.2f})", flush=True)
    print("    (left 변경 시 Δ가 양수여야 measured>0=왼쪽 규약과 일치)", flush=True)

print("\n=== 프로브 완료 ===", flush=True)
