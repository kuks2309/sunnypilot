#!/usr/bin/env python3
"""
차선 변경 중 "lead가 원래 차선 차에 묶이는" 거동을 실주행 로그에서 추출/검증하는 스크립트.

분석한 가설:
  - laneChangeStarting 동안 leadOne(=ego 경로 앞차)이 떠나는(원래) 차선 차에 고정되고,
    longitudinalPlan.speeds 가 그 차를 추종하다가, 변경 완료(off) 후에야 풀린다.

이 스크립트는 시간 동기화된 표를 만들어 그 순간을 눈으로 확인할 수 있게 한다.

사용 예:
  # 로컬 세그먼트(rlog) 직접
  python tools/scripts/lane_change_analysis.py /data/media/0/realdata/<route>--0/rlog.zst

  # comma connect 라우트명 (구간 지정 가능)
  python tools/scripts/lane_change_analysis.py "<dongle>|2024-01-01--00-00-00"
  python tools/scripts/lane_change_analysis.py "<dongle>|2024-01-01--00-00-00/2:5"

  # 전체 CSV 로 저장
  python tools/scripts/lane_change_analysis.py <route> --csv out.csv
"""
import argparse
import csv
import sys

from openpilot.common.constants import CV
from openpilot.tools.lib.logreader import LogReader


def kph(v_ms):
  return None if v_ms is None else round(v_ms * CV.MS_TO_KPH, 1)


def build_rows(lr):
  """시간순 메시지를 순회하며 각 타입 최신값을 유지, longitudinalPlan 프레임마다 한 행 스냅샷."""
  # 각 메시지 타입의 최신 상태 보관
  cs = None          # carState
  md = None          # modelV2
  rs = None          # radarState
  t0 = None
  rows = []

  for msg in lr:
    which = msg.which()
    if t0 is None:
      t0 = msg.logMonoTime

    if which == "carState":
      cs = msg.carState
    elif which == "modelV2":
      md = msg.modelV2
    elif which == "radarState":
      rs = msg.radarState
    elif which == "longitudinalPlan":
      # 종방향 계획(20Hz) = 한 제어 사이클. 이 시점의 모든 최신값을 한 행으로.
      lp = msg.longitudinalPlan
      if cs is None or md is None:
        continue

      meta = md.meta
      # model 이 출력한 ego-경로 lead (leadsV3[0])
      mlead_prob = mlead_x = mlead_y = mlead_v = None
      if len(md.leadsV3) > 0:
        ml = md.leadsV3[0]
        mlead_prob = round(float(ml.prob), 3)
        if len(ml.x):
          mlead_x = round(float(ml.x[0]), 1)
        if len(ml.y):
          mlead_y = round(float(ml.y[0]), 1)
        if len(ml.v):
          mlead_v = float(ml.v[0])

      # 레이더/비전 융합 후 MPC 가 실제로 추종하는 lead (radarState.leadOne)
      rlead_status = rlead_drel = rlead_vlead = rlead_prob = None
      if rs is not None:
        lo = rs.leadOne
        rlead_status = bool(lo.status)
        rlead_drel = round(float(lo.dRel), 1)
        rlead_vlead = float(lo.vLead)
        rlead_prob = round(float(lo.modelProb), 3)

      rows.append({
        "t": round((msg.logMonoTime - t0) / 1e9, 2),
        "lcs": str(meta.laneChangeState).split(".")[-1],       # off/preLaneChange/laneChangeStarting/laneChangeFinishing
        "lcd": str(meta.laneChangeDirection).split(".")[-1],   # none/left/right
        "vEgo_kph": kph(float(cs.vEgo)),
        "blinkL": int(cs.leftBlinker),
        "blinkR": int(cs.rightBlinker),
        "steerTrq": round(float(cs.steeringTorque), 1),
        "plan_v0_kph": kph(float(lp.speeds[0]) if len(lp.speeds) else None),
        "plan_a0": round(float(lp.accels[0]), 2) if len(lp.accels) else None,
        # MPC 가 추종한 lead (핵심)
        "rLead_on": rlead_status,
        "rLead_dRel": rlead_drel,
        "rLead_v_kph": kph(rlead_vlead),
        "rLead_prob": rlead_prob,
        # 모델 원시 lead
        "mLead_prob": mlead_prob,
        "mLead_x": mlead_x,
        "mLead_y": mlead_y,        # 횡오프셋: ~0 이면 정면, 부호로 좌/우
        "mLead_v_kph": kph(mlead_v),
      })

  return rows


def find_lane_change_windows(rows, pad=1.5):
  """laneChangeState != off 인 구간을 앞뒤 pad 초 여유와 함께 묶는다."""
  windows = []
  start_idx = None
  for i, r in enumerate(rows):
    active = r["lcs"] != "off"
    if active and start_idx is None:
      start_idx = i
    elif not active and start_idx is not None:
      windows.append((start_idx, i))
      start_idx = None
  if start_idx is not None:
    windows.append((start_idx, len(rows)))

  # pad 적용 (시간 기준)
  padded = []
  for s, e in windows:
    t_start = rows[s]["t"] - pad
    t_end = rows[e - 1]["t"] + pad
    ps = next((i for i, r in enumerate(rows) if r["t"] >= t_start), s)
    pe = next((i for i in range(len(rows) - 1, -1, -1) if rows[i]["t"] <= t_end), e - 1) + 1
    padded.append((ps, pe))
  return padded


COLS = ["t", "lcs", "lcd", "vEgo_kph", "blinkL", "blinkR", "steerTrq",
        "plan_v0_kph", "plan_a0", "rLead_on", "rLead_dRel", "rLead_v_kph",
        "rLead_prob", "mLead_prob", "mLead_x", "mLead_y", "mLead_v_kph"]


def print_window(rows, s, e):
  hdr = f"{'t':>6} {'state':>17} {'dir':>5} {'vEgo':>5} {'bL':>2} {'bR':>2} " \
        f"{'trq':>5} {'planV':>6} {'planA':>6} {'rLd':>4} {'dRel':>6} {'rLdV':>6} " \
        f"{'mProb':>5} {'mX':>6} {'mY':>5} {'mV':>6}"
  print(hdr)
  print("-" * len(hdr))
  prev_state = None
  for r in rows[s:e]:
    mark = "  <== state change" if r["lcs"] != prev_state else ""
    prev_state = r["lcs"]
    print(f"{r['t']:>6} {r['lcs']:>17} {r['lcd']:>5} {str(r['vEgo_kph']):>5} "
          f"{r['blinkL']:>2} {r['blinkR']:>2} {str(r['steerTrq']):>5} "
          f"{str(r['plan_v0_kph']):>6} {str(r['plan_a0']):>6} "
          f"{str(r['rLead_on'])[:1]:>4} {str(r['rLead_dRel']):>6} {str(r['rLead_v_kph']):>6} "
          f"{str(r['mLead_prob']):>5} {str(r['mLead_x']):>6} {str(r['mLead_y']):>5} "
          f"{str(r['mLead_v_kph']):>6}{mark}")
  print()


def main():
  ap = argparse.ArgumentParser(description="차선 변경 중 lead/속도 거동 추출")
  ap.add_argument("identifier", help="라우트명, 세그먼트 범위, 또는 로컬 rlog.zst 경로")
  ap.add_argument("--csv", help="전체 행을 CSV로 저장할 경로")
  ap.add_argument("--all", action="store_true", help="차선 변경 구간만이 아니라 전체 행 출력")
  ap.add_argument("--pad", type=float, default=1.5, help="차선 변경 구간 앞뒤 여유(초)")
  args = ap.parse_args()

  print(f"[*] 로딩: {args.identifier}", file=sys.stderr)
  lr = LogReader(args.identifier, sort_by_time=True)
  rows = build_rows(lr)
  print(f"[*] {len(rows)} 개 제어 사이클(20Hz) 추출", file=sys.stderr)

  if args.csv:
    with open(args.csv, "w", newline="") as f:
      w = csv.DictWriter(f, fieldnames=COLS)
      w.writeheader()
      w.writerows(rows)
    print(f"[*] CSV 저장: {args.csv}", file=sys.stderr)

  if args.all:
    print_window(rows, 0, len(rows))
    return

  windows = find_lane_change_windows(rows, pad=args.pad)
  if not windows:
    print("[!] 차선 변경(laneChangeState != off) 구간을 찾지 못함. "
          "--all 로 전체를 보거나 다른 라우트를 시도하세요.", file=sys.stderr)
    return

  print(f"[*] 차선 변경 구간 {len(windows)} 개 발견\n", file=sys.stderr)
  for n, (s, e) in enumerate(windows, 1):
    print(f"===== 차선 변경 #{n}  (t={rows[s]['t']}s ~ {rows[e-1]['t']}s) =====")
    print_window(rows, s, e)
    print("해석 가이드:")
    print("  - laneChangeStarting 동안 rLead_dRel/rLead_v 가 거의 일정 = 원래 차선 차에 고정")
    print("  - 같은 구간 plan_v0 가 그 rLead_v 를 추종 = 떠나는 차선 lead 가 속도 지배")
    print("  - state 가 off 로 바뀐 뒤 rLead_on=F 또는 dRel 점프 + plan_v0 상승 = lead 해제 시점\n")


if __name__ == "__main__":
  main()
