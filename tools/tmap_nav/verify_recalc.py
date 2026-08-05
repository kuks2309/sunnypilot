#!/usr/bin/env python3
"""좌표 기반 거리 재계산을 실측으로 검증한다.

nav_logic 의 재계산 경로는 자차 GPS(Global Positioning System) fix 가 있어야 돌기 때문에,
실내에 둔 기기로는 한 번도 실행되지 않았다. 다행히 브리지가 보내는 패킷 일부에
`vpCurrentPosLat/Lon`(티맵이 아는 자차 위치)이 실려 온다. 그것을 자차 fix 로 넣어주면
재계산 경로가 그대로 돌고, 브리지가 보고한 `nSdiDist` 와 대조할 수 있다.

보는 것은 두 가지다.

1. 재계산이 맞는가 — `nSdiDist` 와 크게 어긋나면 좌표나 계산이 틀린 것이다.
2. 재계산이 필요한가 — `ageMs.sdi` 가 클수록 오차가 커져야 한다.
   콜백이 8~10초에 한 번이므로 그 사이 `nSdiDist` 는 늙고 좌표는 안 늙기 때문이다.

사용: python -m tools.tmap_nav.verify_recalc <파일.jsonl>
"""
import argparse
import json
import statistics
import sys

# Windows 콘솔은 cp949 라 em-dash 같은 문자에서 죽는다. 숫자를 보려고 돌리는 도구이므로
# 인코딩 문제로 중단되지 않게 한다.
if hasattr(sys.stdout, "reconfigure"):
  sys.stdout.reconfigure(errors="replace")

from openpilot.selfdrive.tmap_nav.nav_logic import TmapNavLogic


def load(path):
  """(시각, payload bytes, 자차 lat, 자차 lon) 을 뽑는다."""
  with open(path, encoding="utf-8") as f:
    for line in f:
      line = line.strip()
      if not line:
        continue
      try:
        rec = json.loads(line)
      except ValueError:
        continue
      payload = rec.get("raw") if "raw" in rec else rec.get("data")
      if not isinstance(payload, dict):
        continue
      a = payload.get("apilot") or {}
      lat = a.get("vpCurrentPosLat") or 0.0
      lon = a.get("vpCurrentPosLon") or 0.0
      yield rec.get("t") or 0.0, json.dumps(payload).encode(), lat, lon


def main():
  ap = argparse.ArgumentParser()
  ap.add_argument("file")
  ap.add_argument("--buckets", type=int, default=5)
  args = ap.parse_args()

  lg = TmapNavLogic()
  samples = []          # (ageMs.sdi, 보고거리, 재계산거리, 유형)
  no_fix = with_fix = 0

  for t, raw, lat, lon in load(args.file):
    if not lg.on_packet(raw, t):
      continue
    have_fix = bool(lat) and bool(lon)
    out = lg.update(t, lat, lon, have_fix)
    if out["st"] < 0:
      continue
    if not have_fix:
      no_fix += 1
      continue
    with_fix += 1
    # 같은 패킷을 fix 없이 한 번 더 돌려 브리지 원본 거리를 얻는다
    raw_out = lg.update(t, 0.0, 0.0, False)
    samples.append((out["sa"], raw_out["sd"], out["sd"], out["st"]))

  print(f"=== {args.file}")
  print(f"  SDI 통과 중 자차 fix 있음 {with_fix}건, 없음 {no_fix}건")
  if not samples:
    print("  재계산 표본 없음")
    return 1

  diffs = [abs(rep - rec) for _, rep, rec, _ in samples]
  print(f"  재계산 표본 {len(samples)}건")
  print(f"  |보고거리 - 재계산거리|  중앙값 {statistics.median(diffs):.1f}m  "
        f"평균 {statistics.fmean(diffs):.1f}m  최대 {max(diffs):.1f}m")

  print("\n  ageMs.sdi 구간별 오차 — 나이가 클수록 커져야 정상이다")
  ages = sorted(s[0] for s in samples)
  n = len(ages)
  edges = [ages[int(n * i / args.buckets)] for i in range(1, args.buckets)] + [ages[-1] + 1]
  lo = -1
  print(f"    {'age(ms)':>16}  {'표본':>5}  {'중앙오차':>9}  {'최대오차':>9}")
  for hi in edges:
    grp = [abs(rep - rec) for a, rep, rec, _ in samples if lo < a <= hi]
    if grp:
      print(f"    {lo + 1:>7}~{hi:<8}  {len(grp):>5}  "
            f"{statistics.median(grp):>8.1f}m  {max(grp):>8.1f}m")
    lo = hi

  print("\n  유형별 표본")
  types = {}
  for _, rep, rec, t in samples:
    types.setdefault(t, []).append(abs(rep - rec))
  for t in sorted(types):
    g = types[t]
    print(f"    nSdiType {t:>3}  {len(g):>5}건  중앙오차 {statistics.median(g):>7.1f}m")

  return 0


if __name__ == "__main__":
  sys.exit(main())
