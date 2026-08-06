#!/usr/bin/env python3
"""tmap_nav 리플레이 하네스 — 실주행 수신 로그를 nav_logic 에 흘려 게이팅을 검사한다.

기기 없이 PC 에서 도는 것이 요점이다. 판단 로직은 cereal 비의존이므로
Windows 에서도 그대로 검증된다.

두 가지 jsonl 형식을 모두 읽는다.
  원격 수신기:  {"ts": ..., "ch": "udp7707", "peer": ..., "data": {"active":1, "apilot": {...}}}
  본체 캡처:    {"t": ..., "peer": ..., "raw": {"active":1, "apilot": {...}}}

사용:
  python -m tools.tmap_nav.replay <파일.jsonl> [...]
  python -m tools.tmap_nav.replay --expect-no-sdi <구버전.jsonl>

`--expect-no-sdi` 는 회귀 게이트다. `sdiFrom`/`ageMs` 가 없던 시절(2026-08-05 이전)
로그에서는 SDI 가 단 한 번도 통과하면 안 된다. 그 시절 값은 힙에서 주워온 낡은 값일 수
있고, 신선도를 판단할 근거가 패킷 안에 없기 때문이다.
"""
import argparse
import json
import sys
from collections import Counter

from openpilot.selfdrive.tmap_nav.nav_logic import TmapNavLogic

PKT_DT = 0.5   # 브리지 송신 2Hz — 로그에 타임스탬프가 없을 때 쓰는 가정 간격


def iter_packets(path: str):
  """jsonl 한 줄에서 (시각, payload bytes) 를 뽑는다. 형식 차이를 여기서 흡수한다."""
  with open(path, encoding="utf-8") as f:
    for lineno, line in enumerate(f, 1):
      line = line.strip()
      if not line:
        continue
      try:
        rec = json.loads(line)
      except ValueError:
        print(f"  [skip] {path}:{lineno} JSON 파싱 실패")
        continue

      payload = rec.get("data") if "data" in rec else rec.get("raw")
      if payload is None:
        continue
      # 원격 수신기가 파싱 못 한 원본은 _raw_hex 로 남는다 — 재생 대상이 아니다
      if isinstance(payload, dict) and "_raw_hex" in payload:
        continue

      ts = rec.get("t")
      if ts is None:
        ts = rec.get("ts")   # ISO 문자열이면 아래에서 None 처리
        if isinstance(ts, str):
          ts = None
      yield ts, json.dumps(payload).encode()


def replay(path: str) -> dict:
  lg = TmapNavLogic()
  stats = Counter()
  reasons = Counter()
  sdi_seen = []          # (type, dist, lat, lon)
  limits_seen = Counter()
  t = 0.0
  first_ts = None

  for ts, raw in iter_packets(path):
    # 실제 타임스탬프가 있으면 쓰고, 없으면 2Hz 가정으로 진행시킨다
    if ts is not None:
      if first_ts is None:
        first_ts = ts
      t = ts - first_ts
    else:
      t += PKT_DT

    stats["packets"] += 1
    if not lg.on_packet(raw, t):
      stats["parse_error"] += 1
      continue

    out = lg.update(t, 0.0, 0.0, False)   # fix 없음 → 좌표 재계산은 하지 않는다

    if not out["gd"]:
      reasons["guiding=false"] += 1
    if out["sl"] > 0:
      stats["road_limit_ok"] += 1
      limits_seen[out["sl"]] += 1
    if out["st"] >= 0:
      stats["sdi_ok"] += 1
      sdi_seen.append((out["st"], out["sd"], out["sy"], out["sz"]))
    else:
      # 왜 걸렀는지 분류 — 진단에 이게 제일 중요하다
      if not out["gd"]:
        pass                                   # 위에서 이미 셈
      elif out["src"] == "":
        reasons["sdiFrom 없음(구버전)"] += 1
      elif out["src"] != "RGData.sdiInfo":
        reasons[f"sdiFrom={out['src']}"] += 1
      elif out["sa"] < 0:
        reasons["ageMs.sdi 없음"] += 1
      else:
        reasons[f"ageMs.sdi 초과({out['sa']}ms)"] += 1

  return {
    "path": path, "stats": stats, "reasons": reasons,
    "sdi_seen": sdi_seen, "limits": limits_seen,
    "parse_errors": lg.parse_errors,
  }


def report(r: dict, expect_no_sdi: bool) -> bool:
  s = r["stats"]
  print(f"\n=== {r['path']}")
  print(f"  패킷 {s['packets']}건, 파싱실패 {s['parse_error']}건")
  print(f"  도로 제한속도 통과: {s['road_limit_ok']}건", end="")
  print(f"  {dict(r['limits'].most_common(6))}" if r["limits"] else "")
  print(f"  SDI 통과: {s['sdi_ok']}건")

  if r["reasons"]:
    print("  거른 이유:")
    for k, v in r["reasons"].most_common():
      print(f"    {v:>6}  {k}")

  if r["sdi_seen"]:
    uniq_pts = {(lat, lon) for _, _, lat, lon in r["sdi_seen"]}
    uniq_dist = {d for _, d, _, _ in r["sdi_seen"]}
    print(f"  통과한 SDI: 좌표 {len(uniq_pts)}종, 거리 {len(uniq_dist)}종")
    if len(r["sdi_seen"]) > 20 and len(uniq_dist) == 1:
      print("    [!] 거리가 한 값에 고정 — 동결 의심. 게이팅이 놓쳤을 수 있다.")

  ok = True
  if expect_no_sdi and s["sdi_ok"] > 0:
    print(f"  [FAIL] 구버전 로그인데 SDI 가 {s['sdi_ok']}건 통과했다. 게이팅이 샜다.")
    ok = False
  elif expect_no_sdi:
    print("  [OK] 구버전 로그의 SDI 를 전부 걸렀다.")
  return ok


def main():
  ap = argparse.ArgumentParser()
  ap.add_argument("files", nargs="+")
  ap.add_argument("--expect-no-sdi", action="store_true",
                  help="SDI 가 한 건도 통과하면 실패로 본다(구버전 로그 회귀 게이트)")
  args = ap.parse_args()

  all_ok = True
  for path in args.files:
    all_ok &= report(replay(path), args.expect_no_sdi)

  print()
  sys.exit(0 if all_ok else 1)


if __name__ == "__main__":
  main()
