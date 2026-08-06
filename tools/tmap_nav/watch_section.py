#!/usr/bin/env python3
"""구간단속 표본이 나오는 순간을 잡는다.

수신 jsonl 을 따라가며 `nSdiBlockType` 이 0 이 아니거나 `section` 객체가 채워지는
첫 순간을 보고한다. 구간단속은 고정식 카메라와 제어 성격이 완전히 달라
(한 지점 제한속도 vs 진입~종료 평균속도 관리) 표본 없이 감속을 붙일 수 없다.

사용: python -m tools.tmap_nav.watch_section <파일.jsonl> [--minutes 30]
"""
import argparse
import json
import sys
import time

sys.stdout.reconfigure(errors="replace") if hasattr(sys.stdout, "reconfigure") else None

BLOCK_FIELDS = ("nSdiBlockType", "nSdiBlockSpeed", "nSdiBlockDist",
                "nSdiBlockTime", "nSdiBlockAverageSpeed")


def main():
  ap = argparse.ArgumentParser()
  ap.add_argument("file")
  ap.add_argument("--minutes", type=float, default=30.0)
  ap.add_argument("--poll", type=float, default=8.0)
  args = ap.parse_args()

  # 이미 쌓인 것은 건너뛰고 새로 들어오는 것만 본다
  with open(args.file, encoding="utf-8", errors="replace") as f:
    f.seek(0, 2)
    pos = f.tell()
  print(f"watch: {args.file} 끝({pos} bytes)부터 {args.minutes:.0f}분간 감시", flush=True)

  end = time.time() + args.minutes * 60
  scanned = hits = 0
  types_seen = set()
  roads_seen = set()

  while time.time() < end:
    time.sleep(args.poll)
    with open(args.file, encoding="utf-8", errors="replace") as f:
      f.seek(pos)
      chunk = f.readlines()
      pos = f.tell()

    for line in chunk:
      line = line.strip()
      if not line:
        continue
      try:
        a = json.loads(line)["raw"]["apilot"]
      except Exception:
        continue
      scanned += 1

      t = a.get("nSdiType")
      if t is not None:
        types_seen.add(t)
      rn = a.get("szPosRoadName")
      if rn:
        roads_seen.add(rn)

      bt = a.get("nSdiBlockType") or 0
      sect = a.get("section")
      if bt or sect:
        hits += 1
        if hits <= 5:
          print(f"\n★ 구간단속 표본 발견 [{time.strftime('%H:%M:%S')}]", flush=True)
          for k in BLOCK_FIELDS:
            print(f"    {k} = {a.get(k)}", flush=True)
          print(f"    nSdiType={a.get('nSdiType')} ({a.get('sdiTypeName')}) "
                f"nSdiDist={a.get('nSdiDist')}", flush=True)
          print(f"    도로={a.get('szPosRoadName')} 제한={a.get('nRoadLimitSpeed')}", flush=True)
          if sect:
            print(f"    section = {json.dumps(sect, ensure_ascii=False)[:400]}", flush=True)

    if chunk:
      print(f"  [{time.strftime('%H:%M:%S')}] 누적 {scanned}건 검사, 구간단속 {hits}건 | "
            f"유형 {sorted(types_seen)} | 도로 {len(roads_seen)}종", flush=True)

  print(f"\n감시 종료. 검사 {scanned}건, 구간단속 표본 {hits}건", flush=True)
  print(f"  관측 유형: {sorted(types_seen)}", flush=True)
  return 0 if hits else 1


if __name__ == "__main__":
  sys.exit(main())
