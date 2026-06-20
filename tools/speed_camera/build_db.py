#!/usr/bin/env python3
"""단속카메라 CSV → 기기 런타임용 compact 바이너리 변환 (오프라인 빌드 단계).

download_cameras.py 가 받은 speed_cameras.csv(11MB)를 카메라당 10바이트의
패킹 바이너리로 줄여 selfdrive/speed_camera/speed_cameras.bin 으로 저장한다.
런타임(speed_camera_warnd)은 이 bin만 읽으므로 기기 부하·용량이 최소화된다.

레코드 포맷  '<ffBB' (little-endian, 10 bytes)
    lat   : float32   위도
    lon   : float32   경도
    limit : uint8     제한속도 km/h (0=없음)
    flags : uint8     분류 — 0:NOWARN 1:FIXED 2:SECTION_START 3:SECTION_END
헤더 (16 bytes): magic 'SCAM'(4) + version uint32 + count uint32 + refdate uint32(YYYYMMDD)

분류는 speed_camera_warn.py 와 동일한 동작기반 규칙(regltSe 라벨 의존 X):
    regltSctnLcSe 1/2 → 구간단속 시점/종점, lmttVe>0 → 고정과속, 그 외 → 신호/기타.

사용
----
    python build_db.py                       # 기본 입출력 경로
    python build_db.py -i in.csv -o out.bin
"""
from __future__ import annotations

import argparse
import csv
import os
import struct
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
DEFAULT_IN = os.path.join(HERE, "speed_cameras.csv")
DEFAULT_OUT = os.path.normpath(os.path.join(HERE, "..", "..", "selfdrive", "speed_camera", "speed_cameras.bin"))

MAGIC = b"SCAM"
VERSION = 1
REC = struct.Struct("<ffBB")
HDR = struct.Struct("<4sIII")

NOWARN, FIXED, SECTION_START, SECTION_END = 0, 1, 2, 3


def _norm(x):
    x = (x or "").strip()
    return str(int(x)) if x.isdigit() else x


def _int(x):
    try:
        v = int(float((x or "").strip()))
        return v if v > 0 else 0
    except ValueError:
        return 0


def _flags(row) -> int:
    sect = _norm(row.get("regltSctnLcSe"))
    if sect == "1":
        return SECTION_START
    if sect == "2":
        return SECTION_END
    return FIXED if _int(row.get("lmttVe")) else NOWARN


def convert(in_csv: str, out_bin: str) -> int:
    with open(in_csv, encoding="utf-8-sig") as f:
        rows = list(csv.DictReader(f))

    recs = []
    refdate = 0
    for r in rows:
        try:
            lat = float((r.get("latitude") or "").strip())
            lon = float((r.get("longitude") or "").strip())
        except ValueError:
            continue
        limit = min(_int(r.get("lmttVe")), 255)
        recs.append((lat, lon, limit, _flags(r)))
        if not refdate:
            d = (r.get("referenceDate") or "").replace("-", "").strip()
            if d.isdigit() and len(d) == 8:
                refdate = int(d)

    os.makedirs(os.path.dirname(out_bin), exist_ok=True)
    with open(out_bin, "wb") as f:
        f.write(HDR.pack(MAGIC, VERSION, len(recs), refdate))
        for lat, lon, limit, flags in recs:
            f.write(REC.pack(lat, lon, limit, flags))

    size = os.path.getsize(out_bin)
    cats = {}
    for *_, fl in recs:
        cats[fl] = cats.get(fl, 0) + 1
    names = {NOWARN: "NOWARN", FIXED: "FIXED", SECTION_START: "SECT_START", SECTION_END: "SECT_END"}
    print(f"[완료] {out_bin}")
    print(f"  {len(recs):,}건, {size/1024:.0f}KB, refdate={refdate}")
    print("  분류:", {names[k]: v for k, v in sorted(cats.items())})
    return 0


def main() -> int:
    ap = argparse.ArgumentParser(description="단속카메라 CSV → compact bin")
    ap.add_argument("-i", "--input", default=DEFAULT_IN)
    ap.add_argument("-o", "--output", default=DEFAULT_OUT)
    args = ap.parse_args()
    if not os.path.exists(args.input):
        print(f"[에러] 입력 없음: {args.input}", file=sys.stderr)
        return 2
    return convert(args.input, args.output)


if __name__ == "__main__":
    sys.exit(main())
