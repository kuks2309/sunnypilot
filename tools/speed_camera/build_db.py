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
import math
import os
import struct
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
DEFAULT_IN = os.path.join(HERE, "speed_cameras.csv")
DEFAULT_OUT = os.path.normpath(os.path.join(HERE, "..", "..", "selfdrive", "speed_camera", "speed_cameras.bin"))
CONFIRMED = os.path.join(HERE, "confirmed_directions.csv")   # itlpc=정확 방위(DIRECTED)
ENRICHED = os.path.join(HERE, "speed_cameras_enriched.csv")  # OSM 축(AXIS)

MAGIC = b"SCAM"
VERSION = 2                          # v2: 방위 1바이트 + flags에 방향모드 비트
REC = struct.Struct("<ffBBB")        # lat, lon, limit, flags, bearing(deg/2)
HDR = struct.Struct("<4sIII")

NOWARN, FIXED, SECTION_START, SECTION_END = 0, 1, 2, 3
DIR_AXIS, DIR_DIRECTED = 4, 8        # flags 비트(하위2비트=category). 둘다0=방향없음(NONE)


def _round_key(lat, lon):
    return (round(lat, 5), round(lon, 5))


def load_directions():
    """좌표→(bearing_deg, dirmode). itlpc(정확)=DIRECTED, OSM축=AXIS. 파일 없으면 빈 dict."""
    directed, axis = {}, {}
    if os.path.exists(CONFIRMED):
        for r in csv.DictReader(open(CONFIRMED, encoding="utf-8-sig")):
            if (r.get("source") == "itlpc") and r.get("facing"):
                try:
                    directed[_round_key(float(r["latitude"]), float(r["longitude"]))] = float(r["facing"])
                except ValueError:
                    pass
    if os.path.exists(ENRICHED):
        for r in csv.DictReader(open(ENRICHED, encoding="utf-8-sig")):
            if r.get("osm_bearing"):
                try:
                    axis[_round_key(float(r["latitude"]), float(r["longitude"]))] = float(r["osm_bearing"])
                except ValueError:
                    pass
    return directed, axis


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


def dedup(items, merge_dist: float):
    """근접 중복 제거 — 같은 (제한·분류·방향) 이고 merge_dist(m) 안이면 동일 카메라로 보고 1개만 유지.
    방향(drc)·제한·분류가 다르면 병합 안 함 → 방향별/유형별 카메라는 보존.
    items: (lat, lon, limit, flags, drc). 격자 인덱스로 O(n)."""
    cell = max(merge_dist / 111000.0, 1e-9)
    grid: dict = {}
    kept = []
    d2 = merge_dist * merge_dist
    for it in items:
        lat, lon, limit, flags, drc = it
        coslat = math.cos(math.radians(lat))
        cl, co = int(lat / cell), int(lon / cell)
        is_dup = False
        for dcl in (-1, 0, 1):
            for dco in (-1, 0, 1):
                for ki in grid.get((cl + dcl, co + dco), ()):
                    k = kept[ki]
                    if (k[2], k[3], k[4]) != (limit, flags, drc):
                        continue
                    dm = ((lat - k[0]) * 111000.0) ** 2 + ((lon - k[1]) * 111000.0 * coslat) ** 2
                    if dm <= d2:
                        is_dup = True
                        break
                if is_dup:
                    break
            if is_dup:
                break
        if not is_dup:
            grid.setdefault((cl, co), []).append(len(kept))
            kept.append(it)
    return kept


def convert(in_csv: str, out_bin: str, merge_dist: float = 30.0) -> int:
    with open(in_csv, encoding="utf-8-sig") as f:
        rows = list(csv.DictReader(f))

    items = []
    refdate = 0
    for r in rows:
        try:
            lat = float((r.get("latitude") or "").strip())
            lon = float((r.get("longitude") or "").strip())
        except ValueError:
            continue
        limit = min(_int(r.get("lmttVe")), 255)
        items.append((lat, lon, limit, _flags(r), _norm(r.get("roadRouteDrc"))))
        if not refdate:
            d = (r.get("referenceDate") or "").replace("-", "").strip()
            if d.isdigit() and len(d) == 8:
                refdate = int(d)

    raw_n = len(items)
    if merge_dist > 0:
        items = dedup(items, merge_dist)
    print(f"  dedup: {raw_n:,} → {len(items):,} ({raw_n-len(items):,} 제거, merge_dist={merge_dist:.0f}m)")

    directed, axis = load_directions()
    print(f"  방향소스: itlpc(DIRECTED) {len(directed):,} / OSM축(AXIS) {len(axis):,}")

    recs = []
    n_dir = n_axis = n_none = 0
    for lat, lon, limit, flags, _drc in items:
        key = _round_key(lat, lon)
        if key in directed:
            deg, dmode = directed[key], DIR_DIRECTED; n_dir += 1
        elif key in axis:
            deg, dmode = axis[key], DIR_AXIS; n_axis += 1
        else:
            deg, dmode = 0.0, 0; n_none += 1
        bearing = max(0, min(180, int(round((deg % 360) / 2))))  # deg/2 (0~180), 2° 해상도
        recs.append((lat, lon, limit, flags | dmode, bearing))

    os.makedirs(os.path.dirname(out_bin), exist_ok=True)
    with open(out_bin, "wb") as f:
        f.write(HDR.pack(MAGIC, VERSION, len(recs), refdate))
        for lat, lon, limit, flags, bearing in recs:
            f.write(REC.pack(lat, lon, limit, flags, bearing))

    size = os.path.getsize(out_bin)
    cats = {}
    for _la, _lo, _li, fl, _b in recs:
        cat = fl & 3
        cats[cat] = cats.get(cat, 0) + 1
    names = {NOWARN: "NOWARN", FIXED: "FIXED", SECTION_START: "SECT_START", SECTION_END: "SECT_END"}
    print(f"[완료] {out_bin}  (v{VERSION})")
    print(f"  {len(recs):,}건, {size/1024:.0f}KB, refdate={refdate}")
    print("  분류:", {names[k]: v for k, v in sorted(cats.items())})
    print(f"  방향: DIRECTED {n_dir:,} / AXIS {n_axis:,} / NONE {n_none:,}")
    return 0


def main() -> int:
    ap = argparse.ArgumentParser(description="단속카메라 CSV → compact bin")
    ap.add_argument("-i", "--input", default=DEFAULT_IN)
    ap.add_argument("-o", "--output", default=DEFAULT_OUT)
    ap.add_argument("--merge-dist", type=float, default=30.0,
                    help="중복 병합 거리(m). 같은 제한·분류·방향만 병합. 0=dedup끔 (기본 30)")
    args = ap.parse_args()
    if not os.path.exists(args.input):
        print(f"[에러] 입력 없음: {args.input}", file=sys.stderr)
        return 2
    return convert(args.input, args.output, args.merge_dist)


if __name__ == "__main__":
    sys.exit(main())
