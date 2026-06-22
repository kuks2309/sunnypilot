#!/usr/bin/env python3
"""2단계: 제한속도 불일치(공공 vs jjikcam) 140건을 OSM maxspeed 로 3자 재검증.

limit_mismatch.csv(공공 vs jjikcam 제한 불일치) 각 지점을 OSM 카메라노드(osm_speed_cameras.json,
maxspeed 보유)와 좌표매칭(≤MATCH m)해 제3의 표를 얻는다.
  - OSM == 공공  → jjikcam 오류 가능
  - OSM == jjikcam → 공공 오류 가능
  - OSM == 둘다아님 / OSM 없음 → 미결
전부 오프라인(추가 네트워크 0).
"""
from __future__ import annotations

import csv
import json
import math
import os

HERE = os.path.dirname(os.path.abspath(__file__))
MATCH = 120.0  # OSM 카메라 매칭 허용(m). OSM 노드 좌표 오차 고려해 다소 넉넉히


def main():
    mism = list(csv.DictReader(open(os.path.join(HERE, "limit_mismatch.csv"), encoding="utf-8-sig")))
    osm = []
    for e in json.load(open(os.path.join(HERE, "osm_speed_cameras.json"), encoding="utf-8"))["elements"]:
        if e["type"] != "node":
            continue
        ms = e.get("tags", {}).get("maxspeed")
        try:
            ms = int(ms)
        except (ValueError, TypeError):
            ms = None
        osm.append((e["lat"], e["lon"], ms))

    cell = MATCH / 111000.0
    g: dict = {}
    for i, p in enumerate(osm):
        g.setdefault((int(p[0] / cell), int(p[1] / cell)), []).append(i)

    def nearest(lat, lon):
        cl, co = int(lat / cell), int(lon / cell)
        coslat = math.cos(math.radians(lat))
        best, bi = MATCH ** 2, -1
        for dcl in (-1, 0, 1):
            for dco in (-1, 0, 1):
                for i in g.get((cl + dcl, co + dco), ()):
                    p = osm[i]
                    dm = ((lat - p[0]) * 111000.0) ** 2 + ((lon - p[1]) * 111000.0 * coslat) ** 2
                    if dm < best:
                        best, bi = dm, i
        return bi

    osm_eq_ours = osm_eq_jjik = osm_other = no_osm = no_osm_ms = 0
    samples = []
    for m in mism:
        lat, lon = float(m["lat"]), float(m["lon"])
        ours, jjik = int(m["ours"]), int(m["jjik"])
        bi = nearest(lat, lon)
        if bi < 0:
            no_osm += 1
            continue
        oms = osm[bi][2]
        if oms is None:
            no_osm_ms += 1
            continue
        if oms == ours:
            osm_eq_ours += 1
            verdict = "jjik오류?"
        elif oms == jjik:
            osm_eq_jjik += 1
            verdict = "공공오류?"
        else:
            osm_other += 1
            verdict = "셋다다름"
        if len(samples) < 12:
            samples.append((round(lat, 5), round(lon, 5), ours, jjik, oms, verdict))

    n = len(mism)
    print(f"제한 불일치 {n}건, OSM 매칭 허용 {MATCH:.0f}m\n")
    print(f"  OSM 카메라 매칭됨: {n - no_osm:,}  (그중 maxspeed 없음 {no_osm_ms})")
    print(f"  OSM 매칭 안됨(OSM에 카메라 없음): {no_osm}")
    print("\n[3자 판정] (OSM maxspeed 기준)")
    print(f"  OSM == 공공  (jjikcam 오류 의심): {osm_eq_ours}")
    print(f"  OSM == jjikcam (공공 오류 의심): {osm_eq_jjik}")
    print(f"  OSM == 둘다아님: {osm_other}")
    print("\n샘플 (lat,lon,공공,jjik,osm,판정):")
    for s in samples:
        print("  ", s)


if __name__ == "__main__":
    main()
