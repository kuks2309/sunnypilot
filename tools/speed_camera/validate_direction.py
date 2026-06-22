#!/usr/bin/env python3
"""5단계 검증: 우리 도로유도 방위(osm_bearing) vs OSM speed_camera 노드의 direction 태그 비교.

OSM 724노드 중 direction(도) 보유분을, speed_cameras_enriched.csv 의 우리 방위와 좌표매칭(≤50m)해
각도 오차 분포를 측정한다. DIRECTED는 full 각도차, AXIS는 mod180 비교.
오차가 작으면 도로유도 방위가 신뢰할 만함을 입증.
"""
from __future__ import annotations

import csv
import json
import math
import os

HERE = os.path.dirname(os.path.abspath(__file__))


def angdiff(a, b):
    return abs((a - b + 180) % 360 - 180)


def main():
    osm = []
    for e in json.load(open(os.path.join(HERE, "osm_speed_cameras.json"), encoding="utf-8"))["elements"]:
        if e["type"] != "node":
            continue
        d = e.get("tags", {}).get("direction")
        try:
            d = float(d)
        except (ValueError, TypeError):
            continue
        osm.append((e["lat"], e["lon"], d))

    cams = []
    for r in csv.DictReader(open(os.path.join(HERE, "speed_cameras_enriched.csv"), encoding="utf-8-sig")):
        try:
            b = float(r["osm_bearing"]) if r["osm_bearing"] else None
        except ValueError:
            b = None
        cams.append((float(r["latitude"]), float(r["longitude"]), b, r.get("osm_dirmode")))

    cell = 50.0 / 111000.0
    g = {}
    for i, c in enumerate(cams):
        g.setdefault((int(c[0] / cell), int(c[1] / cell)), []).append(i)

    def nearest(lat, lon):
        cl, co = int(lat / cell), int(lon / cell)
        coslat = math.cos(math.radians(lat))
        best, bi = (50.0) ** 2, -1
        for dcl in (-1, 0, 1):
            for dco in (-1, 0, 1):
                for i in g.get((cl + dcl, co + dco), ()):
                    c = cams[i]
                    dm = ((lat - c[0]) * 111000) ** 2 + ((lon - c[1]) * 111000 * coslat) ** 2
                    if dm < best:
                        best, bi = dm, i
        return bi

    errs_dir, errs_axis, nomatch, nobrg = [], [], 0, 0
    for lat, lon, d in osm:
        bi = nearest(lat, lon)
        if bi < 0:
            nomatch += 1
            continue
        b = cams[bi][2]
        if b is None:
            nobrg += 1
            continue
        if cams[bi][3] == "AXIS":
            e = min(angdiff(b, d), angdiff(b + 180, d))
            errs_axis.append(e)
        else:
            errs_dir.append(angdiff(b, d))

    def stats(name, e):
        if not e:
            print(f"  {name}: 표본 0"); return
        e = sorted(e)
        med = e[len(e) // 2]
        w20 = 100 * sum(1 for x in e if x <= 20) // len(e)
        w45 = 100 * sum(1 for x in e if x <= 45) // len(e)
        print(f"  {name}: n={len(e)}  중앙오차 {med:.0f}°  ≤20° {w20}%  ≤45° {w45}%")

    print(f"OSM direction 보유 노드 {len(osm)} (매칭실패 {nomatch}, 우리방위없음 {nobrg})")
    stats("DIRECTED", errs_dir)
    stats("AXIS(mod180)", errs_axis)


if __name__ == "__main__":
    main()
