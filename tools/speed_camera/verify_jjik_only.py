#!/usr/bin/env python3
"""3단계: jjikcam-only(우리 누락) 230건 검증 — 전부 오프라인.

jjikcam_only.csv 각 카메라를:
  - 우리 공공데이터에 100m 안에 있는지(경계효과 → 진짜누락 분리)
  - OSM 카메라노드(독립 소스)가 120m 안에 있는지(실재 교차확인)
  - 타입/제한 분포
로 검증해, "진짜 보강 후보"를 확정한다.
"""
from __future__ import annotations

import csv
import json
import math
import os
import collections

HERE = os.path.dirname(os.path.abspath(__file__))


def _int(x):
    try:
        v = int(float((x or "").strip())); return v if v > 0 else 0
    except (ValueError, TypeError):
        return 0


def load_ours():
    out = []
    with open(os.path.join(HERE, "speed_cameras.csv"), encoding="utf-8-sig") as f:
        for r in csv.DictReader(f):
            try:
                out.append((float(r["latitude"]), float(r["longitude"])))
            except (ValueError, KeyError):
                pass
    return out


def grid(points, cell):
    g = {}
    for i, p in enumerate(points):
        g.setdefault((int(p[0] / cell), int(p[1] / cell)), []).append(i)
    return g


def has_within(lat, lon, pts, g, cell, m):
    cl, co = int(lat / cell), int(lon / cell)
    coslat = math.cos(math.radians(lat)); d2 = m * m
    for dcl in (-1, 0, 1):
        for dco in (-1, 0, 1):
            for i in g.get((cl + dcl, co + dco), ()):
                p = pts[i]
                if ((lat - p[0]) * 111000.0) ** 2 + ((lon - p[1]) * 111000.0 * coslat) ** 2 <= d2:
                    return True
    return False


def main():
    jo = list(csv.DictReader(open(os.path.join(HERE, "jjikcam_only.csv"), encoding="utf-8-sig")))
    ours = load_ours()
    osm = [(e["lat"], e["lon"]) for e in json.load(open(os.path.join(HERE, "osm_speed_cameras.json"), encoding="utf-8"))["elements"] if e["type"] == "node"]

    CELL = 120.0 / 111000.0
    go, go_osm = grid(ours, CELL), grid(osm, CELL)

    boundary = truly = osm_confirmed = 0
    miss_rows = []
    for r in jo:
        lat, lon = float(r["lat"]), float(r["lng"]) if "lng" in r else float(r["lon"])
        if has_within(lat, lon, ours, go, CELL, 100):
            boundary += 1
            continue
        truly += 1
        conf = has_within(lat, lon, osm, go_osm, CELL, 120)
        if conf:
            osm_confirmed += 1
        miss_rows.append((lat, lon, r.get("speedLimit"), r.get("type"), r.get("direction"), r.get("roadName"), conf))

    print(f"jjikcam-only {len(jo)}건 검증 (오프라인)\n")
    print(f"  경계(우리 100m 안에 존재 → 실은 누락 아님): {boundary}")
    print(f"  진짜 누락(우리 100m 없음): {truly}")
    print(f"    └ OSM 카메라로 실재 확인됨: {osm_confirmed}  (나머지 {truly-osm_confirmed}는 미확인)")
    tc = collections.Counter(r[3] for r in miss_rows)
    lc = collections.Counter(r[2] for r in miss_rows)
    print(f"\n  진짜누락 타입분포: {dict(tc)}")
    print(f"  진짜누락 제한분포: {dict(lc.most_common(8))}")
    with open(os.path.join(HERE, "jjik_only_truly_missing.csv"), "w", newline="", encoding="utf-8-sig") as f:
        w = csv.writer(f); w.writerow(["lat", "lon", "speedLimit", "type", "direction", "roadName", "osm_confirmed"])
        w.writerows(miss_rows)
    print(f"\n  저장: jjik_only_truly_missing.csv ({len(miss_rows)})")


if __name__ == "__main__":
    main()
