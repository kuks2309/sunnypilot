#!/usr/bin/env python3
"""5단계: OSM 도로벡터로 카메라 방향(bearing) + 도로 maxspeed enrich (오프라인, PC 1회).

south-korea-latest.osm.pbf 에서 주행도로(highway) 세그먼트를 추출해 방위/oneway/maxspeed 를 만들고,
KDTree로 각 카메라의 최근접 도로 세그먼트를 찾아:
  - bearing(0~360, oneway면 directed / 아니면 axis)
  - dir_mode: DIRECTED / AXIS / NONE(매칭 실패)
  - road_maxspeed (제한 불일치 140 중재 + 검증용)
를 speed_cameras_enriched.csv 로 출력. build_db(v2)가 bearing 컬럼을 읽어 bin v2 생성.

의존: osmium, scipy, numpy (geopandas/GDAL 불필요).
"""
from __future__ import annotations

import argparse
import csv
import math
import os
import sys

import numpy as np
import osmium
from scipy.spatial import cKDTree

HERE = os.path.dirname(os.path.abspath(__file__))
DEFAULT_PBF = os.path.normpath(os.path.join(os.environ.get("TEMP", "/tmp"), "south-korea-latest.osm.pbf"))
DEFAULT_IN = os.path.join(HERE, "speed_cameras_merged.csv")
DEFAULT_OUT = os.path.join(HERE, "speed_cameras_enriched.csv")

DRIVABLE = {
    "motorway", "trunk", "primary", "secondary", "tertiary", "unclassified", "residential",
    "motorway_link", "trunk_link", "primary_link", "secondary_link", "tertiary_link", "living_street",
}
MATCH_MAX = 60.0   # 도로 매칭 허용(m). 초과 시 NONE
LAT0 = 36.5        # 한국 중위도(등거리 투영 기준)
KX = 111000.0 * math.cos(math.radians(LAT0))
KY = 111000.0
DIR_AXIS, DIR_NONE = 4, 8  # build_db flags 비트와 일치 예정


def bearing(la1, lo1, la2, lo2):
    p1, p2 = math.radians(la1), math.radians(la2)
    dl = math.radians(lo2 - lo1)
    y = math.sin(dl) * math.cos(p2)
    x = math.cos(p1) * math.sin(p2) - math.sin(p1) * math.cos(p2) * math.cos(dl)
    return (math.degrees(math.atan2(y, x)) + 360) % 360


def parse_ms(s):
    if not s:
        return 0
    s = s.strip()
    if s.isdigit():
        return int(s)
    for tok in s.replace(";", " ").split():
        if tok.isdigit():
            return int(tok)
    return 0


class RoadHandler(osmium.SimpleHandler):
    def __init__(self):
        super().__init__()
        self.lat, self.lon, self.brg, self.oneway, self.ms = [], [], [], [], []

    def way(self, w):
        hw = w.tags.get("highway")
        if hw not in DRIVABLE:
            return
        ow = w.tags.get("oneway")
        oneway = 1 if ow in ("yes", "true", "1") else (-1 if ow == "-1" else 0)
        ms = parse_ms(w.tags.get("maxspeed"))
        pts = []
        try:
            for n in w.nodes:
                if n.location.valid():
                    pts.append((n.location.lat, n.location.lon))
        except Exception:  # noqa: BLE001
            return
        for i in range(len(pts) - 1):
            la1, lo1 = pts[i]
            la2, lo2 = pts[i + 1]
            b = bearing(la1, lo1, la2, lo2)
            if oneway == -1:
                b = (b + 180) % 360
            self.lat.append((la1 + la2) / 2)
            self.lon.append((lo1 + lo2) / 2)
            self.brg.append(b)
            self.oneway.append(1 if oneway != 0 else 0)
            self.ms.append(ms)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--pbf", default=DEFAULT_PBF)
    ap.add_argument("-i", "--input", default=DEFAULT_IN)
    ap.add_argument("-o", "--output", default=DEFAULT_OUT)
    args = ap.parse_args()
    if not os.path.exists(args.pbf):
        print(f"[에러] pbf 없음: {args.pbf}", file=sys.stderr)
        return 2

    print("OSM 도로 파싱 중...")
    h = RoadHandler()
    h.apply_file(args.pbf, locations=True)
    n = len(h.brg)
    print(f"  주행도로 세그먼트 {n:,}")

    seg_xy = np.column_stack([np.array(h.lon) * KX, np.array(h.lat) * KY])
    brg = np.array(h.brg)
    oneway = np.array(h.oneway, dtype=np.uint8)
    ms = np.array(h.ms, dtype=np.int16)
    tree = cKDTree(seg_xy)

    with open(args.input, encoding="utf-8-sig") as f:
        rows = list(csv.DictReader(f))
        cols = list(rows[0].keys())
    cam_xy = np.array([[float(r["longitude"]) * KX, float(r["latitude"]) * KY] for r in rows])
    dist, idx = tree.query(cam_xy, k=1)

    out_cols = cols + ["osm_bearing", "osm_dirmode", "osm_maxspeed"]
    directed = axis = none = flipped = 0
    with open(args.output, "w", newline="", encoding="utf-8-sig") as f:
        w = csv.DictWriter(f, fieldnames=out_cols)
        w.writeheader()
        for i, (r, d, si) in enumerate(zip(rows, dist, idx)):
            if d > MATCH_MAX:
                r["osm_bearing"], r["osm_dirmode"], r["osm_maxspeed"] = "", "NONE", ""
                none += 1
            else:
                b = float(brg[si])
                r["osm_maxspeed"] = str(int(ms[si])) if ms[si] > 0 else ""
                if oneway[si]:
                    # RHT 우측배치: 카메라가 스냅 세그먼트 진행방향 '왼쪽'이면 반대차로 소속 → 180° 보정
                    mid_lon, mid_lat = seg_xy[si][0] / KX, seg_xy[si][1] / KY
                    clat = float(r["latitude"]); clon = float(r["longitude"])
                    ve = (clon - mid_lon) * 111000.0 * math.cos(math.radians(clat))
                    vn = (clat - mid_lat) * 111000.0
                    rad = math.radians(b)
                    cross = math.sin(rad) * vn - math.cos(rad) * ve  # >0: 카메라가 왼쪽
                    if cross > 0:
                        b = (b + 180) % 360
                        flipped += 1
                    r["osm_bearing"] = f"{b:.0f}"; r["osm_dirmode"] = "DIRECTED"; directed += 1
                else:
                    r["osm_bearing"] = f"{b:.0f}"; r["osm_dirmode"] = "AXIS"; axis += 1
            w.writerow(r)
    print(f"  RHT 보정(왼쪽→180°): {flipped:,}건")
    print(f"  카메라 {len(rows):,}: DIRECTED {directed:,} / AXIS {axis:,} / NONE {none:,}")
    print(f"  저장: {args.output}")


if __name__ == "__main__":
    main()
