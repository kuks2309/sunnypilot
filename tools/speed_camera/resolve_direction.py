#!/usr/bin/env python3
"""5단계-2: jjikcam 방향 + OSM 축으로 카메라 directed 방향 확정 (차로 뒤집힘 해결).

OSM 축(speed_cameras_enriched.csv osm_bearing; 정확도 3°)은 도로의 "선"만 정확하고, oneway
directed 는 21% 반대차로로 뒤집힘. → jjikcam direction(상행/하행/양방향/목적지)으로 두 축방향 중
맞는 쪽을 골라 directed 를 확정한다.
  - 양방향/미상 → AXIS(축, mod180)
  - 상행 → 서울방향 / 하행 → 반대 / 목적지명 → 그 도시방향
  → OSM 축 후보(b, b+180) 중 target 에 가까운 쪽 = travel 방향(차량 heading 비교용)
출력 speed_cameras_resolved.csv (final_bearing, final_dirmode). 카카오 불요, 보유 데이터만.
"""
from __future__ import annotations

import csv
import json
import math
import os

HERE = os.path.dirname(os.path.abspath(__file__))
SEOUL = (37.5665, 126.9780)
# 주요 목적지 도시 좌표(jjikcam direction 목적지명 매핑; 없으면 AXIS 폴백)
CITIES = {
    "서울": (37.5665, 126.978), "부산": (35.1796, 129.0756), "대구": (35.8714, 128.6014),
    "인천": (37.4563, 126.7052), "광주": (35.1595, 126.8526), "대전": (36.3504, 127.3845),
    "울산": (35.5384, 129.3114), "세종": (36.48, 127.289), "수원": (37.2636, 127.0286),
    "춘천": (37.8813, 127.7300), "원주": (37.3422, 127.9202), "강릉": (37.7519, 128.8761),
    "청주": (36.6424, 127.489), "천안": (36.8151, 127.1139), "전주": (35.8242, 127.148),
    "군산": (35.9676, 126.7370), "목포": (34.8118, 126.3922), "순천": (34.9506, 127.4872),
    "여수": (34.7604, 127.6622), "포항": (36.019, 129.3435), "경주": (35.8562, 129.2247),
    "안동": (36.5684, 128.7294), "창원": (35.2280, 128.6811), "진주": (35.1800, 128.1076),
    "김해": (35.2342, 128.8894), "양양": (38.0754, 128.6190), "속초": (38.207, 128.5918),
    "통영": (34.8544, 128.4331), "거제": (34.8806, 128.6211), "제천": (37.1326, 128.191),
    "상주": (36.4109, 128.1592), "김천": (36.1398, 128.1135), "구미": (36.1196, 128.3441),
}


def bearing(la1, lo1, la2, lo2):
    p1, p2 = math.radians(la1), math.radians(la2)
    dl = math.radians(lo2 - lo1)
    y = math.sin(dl) * math.cos(p2)
    x = math.cos(p1) * math.sin(p2) - math.sin(p1) * math.cos(p2) * math.cos(dl)
    return (math.degrees(math.atan2(y, x)) + 360) % 360


def angdiff(a, b):
    return abs((a - b + 180) % 360 - 180)


def main():
    jj = [(c["lat"], c["lng"], (c.get("direction") or "").strip())
          for c in json.load(open(os.path.join(HERE, "jjikcam_cameras.json"), encoding="utf-8"))]
    cell = 50 / 111000.0
    gj = {}
    for i, c in enumerate(jj):
        gj.setdefault((int(c[0] / cell), int(c[1] / cell)), []).append(i)

    def jdir(lat, lon):
        cl, co = int(lat / cell), int(lon / cell)
        coslat = math.cos(math.radians(lat))
        best, bi = 2500, -1
        for a in (-1, 0, 1):
            for o in (-1, 0, 1):
                for i in gj.get((cl + a, co + o), ()):
                    c = jj[i]
                    d = ((lat - c[0]) * 111000) ** 2 + ((lon - c[1]) * 111000 * coslat) ** 2
                    if d < best:
                        best, bi = d, i
        return jj[bi][2] if bi >= 0 else None

    rows = list(csv.DictReader(open(os.path.join(HERE, "speed_cameras_enriched.csv"), encoding="utf-8-sig")))
    cols = list(rows[0].keys()) + ["final_bearing", "final_dirmode"]
    n_dir = n_axis = n_none = 0
    src = {"양방향": 0, "상/하행": 0, "목적지": 0, "미매칭": 0}
    with open(os.path.join(HERE, "speed_cameras_resolved.csv"), "w", newline="", encoding="utf-8-sig") as f:
        w = csv.DictWriter(f, fieldnames=cols)
        w.writeheader()
        for r in rows:
            mode = r.get("osm_dirmode")
            b = r.get("osm_bearing")
            if mode == "NONE" or not b:
                r["final_bearing"], r["final_dirmode"] = "", "NONE"
                n_none += 1
                w.writerow(r); continue
            b = float(b)
            lat, lon = float(r["latitude"]), float(r["longitude"])
            d = jdir(lat, lon)
            target = None
            if d in ("상행",):
                target = bearing(lat, lon, *SEOUL); src["상/하행"] += 1
            elif d in ("하행",):
                target = (bearing(lat, lon, *SEOUL) + 180) % 360; src["상/하행"] += 1
            elif d in CITIES:
                target = bearing(lat, lon, *CITIES[d]); src["목적지"] += 1
            elif d == "양방향":
                src["양방향"] += 1
            else:
                src["미매칭"] += 1
            if target is None:
                r["final_bearing"], r["final_dirmode"] = f"{b:.0f}", "AXIS"
                n_axis += 1
            else:
                chosen = b if angdiff(b, target) <= angdiff(b + 180, target) else (b + 180) % 360
                r["final_bearing"], r["final_dirmode"] = f"{chosen:.0f}", "DIRECTED"
                n_dir += 1
            w.writerow(r)
    print(f"DIRECTED {n_dir:,} / AXIS {n_axis:,} / NONE {n_none:,}")
    print("방향소스:", src)
    print("저장: speed_cameras_resolved.csv")


if __name__ == "__main__":
    main()
