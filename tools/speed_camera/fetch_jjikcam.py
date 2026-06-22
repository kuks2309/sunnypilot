#!/usr/bin/env python3
"""jjikcam(찍캠) 전국 단속카메라 수집 — 교차검증용 (개인 검증 목적).

jjikcam 공개 마커 API(zoom=17이면 bbox 전량 uncapped 반환)를 전국 격자로 호출해
카메라(id,lat,lng,speedLimit,cameraType,roadName,direction)를 모아 JSON으로 저장한다.
공공데이터(우리 DB)와 좌표 매칭해 타입·존재 불일치를 점검(compare_jjikcam.py)하는 데 쓴다.

주의: 개인 검증용. 격자 호출 사이 지연을 둬 서버 부담 최소화.
"""
from __future__ import annotations

import json
import time
import urllib.parse
import urllib.request

API = "https://jjikcam.com/api/cameras"
HEADERS = {"User-Agent": "Mozilla/5.0", "Referer": "https://jjikcam.com/", "Accept": "application/json"}

# 대한민국 대략 경계
LAT0, LAT1 = 33.0, 38.7
LNG0, LNG1 = 124.5, 131.2
STEP = 0.5          # 격자 한 변(도) ~50km. zoom17이면 uncapped라 충분
DELAY = 0.25        # 호출 간 지연(s)
OUT = "jjikcam_cameras.json"


def fetch(sw_lat, sw_lng, ne_lat, ne_lng):
    q = urllib.parse.urlencode({"swLat": sw_lat, "swLng": sw_lng, "neLat": ne_lat, "neLng": ne_lng, "zoom": 17})
    req = urllib.request.Request(f"{API}?{q}", headers=HEADERS)
    with urllib.request.urlopen(req, timeout=30) as r:
        return json.loads(r.read().decode("utf-8"))


def main():
    cams: dict[int, dict] = {}
    capped_cells = 0
    lat = LAT0
    cells = 0
    nlat = int((LAT1 - LAT0) / STEP) + 1
    nlng = int((LNG1 - LNG0) / STEP) + 1
    print(f"격자 {nlat}x{nlng} (~{nlat*nlng}셀) 수집 시작...")
    while lat < LAT1:
        lng = LNG0
        while lng < LNG1:
            try:
                d = fetch(lat, lng, lat + STEP, lng + STEP)
                if d.get("type") == "markers":
                    for c in d.get("cameras", []):
                        cams[c["id"]] = c
                    if d.get("capped"):
                        capped_cells += 1
            except Exception as e:  # noqa: BLE001
                print(f"  [경고] 셀({lat:.1f},{lng:.1f}) 실패: {e}")
            cells += 1
            if cells % 20 == 0:
                print(f"  {cells}셀, 누적 {len(cams):,}건")
            time.sleep(DELAY)
            lng += STEP
        lat += STEP
    print(f"[완료] {len(cams):,}건 (capped 셀 {capped_cells})")
    with open(OUT, "w", encoding="utf-8") as f:
        json.dump(list(cams.values()), f, ensure_ascii=False)
    print(f"저장: {OUT}")
    # 요약
    import collections
    ct = collections.Counter(c.get("cameraType") for c in cams.values())
    dr = collections.Counter(c.get("direction") for c in cams.values())
    print("cameraType:", dict(ct.most_common()))
    print("direction:", dict(dr.most_common()))


if __name__ == "__main__":
    main()
