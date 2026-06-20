#!/usr/bin/env python3
"""단속카메라 거리경고 — 오프라인 검증 도구 (2단계-A).

download_cameras.py 가 받은 전국 단속카메라 CSV를 좌표 DB로 로드하고,
"현재 위경도 → 반경 내 전방 카메라 + 제한속도 + 권장 감속속도"를 출력한다.
openpilot 런타임을 건드리지 않고 경고 로직을 검증하기 위한 도구(표준 라이브러리만 사용).

설계 — regltSe 라벨 모호성 우회, 동작 기반 분류
-----------------------------------------------
  · regltSctnLcSe 존재(1=시점/2=종점)  → SECTION  (구간단속)
  · lmttVe>0 (구간 아님)               → FIXED    (고정식 과속)
  · 제한속도 없음                       → NOWARN   (신호/기타: 속도경고 제외)
regltSe(공공데이터)와 carrot의 nSdiType(내비 SDI)은 다른 코드체계이므로 라벨을 직접 매핑하지 않는다.

감속 모델 — carrot 검증식 차용
------------------------------
  허용속도 v_allowed = sqrt(v_f^2 + 2*a*d)   (v in m/s, d=남은거리 m, a=편안한 감속도)
  a 기본 2.0 m/s² = openpilot COMFORT_BRAKE 튜닝값. v_f = 카메라 제한속도.
  현재속도가 v_allowed를 넘으면 "지금부터 감속 필요".

예시
----
    python speed_camera_warn.py 35.8185 128.7382                 # 반경 1km 내 전방 카메라
    python speed_camera_warn.py 35.8185 128.7382 -s 90 -r 2000   # 현재 90km/h, 반경 2km
    python speed_camera_warn.py 35.8185 128.7382 --heading 270   # 진행방향 270°(서) 전방만
"""
from __future__ import annotations

import argparse
import csv
import math
import os
import sys

DEFAULT_CSV = os.path.join(os.path.dirname(os.path.abspath(__file__)), "speed_cameras.csv")
COMFORT_BRAKE = 2.0  # m/s²  (openpilot 종방향 튜닝값과 동일)


def _norm_code(x: str | None) -> str:
    x = (x or "").strip()
    return str(int(x)) if x.isdigit() else x


def _to_float(x: str | None):
    try:
        return float((x or "").strip())
    except ValueError:
        return None


def _to_int(x: str | None):
    s = (x or "").strip()
    try:
        v = int(float(s))
        return v if v > 0 else None
    except ValueError:
        return None


def classify(row: dict) -> str:
    """동작 기반 카메라 분류 (regltSe 라벨에 의존하지 않음)."""
    if _norm_code(row.get("regltSctnLcSe")) in ("1", "2"):
        return "SECTION"
    if _to_int(row.get("lmttVe")):
        return "FIXED"
    return "NOWARN"


def load_cameras(csv_path: str) -> list[dict]:
    with open(csv_path, encoding="utf-8-sig") as f:
        rows = list(csv.DictReader(f))
    cams = []
    for r in rows:
        lat, lon = _to_float(r.get("latitude")), _to_float(r.get("longitude"))
        if lat is None or lon is None:
            continue
        cams.append({
            "lat": lat, "lon": lon,
            "limit": _to_int(r.get("lmttVe")),
            "category": classify(r),
            "sect": _norm_code(r.get("regltSctnLcSe")),  # 1=시점 2=종점
            "regltSe": _norm_code(r.get("regltSe")),
            "loc": (r.get("itlpc") or r.get("rdnmadr") or "").strip(),
            "ctprvn": (r.get("ctprvnNm") or "").strip(),
            "signgu": (r.get("signguNm") or "").strip(),
        })
    return cams


def haversine_m(lat1, lon1, lat2, lon2) -> float:
    R = 6371000.0
    p1, p2 = math.radians(lat1), math.radians(lat2)
    dp = math.radians(lat2 - lat1)
    dl = math.radians(lon2 - lon1)
    a = math.sin(dp / 2) ** 2 + math.cos(p1) * math.cos(p2) * math.sin(dl / 2) ** 2
    return 2 * R * math.asin(math.sqrt(a))


def bearing_deg(lat1, lon1, lat2, lon2) -> float:
    p1, p2 = math.radians(lat1), math.radians(lat2)
    dl = math.radians(lon2 - lon1)
    y = math.sin(dl) * math.cos(p2)
    x = math.cos(p1) * math.sin(p2) - math.sin(p1) * math.cos(p2) * math.cos(dl)
    return (math.degrees(math.atan2(y, x)) + 360) % 360


def recommended_speed_kph(limit_kph: int, dist_m: float, a: float = COMFORT_BRAKE) -> float:
    """남은거리 dist_m 에서 허용되는 현재속도(km/h). carrot 등감속식."""
    vf = limit_kph / 3.6
    v = math.sqrt(vf * vf + 2 * a * max(dist_m, 0))
    return v * 3.6


def query(cams, lat, lon, radius_m, heading=None, fov=120.0):
    # 위경도 바운딩박스로 1차 필터(43k 전수 haversine 회피)
    dlat = radius_m / 111_000.0
    dlon = radius_m / (111_000.0 * max(math.cos(math.radians(lat)), 1e-6))
    out = []
    for c in cams:
        if abs(c["lat"] - lat) > dlat or abs(c["lon"] - lon) > dlon:
            continue
        d = haversine_m(lat, lon, c["lat"], c["lon"])
        if d > radius_m:
            continue
        brg = bearing_deg(lat, lon, c["lat"], c["lon"])
        if heading is not None:
            diff = abs((brg - heading + 180) % 360 - 180)
            if diff > fov / 2:  # 전방 시야각 밖 = 후방 카메라, 제외
                continue
        out.append({**c, "dist": d, "bearing": brg})
    out.sort(key=lambda x: x["dist"])
    return out


def main() -> int:
    ap = argparse.ArgumentParser(description="단속카메라 거리경고 오프라인 검증")
    ap.add_argument("lat", type=float, help="현재 위도")
    ap.add_argument("lon", type=float, help="현재 경도")
    ap.add_argument("speed", nargs="?", type=float, default=None, help="현재 속도 km/h (선택)")
    ap.add_argument("-r", "--radius", type=float, default=1000, help="검색 반경 m (기본 1000)")
    ap.add_argument("-s", "--speed-opt", type=float, dest="speed_opt", help="현재 속도 km/h(대체 플래그)")
    ap.add_argument("--heading", type=float, default=None, help="진행방향 deg(0=북). 주면 전방만")
    ap.add_argument("--fov", type=float, default=120.0, help="전방 시야각 deg (기본 120)")
    ap.add_argument("--csv", default=DEFAULT_CSV, help="카메라 CSV 경로")
    ap.add_argument("--show-nowarn", action="store_true", help="신호/기타(NOWARN)도 표시")
    args = ap.parse_args()

    if not os.path.exists(args.csv):
        print(f"[에러] CSV 없음: {args.csv} (먼저 download_cameras.py 실행)", file=sys.stderr)
        return 2
    speed = args.speed if args.speed is not None else args.speed_opt

    cams = load_cameras(args.csv)
    hits = query(cams, args.lat, args.lon, args.radius, args.heading, args.fov)
    if not args.show_nowarn:
        hits = [h for h in hits if h["category"] != "NOWARN"]

    print(f"위치 ({args.lat}, {args.lon}) 반경 {args.radius:.0f}m"
          + (f", 진행 {args.heading:.0f}°, 시야 {args.fov:.0f}°" if args.heading is not None else "")
          + (f", 현재 {speed:.0f}km/h" if speed else ""))
    print(f"카메라 DB {len(cams):,}건 · 검색결과 {len(hits)}건\n")

    SECT = {"1": "시점", "2": "종점"}
    for h in hits:
        tag = h["category"]
        extra = f"({SECT.get(h['sect'],'')})" if tag == "SECTION" else ""
        lim = f"{h['limit']}km/h" if h["limit"] else "제한없음"
        line = (f"  {h['dist']:6.0f}m  {tag:7s}{extra:4s} {lim:>9s}  "
                f"방위{h['bearing']:3.0f}°  {h['ctprvn']} {h['signgu']} {h['loc']}")
        print(line)
        if speed and h["limit"]:
            rec = recommended_speed_kph(h["limit"], h["dist"])
            if speed > rec:
                print(f"          [!] 감속필요: 현재 {speed:.0f} > 허용 {rec:.0f}km/h "
                      f"(이 거리에서 {h['limit']}km/h까지 a={COMFORT_BRAKE}m/s²로 감속 가정)")
    if not hits:
        print("  (반경 내 경고 대상 카메라 없음)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
