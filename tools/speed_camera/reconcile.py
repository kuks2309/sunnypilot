#!/usr/bin/env python3
"""4단계: 3소스 정합 — 공공데이터 + jjikcam 검증 보강분(고속도로 구간단속) 병합.

공공 speed_cameras.csv(43k) 에, 교차검증으로 확인된 jjik_only_truly_missing.csv(고속도로
구간단속 224, 한국도로공사 공식파일과 노선 31/32 일치) 를 공공 23컬럼 스키마로 변환해 추가한다.
결과 speed_cameras_merged.csv 를 build_db.py 가 그대로 읽어 dedup→bin 생성(도구 재사용=기술부채 0).

jjikcam 보강분 매핑:
  latitude/longitude, lmttVe=speedLimit, regltSctnLcSe="1"(→SECTION_START 분류),
  roadRouteDrc(상행1/하행2/기타3), roadRouteNm/itlpc=노선명, referenceDate=공공기준일, 관리번호=JJIKxxxx
"""
from __future__ import annotations

import csv
import os

HERE = os.path.dirname(os.path.abspath(__file__))
PUB = os.path.join(HERE, "speed_cameras.csv")
SUPP = os.path.join(HERE, "jjik_only_truly_missing.csv")
OUT = os.path.join(HERE, "speed_cameras_merged.csv")


def dir_code(d: str) -> str:
    d = (d or "").strip()
    if d == "상행":
        return "1"
    if d == "하행":
        return "2"
    return "3"  # 양방향/목적지명 등


def main():
    with open(PUB, encoding="utf-8-sig") as f:
        rdr = csv.DictReader(f)
        cols = rdr.fieldnames
        pub_rows = list(rdr)
    refdate = next((r.get("referenceDate") for r in pub_rows if r.get("referenceDate")), "")

    supp = list(csv.DictReader(open(SUPP, encoding="utf-8-sig")))
    add = []
    for i, r in enumerate(supp):
        row = {c: "" for c in cols}
        row["mnlssRegltCameraManageNo"] = f"JJIK{i:04d}"
        row["roadKnd"] = "고속국도"
        row["roadRouteNm"] = r.get("roadName", "")
        row["roadRouteDrc"] = dir_code(r.get("direction"))
        row["latitude"] = r["lat"]
        row["longitude"] = r["lon"]
        row["itlpc"] = (r.get("roadName", "") + " 구간단속").strip()
        row["regltSe"] = "4"            # 표시용(구간), _flags는 regltSctnLcSe로 분류
        row["lmttVe"] = r.get("speedLimit", "") or ""
        row["regltSctnLcSe"] = "1"      # → SECTION_START 로 분류
        row["referenceDate"] = refdate
        row["insttNm"] = "한국도로공사(jjikcam검증)"
        add.append(row)

    with open(OUT, "w", newline="", encoding="utf-8-sig") as f:
        w = csv.DictWriter(f, fieldnames=cols)
        w.writeheader()
        w.writerows(pub_rows)
        w.writerows(add)

    print(f"공공 {len(pub_rows):,} + 보강(고속도로 구간단속) {len(add):,} = {len(pub_rows)+len(add):,}")
    print(f"저장: {OUT}")
    print("다음: python build_db.py -i speed_cameras_merged.csv  (dedup→bin)")


if __name__ == "__main__":
    main()
