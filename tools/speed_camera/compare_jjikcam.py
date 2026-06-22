#!/usr/bin/env python3
"""공공데이터(우리) vs jjikcam 교차검증 — 정밀·다중 재검증판.

fetch_jjikcam.py 의 jjikcam_cameras.json 과 speed_cameras.csv 를 좌표 매칭해:
  - 여러 임계거리(20/30/50/100m)로 매칭률 민감도 재검증
  - ours-only(매칭 안 됨)를 "내부중복(다른 우리카메라 근접) vs 진짜 단독"으로 분류
  - ours-only 가 100m 안에선 매칭되는지(경계효과) 재확인
  - 매칭된 것 중 제한속도 불일치 목록
  - jjikcam-only(우리 누락) 목록
검토용 CSV 출력: ours_only_isolated.csv, jjikcam_only.csv, limit_mismatch.csv
"""
from __future__ import annotations

import csv
import json
import math
import os

HERE = os.path.dirname(os.path.abspath(__file__))
OUR_CSV = os.path.join(HERE, "speed_cameras.csv")
JJIK = os.path.join(HERE, "jjikcam_cameras.json")


def _int(x):
    try:
        v = int(float((x or "").strip()))
        return v if v > 0 else 0
    except (ValueError, TypeError):
        return 0


def load_ours():
    out = []
    with open(OUR_CSV, encoding="utf-8-sig") as f:
        for r in csv.DictReader(f):
            try:
                out.append((float(r["latitude"]), float(r["longitude"]), _int(r.get("lmttVe")),
                            (r.get("itlpc") or r.get("rdnmadr") or "").strip()))
            except (ValueError, KeyError):
                pass
    return out


def grid_index(points, cell):
    g: dict = {}
    for i, p in enumerate(points):
        g.setdefault((int(p[0] / cell), int(p[1] / cell)), []).append(i)
    return g


def nearest(lat, lon, pts, g, cell, max_m, skip=-1):
    """max_m 이내 최근접 인덱스와 거리(m). 없으면 (-1, inf). skip 인덱스는 제외(자기 자신)."""
    cl, co = int(lat / cell), int(lon / cell)
    coslat = math.cos(math.radians(lat))
    best, bi = max_m ** 2, -1
    for dcl in (-1, 0, 1):
        for dco in (-1, 0, 1):
            for i in g.get((cl + dcl, co + dco), ()):
                if i == skip:
                    continue
                p = pts[i]
                dm = ((lat - p[0]) * 111000.0) ** 2 + ((lon - p[1]) * 111000.0 * coslat) ** 2
                if dm < best:
                    best, bi = dm, i
    return (bi, math.sqrt(best)) if bi >= 0 else (-1, float("inf"))


def main():
    ours = load_ours()
    jj = [(c["lat"], c["lng"], c.get("speedLimit"), c.get("cameraType"), c.get("direction"),
           c.get("roadName"), c.get("isNew")) for c in json.load(open(JJIK, encoding="utf-8"))]
    print(f"우리(공공) {len(ours):,} vs jjikcam {len(jj):,}\n")

    CELL = 100.0 / 111000.0  # 최대 검사 100m 커버
    gj = grid_index(jj, CELL)
    go = grid_index(ours, CELL)

    # (재검증 1) 임계거리 민감도
    print("[재검증①] 매칭 임계거리별 우리→jjikcam 매칭률:")
    for thr in (20, 30, 50, 100):
        m = sum(1 for lat, lon, *_ in ours if nearest(lat, lon, jj, gj, CELL, thr)[0] >= 0)
        print(f"  ≤{thr:3d}m: {m:,} ({100*m//len(ours)}%)  ours-only {len(ours)-m:,}")

    # 기준 50m 로 분류
    THR = 50
    ours_only, limit_mismatch = [], []
    lim_match = lim_diff = 0
    for idx, (lat, lon, lim, loc) in enumerate(ours):
        j, _d = nearest(lat, lon, jj, gj, CELL, THR)
        if j < 0:
            ours_only.append(idx)
            continue
        jl = jj[j][2]
        if jl and lim:
            if int(jl) == lim:
                lim_match += 1
            else:
                lim_diff += 1
                limit_mismatch.append((lat, lon, lim, int(jl), jj[j][3], loc))

    # ours-only(@50m) 상호배타 분할: 경계(jjik 50~100m) / 진짜부재(jjik 없음 ≤100m){내부중복 vs 단독}
    boundary, absent = [], []
    for idx in ours_only:
        j100, _ = nearest(ours[idx][0], ours[idx][1], jj, gj, CELL, 100)
        (boundary if j100 >= 0 else absent).append(idx)
    dup_cluster, isolated = [], []
    for idx in absent:
        lat, lon, lim, loc = ours[idx]
        nj, _ = nearest(lat, lon, ours, go, CELL, 50, skip=idx)
        (dup_cluster if (nj >= 0 and ours[nj][2] == lim) else isolated).append(idx)
    assert len(boundary) + len(dup_cluster) + len(isolated) == len(ours_only)  # 분할 검증

    # jjikcam-only
    jj_only = [k for k in range(len(jj)) if nearest(jj[k][0], jj[k][1], ours, go, CELL, THR)[0] < 0]

    print(f"\n[기준 {THR}m 분류] (상호배타)")
    print(f"  매칭: {len(ours)-len(ours_only):,}")
    print(f"  ours-only: {len(ours_only):,}")
    print(f"    ├ 경계(jjik 50~100m, 동일카메라 좌표오차): {len(boundary):,}")
    print(f"    └ 진짜부재(jjik ≤100m 없음): {len(absent):,}")
    print(f"        ├ 내부중복(다른 우리카메라 50m+동일제한): {len(dup_cluster):,}")
    print(f"        └ 단독(stale/jjikcam누락 의심): {len(isolated):,}")
    print(f"  jjikcam-only(우리 누락): {len(jj_only):,}  (isNew={sum(1 for k in jj_only if jj[k][6])})")
    tot = lim_match + lim_diff
    print(f"\n[제한속도] 매칭+양쪽값 {tot:,} 중 일치 {lim_match:,} ({100*lim_match//max(tot,1)}%), 불일치 {lim_diff:,}")

    # 검토용 파일 출력
    with open(os.path.join(HERE, "ours_only_isolated.csv"), "w", newline="", encoding="utf-8-sig") as f:
        w = csv.writer(f); w.writerow(["lat", "lon", "limit", "loc"])
        for idx in isolated:
            w.writerow(ours[idx])
    with open(os.path.join(HERE, "jjikcam_only.csv"), "w", newline="", encoding="utf-8-sig") as f:
        w = csv.writer(f); w.writerow(["lat", "lon", "speedLimit", "type", "direction", "roadName", "isNew"])
        for k in jj_only:
            w.writerow(jj[k])
    with open(os.path.join(HERE, "limit_mismatch.csv"), "w", newline="", encoding="utf-8-sig") as f:
        w = csv.writer(f); w.writerow(["lat", "lon", "ours", "jjik", "jjik_type", "loc"])
        for row in limit_mismatch:
            w.writerow(row)
    print("\n검토파일: ours_only_isolated.csv, jjikcam_only.csv, limit_mismatch.csv")


if __name__ == "__main__":
    main()
