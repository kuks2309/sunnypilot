#!/usr/bin/env python3
"""단속카메라 경고/감속 SIL(Software-In-the-Loop) 검증 — 합성 주행 폐루프.

cereal/기기 없이 PC에서 검증한다. 실제 데몬이 쓰는 순수 로직(SpeedCameraLogic)과
동일 카메라 DB(speed_cameras.bin)를 그대로 사용하고, resolver의 fail-safe min +
SLA 감속지령을 모사해 차속을 적분한다 → "카메라 지점에서 제한+5km/h로 감속하는가"를 확인.

시나리오
  1) 경고만(비engage): 화면/소리 단계가 올바른 거리에서 발동, 감속은 0(차속 유지)
  2) 경고+감속(engage): 카메라에서 제한+5 로 수렴
  3) 구간단속(engage): 시점 통과 후 종점까지 제한 유지

실행:  python tools/speed_camera/sil_test.py
"""
from __future__ import annotations

import os
import sys

# openpilot.* 네임스페이스(PEP 420)로 import — repo 루트의 부모를 path에 추가
sys.path.insert(0, os.path.normpath(os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "..", "..")))

from openpilot.selfdrive.speed_camera.camera_db import CameraDB, FIXED, SECTION_END  # noqa: E402
from openpilot.selfdrive.speed_camera.warn_logic import SpeedCameraLogic, DECEL_OFFSET_KPH, COMFORT_BRAKE  # noqa: E402

RATE = 5.0
DT = 1.0 / RATE
M_PER_DEG = 111_000.0


def offset_south(lat, lon, meters):
    """카메라에서 남쪽으로 meters 떨어진 점(=북쪽으로 접근). 경도 동일."""
    return lat - meters / M_PER_DEG, lon


def sla_accel(v, dl_kph, dd_m):
    """SLA 감속지령 모사: dd>0면 카메라 지점에서 vf 도달하는 등감속, dd==0(구간유지)면 시정수 감속."""
    vf = dl_kph / 3.6
    if v <= vf:
        return max((vf - v) / 1.0, 0.0)            # 이미 이하 → 천천히 복귀(상한 없음, 시나리오상 미사용)
    if dd_m > 0:
        a = (vf * vf - v * v) / (2 * max(dd_m, 1.0))  # 음수(감속)
    else:
        a = (vf - v) / 2.0                          # 구간 유지: 시정수 2s
    return max(a, -COMFORT_BRAKE - 0.5)             # 편안한 감속 + 약간 여유로 클램프


def run(db, cam_lat, cam_lon, label, start_dist, v0_kph, warn, decel, engaged, expect_hold=False):
    logic = SpeedCameraLogic(db, RATE)
    v = v0_kph / 3.6
    d = float(start_dist)                            # 카메라까지 1D 남은거리
    cruise_v = v0_kph / 3.6
    rows = []
    min_dist_seen = d
    fired_headsup = fired_decel = False
    v_at_camera = None

    t = 0.0
    for _ in range(int(start_dist / max(v, 1.0) / DT) + 400):
        lat, lon = offset_south(cam_lat, cam_lon, d)
        payload = logic.update(lat, lon, 0.0, v, warn, decel, sound=1, have_fix=True)
        s, dl, dd = payload["s"], payload["dl"], payload["dd"]
        if s == 1:
            fired_headsup = True
        if s == 2 or dl > 0:
            fired_decel = True

        # resolver fail-safe min(기존제한 없음=cruise) + SLA(engage 게이트)
        if engaged and decel and dl > 0:
            a = sla_accel(v, dl, dd)
        else:
            a = (cruise_v - v) / 1.0                 # 크루즈 복귀(상한)
        v = max(v + a * DT, 0.0)

        # 카메라 통과 직전 기록
        if d <= min_dist_seen:
            min_dist_seen = d
        if v_at_camera is None and d <= 5.0:
            v_at_camera = v

        rows.append((t, d, v * 3.6, s, dl, payload["f"]))
        # 전진(구간유지 검증은 종점 통과까지 더 진행)
        d -= v * DT
        t += DT
        if d < (-300 if expect_hold else -20):
            break

    # 결과 출력(샘플)
    print(f"\n=== {label} ===")
    print(f"  카메라({cam_lat:.5f},{cam_lon:.5f}) 접근 {start_dist}m, 초기 {v0_kph}km/h, "
          f"warn={warn} decel={decel} engaged={engaged}")
    print(f"  {'t':>5} {'남은m':>7} {'속도kph':>7} {'stage':>5} {'dl':>4} {'flags':>5}")
    step = max(1, len(rows) // 14)
    for i in range(0, len(rows), step):
        t_, d_, vk, s, dl, fl = rows[i]
        mark = "  <감속발동" if dl > 0 else ("  <사전알림" if s == 1 else "")
        print(f"  {t_:5.1f} {d_:7.0f} {vk:7.1f} {s:5d} {dl:4d} {fl:5d}{mark}")
    return fired_headsup, fired_decel, v_at_camera


def find_camera(db, want_flags, want_limit=None):
    for i in range(len(db)):
        if db.flags[i] == want_flags and (want_limit is None or db.limits[i] == want_limit):
            return db.lats[i], db.lons[i], db.limits[i]
    return None


def main():
    db = CameraDB()
    if not db.loaded:
        print("[에러] speed_cameras.bin 없음 → 먼저 build_db.py 실행")
        return 1
    print(f"카메라 DB {len(db):,}건 로드 (refdate={db.refdate})")

    fixed = find_camera(db, FIXED, 50) or find_camera(db, FIXED)
    flat, flon, flim = fixed
    target = flim + DECEL_OFFSET_KPH
    fails = []

    # 1) 경고만(비engage) — 감속 0, 화면 단계만
    hu, dec, vcam = run(db, flat, flon, f"경고만(비engage) FIXED {flim}", 900, 90, warn=True, decel=False, engaged=False)
    if not hu:
        fails.append("경고: 사전알림 미발동")
    if vcam is not None and vcam < 85 / 3.6:
        fails.append(f"경고만인데 감속됨(v_at_cam={vcam*3.6:.0f})")

    # 2) 경고+감속(engage) — 카메라에서 제한+5 수렴
    hu2, dec2, vcam2 = run(db, flat, flon, f"경고+감속(engage) FIXED {flim}→목표{target}", 900, 90, warn=True, decel=True, engaged=True)
    if not dec2:
        fails.append("감속: 발동 안됨")
    if vcam2 is None:
        fails.append("감속: 카메라 도달 전 종료")
    elif vcam2 * 3.6 > target + 3:
        fails.append(f"감속 부족: 카메라서 {vcam2*3.6:.0f}km/h > 목표 {target}+3")

    # 3) 구간단속(engage) — 종점까지 유지
    sect = find_camera(db, SECTION_END)
    if sect:
        slat, slon, slim = sect
        st = slim + DECEL_OFFSET_KPH
        hu3, dec3, vcam3 = run(db, slat, slon, f"구간단속 종점접근(engage) {slim}→유지{st}", 700, 90, warn=True, decel=True, engaged=True, expect_hold=True)
        if not dec3:
            fails.append("구간단속: 감속/유지 미발동")
        elif vcam3 is not None and vcam3 * 3.6 > st + 3:
            fails.append(f"구간단속 유지 부족: {vcam3*3.6:.0f} > {st}+3")
    else:
        print("\n[경고] SECTION_END 카메라 없음 — 구간단속 시나리오 생략")

    print("\n" + "=" * 50)
    if fails:
        print("SIL 검정 실패:")
        for f in fails:
            print("  - " + f)
        return 1
    print("SIL 검정 통과 ✅  (경고 단계·감속 수렴·구간유지 정상)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
