#!/usr/bin/env python3
"""speed_camera_warnd — 단속카메라 거리경고/감속 데몬 (얇은 런타임 글루).

내장 GNSS(gpsLocation/gpsLocationExternal, ublox 유무로 자동선택)를 저주파로 구독해 SpeedCameraLogic 에 넘기고,
결과 payload 를 customReservedRawData0(raw bytes)로 발행한다.
무거운 일(43k DB 격자검색)은 전부 여기서만 → 안전 루프(selfdrived 100Hz)에 부담 0.
결정 로직은 warn_logic.py 에 분리(cereal 비의존)되어 PC에서 SIL 검증 가능.

- selfdrived: payload.s>0 → EventName.speedCameraWarning(ET.PERMANENT) → engage 무관 화면+소리
- plannerd/SLA: payload.dl/dd → 카메라 제한속도 fail-safe 주입 → engage 중에만 감속
"""
import json

from cereal import messaging
from openpilot.common.gps import get_gps_location_service
from openpilot.common.params import Params
from openpilot.common.realtime import Ratekeeper, config_realtime_process
from openpilot.common.swaglog import cloudlog
from openpilot.selfdrive.speed_camera.camera_db import CameraDB
from openpilot.selfdrive.speed_camera.warn_logic import SpeedCameraLogic, DECEL_MARGIN_OPTIONS

RATE_HZ = 5.0          # 데몬 주기(저주파)
PARAM_REFRESH = 1.0    # 파라미터 재읽기 주기(s)


def main():
    config_realtime_process([0, 1, 2, 3], priority=5)
    cloudlog.info("speed_camera_warnd: loading camera DB")
    db = CameraDB()
    cloudlog.info(f"speed_camera_warnd: {len(db)} cameras, refdate={db.refdate}, loaded={db.loaded}")

    logic = SpeedCameraLogic(db, RATE_HZ)
    params = Params()
    gps_service = get_gps_location_service(params)  # ublox 유무에 따라 gpsLocationExternal/gpsLocation
    cloudlog.info(f"speed_camera_warnd: gps service = {gps_service}")
    sm = messaging.SubMaster([gps_service])
    pm = messaging.PubMaster(['customReservedRawData0'])
    rk = Ratekeeper(RATE_HZ, print_delay_threshold=None)

    enabled = decel_enabled = False
    sound = 1
    param_timer = 0.0

    while True:
        sm.update(0)

        # 파라미터는 저주파로만 갱신(파일 IO 최소화)
        param_timer += 1.0 / RATE_HZ
        if param_timer >= PARAM_REFRESH:
            param_timer = 0.0
            try:
                enabled = params.get_bool("SpeedCameraWarnEnabled")
                decel_enabled = params.get_bool("SpeedCameraDecelEnabled")
                sound = int(params.get("SpeedCameraWarnSound") or 1)
                midx = int(params.get("SpeedCameraDecelMargin") or 1)
                logic.decel_margin = DECEL_MARGIN_OPTIONS[midx % len(DECEL_MARGIN_OPTIONS)]
            except (ValueError, TypeError):
                enabled, decel_enabled, sound = False, False, 1

        gps = sm[gps_service]
        have_fix = sm.valid[gps_service] and (gps.latitude != 0.0 or gps.longitude != 0.0)

        payload = logic.update(gps.latitude, gps.longitude, gps.bearingDeg, max(gps.speed, 0.0),
                               enabled, decel_enabled, sound, have_fix)

        # customReservedRawData0 은 Data(raw bytes) 필드 → new_message 의 init(service) 가 안 먹힘.
        # service=None 으로 빈 Event 생성 후 Data 필드에 직접 대입.
        msg = messaging.new_message(None, valid=True)
        msg.customReservedRawData0 = json.dumps(payload).encode()
        pm.send('customReservedRawData0', msg)

        rk.keep_time()


if __name__ == "__main__":
    main()
