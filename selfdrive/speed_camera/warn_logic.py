"""단속카메라 경고·감속 결정 로직 (cereal 비의존 순수 모듈).

speed_camera_warnd(데몬)와 SIL 하니스가 공유한다. messaging/params 같은 런타임 의존이 없어
PC에서 합성 주행으로 단위·SIL 검증이 가능하다. 데몬은 이 클래스에 GPS/속도/토글만 넘긴다.

출력 payload(dict): {"s":stage, "d":dist, "l":limit, "f":flags, "snd":sound, "c":chime, "dl":decel_kph, "dd":decel_dist}
    stage  0:없음 1:사전알림 2:감속경고
    dl     engage 감속 목표 km/h (0=비활성, 제한속도+offset)
"""
from __future__ import annotations

from openpilot.selfdrive.speed_camera.camera_db import FIXED, SECTION_START, SECTION_END, NOWARN

HEADSUP_DIST = 600.0     # 사전 알림 시작 거리(m)
SEARCH_RADIUS = 800.0    # DB 검색 반경(m)
FOV_DEG = 120.0          # 전방 시야각(후방 카메라 제외)
COMFORT_BRAKE = 2.0      # 편안한 감속도 m/s² (openpilot 종방향 튜닝값)
REACTION_S = 1.0         # 반응 여유시간
DECEL_MARGIN = 15.0      # 감속 트리거 추가 여유(m)
MIN_OVERSPEED_MS = 1.0   # 이 이상 초과해야 감속(노이즈 방지)
DECEL_OFFSET_KPH = 5     # engage 감속 목표 = 제한속도 + 5km/h
CHIME_S = 1.2            # 단계 진입 시 소리 윈도우(s)
HEADING_MIN_SPEED = 1.0  # 이 이하 속도는 방위 부정확 → 전방필터 해제

# 감속/경고 시작 "여유 거리"(m) GUI 선택지. param SpeedCameraDecelMargin 의 인덱스 → 이 값.
# margin 을 키우면 그만큼 더 일찍 시작. 감속 강도(COMFORT_BRAKE)는 차간거리 감속과 동일(고정).
DECEL_MARGIN_OPTIONS = [0.0, 15.0, 30.0, 50.0, 80.0]


def decel_trigger_dist(v_ms: float, limit_kph: float, margin: float = DECEL_MARGIN) -> float:
    """현재속도 v 에서 limit 까지 편안히 감속하는 데 필요한 거리(m) + 시작 여유(margin)."""
    vf = limit_kph / 3.6
    if v_ms <= vf:
        return 0.0
    return (v_ms * v_ms - vf * vf) / (2 * COMFORT_BRAKE) + v_ms * REACTION_S + margin


class SpeedCameraLogic:
    def __init__(self, db, rate_hz: float = 5.0):
        self.db = db
        self.prev_stage = 0
        self.chime_left = 0
        self.chime_cycles = max(1, int(CHIME_S * rate_hz))
        self.decel_margin = DECEL_MARGIN  # 시작 여유(m). 데몬이 param 으로 갱신.

    def update(self, lat: float, lon: float, bearing: float, speed_ms: float,
               warn_enabled: bool, decel_enabled: bool, sound: int, have_fix: bool = True) -> dict:
        stage, dist, limit, flags = 0, 0, 0, 0
        dl, dd = 0, 0

        if (warn_enabled or decel_enabled) and self.db.loaded and have_fix:
            heading = bearing if speed_ms > HEADING_MIN_SPEED else None  # 정지 시 전방필터 해제
            hit = self.db.nearest_forward(lat, lon, heading=heading,
                                          radius_m=SEARCH_RADIUS, fov=FOV_DEG)
            if hit and hit["flags"] != NOWARN:
                dist = int(hit["dist"])
                limit = int(hit["limit"])
                flags = int(hit["flags"])
                v = max(speed_ms, 0.0)
                target = limit + DECEL_OFFSET_KPH if limit > 0 else 0

                if warn_enabled:
                    if (limit > 0 and flags != SECTION_END
                            and v > limit / 3.6 + MIN_OVERSPEED_MS
                            and hit["dist"] <= decel_trigger_dist(v, limit, self.decel_margin)):
                        stage = 2
                    elif hit["dist"] <= HEADSUP_DIST:
                        stage = 1

                if decel_enabled and target > 0:
                    if flags == SECTION_END:          # 전방 최근접=종점 → 구간 내부 → 제한 유지
                        dl, dd = target, 0
                    elif (flags in (FIXED, SECTION_START)
                          and v > target / 3.6
                          and hit["dist"] <= decel_trigger_dist(v, target, self.decel_margin)):
                        dl, dd = target, dist

        # chime: 단계 신규/상향 시에만 윈도우 동안 1 (wav 반복특성과 무관한 단발)
        if stage > self.prev_stage and stage > 0:
            self.chime_left = self.chime_cycles
        self.prev_stage = stage
        if stage == 0:
            self.chime_left = 0
        chime = 1 if self.chime_left > 0 else 0
        if self.chime_left > 0:
            self.chime_left -= 1

        return {"s": stage, "d": dist, "l": limit, "f": flags,
                "snd": sound, "c": chime, "dl": dl, "dd": dd}
