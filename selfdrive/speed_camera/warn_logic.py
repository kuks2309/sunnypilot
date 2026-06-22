"""단속카메라 경고·감속 결정 로직 (cereal 비의존 순수 모듈).

speed_camera_warnd(데몬)와 SIL 하니스가 공유한다. messaging/params 같은 런타임 의존이 없어
PC에서 합성 주행으로 단위·SIL 검증이 가능하다. 데몬은 이 클래스에 위치/속도/토글만 넘긴다.

래치(latch): 전방 카메라를 한 번 잡으면 **통과할 때까지** 그 카메라를 계속 추적·표시한다.
곡선/교차로에서 전방 시야각을 잠깐 벗어나도(혹은 GPS 음영) 깜빡이지 않고 거리(haversine)를 직접 갱신.

출력 payload(dict): {"s":stage, "d":dist, "l":limit, "f":flags, "snd":sound, "c":chime, "dl":decel_kph, "dd":decel_dist}
    stage  0:없음 1:사전알림 2:감속경고
    dl     engage 감속 목표 km/h (0=비활성, 제한속도+offset)
"""
from __future__ import annotations

from openpilot.selfdrive.speed_camera.camera_db import (
    haversine_m, bearing_deg, FIXED, SECTION_START, SECTION_END, NOWARN)

HEADSUP_DIST = 600.0     # 사전 알림(=래치) 시작 거리(m)
SEARCH_RADIUS = 800.0    # DB 검색 반경(m)
FOV_DEG = 120.0          # 전방 시야각(래치 채택 시 후방 카메라 제외)
COMFORT_BRAKE = 2.0      # 편안한 감속도 m/s² (openpilot 종방향 튜닝값)
REACTION_S = 1.0         # 반응 여유시간
DECEL_MARGIN = 15.0      # 감속 트리거 추가 여유(m)
MIN_OVERSPEED_MS = 1.0   # 이 이상 초과해야 감속(노이즈 방지)
DECEL_OFFSET_KPH = 5     # engage 감속 목표 = 제한속도 + 5km/h
CHIME_S = 1.2            # 단계 진입 시 소리 윈도우(s)
HEADING_MIN_SPEED = 1.0  # 이 이하 속도는 방위 부정확 → 전방필터 해제
PASS_MARGIN_M = 12.0     # 최근접 후 이만큼 멀어지면 통과로 간주(래치 해제) — 빠른 해제
BEHIND_DEG = 95.0        # 진행방향 대비 이 각도 이상 뒤면 통과로 간주(통과 직후 해제)
PASS_MIN_M = 15.0        # 통과판정 최소 거리하한(이보다 가까우면 아직 통과 전)
SWITCH_CLOSER_M = 50.0   # 래치 중 더 가까운 다른 카메라가 나타나면 전환하는 거리차

# 감속/경고 시작 "여유 거리"(m) GUI 선택지. param SpeedCameraDecelMargin 의 인덱스 → 이 값.
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
        self.decel_margin = DECEL_MARGIN
        self.target: dict | None = None  # 래치된 카메라 {lat,lon,limit,flags,min_dist}

    def _update_target(self, lat, lon, heading, hit):
        # 1) 래치된 카메라 통과 판정 → 해제
        if self.target is not None:
            t = self.target
            d = haversine_m(lat, lon, t["lat"], t["lon"])
            t["min_dist"] = min(t["min_dist"], d)
            behind = heading is not None and abs((bearing_deg(lat, lon, t["lat"], t["lon"]) - heading + 180) % 360 - 180) > BEHIND_DEG
            grew = d > t["min_dist"] + PASS_MARGIN_M and d > PASS_MIN_M
            if behind or grew:
                self.target = None

        # 2) 전방 카메라 채택/전환 (HEADSUP 안으로 들어온 것만 래치)
        if hit and hit["flags"] != NOWARN and hit["dist"] <= HEADSUP_DIST:
            if self.target is None:
                self.target = {"lat": hit["lat"], "lon": hit["lon"], "limit": hit["limit"],
                               "flags": hit["flags"], "min_dist": hit["dist"]}
            else:
                dt = haversine_m(lat, lon, self.target["lat"], self.target["lon"])
                diff_cam = abs(hit["lat"] - self.target["lat"]) > 1e-5 or abs(hit["lon"] - self.target["lon"]) > 1e-5
                if diff_cam and hit["dist"] < dt - SWITCH_CLOSER_M:
                    self.target = {"lat": hit["lat"], "lon": hit["lon"], "limit": hit["limit"],
                                   "flags": hit["flags"], "min_dist": hit["dist"]}

    def update(self, lat: float, lon: float, bearing: float, speed_ms: float,
               warn_enabled: bool, decel_enabled: bool, sound: int, have_fix: bool = True) -> dict:
        stage, dist, limit, flags = 0, 0, 0, 0
        dl, dd = 0, 0

        if not (warn_enabled or decel_enabled):
            self.target = None
        elif self.db.loaded and have_fix:
            heading = bearing if speed_ms > HEADING_MIN_SPEED else None
            hit = self.db.nearest_forward(lat, lon, heading=heading, radius_m=SEARCH_RADIUS, fov=FOV_DEG)
            self._update_target(lat, lon, heading, hit)

            if self.target is not None:
                t = self.target
                d = haversine_m(lat, lon, t["lat"], t["lon"])  # 래치 대상까지 직접 거리(시야각 무관)
                dist, limit, flags = int(d), t["limit"], t["flags"]
                v = max(speed_ms, 0.0)
                target_kph = limit + DECEL_OFFSET_KPH if limit > 0 else 0

                if warn_enabled:
                    if (limit > 0 and flags != SECTION_END
                            and v > limit / 3.6 + MIN_OVERSPEED_MS
                            and d <= decel_trigger_dist(v, limit, self.decel_margin)):
                        stage = 2
                    else:
                        stage = 1  # 래치된 동안 통과 전까지 계속 표시

                if decel_enabled and target_kph > 0:
                    if flags == SECTION_END:
                        dl, dd = target_kph, 0
                    elif (flags in (FIXED, SECTION_START)
                          and v > target_kph / 3.6
                          and d <= decel_trigger_dist(v, target_kph, self.decel_margin)):
                        dl, dd = target_kph, dist
        # have_fix 없음(터널 등): 래치는 유지하되 이번 사이클은 표시 안 함(드리프트 오표시 방지) → 복귀 시 재개

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
