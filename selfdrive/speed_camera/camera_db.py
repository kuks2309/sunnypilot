"""단속카메라 좌표 DB 런타임 로더 — compact bin + 격자(grid) 인덱스.

tools/speed_camera/build_db.py 가 만든 speed_cameras.bin(카메라당 10바이트)을 읽어
격자 버킷에 색인하고, 현재 위경도에서 반경 내 전방 최근접 카메라를 빠르게 찾는다.
표준 라이브러리만 사용(기기 부하 최소화). 무거운 일은 이 모듈을 쓰는 데몬에서만 수행.

좌표→격자: 약 0.02도(~2.2km) 셀. 질의 시 현재 셀 + 8이웃만 검사 → 43k 전수 스캔 회피.
"""
from __future__ import annotations

import math
import os
import struct

HERE = os.path.dirname(os.path.abspath(__file__))
DEFAULT_BIN = os.path.join(HERE, "speed_cameras.bin")

MAGIC = b"SCAM"
REC = struct.Struct("<ffBBB")  # v2: lat, lon, limit, flags(category|dirmode), bearing(deg/2)
HDR = struct.Struct("<4sIII")

# 분류(flags 하위2비트) / 방향모드 비트 (build_db.py 와 동일)
NOWARN, FIXED, SECTION_START, SECTION_END = 0, 1, 2, 3
DIR_AXIS, DIR_DIRECTED = 4, 8

CELL_DEG = 0.02  # 격자 한 변(도). ~2.2km
EARTH_R = 6371000.0
CROSS_TRACK_MAX = 30.0  # 진행 경로선에서 횡방향 이격 한계(m). 옆/아래 다른 도로 카메라 배제
DIR_TOL = 70.0          # DIRECTED: 내 진행방향과 카메라 방위 차가 이 이상이면 다른 방향 → 배제
AXIS_TOL = 55.0         # AXIS: 도로 축(mod180)과 차가 이 이상이면 교차로 → 배제


def haversine_m(lat1, lon1, lat2, lon2) -> float:
    p1, p2 = math.radians(lat1), math.radians(lat2)
    dp = math.radians(lat2 - lat1)
    dl = math.radians(lon2 - lon1)
    a = math.sin(dp / 2) ** 2 + math.cos(p1) * math.cos(p2) * math.sin(dl / 2) ** 2
    return 2 * EARTH_R * math.asin(math.sqrt(a))


def bearing_deg(lat1, lon1, lat2, lon2) -> float:
    p1, p2 = math.radians(lat1), math.radians(lat2)
    dl = math.radians(lon2 - lon1)
    y = math.sin(dl) * math.cos(p2)
    x = math.cos(p1) * math.sin(p2) - math.sin(p1) * math.cos(p2) * math.cos(dl)
    return (math.degrees(math.atan2(y, x)) + 360) % 360


def _cell(lat, lon):
    return (int(math.floor(lat / CELL_DEG)), int(math.floor(lon / CELL_DEG)))


class CameraDB:
    def __init__(self, bin_path: str = DEFAULT_BIN):
        self.lats: list[float] = []
        self.lons: list[float] = []
        self.limits: list[int] = []
        self.flags: list[int] = []      # category (NOWARN/FIXED/SECTION_*)
        self.dirmodes: list[int] = []   # 0=NONE / DIR_AXIS / DIR_DIRECTED
        self.cam_brg: list[float] = []  # 카메라 방위(도). DIRECTED=주행방향, AXIS=도로축
        self.grid: dict[tuple[int, int], list[int]] = {}
        self.refdate = 0
        self.loaded = False
        self.bin_path = bin_path
        self._load()

    def _load(self):
        if not os.path.exists(self.bin_path):
            return
        with open(self.bin_path, "rb") as f:
            raw = f.read()
        magic, _ver, count, refdate = HDR.unpack_from(raw, 0)
        if magic != MAGIC:
            raise ValueError(f"bad camera DB magic: {magic!r}")
        self.refdate = refdate
        off = HDR.size
        for i in range(count):
            lat, lon, limit, fl, bearing = REC.unpack_from(raw, off + i * REC.size)
            self.lats.append(lat)
            self.lons.append(lon)
            self.limits.append(limit)
            self.flags.append(fl & 3)                       # category
            self.dirmodes.append(fl & (DIR_AXIS | DIR_DIRECTED))
            self.cam_brg.append(bearing * 2.0)              # deg/2 → deg
            self.grid.setdefault(_cell(lat, lon), []).append(i)
        self.loaded = True

    def __len__(self):
        return len(self.lats)

    def nearest_forward(self, lat, lon, heading=None, radius_m=1000.0, fov=120.0):
        """반경 내 (전방) 최근접 카메라 1건. 없으면 None.

        반환 dict: dist, limit, flags, bearing, lat, lon
        heading 가 주어지면 진행방향 ±fov/2 안의 카메라만(후방 제외).
        """
        if not self.loaded:
            return None
        cl, co = _cell(lat, lon)
        best = None
        for dcl in (-1, 0, 1):
            for dco in (-1, 0, 1):
                for i in self.grid.get((cl + dcl, co + dco), ()):  # noqa: E1133
                    if self.flags[i] == NOWARN:
                        continue
                    d = haversine_m(lat, lon, self.lats[i], self.lons[i])
                    if d > radius_m:
                        continue
                    brg = bearing_deg(lat, lon, self.lats[i], self.lons[i])
                    if heading is not None:
                        diff = abs((brg - heading + 180) % 360 - 180)
                        if diff > fov / 2:
                            continue
                        # 횡방향 이격: 다른 도로(옆/아래) 카메라 배제, 같은 경로상만 통과
                        if d * math.sin(math.radians(diff)) > CROSS_TRACK_MAX:
                            continue
                        # 방향 게이트: 카메라 방위 vs 내 진행방향
                        dm = self.dirmodes[i]
                        if dm == DIR_DIRECTED:
                            if abs((self.cam_brg[i] - heading + 180) % 360 - 180) > DIR_TOL:
                                continue   # 반대편/교차 → 배제
                        elif dm == DIR_AXIS:
                            ax = abs((self.cam_brg[i] - heading + 180) % 360 - 180)
                            if min(ax, 180 - ax) > AXIS_TOL:
                                continue   # 도로축과 큰 각 = 교차로 → 배제
                    if best is None or d < best["dist"]:
                        best = {"dist": d, "limit": self.limits[i], "flags": self.flags[i],
                                "bearing": brg, "lat": self.lats[i], "lon": self.lons[i]}
        return best
