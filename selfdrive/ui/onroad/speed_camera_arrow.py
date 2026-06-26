"""온로드 HUD: 단속카메라 상대방위 화살표 + 거리 (표시 전용).

speed_camera_warnd 가 customReservedRawData0(JSON)로 보낸 payload 를 읽어
카메라가 내 차 기준 어느 방향(rb=상대방위, 0=전방, +우/−좌)인지 회전 화살표로,
남은 거리는 텍스트로 표시한다. 제어에는 영향 없음.

폰트는 라틴 글리프만이라 텍스트는 거리(숫자+m)만 — 방향은 화살표 그래픽으로.
"""
import json
import math

import pyray as rl

from openpilot.selfdrive.ui.ui_state import ui_state
from openpilot.system.ui.lib.application import gui_app, FontWeight
from openpilot.system.ui.lib.text_measure import measure_text_cached
from openpilot.system.ui.widgets import Widget

ARROW_LEN = 64.0
ARROW_HALF = 42.0
BG_RADIUS = 86
DIST_FONT = 48


class SpeedCameraArrow(Widget):
    def __init__(self):
        super().__init__()
        self._stage = 0
        self._dist = 0
        self._rb = 999      # 상대방위(도). 999=미상 → 전방 표시
        self._font = gui_app.font(FontWeight.BOLD)

    def _update_state(self):
        self._stage = 0
        sm = ui_state.sm
        try:
            raw = sm['customReservedRawData0']
            if raw:
                d = json.loads(bytes(raw))
                self._stage = d.get("s", 0)
                self._dist = d.get("d", 0)
                self._rb = d.get("rb", 999)
        except (ValueError, KeyError, TypeError):
            self._stage = 0

    def _render(self, rect: rl.Rectangle):
        if self._stage <= 0:
            return
        cx = rect.x + rect.width / 2.0
        cy = rect.y + 430.0   # 헤더(속도) 아래. 실차에서 위치 미세조정 가능

        # 배경 원(대비)
        rl.draw_circle(int(cx), int(cy), BG_RADIUS, rl.Color(0, 0, 0, 160))
        rl.draw_circle_lines(int(cx), int(cy), BG_RADIUS, rl.Color(255, 255, 255, 90))

        # 회전 화살표: rb=0 → 위(전방), +우(시계). 미상이면 전방.
        th = math.radians(0.0 if self._rb == 999 else float(self._rb))
        c, s = math.cos(th), math.sin(th)

        def rot(x, y):
            return rl.Vector2(cx + x * c - y * s, cy + x * s + y * c)

        tip = rot(0.0, -ARROW_LEN)
        left = rot(-ARROW_HALF, ARROW_LEN * 0.55)
        right = rot(ARROW_HALF, ARROW_LEN * 0.55)
        col = rl.Color(255, 80, 80, 255) if self._stage == 2 else rl.Color(0, 220, 160, 255)
        rl.draw_triangle(tip, left, right, col)
        rl.draw_triangle(tip, right, left, col)  # 양방향 winding(컬링 회피)

        # 남은 거리(숫자만, 라틴 폰트)
        txt = f"{self._dist}m"
        sz = measure_text_cached(self._font, txt, DIST_FONT)
        rl.draw_text_ex(self._font, txt, rl.Vector2(cx - sz.x / 2.0, cy + BG_RADIUS + 6.0),
                        DIST_FONT, 0, rl.WHITE)
