"""온로드 HUD: 신호등(정지/출발) 검출 인디케이터.

검출모듈(sunnypilot/selfdrive/traffic_light/detector.py)을 UI 위젯에서 직접 돌려
정차 중 모델 예측경로 길이로 RED(대기)/GREEN(출발 가능)을 화면 좌상단에 표시한다.
기기엔 없던 시각화 — greenLightAlert 는 소리만 났음. 제어에는 영향 없음(표시 전용).
"""
import pyray as rl

from openpilot.selfdrive.ui.ui_state import ui_state
from openpilot.system.ui.widgets import Widget
from openpilot.sunnypilot.selfdrive.traffic_light.detector import (
    TrafficLightDetector, DetectorInputs, TrafficLightState,
)

RADIUS = 28
MARGIN_X = 60
MARGIN_Y = 110
COLOR = {
    TrafficLightState.RED: rl.Color(255, 0, 21, 255),
    TrafficLightState.GREEN: rl.Color(0, 255, 38, 255),
}


class TrafficLightIndicator(Widget):
    def __init__(self):
        super().__init__()
        self._detector = TrafficLightDetector()
        self._state = TrafficLightState.OFF

    def _update_state(self):
        sm = ui_state.sm
        # 새 modelV2 가 온 프레임에만 검출 실행(20Hz 기준 유지). 어떤 경우에도 UI를 죽이지 않음.
        if not ui_state.started or not sm.updated.get('modelV2', False):
            return
        try:
            m = sm['modelV2']
            cs = sm['carState']
            if len(m.position.x) == 0:
                return
            inp = DetectorInputs(
                model_pos_x=list(m.position.x),
                v_ego=cs.vEgo,
                standstill=cs.standstill,
                has_lead=False,
                gas_pressed=False,
                cc_enabled=False,
            )
            self._state = self._detector.update(inp).state
        except Exception:
            self._state = TrafficLightState.OFF

    def _render(self, _):
        color = COLOR.get(self._state)
        if color is None:   # OFF(주행중)엔 표시 안 함
            return
        cx = int(self.rect.x + MARGIN_X + RADIUS)
        cy = int(self.rect.y + MARGIN_Y + RADIUS)
        rl.draw_circle(cx, cy, RADIUS, color)
        rl.draw_circle_lines(cx, cy, RADIUS, rl.BLACK)
