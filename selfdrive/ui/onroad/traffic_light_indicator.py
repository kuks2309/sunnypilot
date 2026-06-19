"""온로드 HUD: 신호등(정지/출발) 검출 인디케이터.

검출모듈(sunnypilot/selfdrive/traffic_light/detector.py)을 UI 위젯에서 직접 돌려
정차 중 모델 예측경로 길이로 RED(대기)/GREEN(출발 가능)을 표시한다.
제어에는 영향 없음(표시 전용).

⚠️ 현재 진단(DEBUG) 모드: state 무관 항상 화면 정중앙에 큰 점을 그림(색=state).
   안 보이면 _render 미호출, 노랑=OFF(게이트/검출 실패), 빨강/초록=정상. 원인 확인 후 원복.
"""
import pyray as rl

from openpilot.common.swaglog import cloudlog
from openpilot.selfdrive.ui.ui_state import ui_state
from openpilot.system.ui.widgets import Widget
from openpilot.sunnypilot.selfdrive.traffic_light.detector import (
    TrafficLightDetector, DetectorInputs, TrafficLightState,
)

DIAG_COLOR = {
    TrafficLightState.OFF: rl.Color(255, 230, 0, 255),    # 노랑 = OFF/검출 미작동
    TrafficLightState.RED: rl.Color(255, 0, 21, 255),     # 빨강
    TrafficLightState.GREEN: rl.Color(0, 255, 38, 255),   # 초록
}
DIAG_RADIUS = 50


class TrafficLightIndicator(Widget):
    def __init__(self):
        super().__init__()
        self._detector = TrafficLightDetector()
        self._state = TrafficLightState.OFF
        self._frame = 0
        self._ran = False
        self._last_err = ""

    def _update_state(self):
        self._frame += 1
        sm = ui_state.sm
        if not ui_state.started or not sm.updated.get('modelV2', False):
            if self._frame % 60 == 0:
                cloudlog.warning(f"[TLI] gate closed started={ui_state.started} "
                                 f"updated={sm.updated.get('modelV2', False)}")
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
            self._ran = True
        except Exception as e:
            self._state = TrafficLightState.OFF
            self._last_err = repr(e)[:120]
        if self._frame % 30 == 0:
            cloudlog.warning(f"[TLI] state={self._state.name} ran={self._ran} "
                             f"err={self._last_err} rect=({int(self.rect.x)},{int(self.rect.y)},"
                             f"{int(self.rect.width)},{int(self.rect.height)})")

    def _render(self, _):
        # 진단: state 무관 항상 화면 정중앙에 큰 점(색=state)
        color = DIAG_COLOR.get(self._state, rl.Color(255, 0, 255, 255))  # 핑크 = unknown
        cx = int(self.rect.x + self.rect.width / 2)
        cy = int(self.rect.y + self.rect.height / 2)
        rl.draw_circle(cx, cy, DIAG_RADIUS, color)
        rl.draw_circle_lines(cx, cy, DIAG_RADIUS, rl.BLACK)
