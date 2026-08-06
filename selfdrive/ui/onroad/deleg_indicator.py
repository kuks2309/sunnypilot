"""온로드 HUD: 종방향 제어 주체 표시 (표시 전용).

Tesla 종방향 위임(delegation)의 blend 값을 carOutput.actuatorsOutput.speed(디버그 필드,
carcontroller 가 0=openpilot ~ 1=Tesla 로 기록)에서 읽어 화면 오른쪽에 표시한다.

  OP    (초록)  = openpilot 이 종방향 제어 중
  TESLA (빨강)  = Tesla pass-through 위임 중
  T 45% (주황)  = 전환 램프 구간

engage 중에만 표시. 폰트는 라틴 글리프만이라 문구는 영문 ASCII.
"""
import pyray as rl

from openpilot.selfdrive.ui.ui_state import ui_state
from openpilot.system.ui.lib.application import gui_app, FontWeight
from openpilot.system.ui.lib.text_measure import measure_text_cached
from openpilot.system.ui.widgets import Widget

FONT_SIZE = 52
PAD_X = 22
PAD_Y = 12
RIGHT_MARGIN = 40

COLOR_OP = rl.Color(0, 201, 101, 230)       # green
COLOR_TESLA = rl.Color(226, 33, 60, 230)    # tesla red
COLOR_BLEND = rl.Color(255, 166, 0, 230)    # amber
COLOR_TEXT = rl.Color(255, 255, 255, 255)
COLOR_BG = rl.Color(0, 0, 0, 120)


class DelegIndicator(Widget):
    def __init__(self):
        super().__init__()
        self._font = gui_app.font(FontWeight.BOLD)
        self._blend = 0.0
        self._engaged = False

    def _update_state(self):
        self._engaged = False
        sm = ui_state.sm
        if not ui_state.started:
            return
        try:
            self._engaged = sm['selfdriveState'].enabled
            self._blend = float(sm['carOutput'].actuatorsOutput.speed)
        except (KeyError, AttributeError):
            self._blend = 0.0

    def _render(self, _):
        if not self._engaged:
            return
        if self._blend >= 0.99:
            txt, color = "TESLA", COLOR_TESLA
        elif self._blend <= 0.01:
            txt, color = "OP", COLOR_OP
        else:
            txt, color = f"T {int(self._blend * 100):d}%", COLOR_BLEND

        sz = measure_text_cached(self._font, txt, FONT_SIZE)
        x = self.rect.x + self.rect.width - RIGHT_MARGIN - sz.x - 2 * PAD_X
        y = self.rect.y + self.rect.height / 2 - sz.y / 2 - PAD_Y
        box = rl.Rectangle(x, y, sz.x + 2 * PAD_X, sz.y + 2 * PAD_Y)
        rl.draw_rectangle_rounded(box, 0.3, 8, COLOR_BG)
        rl.draw_rectangle_rounded_lines_ex(box, 0.3, 8, 3, color)
        rl.draw_text_ex(self._font, txt, rl.Vector2(x + PAD_X, y + PAD_Y), FONT_SIZE, 0, color)
