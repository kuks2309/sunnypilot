"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
from openpilot.common.params import Params
from openpilot.selfdrive.ui.ui_state import ui_state
from openpilot.system.ui.lib.multilang import tr, tr_noop
from openpilot.system.ui.sunnypilot.widgets.list_view import toggle_item_sp, multiple_button_item_sp
from openpilot.system.ui.widgets.scroller_tici import Scroller
from openpilot.system.ui.widgets import Widget

CHEVRON_INFO_DESCRIPTION = {
  "enabled": tr_noop("Display useful metrics below the chevron that tracks the lead car " +
                     "only applicable to cars with sunnypilot longitudinal control."),
  "disabled": tr_noop("This feature requires sunnypilot longitudinal control to be available.")
}


class VisualsLayout(Widget):
  def __init__(self):
    super().__init__()

    self._params = Params()
    items = self._initialize_items()
    self._scroller = Scroller(items, line_separator=True, spacing=0)

  def _initialize_items(self):
    self._toggle_defs = {
      "BlindSpot": (
        lambda: tr("Show Blind Spot Warnings"),
        tr("Enabling this will display warnings when a vehicle is detected in your " +
           "blind spot as long as your car has BSM supported."),
        None,
      ),
      "TorqueBar": (
        lambda: tr("Steering Arc"),
        tr("Display steering arc on the driving screen when lateral control is enabled."),
        None,
      ),
      "RainbowMode": (
        lambda: tr("Enable Tesla Rainbow Mode"),
        tr("A beautiful rainbow effect on the path the model wants to take. " +
           "It does not affect driving in any way."),
        None,
      ),
      "StandstillTimer": (
        lambda: tr("Enable Standstill Timer"),
        tr("Show a timer on the HUD when the car is at a standstill."),
        None,
      ),
      "RoadNameToggle": (
        lambda: tr("Display Road Name"),
        tr("Displays the name of the road the car is traveling on." +
           "<br>The OpenStreetMap database of the location must be downloaded from " +
           "the OSM panel to fetch the road name."),
        None,
      ),
      "GreenLightAlert": (
        lambda: tr("Green Traffic Light Alert (Beta)"),
        tr("A chime and on-screen alert will play when the traffic light you are waiting for " +
           "turns green and you have no vehicle in front of you." +
           "<br>Note: This chime is only designed as a notification. " +
           "It is the driver's responsibility to observe their environment and make decisions accordingly."),
        None,
      ),
      "LeadDepartAlert": (
        lambda: tr("Lead Departure Alert (Beta)"),
        tr("A chime and on-screen alert will play when you are stopped, and the vehicle in front of you start moving." +
           "<br>Note: This chime is only designed as a notification. " +
           "It is the driver's responsibility to observe their environment and make decisions accordingly."),
        None,
      ),
      "TrueVEgoUI": (
        lambda: tr("Speedometer: Always Display True Speed"),
        tr("For applicable vehicles, always display the true vehicle current speed from wheel speed sensors."),
        None,
      ),
      "HideVEgoUI": (
        lambda: tr("Speedometer: Hide from Onroad Screen"),
        tr("When enabled, the speedometer on the onroad screen is not displayed."),
        None,
      ),
      "ShowTurnSignals": (
        lambda: tr("Display Turn Signals"),
        tr("When enabled, visual turn indicators are drawn on the HUD."),
        None,
      ),
      "RocketFuel": (
        lambda: tr("Real-time Acceleration Bar"),
        tr("Show an indicator on the left side of the screen to display real-time vehicle acceleration and deceleration. " +
           "This displays what the car is currently doing, not what the planner is requesting."),
        None,
      ),
      # [단속카메라 비활성화 2026-08-06] 동작하지 않는 토글을 남기지 않도록 설정 목록에서 제외.
      # "SpeedCameraWarnEnabled": (
      #   lambda: tr("Speed Camera Warning (Korea)"),
      #   tr("Show an on-screen alert and chime when approaching a Korean fixed speed / section enforcement camera. " +
      #      "Two stages: heads-up at ~600m, then a deceleration warning when overspeeding. " +
      #      "Works even when openpilot is not engaged. Requires the camera database to be installed on device."),
      #   None,
      # ),
      # "SpeedCameraDecelEnabled": (
      #   lambda: tr("Speed Camera: Slow Down (when engaged)"),
      #   tr("When engaged, automatically slow down to the camera speed limit + 5 km/h before a speed camera, " +
      #      "and hold the limit through a section-control zone. Only acts while openpilot longitudinal is engaged; " +
      #      "it can only lower speed, never raise it. Requires the warning above and the camera database."),
      #   None,
      # ),
      "TmapNavEnabled": (
        lambda: tr("TMAP Navigation Data (Korea)"),
        tr("Receive live navigation data (road speed limit, enforcement points, turn-by-turn) from the TMAP " +
           "phone bridge over UDP. Receive and log only for now — it does not affect control. " +
           "Stale or unverifiable data is discarded rather than used."),
        None,
      ),
    }
    self._toggles = {}
    for param, (title, desc, callback) in self._toggle_defs.items():
      toggle = toggle_item_sp(
        title=title,
        description=desc,
        param=param,
        initial_state=ui_state.params.get_bool(param),
        callback=callback,
      )
      self._toggles[param] = toggle

    self._chevron_info = multiple_button_item_sp(
      title=lambda: tr("Display Metrics Below Chevron"),
      description="",
      buttons=[lambda: tr("Off"), lambda: tr("Distance"), lambda: tr("Speed"), lambda: tr("Time"), lambda: tr("All")],
      param="ChevronInfo",
      inline=False
    )
    self._dev_ui_info = multiple_button_item_sp(
      title=lambda: tr("Developer UI"),
      description=lambda: tr("Display real-time parameters and metrics from various sources."),
      buttons=[lambda: tr("Off"), lambda: tr("Bottom"), lambda: tr("Right"), lambda: tr("Right & Bottom")],
      param="DevUIInfo",
      button_width=350,
      inline=False
    )
    # [단속카메라 비활성화 2026-08-06] 경고음·시작여유 선택기 제외.
    # self._speed_cam_sound = multiple_button_item_sp(
    #   title=lambda: tr("Speed Camera Warning Sound"),
    #   description=lambda: tr("Sound played for speed camera warnings (when the warning starts). Mute = on-screen only."),
    #   buttons=[lambda: tr("Mute"), lambda: tr("Soft"), lambda: tr("Immediate"), lambda: tr("Prompt")],
    #   param="SpeedCameraWarnSound",
    #   button_width=350,
    #   inline=False
    # )
    # self._speed_cam_margin = multiple_button_item_sp(
    #   title=lambda: tr("Speed Camera Slow Down: Start Margin"),
    #   description=lambda: tr("Extra distance before the computed deceleration point. Larger = start slowing earlier. " +
    #                         "The deceleration rate matches normal lead-following braking and is not changed."),
    #   buttons=[lambda: tr("0 m"), lambda: tr("15 m"), lambda: tr("30 m"), lambda: tr("50 m"), lambda: tr("80 m")],
    #   param="SpeedCameraDecelMargin",
    #   button_width=220,
    #   inline=False
    # )

    items = list(self._toggles.values()) + [
      self._chevron_info,
      self._dev_ui_info,
      # self._speed_cam_sound,    # [단속카메라 비활성화 2026-08-06]
      # self._speed_cam_margin,   # [단속카메라 비활성화 2026-08-06]
    ]
    return items

  def _update_state(self):
    super()._update_state()

    for param in self._toggle_defs:
      self._toggles[param].action_item.set_state(self._params.get_bool(param))

    self._dev_ui_info.action_item.set_selected_button(ui_state.params.get("DevUIInfo", return_default=True))
    # [단속카메라 비활성화 2026-08-06] 위젯이 생성되지 않으므로 참조도 제거(남기면 AttributeError).
    # self._speed_cam_sound.action_item.set_selected_button(ui_state.params.get("SpeedCameraWarnSound", return_default=True))
    # self._speed_cam_margin.action_item.set_selected_button(ui_state.params.get("SpeedCameraDecelMargin", return_default=True))

    if ui_state.has_longitudinal_control:
      self._chevron_info.set_description(tr(CHEVRON_INFO_DESCRIPTION["enabled"]))
      self._chevron_info.action_item.set_selected_button(ui_state.params.get("ChevronInfo", return_default=True))
      self._chevron_info.action_item.set_enabled(True)
    else:
      self._chevron_info.set_description(tr(CHEVRON_INFO_DESCRIPTION["disabled"]))
      self._chevron_info.action_item.set_enabled(False)
      ui_state.params.put("ChevronInfo", 0)

  def _render(self, rect):
    self._scroller.render(rect)

  def show_event(self):
    self._scroller.show_event()
    if not ui_state.has_longitudinal_control:
      self._chevron_info.set_description(tr(CHEVRON_INFO_DESCRIPTION["disabled"]))
      self._chevron_info.show_description(True)
