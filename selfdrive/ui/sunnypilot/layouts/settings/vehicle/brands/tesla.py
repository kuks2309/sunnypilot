"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
from openpilot.selfdrive.ui.sunnypilot.layouts.settings.vehicle.brands.base import BrandSettings
from openpilot.selfdrive.ui.ui_state import ui_state
from openpilot.system.ui.lib.multilang import tr
from openpilot.system.ui.sunnypilot.widgets.list_view import toggle_item_sp

class TeslaSettings(BrandSettings):
  def __init__(self):
    super().__init__()
    self.coop_steering_toggle = toggle_item_sp(tr("Cooperative Steering"), "", param="TeslaCoopSteering")
    self.long_fusion_toggle = toggle_item_sp(tr("Tesla ACC Longitudinal Fusion (Experimental A/B)"), "",
                                             param="TeslaLongitudinalFusion")
    self.curve_assist_toggle = toggle_item_sp(tr("Tesla Curve Assist Delegation (Experimental A/B)"), "",
                                              param="TeslaCurveAssistDelegation")
    self.items = [self.coop_steering_toggle, self.long_fusion_toggle, self.curve_assist_toggle]

  def update_settings(self):
    coop_steering_desc = (
      f"{tr('Converts light steering input into steering-wheel rotation.')}<br>" +
      f"{tr('The faster you go, the stiffer the steering gets.')}"
    )

    long_fusion_desc = (
      f"{tr('Experimental A/B option. When openpilot longitudinal is active, floors openpilot accel with Teslas own')}<br>" +
      f"{tr('ACC deceleration so Teslas lead braking can override openpilots weaker vision braking. Only adds braking, never speeds up.')}"
    )

    curve_assist_desc = (
      f"{tr('Experimental A/B option, requires Longitudinal Fusion. When Tesla lowers its own target speed for an')}<br>" +
      f"{tr('upcoming curve (curve assist), hand longitudinal control to Tesla so the car slows before the corner.')}"
    )

    enable_offroad_msg = tr("Enable \"Always Offroad\" in Device panel, or turn vehicle off to toggle.")
    if not ui_state.is_offroad():
      coop_steering_desc = f"<b>{enable_offroad_msg}</b><br><br>{coop_steering_desc}"
      long_fusion_desc = f"<b>{enable_offroad_msg}</b><br><br>{long_fusion_desc}"
      curve_assist_desc = f"<b>{enable_offroad_msg}</b><br><br>{curve_assist_desc}"

    self.coop_steering_toggle.set_description(coop_steering_desc)
    self.long_fusion_toggle.set_description(long_fusion_desc)
    self.curve_assist_toggle.set_description(curve_assist_desc)

    self.coop_steering_toggle.action_item.set_enabled(ui_state.is_offroad())
    self.long_fusion_toggle.action_item.set_enabled(ui_state.is_offroad())
    self.curve_assist_toggle.action_item.set_enabled(ui_state.is_offroad())
