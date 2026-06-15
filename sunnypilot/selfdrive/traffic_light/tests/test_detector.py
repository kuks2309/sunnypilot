import math

from sunnypilot.selfdrive.traffic_light.detector import (
    TrafficLightState, DetectorInputs, DetectorOutput, TrafficLightDetector,
)


def test_state_enum_values():
    assert TrafficLightState.OFF.value == 0
    assert TrafficLightState.RED.value == 1
    assert TrafficLightState.GREEN.value == 2


def test_detector_constructs():
    d = TrafficLightDetector()
    assert d.state is TrafficLightState.OFF
