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


def _inputs(stopping: bool, v_ego=15.0, steer=0.0, a_ego=0.0, d_rel=250.0):
    # stopping=True: 예측 path가 가까이서 멈춤(짧은 x, 낮은 v)
    if stopping:
        px = [min(40.0, i * 1.3) for i in range(33)]   # ~40m에서 포화
        vx = [max(0.0, v_ego - i * 1.0) for i in range(33)]  # 끝에서 0
    else:
        px = [i * 7.0 for i in range(33)]              # 계속 전진
        vx = [v_ego for _ in range(33)]
    py = [0.0 for _ in range(33)]
    return DetectorInputs(px, py, vx, v_ego, a_ego, steer, d_rel)


def test_red_on_stopping_trajectory():
    d = TrafficLightDetector()
    out = None
    for _ in range(5):  # 디바운스 통과
        out = d.update(_inputs(stopping=True))
    assert out.state is TrafficLightState.RED
    assert out.x_stop > 0.0


def test_off_on_cruising_trajectory():
    d = TrafficLightDetector()
    out = d.update(_inputs(stopping=False))
    assert out.state is TrafficLightState.OFF


def test_steer_guard_suppresses_red():
    d = TrafficLightDetector()
    out = None
    for _ in range(5):
        out = d.update(_inputs(stopping=True, steer=30.0))
    assert out.state is not TrafficLightState.RED
    assert "steer" in out.diagnostics["guards"]


def test_regen_guard_suppresses_red():
    d = TrafficLightDetector()
    out = None
    for _ in range(5):
        out = d.update(_inputs(stopping=True, a_ego=-1.5))
    assert "a_ego_regen" in out.diagnostics["guards"]
