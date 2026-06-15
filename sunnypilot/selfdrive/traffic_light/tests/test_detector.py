from sunnypilot.selfdrive.traffic_light.detector import (
    TrafficLightState, DetectorInputs, TrafficLightDetector,
)


def _frame(model_x_end, *, v_ego, standstill, has_lead=False, gas=False, cc=False):
    px = [0.0] * 32 + [float(model_x_end)]
    return DetectorInputs(px, v_ego=v_ego, standstill=standstill,
                          has_lead=has_lead, gas_pressed=gas, cc_enabled=cc)


def _moving(model_x_end=100.0):
    return _frame(model_x_end, v_ego=10.0, standstill=False)


def _stopped(model_x_end, **kw):
    return _frame(model_x_end, v_ego=0.0, standstill=True, **kw)


def test_state_enum_values():
    assert TrafficLightState.OFF.value == 0
    assert TrafficLightState.RED.value == 1
    assert TrafficLightState.GREEN.value == 2


def test_moving_is_off():
    d = TrafficLightDetector()
    assert d.update(_moving()).state is TrafficLightState.OFF


def test_red_when_stopped_path_short():
    d = TrafficLightDetector()
    out = d.update(_stopped(10.0))           # 정차 + 짧은 경로 → 대기(RED)
    assert out.state is TrafficLightState.RED


def test_green_when_stopped_path_opens():
    d = TrafficLightDetector()
    d.update(_stopped(10.0))                  # 대기
    out = None
    for _ in range(10):                       # 경로 열림 0.3s 유지 → GREEN
        out = d.update(_stopped(120.0))
    assert out.state is TrafficLightState.GREEN


def test_would_alert_true_when_manual_no_lead():
    d = TrafficLightDetector()
    d.update(_moving())                       # 움직였다가
    out = None
    for _ in range(50):                       # 정차(recent_moving 해제)+경로열림+수동+앞차X
        out = d.update(_stopped(120.0))
    assert out.state is TrafficLightState.GREEN
    assert out.diagnostics["would_alert"] is True


def test_would_alert_false_when_engaged():
    d = TrafficLightDetector()
    d.update(_moving())
    out = None
    for _ in range(50):                       # openpilot 개입중 → 알림 게이트 막힘
        out = d.update(_stopped(120.0, cc=True))
    assert out.state is TrafficLightState.GREEN     # 표시상태는 GREEN
    assert out.diagnostics["would_alert"] is False  # 실제 알림은 안 울림
