import pytest
try:
    from openpilot.sunnypilot.selfdrive.controls.lib.alb.offset_state import OffsetState, OffsetInput
except ModuleNotFoundError:
    import os, sys
    sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))
    from offset_state import OffsetState, OffsetInput

DT = 0.05
def mk(left=False, right=False, v=30.0, pl=0.9, pr=0.9, lc=False, lat=True, cl=1.8, cr=1.8):
    return OffsetInput(left, right, v, pl, pr, lc, lat, cl, cr, DT)
def settle(s, inp, secs=8.0):
    o=0.0
    for _ in range(int(secs/DT)): o=s.update(inp)
    return o

def test_no_detect_zero():
    assert settle(OffsetState(), mk()) == pytest.approx(0.0, abs=1e-3)

def test_right_only_positive_left_bias():
    assert settle(OffsetState(offset_m=0.2), mk(right=True)) == pytest.approx(0.2, abs=0.02)

def test_left_only_negative():
    assert settle(OffsetState(offset_m=0.2), mk(left=True)) == pytest.approx(-0.2, abs=0.02)

def test_both_release_zero():
    assert settle(OffsetState(offset_m=0.2), mk(left=True, right=True)) == pytest.approx(0.0, abs=0.02)

def test_speed_gate():
    assert settle(OffsetState(offset_m=0.2), mk(right=True, v=10.0)) == pytest.approx(0.0, abs=0.02)

def test_arming_requires_sustained():
    # arm_frames=10: 9프레임만 감지되면 오프셋 안 뜸
    s = OffsetState(offset_m=0.2, arm_frames=10)
    o=0.0
    for _ in range(9): o=s.update(mk(right=True))
    assert abs(o) < 0.02

def test_lane_gap_cap():
    # 오른쪽 여유 0.30m 뿐 → +offset(왼쪽)엔 무관, 하지만 좁은 왼쪽 여유로 캡 확인
    # 왼쪽으로 0.2 가려는데 왼쪽 차선 여유 cl=0.30 → 0.30-0.20(min_gap)=0.10 로 캡
    s = OffsetState(offset_m=0.2, min_lane_gap=0.20)
    o = settle(s, mk(right=True, cl=0.30))
    assert o == pytest.approx(0.10, abs=0.03)

def test_ramp_gradual():
    s = OffsetState(offset_m=0.2, tau=2.0)
    first = s.update(mk(right=True))
    assert abs(first) < 0.03

def test_latch_holds_flicker():
    s = OffsetState(offset_m=0.2, latch_s=1.2)
    settle(s, mk(right=True))
    assert s.update(mk(right=False)) == pytest.approx(0.2, abs=0.03)

def test_lane_change_gate():
    assert settle(OffsetState(offset_m=0.2), mk(right=True, lc=True)) == pytest.approx(0.0, abs=0.02)

def test_latch_decays_during_gate():
    # right detected (latch on), then a long gated interval (lane change) with BSM off
    # must NOT resurrect stale bias once the gate lifts
    s = OffsetState(offset_m=0.2, latch_s=1.2)
    settle(s, mk(right=True))                      # latch primed
    for _ in range(int(2.0/DT)):                   # 2s gated, BSM now off
        s.update(mk(right=False, lc=True))
    out = settle(s, mk(right=False), secs=3.0)     # gate lifts, BSM still off; filter settles
    assert out == pytest.approx(0.0, abs=0.02)
