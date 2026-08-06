import pytest
try:
    from openpilot.sunnypilot.selfdrive.controls.lib.alb.blend import BlendState
except ModuleNotFoundError:
    import os, sys
    sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))
    from blend import BlendState
DT=0.05
def settle(b, off, tq=0.0, secs=3.0):
    r=(0.0,False)
    for _ in range(int(secs/DT)): r=b.update(off, tq, DT)
    return r

def test_zero_offset_zero_blend():
    assert settle(BlendState(offset_m=0.2), 0.0)[0] == pytest.approx(0.0, abs=1e-3)

def test_full_offset_blend_near_one():
    b,_ = settle(BlendState(offset_m=0.2), 0.2)
    assert b > 0.9

def test_small_offset_saturates_blend():
    # activation_m=0.05(기본값) 초과 오프셋이면 blend는 1로 포화되어야 함
    b,_ = settle(BlendState(offset_m=0.2), 0.06)
    assert b > 0.9

def test_tiny_offset_partial():
    # activation_m 미만의 작은 오프셋(램프 구간)에서는 부분 blend 유지
    b,_ = settle(BlendState(offset_m=0.2, activation_m=0.05), 0.02)
    assert 0.2 < b < 0.9

def test_blend_is_filtered_not_step():
    b = BlendState(offset_m=0.2, tau=0.5)
    first,_ = b.update(0.2, 0.0, DT)
    assert first < 0.3   # 한 프레임에 튀지 않음

def test_torque_override_snaps_zero():
    b = BlendState(offset_m=0.2, torque_override_nm=1.0)
    settle(b, 0.2)                      # blend 올려놓고
    blend, ov = b.update(0.2, 1.5, DT)  # 운전자 토크 1.5Nm
    assert ov is True and blend == 0.0
