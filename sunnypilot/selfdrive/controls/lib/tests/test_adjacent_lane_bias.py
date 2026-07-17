import pytest

# 프로덕션 임포트 경로 우선. Windows PC 는 openpilot->. 심링크를 Python 임포터가
# 하위패키지로 못 따라가므로(기기 Linux 에선 정상), 직접 임포트로 폴백한다.
try:
    from openpilot.sunnypilot.selfdrive.controls.lib.adjacent_lane_bias import AdjacentLaneBias, Frame
except ModuleNotFoundError:
    import os
    import sys
    sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))
    from adjacent_lane_bias import AdjacentLaneBias, Frame

DT = 0.05


def mkframe(left=False, right=False, v_ego=30.0, y_l=1.8, y_r=-1.8,
            p_l=0.9, p_r=0.9, lc=False, lat=True):
    return Frame(left_blindspot=left, right_blindspot=right, v_ego=v_ego,
                 lane_y_left=y_l, lane_y_right=y_r, prob_left=p_l, prob_right=p_r,
                 lane_change_active=lc, lat_active=lat, dt=DT)


def settle(alb, frame, seconds=6.0):
    """필터가 정착할 때까지 반복 후 최종 target_offset 반환."""
    out = 0.0
    for _ in range(int(seconds / DT)):
        out = alb.target_offset(frame)
    return out


def test_no_detection_target_zero():
    alb = AdjacentLaneBias()
    assert settle(alb, mkframe()) == pytest.approx(0.0, abs=1e-3)


def test_right_only_biases_left_positive():
    # 오른쪽 감지 → 왼쪽으로 편향 → target = +d
    alb = AdjacentLaneBias(offset_m=0.3)
    assert settle(alb, mkframe(right=True)) == pytest.approx(0.3, abs=0.02)


def test_left_only_biases_right_negative():
    # 왼쪽 감지 → 오른쪽으로 편향 → target = -d
    alb = AdjacentLaneBias(offset_m=0.3)
    assert settle(alb, mkframe(left=True)) == pytest.approx(-0.3, abs=0.02)


def test_both_detected_releases_to_zero():
    # 양쪽 동시 → 편향 해제
    alb = AdjacentLaneBias(offset_m=0.3)
    assert settle(alb, mkframe(left=True, right=True)) == pytest.approx(0.0, abs=0.02)


def test_speed_gate_below_60kph_releases():
    alb = AdjacentLaneBias(offset_m=0.3)
    assert settle(alb, mkframe(right=True, v_ego=10.0)) == pytest.approx(0.0, abs=0.02)


def test_ramp_is_gradual_not_step():
    # 스텝 입력에도 첫 프레임에 목표값으로 튀지 않아야 함
    alb = AdjacentLaneBias(offset_m=0.3, tau=1.0)
    first = alb.target_offset(mkframe(right=True))
    assert abs(first) < 0.05


def test_debounce_latch_holds_through_flicker():
    # 신호가 순간 빠져도 래치(1.2s) 동안 유지
    alb = AdjacentLaneBias(offset_m=0.3, latch_s=1.2)
    settle(alb, mkframe(right=True))
    held = alb.target_offset(mkframe(right=False))  # 1프레임 드롭
    assert held == pytest.approx(0.3, abs=0.03)


def test_latch_expires_and_releases():
    alb = AdjacentLaneBias(offset_m=0.3, latch_s=1.2)
    settle(alb, mkframe(right=True))
    out = settle(alb, mkframe(right=False), seconds=8.0)  # 래치 만료 + 필터 복귀
    assert out == pytest.approx(0.0, abs=0.02)


def test_new_detection_never_biases_toward_it():
    # 안전 불변식: 오른쪽만(+d) → 왼쪽 추가 감지 → target 이 +d 보다 커지지 않음
    alb = AdjacentLaneBias(offset_m=0.3)
    peak = settle(alb, mkframe(right=True))
    both = mkframe(left=True, right=True)
    for _ in range(40):
        assert alb.target_offset(both) <= peak + 1e-6


MAX_ABS_OFFSET = 0.5


def test_centered_no_detection_zero_curvature():
    alb = AdjacentLaneBias()
    for _ in range(120):
        d = alb.update(mkframe())
    assert d == pytest.approx(0.0, abs=1e-6)


def test_right_detected_steers_toward_left():
    # 오른쪽 감지 + 차가 중앙 → 왼쪽으로 가야 함 → Δκ > 0 (좌회전 곡률)
    alb = AdjacentLaneBias(offset_m=0.3)
    d = 0.0
    for _ in range(40):
        d = alb.update(mkframe(right=True))
    assert d > 0.0


def test_left_detected_steers_toward_right():
    alb = AdjacentLaneBias(offset_m=0.3)
    d = 0.0
    for _ in range(40):
        d = alb.update(mkframe(left=True))
    assert d < 0.0


def test_curvature_is_clamped():
    # 거대한 오차에도 max_curv 초과 금지
    alb = AdjacentLaneBias(offset_m=0.3, kp=10.0, max_curv=0.002)
    d = 0.0
    for _ in range(40):
        d = alb.update(mkframe(right=True))
    assert abs(d) <= 0.002 + 1e-9


def test_error_shrinks_as_offset_achieved():
    # 이미 목표만큼 왼쪽에 있으면(measured=+0.3) Δκ ≈ 0
    alb = AdjacentLaneBias(offset_m=0.3)
    # measured = -(y_l + y_r)/2 = +0.3  →  y_l=1.5, y_r=-2.1
    f = mkframe(right=True, y_l=1.5, y_r=-2.1)
    d = 0.0
    for _ in range(200):
        d = alb.update(f)
    assert abs(d) < 1e-3


def test_hard_limit_blocks_further_bias():
    # |measured| > 0.5m 이면 추가 편향 금지
    alb = AdjacentLaneBias(offset_m=0.3)
    # measured = +0.6 → y_l=1.2, y_r=-2.4
    f = mkframe(right=True, y_l=1.2, y_r=-2.4)
    d = 0.0
    for _ in range(40):
        d = alb.update(f)
    assert d == pytest.approx(0.0, abs=1e-6)


def test_lane_change_gate_zero_curvature():
    alb = AdjacentLaneBias(offset_m=0.3)
    d = 0.0
    for _ in range(40):
        d = alb.update(mkframe(right=True, lc=True))
    assert d == pytest.approx(0.0, abs=1e-6)


def test_low_prob_lane_gate_zero_curvature():
    alb = AdjacentLaneBias(offset_m=0.3)
    d = 0.0
    for _ in range(40):
        d = alb.update(mkframe(right=True, p_l=0.2))
    assert d == pytest.approx(0.0, abs=1e-6)


def test_lat_inactive_zero_curvature():
    alb = AdjacentLaneBias(offset_m=0.3)
    d = 0.0
    for _ in range(40):
        d = alb.update(mkframe(right=True, lat=False))
    assert d == pytest.approx(0.0, abs=1e-6)
