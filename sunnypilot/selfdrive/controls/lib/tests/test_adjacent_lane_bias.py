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
