# tools/lane_change_lead_release/tests/test_decider.py
import numpy as np
from tools.lane_change_lead_release.decider import (
    LaneLine, TrackPt, compute_target_occupancy,
)

def _lines():
    # x 0..100m. 좌측 변경 기준 차선 라인 4개(far-left[0], left[1], right[2], far-right[3]).
    # 단순화: y 는 거리무관 상수. 좌측 인접 차선 = [0]~[1] 사이 (y ≈ -5.25 ~ -1.75 중심 -3.5)
    x = np.linspace(0, 100, 11)
    return [
        LaneLine(x, np.full_like(x, -5.25)),  # far-left  [0]
        LaneLine(x, np.full_like(x, -1.75)),  # left      [1]
        LaneLine(x, np.full_like(x,  1.75)),  # right     [2]
        LaneLine(x, np.full_like(x,  5.25)),  # far-right [3]
    ]

def test_occupied_when_track_in_left_target_lane():
    # 좌측 목표 차선 중심(-3.5) 근처 트랙 → 점유. 부호: dist = yRel + center.
    # center ≈ -3.5 이므로 yRel ≈ +3.5 이면 dist≈0 (band 중심).
    tracks = [TrackPt(dRel=30.0, yRel=3.5)]
    assert compute_target_occupancy(_lines(), "left", tracks) is True

def test_empty_when_track_in_ego_lane():
    # ego 차선(중심 0) 트랙 → 좌측 목표 차선엔 미점유.
    tracks = [TrackPt(dRel=30.0, yRel=0.0)]
    assert compute_target_occupancy(_lines(), "left", tracks) is False

def test_empty_when_track_beyond_lookahead():
    tracks = [TrackPt(dRel=200.0, yRel=3.5)]
    assert compute_target_occupancy(_lines(), "left", tracks) is False

def test_none_direction_never_occupied():
    tracks = [TrackPt(dRel=30.0, yRel=3.5)]
    assert compute_target_occupancy(_lines(), "none", tracks) is False

from tools.lane_change_lead_release.decider import Frame, LeadReleaseDecider

def _frame(**kw):
    base = dict(
        lane_change_state="laneChangeStarting",
        lane_change_direction="left",
        lane_lines=_lines(),
        tracks=[],                 # 목표 차선 빔
        lead_status=True,
        lead_v=16.0,               # m/s (~58km/h)
        v_ego=16.0,
        v_cruise=27.0,             # ~97km/h
        blindspot_left=False,
        blindspot_right=False,
        dt=0.05,
    )
    base.update(kw)
    return Frame(**base)

def test_release_after_hold_when_target_empty():
    d = LeadReleaseDecider(t_hold=0.5)
    outs = [d.update(_frame()) for _ in range(12)]
    rel = [o.release for o in outs]
    assert rel[:10] == [False]*10          # holding
    assert rel[10] is True and rel[11] is True
    assert outs[0].reason == "holding"
    assert outs[10].reason == "released"

def test_no_release_when_not_starting():
    d = LeadReleaseDecider(t_hold=0.5)
    for _ in range(20):
        out = d.update(_frame(lane_change_state="preLaneChange"))
        assert out.release is False and out.reason == "not_starting"

def test_no_release_when_target_occupied():
    d = LeadReleaseDecider(t_hold=0.5)
    occ = [TrackPt(dRel=30.0, yRel=3.5)]   # 목표 차선에 차
    for _ in range(20):
        out = d.update(_frame(tracks=occ))
        assert out.release is False and out.reason == "target_occupied"

def test_no_release_when_blindspot():
    d = LeadReleaseDecider(t_hold=0.5)
    for _ in range(20):
        out = d.update(_frame(blindspot_left=True))
        assert out.release is False and out.reason == "blindspot"

def test_no_release_when_no_blocking_lead():
    d = LeadReleaseDecider(t_hold=0.5)
    for _ in range(20):  # lead 가 이미 크루즈 이상 → 발목 안 잡음
        out = d.update(_frame(lead_status=True, lead_v=30.0))
        assert out.release is False and out.reason == "no_blocking_lead"

def test_instant_restore_resets_hold():
    d = LeadReleaseDecider(t_hold=0.5)
    for _ in range(10):
        d.update(_frame())                 # 클리어 누적
    d.update(_frame(tracks=[TrackPt(30.0, 3.5)]))  # 트랙 진입 → 즉시 복원
    # 직후 다시 비어도 hold 부터 재시작
    assert d.update(_frame()).release is False


def test_release_right_direction_when_target_empty():
    d = LeadReleaseDecider(t_hold=0.5)
    rel = [d.update(_frame(lane_change_direction="right")).release for _ in range(12)]
    assert rel[:10] == [False]*10
    assert rel[10] is True


def test_no_release_when_direction_none_even_if_lane_clear():
    d = LeadReleaseDecider(t_hold=0.5)
    for _ in range(20):
        out = d.update(_frame(lane_change_direction="none"))
        assert out.release is False and out.reason == "no_direction"


def test_no_release_when_direction_none_with_car_present():
    # C1 회귀: 방향 미상 + 옆차선에 차 → 절대 해제 금지 (fail-closed)
    d = LeadReleaseDecider(t_hold=0.5)
    car = [TrackPt(dRel=30.0, yRel=3.5)]
    for _ in range(20):
        assert d.update(_frame(lane_change_direction="none", tracks=car)).release is False


def test_blindspot_after_hold_resets_and_blocks():
    d = LeadReleaseDecider(t_hold=0.5)
    for _ in range(10):
        d.update(_frame())                         # 클리어 누적
    out = d.update(_frame(blindspot_left=True))    # 사각지대 트립
    assert out.release is False
    assert d.clear_time == 0.0                     # hold 리셋 확인
    assert d.update(_frame()).release is False     # 직후 다시 비어도 hold 재시작
