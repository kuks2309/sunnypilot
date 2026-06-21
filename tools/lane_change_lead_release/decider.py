# tools/lane_change_lead_release/decider.py
"""차선 변경 중 lead 조기 해제 결정 로직 (순수, openpilot 비의존).

검증된 이 로직이 그대로 실차(radard/longitudinal)로 이식된다.
"""
from dataclasses import dataclass
import numpy as np

# ── 튜너블 파라미터 (검증 중 확정) ──────────────────
D_LOOKAHEAD = 75.0   # m, 목표 차선 트랙 탐색 거리
T_HOLD = 0.5         # s, 목표차선 클리어 지속 요구 시간
P_TH = 0.3           # 목표 차선 in-lane 확률 점유 임계값


@dataclass
class LaneLine:
    x: np.ndarray
    y: np.ndarray


@dataclass
class TrackPt:
    dRel: float
    yRel: float


def target_lane_in_lane_prob(line_inner: LaneLine, line_outer: LaneLine,
                             dRel: float, yRel: float) -> float:
    """Carrot d_path 공식. 목표 차선 band 기준 in-lane 확률(0~1)."""
    iy = float(np.interp(dRel, line_inner.x, line_inner.y))
    oy = float(np.interp(dRel, line_outer.x, line_outer.y))
    center = (iy + oy) / 2.0
    half_w = max(0.1, abs(oy - iy) / 2.0)
    dist = yRel + center               # Carrot 부호 컨벤션
    return max(0.0, 1.0 - abs(dist) / half_w)


def compute_target_occupancy(lane_lines, direction: str, tracks,
                             d_lookahead: float = D_LOOKAHEAD,
                             p_th: float = P_TH) -> bool:
    """목표 차선 band(look-ahead 내)에 트랙이 있으면 True."""
    if direction == "left":
        inner, outer = lane_lines[1], lane_lines[0]
    elif direction == "right":
        inner, outer = lane_lines[2], lane_lines[3]
    else:
        return False
    for t in tracks:
        if t.dRel <= 0.0 or t.dRel > d_lookahead:
            continue
        if target_lane_in_lane_prob(inner, outer, t.dRel, t.yRel) > p_th:
            return True
    return False


@dataclass
class Frame:
    lane_change_state: str        # off|preLaneChange|laneChangeStarting|laneChangeFinishing
    lane_change_direction: str    # none|left|right
    lane_lines: list              # list[LaneLine], 길이>=4
    tracks: list                  # list[TrackPt]
    lead_status: bool
    lead_v: float                 # m/s
    v_ego: float                  # m/s
    v_cruise: float               # m/s
    blindspot_left: bool
    blindspot_right: bool
    dt: float                     # s


@dataclass
class Decision:
    release: bool
    reason: str                   # released|not_starting|no_direction|blindspot|no_blocking_lead|target_occupied|holding
    target_occupied: bool         # True=목표차선 점유 확인. False=미점유 또는 '이전 게이트로 미평가'(='확정 클리어' 아님)
    clear_time: float


class LeadReleaseDecider:
    """프레임당 update(). 보수적 5-AND 게이트 + hold 타이머 + 즉시 복원."""

    def __init__(self, d_lookahead: float = D_LOOKAHEAD,
                 t_hold: float = T_HOLD, p_th: float = P_TH):
        self.d_lookahead = d_lookahead
        self.t_hold = t_hold
        self.p_th = p_th
        self.clear_time = 0.0

    def _reset(self):
        self.clear_time = 0.0

    def update(self, f: Frame) -> Decision:
        # 게이트 1: laneChangeStarting 에서만
        if f.lane_change_state != "laneChangeStarting":
            self._reset()
            return Decision(False, "not_starting", False, 0.0)

        # 게이트 1b (fail-closed): 차선 변경 방향 미상이면 목표 차선을 검사할 수 없음 → 해제 금지
        if f.lane_change_direction not in ("left", "right"):
            self._reset()
            return Decision(False, "no_direction", False, 0.0)

        # 게이트 5: 해당 측 사각지대 클리어 (안전)
        bs = f.blindspot_left if f.lane_change_direction == "left" else f.blindspot_right
        if bs:
            self._reset()
            return Decision(False, "blindspot", False, 0.0)

        # 게이트 2: 실제로 발목 잡는 lead 존재
        if not (f.lead_status and f.lead_v < f.v_cruise):
            self._reset()
            return Decision(False, "no_blocking_lead", False, 0.0)

        # 게이트 3: 목표 차선 점유 → 즉시 복원
        occupied = compute_target_occupancy(
            f.lane_lines, f.lane_change_direction, f.tracks,
            self.d_lookahead, self.p_th)
        if occupied:
            self._reset()
            return Decision(False, "target_occupied", True, 0.0)

        # 게이트 4: 클리어 지속(hold) 충족 시 발동
        self.clear_time += f.dt
        if self.clear_time < self.t_hold:
            return Decision(False, "holding", False, self.clear_time)
        return Decision(True, "released", False, self.clear_time)
