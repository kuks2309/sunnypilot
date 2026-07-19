"""ALB(Adjacent Lane Bias) 오프셋 상태기계 (순수 로직).
설계: docs/superpowers/specs/2026-07-19-adjacent-lane-bias-v2-design.md
"""
from dataclasses import dataclass

SIGN = 1.0  # +offset = 왼쪽 (§7.1 B 실측 확정)


@dataclass
class OffsetInput:
    left_bsm: bool
    right_bsm: bool
    v_ego: float
    prob_left: float
    prob_right: float
    lane_change_active: bool
    lat_active: bool
    side_clearance_left: float    # 차 좌측면 ~ 왼쪽 차선선 거리(m)
    side_clearance_right: float   # 차 우측면 ~ 오른쪽 차선선 거리(m)
    dt: float


class _Latch:
    def __init__(self, hold_s): self.hold_s=hold_s; self.t=0.0
    def update(self, on, dt):
        self.t = self.hold_s if on else max(0.0, self.t-dt)
        return self.t > 0.0


class OffsetState:
    def __init__(self, offset_m=0.2, tau=2.0, latch_s=1.2, arm_frames=10,
                 min_lane_gap=0.20, speed_gate_ms=60/3.6):
        self.offset_m=offset_m; self.tau=tau; self.arm_frames=arm_frames
        self.min_lane_gap=min_lane_gap; self.speed_gate_ms=speed_gate_ms
        self._ll=_Latch(latch_s); self._rl=_Latch(latch_s)
        self._filt=0.0; self._arm=0

    def _target(self, inp: OffsetInput) -> float:
        if not inp.lat_active or inp.lane_change_active: self._arm=0; return 0.0
        if inp.v_ego < self.speed_gate_ms: self._arm=0; return 0.0
        if inp.prob_left < 0.5 or inp.prob_right < 0.5: self._arm=0; return 0.0
        left = self._ll.update(inp.left_bsm, inp.dt)
        right = self._rl.update(inp.right_bsm, inp.dt)
        if left == right:  # 양쪽/무감지
            self._arm=0; return 0.0
        # 지속감지 arming (리뷰 #7)
        self._arm = min(self.arm_frames, self._arm+1)
        if self._arm < self.arm_frames: return 0.0
        raw = self.offset_m if right else -self.offset_m
        # 차선선 최소거리 캡 (리뷰 #3): 이동 방향의 side_clearance로 상한
        if raw > 0:   # 왼쪽으로 이동
            cap = max(0.0, inp.side_clearance_left - self.min_lane_gap)
            raw = min(raw, cap)
        else:         # 오른쪽으로 이동
            cap = max(0.0, inp.side_clearance_right - self.min_lane_gap)
            raw = max(raw, -cap)
        return SIGN * raw

    def update(self, inp: OffsetInput) -> float:
        raw = self._target(inp)
        a = inp.dt / (self.tau + inp.dt)
        self._filt += a * (raw - self._filt)
        return self._filt
