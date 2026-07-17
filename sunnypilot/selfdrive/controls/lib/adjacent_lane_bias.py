"""
Adjacent Lane Bias — 인접 차량 감지 시 반대쪽으로 차로 내 편향.

설계: docs/superpowers/specs/2026-07-13-adjacent-lane-bias-design.md
순수 로직(numpy 불필요, 표준 라이브러리만) — openpilot 없이 테스트 가능.
"""
from dataclasses import dataclass

SPEED_GATE_MS = 60 / 3.6  # 스펙 §2.2 속도 게이트
MAX_ABS_OFFSET = 0.5      # 스펙 §5 하드 상한
SIGN = 1.0                # §7.1 B 실측 확정(2026-07-18). +1 = measured>0 이 '중앙보다 왼쪽'


@dataclass
class Frame:
    left_blindspot: bool
    right_blindspot: bool
    v_ego: float
    lane_y_left: float    # modelV2.laneLines[1].y[0]
    lane_y_right: float   # modelV2.laneLines[2].y[0]
    prob_left: float
    prob_right: float
    lane_change_active: bool
    lat_active: bool
    dt: float


class _Latch:
    """신호가 빠져도 hold_s 동안 True 유지 (BSM 경계 깜빡임 완충)."""

    def __init__(self, hold_s: float):
        self.hold_s = hold_s
        self.timer = 0.0

    def update(self, on: bool, dt: float) -> bool:
        if on:
            self.timer = self.hold_s
        else:
            self.timer = max(0.0, self.timer - dt)
        return self.timer > 0.0


class AdjacentLaneBias:
    def __init__(self, offset_m: float = 0.3, kp: float = 0.02,
                 max_curv: float = 0.002, tau: float = 1.0, latch_s: float = 1.2):
        self.offset_m = offset_m
        self.kp = kp
        self.max_curv = max_curv
        self.tau = tau
        self._latch_l = _Latch(latch_s)
        self._latch_r = _Latch(latch_s)
        self._filtered = 0.0

    def _raw_target(self, f: Frame) -> float:
        """게이트 통과 시 ±offset, 아니면 0. 부호: + = 왼쪽으로 편향."""
        if not f.lat_active or f.lane_change_active:
            return 0.0
        if f.v_ego < SPEED_GATE_MS:
            return 0.0
        if f.prob_left < 0.5 or f.prob_right < 0.5:
            return 0.0

        left = self._latch_l.update(f.left_blindspot, f.dt)
        right = self._latch_r.update(f.right_blindspot, f.dt)

        if left == right:      # 양쪽 동시 또는 무감지 → 해제
            return 0.0
        return self.offset_m if right else -self.offset_m

    def target_offset(self, f: Frame) -> float:
        """1차 필터로 램프된 목표 오프셋(m)."""
        raw = self._raw_target(f)
        alpha = f.dt / (self.tau + f.dt)
        self._filtered += alpha * (raw - self._filtered)
        return self._filtered

    @staticmethod
    def measured_offset(f: Frame) -> float:
        """차로 중앙 대비 현재 횡방향 위치(m). SIGN=+1 이면 양수 = 중앙보다 왼쪽.

        measured = -(y1+y2)/2 는 y1·y2 에 대칭이라 laneLines[1]/[2] 의 좌/우
        라벨·y± 방향과 무관하게 값이 동일하다(§7.1 B 실측: centered≈+0.06m).
        """
        return SIGN * (-(f.lane_y_left + f.lane_y_right) / 2.0)

    def update(self, f: Frame) -> float:
        """Δκ(곡률 보정, 1/m). controlsd 가 desired_curvature 에 더한다."""
        target = self.target_offset(f)
        measured = self.measured_offset(f)

        # 편향 해제 시엔 Δκ=0 → 중앙 복원은 모델에 맡긴다 (스펙 §2.1).
        # 여기서 error=-measured 로 두면 우리가 중앙 복원을 떠맡아 모델과 길항한다.
        if target == 0.0:
            return 0.0

        # 하드 상한: 이미 크게 벗어났으면 추가 편향 금지 (스펙 §5)
        if abs(measured) > MAX_ABS_OFFSET:
            return 0.0

        error = target - measured
        delta = self.kp * error
        return max(-self.max_curv, min(self.max_curv, delta))
