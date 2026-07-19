"""ALB blend 계수 + 수동토크 오버라이드 (순수 로직). 스펙 §3.4, §4."""


class BlendState:
    def __init__(self, offset_m=0.2, tau=0.5, torque_override_nm=1.0, activation_m=0.05):
        self.offset_m=max(1e-3, offset_m); self.tau=tau
        self.torque_override_nm=torque_override_nm
        self.activation_m=max(1e-3, activation_m)
        self._blend=0.0

    def update(self, offset_filtered: float, driver_torque_nm: float, dt: float):
        # 수동 토크 오버라이드 → 즉시 0 스냅 (리뷰 #4)
        if abs(driver_torque_nm) > self.torque_override_nm:
            self._blend = 0.0
            return 0.0, True
        # 오프셋 활성도 = |offset|/activation_m, activation_m 이상이면 1로 포화
        # (offset_m에 비례시키면 풀오프셋에서만 blend=1이 되어 램프 구간에서
        #  E2E 재중앙화와 MPC가 다투는 평형점이 생김 — 5cm만 넘으면 즉시 MPC 전권)
        target = min(1.0, abs(offset_filtered) / self.activation_m)
        a = dt / (self.tau + dt)      # blend 계수 필터 (리뷰 #2)
        self._blend += a * (target - self._blend)
        return self._blend, False
