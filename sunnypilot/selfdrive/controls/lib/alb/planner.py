"""ALB: 모델 position + 오프셋 → lateral MPC → 곡률. 스펙 §3.1,§3.3.
carrot lateral_planner.py MPC 구동부 이식(차선선 재구성 없이 position에 offset만).
"""
import numpy as np
from openpilot.selfdrive.controls.lib.lateral_mpc_lib.lat_mpc import LateralMpc, N as LAT_MPC_N
from openpilot.selfdrive.controls.lib.drive_helpers import get_lag_adjusted_curvature

CONTROL_N = 17
MIN_SPEED = 1.0
# openpilot 표준 lateral MPC 가중치 (carrot 상수와 동일 계열)
PATH_COST = 1.0
LATERAL_MOTION_COST = 0.11
LATERAL_ACCEL_COST = 0.0
LATERAL_JERK_COST = 0.04
STEERING_RATE_COST = 700.0
DT_MDL = 0.05


class ALBPlanner:
    def __init__(self, CP):
        self.lat_mpc = LateralMpc()
        self.factor1 = CP.wheelbase - CP.centerToFront
        self.factor2 = (CP.centerToFront * CP.mass) / (CP.wheelbase * CP.tireStiffnessRear)
        self.reset()

    def reset(self):
        self.x0 = np.zeros(4)
        self.lat_mpc.reset(x0=self.x0)

    @staticmethod
    def _smooth_moving_avg(arr, window=5):
        # carrot lateral_planner.py smooth_moving_avg() 이식 (~line 277-285)
        if window < 2:
            return arr
        if window % 2 == 0:
            window += 1
        pad = window // 2
        arr_pad = np.pad(arr, (pad, pad), mode='edge')
        kernel = np.ones(window) / window
        return np.convolve(arr_pad, kernel, mode='same')[pad:-pad]

    @classmethod
    def _plan_yaw(cls, path_xyz, v_plan, smooth_window=5, clip_rate=2.0):
        # carrot lateral_planner.py yaw_from_path_no_scipy() 이식 (~line 287-347).
        # 호길이(arc-length) 파라미터화 + 이동평균 스무딩 후 미분 → 원본 위치의
        # np.gradient 2중 미분(스무딩 없음) 대비 model position 노이즈 증폭을 방지.
        v0 = float(np.asarray(v_plan)[0]) if len(v_plan) else 0.0
        if v0 <= 6.0:  # 저속에서는 창을 크게(9~11 권장)
            smooth_window = max(smooth_window, 9)

        N = path_xyz.shape[0]
        x = path_xyz[:, 0].astype(float)
        y = path_xyz[:, 1].astype(float)

        if N < 5:
            return np.zeros(N, np.float32), np.zeros(N, np.float32)

        # 1) s(호길이) 계산 — 음수/영(degenerate) 구간 가드
        dx = np.diff(x)
        dy = np.diff(y)
        ds_seg = np.sqrt(dx * dx + dy * dy)
        ds_seg[ds_seg < 0.05] = 0.05
        s = np.zeros(N, float)
        s[1:] = np.cumsum(ds_seg)
        if s[-1] < 0.5:  # 총 호길이 < 0.5m면 미분 결과 의미가 약함
            return np.zeros(N, np.float32), np.zeros(N, np.float32)

        # 2) smoothing (이동평균) — 미분 전에 적용해 모델 노이즈 증폭 방지
        x_smooth = cls._smooth_moving_avg(x, smooth_window)
        y_smooth = cls._smooth_moving_avg(y, smooth_window)

        # 3) 1·2차 도함수(s축 미분)
        dx_ds = np.gradient(x_smooth, s)
        dy_ds = np.gradient(y_smooth, s)
        d2x_ds2 = np.gradient(dx_ds, s)
        d2y_ds2 = np.gradient(dy_ds, s)

        # 4) yaw = atan2(dy/ds, dx/ds)
        yaw = np.unwrap(np.arctan2(dy_ds, dx_ds))

        # 5) 곡률 kappa
        denom = (dx_ds * dx_ds + dy_ds * dy_ds) ** 1.5
        denom[denom < 1e-9] = 1e-9
        kappa = (dx_ds * d2y_ds2 - dy_ds * d2x_ds2) / denom

        # 6) yaw_rate = kappa * v
        v = np.asarray(v_plan, float)
        yaw_rate = kappa * v
        if v0 <= 6.0:  # 이동평균으로 미세 요동 감쇄(창 5~7)
            yaw_rate = cls._smooth_moving_avg(yaw_rate, window=7)

        # 7) 안정화
        yaw = np.where(np.isfinite(yaw), yaw, 0.0)
        yaw_rate = np.where(np.isfinite(yaw_rate), yaw_rate, 0.0)
        yaw_rate = np.clip(yaw_rate, -abs(clip_rate), abs(clip_rate))

        return yaw.astype(np.float32), yaw_rate.astype(np.float32)

    def update(self, position_x, position_y, position_z, t_idxs, offset_m, v_ego, CP, VM):
        path_xyz = np.column_stack([position_x, np.asarray(position_y) + offset_m, position_z])
        t_idxs = np.asarray(t_idxs, dtype=float)
        v_plan = np.full(len(path_xyz), max(v_ego, MIN_SPEED))
        plan_yaw, plan_yaw_rate = self._plan_yaw(path_xyz, v_plan)

        self.lat_mpc.set_weights(PATH_COST, LATERAL_MOTION_COST,
                                 LATERAL_ACCEL_COST, LATERAL_JERK_COST, STEERING_RATE_COST)
        y_pts = path_xyz[:LAT_MPC_N + 1, 1]
        heading_pts = plan_yaw[:LAT_MPC_N + 1]
        yaw_rate_pts = plan_yaw_rate[:LAT_MPC_N + 1]
        lateral_factor = np.clip(self.factor1 - self.factor2 * v_plan**2, 0.0, np.inf)
        p = np.column_stack([v_plan, lateral_factor])
        self.lat_mpc.run(self.x0, p, y_pts, heading_pts, yaw_rate_pts)
        self.x0[3] = np.interp(DT_MDL, t_idxs[:LAT_MPC_N + 1], self.lat_mpc.x_sol[:, 3])

        if np.isnan(self.lat_mpc.x_sol[:, 3]).any() or self.lat_mpc.solution_status != 0:
            self.reset()
            return 0.0  # MPC 실패 → 0 (controlsd가 blend로 E2E 유지)

        psis = self.lat_mpc.x_sol[0:CONTROL_N, 2]
        # carrot lateral_planner.py publish() v_div (~line 224): 저속 불안정 방지 위해
        # 0.1m/s가 아닌 6.0m/s를 divisor 하한으로 사용.
        v_div = np.maximum(v_plan[:CONTROL_N], 6.0)
        curvs = self.lat_mpc.x_sol[0:CONTROL_N, 3] / v_div
        dists = self.lat_mpc.x_sol[0:CONTROL_N, 0]
        delay = CP.steerActuatorDelay + 0.1
        return get_lag_adjusted_curvature(CP, v_ego, psis.tolist(), curvs.tolist(), delay, dists.tolist())
