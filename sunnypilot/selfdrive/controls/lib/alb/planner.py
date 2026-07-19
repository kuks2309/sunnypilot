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
    def _plan_yaw(path_xyz, v_plan):
        x = path_xyz[:, 0]; y = path_xyz[:, 1]
        dx = np.gradient(x); dy = np.gradient(y)
        yaw = np.arctan2(dy, np.maximum(dx, 1e-3))
        yaw_rate = np.gradient(yaw) * v_plan / np.maximum(dx, 1e-3)
        return yaw, yaw_rate

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
        curvs = self.lat_mpc.x_sol[0:CONTROL_N, 3] / max(v_ego, 0.1)
        dists = self.lat_mpc.x_sol[0:CONTROL_N, 0]
        delay = CP.steerActuatorDelay + 0.1
        return get_lag_adjusted_curvature(CP, v_ego, psis.tolist(), curvs.tolist(), delay, dists.tolist())
