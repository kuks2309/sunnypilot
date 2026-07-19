import numpy as np
import pytest
import cereal.messaging as messaging
from cereal import car
from opendbc.car.vehicle_model import VehicleModel
from openpilot.common.params import Params
from openpilot.sunnypilot.selfdrive.controls.lib.alb.planner import ALBPlanner


@pytest.fixture
def alb_cp():
    return messaging.log_from_bytes(Params().get("CarParamsPersistent"), car.CarParams)


@pytest.fixture
def alb_vm(alb_cp):
    return VehicleModel(alb_cp)


def test_zero_offset_straight_near_zero_curv(alb_cp, alb_vm):
    N = 33
    px = np.arange(N, dtype=float); py = np.zeros(N); pz = np.zeros(N); t = np.linspace(0, 10, N)
    p = ALBPlanner(alb_cp)
    c = p.update(px, py, pz, t, 0.0, 25.0, alb_cp, alb_vm)
    assert abs(c) < 1e-2


def test_offset_produces_transient_curvature(alb_cp, alb_vm):
    N = 33
    px = np.arange(N, dtype=float); py = np.zeros(N); pz = np.zeros(N); t = np.linspace(0, 10, N)
    p = ALBPlanner(alb_cp)
    c = p.update(px, py, pz, t, 0.3, 25.0, alb_cp, alb_vm)  # 왼쪽 0.3
    assert c > 0   # 좌회전 곡률


def test_noisy_path_curvature_bounded(alb_cp, alb_vm):
    # 리뷰어 지적: 순수 직선 path 테스트는 노이즈 안정성을 증명하지 못함.
    # 직선 + 소진폭(~0.05m) 다중주파수 pseudo-noise(np.sin 합성, 결정적/재현가능 —
    # np.random 미사용)로 model position 노이즈를 흉내내고, _plan_yaw의 arc-length
    # 스무딩(carrot yaw_from_path_no_scipy 이식)이 곡률 폭주를 막는지 확인한다.
    N = 33
    px = np.arange(N, dtype=float)
    # 3개 주파수 합성 sin 노이즈, 진폭 0.05/0.03/0.02m (offset 0.0)
    py = (0.05 * np.sin(px * 1.7) + 0.03 * np.sin(px * 0.6 + 1.1) + 0.02 * np.sin(px * 3.3 + 0.4))
    pz = np.zeros(N)
    t = np.linspace(0, 10, N)
    p = ALBPlanner(alb_cp)
    c = p.update(px, py, pz, t, 0.0, 25.0, alb_cp, alb_vm)
    # 스무딩 없는 이전 구현은 노이즈를 그대로 2중미분 → 곡률이 크게 흔들림.
    # arc-length 스무딩 이식 후에는 노이즈 진폭(5cm급)에 대해 MPC가 산출하는
    # 곡률이 작게 유지되어야 함(잦은 조향 떨림 방지). 임계값 5e-3(1/m)은
    # 25m/s에서 약 0.125 rad/s 요레이트에 해당 — 5cm 노이즈로 이 정도의
    # 조향 반응이 나오면 과도한 떨림으로 간주.
    assert abs(c) < 5e-3
