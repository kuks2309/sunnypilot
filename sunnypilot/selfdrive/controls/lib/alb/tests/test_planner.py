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


def test_noisy_path_curvature_bounded_e2e_sanity(alb_cp, alb_vm):
    # 종단(end-to-end) sanity check일 뿐, 회귀 가드가 아님: MPC 자체의
    # 정규화(STEERING_RATE_COST=700, LATERAL_JERK_COST=0.04)가 스무딩 유무와
    # 무관하게 최종 곡률을 ~2-3e-5 수준으로 짓눌러, 이 assert는 스무딩을
    # 되돌려도 계속 통과한다(리뷰어 지적). 실제 회귀 가드는 아래
    # test_plan_yaw_smoothing_bounds_jitter (yaw_rate std 직접단언)가 담당.
    N = 33
    px = np.arange(N, dtype=float)
    # 3개 주파수 합성 sin 노이즈, 진폭 0.05/0.03/0.02m (offset 0.0)
    py = (0.05 * np.sin(px * 1.7) + 0.03 * np.sin(px * 0.6 + 1.1) + 0.02 * np.sin(px * 3.3 + 0.4))
    pz = np.zeros(N)
    t = np.linspace(0, 10, N)
    p = ALBPlanner(alb_cp)
    c = p.update(px, py, pz, t, 0.0, 25.0, alb_cp, alb_vm)
    assert abs(c) < 5e-3


def test_plan_yaw_smoothing_bounds_jitter():
    # 회귀 가드(reviewer, Important): 위 e2e 테스트는 MPC 정규화가 old/new
    # _plan_yaw 구현 모두를 최종 곡률 ~2-3e-5로 짓눌러 스무딩 되돌리기를
    # 잡아내지 못한다. 스무딩의 실제 효과는 _plan_yaw()가 반환하는
    # plan_yaw_rate 배열 자체에 있으므로, MPC를 거치지 않고 이 배열의
    # std(표준편차)를 직접 단언한다. ALBPlanner._plan_yaw는 classmethod라
    # ALBPlanner()를 만들지 않고도(=acados 불필요) 직접 호출 가능하다.
    N = 33
    px = np.arange(N, dtype=float)
    py = 0.05 * np.sin(px) + 0.03 * np.sin(2.3 * px)
    pz = np.zeros(N)
    v_plan = np.full(N, 25.0)

    plan_yaw, plan_yaw_rate = ALBPlanner._plan_yaw(np.column_stack([px, py, pz]), v_plan)

    # 이전(스무딩 없는 이중 np.gradient) 구현은 노이즈를 그대로 증폭해
    # std(yaw_rate) ≈ 0.96 rad/s(기기 실측). arc-length 스무딩 이식 후에는
    # ≈ 0.27 rad/s로 줄어든다. 0.5는 두 값 사이 중간 임계값 — 스무딩을
    # 되돌리면 이 테스트는 반드시 FAIL해야 한다.
    assert np.std(plan_yaw_rate) < 0.5
