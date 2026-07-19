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
