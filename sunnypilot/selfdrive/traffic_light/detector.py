"""모델궤적 기반 신호등 검출 휴리스틱.

CarrotPilot `selfdrive/carrot/carrot_functions.py::check_model_stopping` 포팅.
제어에 사용하지 않음 — 검출/표시 전용. 임계상수는 본 파일 상단에 모아 튜닝한다.
"""
from dataclasses import dataclass, field
from enum import IntEnum

DT_MDL = 0.05  # modelV2 주기(20Hz)

# --- 튜닝 상수 (CarrotPilot 포팅값, 검증 후 조정) ---
STOP_VEGO_KPH_LOW = 1.0
STOP_VEGO_KPH_HIGH = 82.0
STOP_MODELX_LOW = 20.0
STOP_MODELV_LOW = 10.0
STOP_MODELV_MID = 3.0
STOP_MODELV_RATIO = 0.7
STOP_LATERAL_MAX = 5.0
STOP_DREL_MARGIN = 3.0
STOP_MODELX_V_BP = [60.0, 80.0]      # km/h
STOP_MODELX_V_V = [120.0, 150.0]     # m
GUARD_STEER_DEG = 20.0
GUARD_AEGO = -1.0
RED_DEBOUNCE_S = 0.0
GREEN_DEBOUNCE_S = 0.2


class TrafficLightState(IntEnum):
    OFF = 0
    RED = 1
    GREEN = 2


@dataclass
class DetectorInputs:
    model_pos_x: list      # modelV2.position.x (33,)
    model_pos_y: list      # modelV2.position.y (33,)
    model_vel_x: list      # modelV2.velocity.x (33,)
    v_ego: float           # m/s
    a_ego: float           # m/s^2
    steering_angle_deg: float
    d_rel: float           # 선행차 거리 (없으면 250.0)


@dataclass
class DetectorOutput:
    state: TrafficLightState
    x_stop: float                      # 정지 예측 거리 (m)
    diagnostics: dict = field(default_factory=dict)


class TrafficLightDetector:
    def __init__(self):
        self.state = TrafficLightState.OFF
        self.stop_sign_count = 0
        self.start_sign_count = 0
        self.x_stop = 0.0

    def update(self, inp: DetectorInputs) -> DetectorOutput:
        raise NotImplementedError
