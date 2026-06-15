"""모델궤적 기반 신호등 검출 휴리스틱.

CarrotPilot `selfdrive/carrot/carrot_functions.py::check_model_stopping` 포팅.
제어에 사용하지 않음 — 검출/표시 전용. 임계상수는 본 파일 상단에 모아 튜닝한다.
"""
from dataclasses import dataclass, field
from enum import IntEnum

import numpy as np

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

    def _stop_sign(self, inp: DetectorInputs) -> bool:
        x = inp.model_pos_x
        y = inp.model_pos_y
        v = inp.model_vel_x
        model_x = x[-1]            # 예측 총 전진거리
        model_v = v[-1]            # 예측 종단 속도
        v0 = v[0]
        v_ego_kph = inp.v_ego * 3.6
        if v_ego_kph < STOP_VEGO_KPH_LOW:
            return model_x < STOP_MODELX_LOW and model_v < STOP_MODELV_LOW
        if v_ego_kph < STOP_VEGO_KPH_HIGH:
            x_thresh = float(np.interp(v0 * 3.6, STOP_MODELX_V_BP, STOP_MODELX_V_V))
            return (model_x < inp.d_rel - STOP_DREL_MARGIN
                    and model_x < x_thresh
                    and ((model_v < STOP_MODELV_MID) or (model_v < v0 * STOP_MODELV_RATIO))
                    and abs(y[-1]) < STOP_LATERAL_MAX)
        return False  # ~82km/h 이상 비활성

    def update(self, inp: DetectorInputs) -> DetectorOutput:
        guards = []
        if abs(inp.steering_angle_deg) > GUARD_STEER_DEG:
            guards.append("steer")
        if inp.a_ego < GUARD_AEGO:
            guards.append("a_ego_regen")

        raw_stop = self._stop_sign(inp) and not guards
        if raw_stop:
            self.stop_sign_count += 1
            self.start_sign_count = 0
        else:
            self.start_sign_count += 1
            self.stop_sign_count = 0

        if self.stop_sign_count * DT_MDL > RED_DEBOUNCE_S and self.stop_sign_count > 0:
            self.state = TrafficLightState.RED
        elif self.start_sign_count * DT_MDL > GREEN_DEBOUNCE_S:
            self.state = TrafficLightState.GREEN if self.state is TrafficLightState.RED else TrafficLightState.OFF

        self.x_stop = inp.model_pos_x[-1]
        diag = {
            "model_x": round(inp.model_pos_x[-1], 1),
            "model_v": round(inp.model_vel_x[-1], 2),
            "y_end": round(inp.model_pos_y[-1], 2),
            "v_ego": round(inp.v_ego, 2),
            "a_ego": round(inp.a_ego, 2),
            "steer": round(inp.steering_angle_deg, 1),
            "stop_cnt": self.stop_sign_count,
            "start_cnt": self.start_sign_count,
            "guards": guards,
        }
        return DetectorOutput(self.state, self.x_stop, diag)
