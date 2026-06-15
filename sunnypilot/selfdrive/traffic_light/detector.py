"""신호등(정지/출발) 검출 — sunnypilot greenLightAlert 메커니즘 정합.

기준: sunnypilot/selfdrive/controls/lib/e2e_alerts_helper.py 의 Green Light Alert.
카메라로 램프색을 보는 게 아니라, 모델 예측경로 길이(model_x[-1])로 판단한다:
  - 정차 + 앞차없음 + 미개입 = "신호 대기"(armed)
      · 경로가 30m 넘게 열리면(0.3s 유지) → GREEN (갈 수 있음 = 초록)
      · 경로가 아직 짧으면              → RED   (대기 = 빨강)
  - 그 외(주행/앞차있음/개입)          → OFF
제어에 사용하지 않음 — 검출/표시 전용.
"""
from dataclasses import dataclass, field
from enum import IntEnum

DT_MDL = 0.05  # modelV2 주기(20Hz)

# --- 튜닝 상수 (e2e_alerts_helper 값과 정합) ---
GREEN_X_THRESHOLD = 30.0       # 경로 끝점이 이보다 멀면 "길 트임"
MOVING_VEGO = 0.1              # 이보다 빠르면 주행 중으로 간주
RECENT_MOVING_S = 2.0          # 직전 주행 후 이 시간까진 armed 안 함
GREEN_TRIGGER_S = 0.3          # GREEN 확정까지 유지시간


class TrafficLightState(IntEnum):
    OFF = 0
    RED = 1
    GREEN = 2


@dataclass
class DetectorInputs:
    model_pos_x: list      # modelV2.position.x (33,)
    v_ego: float           # m/s
    standstill: bool       # carState.standstill
    has_lead: bool         # radarState.leadOne.status
    gas_pressed: bool      # carState.gasPressed
    cc_enabled: bool       # carControl.enabled (openpilot 종방향 개입중)


@dataclass
class DetectorOutput:
    state: TrafficLightState
    x_stop: float                      # 예측 경로 끝점 거리 (m)
    diagnostics: dict = field(default_factory=dict)


class TrafficLightDetector:
    def __init__(self):
        self.state = TrafficLightState.OFF
        self.frames_since_moving = None    # None = 아직 움직인 적 없음
        self.green_timer = 0
        self.x_stop = 0.0

    def update(self, inp: DetectorInputs) -> DetectorOutput:
        moving = (not inp.standstill) and inp.v_ego > MOVING_VEGO
        if moving:
            self.frames_since_moving = 0
        elif self.frames_since_moving is not None:
            self.frames_since_moving += 1

        recent_moving = (self.frames_since_moving is None or
                         self.frames_since_moving * DT_MDL < RECENT_MOVING_S)

        model_x_end = inp.model_pos_x[-1]
        path_open = model_x_end > GREEN_X_THRESHOLD

        # 표시상태: 정차+모델경로 길이만으로 RED/GREEN (시각화용 핵심 AI 신호)
        if path_open and not moving:
            self.green_timer += 1
        else:
            self.green_timer = 0

        if moving:
            self.state = TrafficLightState.OFF
        elif self.green_timer * DT_MDL > GREEN_TRIGGER_S:
            self.state = TrafficLightState.GREEN
        else:
            self.state = TrafficLightState.RED

        # would_alert: 실제 greenLightAlert 가 울렸을 조건(개입X·앞차X·수동정차)
        would_alert = (self.state == TrafficLightState.GREEN
                       and not inp.gas_pressed and not inp.cc_enabled
                       and not recent_moving and not inp.has_lead)

        self.x_stop = model_x_end
        diag = {
            "model_x_end": round(model_x_end, 1),
            "v_ego": round(inp.v_ego, 2),
            "standstill": inp.standstill,
            "has_lead": inp.has_lead,
            "cc_enabled": inp.cc_enabled,
            "would_alert": would_alert,
            "green_timer": self.green_timer,
        }
        return DetectorOutput(self.state, self.x_stop, diag)
