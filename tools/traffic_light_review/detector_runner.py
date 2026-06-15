"""프레임 시퀀스에 검출모듈을 재생 → (states, outputs)."""
from sunnypilot.selfdrive.traffic_light.detector import TrafficLightDetector


def run_detector(frames):
    det = TrafficLightDetector()
    states, outs = [], []
    for fr in frames:
        out = det.update(fr.inputs)
        states.append(int(out.state))
        outs.append(out)
    return states, outs
