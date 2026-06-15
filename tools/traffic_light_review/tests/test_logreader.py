from tools.traffic_light_review.logreader import Frame, frames_from_events


class FakeModel:
    def __init__(self):
        self.position = type("P", (), {"x": [1.0] * 33, "y": [0.0] * 33})()
        self.velocity = type("V", (), {"x": [10.0] * 33})()
        self.frameId = 7


class FakeCarState:
    vEgo = 10.0
    aEgo = 0.0
    steeringAngleDeg = 0.0


class FakeLead:
    dRel = 40.0
    status = True


class FakeRadar:
    leadOne = FakeLead()


class Evt:
    def __init__(self, which, payload, t):
        self._which = which
        self._payload = payload
        self.logMonoTime = t

    def which(self):
        return self._which

    def __getattr__(self, name):
        if name == self.__dict__.get("_which"):
            return self.__dict__["_payload"]
        raise AttributeError(name)


def test_frames_from_events_pairs_latest_state():
    events = [
        Evt("carState", FakeCarState(), 100),
        Evt("radarState", FakeRadar(), 110),
        Evt("modelV2", FakeModel(), 120),
        Evt("modelV2", FakeModel(), 170),
    ]
    frames = frames_from_events(events)
    assert len(frames) == 2                      # modelV2 마다 1프레임
    assert isinstance(frames[0], Frame)
    assert frames[0].inputs.v_ego == 10.0
    assert frames[0].inputs.d_rel == 40.0
    assert frames[0].t_mono == 120
