# tools/lane_change_lead_release/extractor.py
"""rlog → decider.Frame 스트림. 종방향 프레임(20Hz)마다 최신값 스냅샷."""
from openpilot.tools.lib.logreader import LogReader
from openpilot.common.constants import CV
from tools.lane_change_lead_release.decider import Frame, LaneLine, TrackPt

DT_MDL = 0.05


def _lane_lines(md):
    out = []
    for ll in md.laneLines:
        out.append(LaneLine(list(ll.x), list(ll.y)))
    return out


def _tracks(lt):
    # liveTracks 는 Car.RadarData 구조체. 트랙 리스트는 .points (List[RadarPoint]).
    out = []
    for t in lt.points:
        out.append(TrackPt(dRel=float(t.dRel), yRel=float(t.yRel)))
    return out


def frames(path):
    """(t, Frame) 제너레이터. modelV2 도착마다 한 프레임."""
    cs = lt = None
    rs = None
    t0 = None
    for m in LogReader(path, sort_by_time=True):
        w = m.which()
        if w == "carState":
            cs = m.carState
        elif w == "liveTracks":
            lt = m.liveTracks
        elif w == "radarState":
            rs = m.radarState
        elif w == "modelV2":
            if cs is None:
                continue
            if t0 is None:
                t0 = m.logMonoTime
            md = m.modelV2
            lead = rs.leadOne if rs is not None else None
            # vCruise(kph): 255=V_CRUISE_UNSET(크루즈 미설정) → v_cruise=0 으로 게이트2 해제 억제
            vc_kph = float(cs.vCruise)
            v_cruise = 0.0 if vc_kph >= 250.0 else vc_kph * CV.KPH_TO_MS
            f = Frame(
                lane_change_state=str(md.meta.laneChangeState).split(".")[-1],
                lane_change_direction=str(md.meta.laneChangeDirection).split(".")[-1],
                lane_lines=_lane_lines(md),
                tracks=_tracks(lt) if lt is not None else [],
                lead_status=bool(lead.status) if lead is not None else False,
                lead_v=float(lead.vLead) if lead is not None else 0.0,
                v_ego=float(cs.vEgo),
                v_cruise=v_cruise,
                blindspot_left=bool(cs.leftBlindspot),
                blindspot_right=bool(cs.rightBlindspot),
                dt=DT_MDL,
            )
            yield round((m.logMonoTime - t0) / 1e9, 2), f
