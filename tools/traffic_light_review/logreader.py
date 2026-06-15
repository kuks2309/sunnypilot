"""rlog → 프레임별 DetectorInputs 추출 + qcamera 경로.

modelV2 매 메시지를 1프레임으로 보고, 직전 carState/radarState를 결합한다.
cereal/capnp 의존은 지연 import — 순수함수 frames_from_events 는 의존 없이 테스트 가능.

Windows 주의: cereal 의 .capnp 는 git symlink 라 PC 체크아웃 시 텍스트파일이 됨.
_resolve_winlink 로 풀어 temp 에 모은 뒤 capnp.load 한다 (car_identify_standalone 패턴).
`from cereal import log` 은 PC 에서 크래시하므로 쓰지 않는다.
"""
import io
import os
import glob
import shutil
import tempfile
from dataclasses import dataclass

from sunnypilot.selfdrive.traffic_light.detector import DetectorInputs

_REPO_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
_CEREAL = os.path.join(_REPO_ROOT, "cereal")
_EVENT_SCHEMA = None


@dataclass
class Frame:
    t_mono: int
    inputs: DetectorInputs
    seg_road_frame_id: int  # 동기화용 modelV2.frameId


def _resolve_winlink(src):
    """Windows git symlink(작은 텍스트파일)을 실제 .capnp 경로로 해소."""
    try:
        if 0 < os.path.getsize(src) < 300:
            txt = open(src, "rb").read().decode("utf-8", "ignore").strip()
            if "\n" not in txt and txt.endswith(".capnp"):
                cand = os.path.normpath(os.path.join(os.path.dirname(src), txt))
                if os.path.isfile(cand):
                    return _resolve_winlink(cand)
    except OSError:
        pass
    return src


def _load_event_schema():
    """cereal log.capnp 스키마를 standalone 으로 로드해 Event 반환(캐시)."""
    global _EVENT_SCHEMA
    if _EVENT_SCHEMA is not None:
        return _EVENT_SCHEMA
    import capnp
    capnp.remove_import_hook()
    tmp = tempfile.mkdtemp(prefix="cer_tl_")
    os.makedirs(os.path.join(tmp, "include"), exist_ok=True)
    for name in ("log.capnp", "car.capnp", "custom.capnp", "deprecated.capnp"):
        shutil.copyfile(_resolve_winlink(os.path.join(_CEREAL, name)),
                        os.path.join(tmp, name))
    shutil.copyfile(_resolve_winlink(os.path.join(_CEREAL, "include", "c++.capnp")),
                    os.path.join(tmp, "include", "c++.capnp"))
    log = capnp.load(os.path.join(tmp, "log.capnp"),
                     imports=[tmp, os.path.join(tmp, "include")])
    _EVENT_SCHEMA = log.Event
    return _EVENT_SCHEMA


def _safe_iter(reader):
    """capnp 스트림 끝 truncation(불완전 세그먼트)에서 예외 대신 정상 종료."""
    it = iter(reader)
    while True:
        try:
            yield next(it)
        except StopIteration:
            return
        except Exception:  # capnp KjException: Message ends prematurely
            return


def _read_rlog(path):
    import zstandard
    Event = _load_event_schema()
    raw = open(path, "rb").read()
    if path.endswith(".zst"):
        raw = zstandard.ZstdDecompressor().stream_reader(io.BytesIO(raw)).read()
    return _safe_iter(Event.read_multiple_bytes(raw))


def frames_from_events(events):
    """이벤트 시퀀스 → Frame 리스트. modelV2 마다 직전 carState/radarState/carControl 결합."""
    cs = None
    radar = None
    cc = None
    frames = []
    for e in events:
        w = e.which()
        if w == "carState":
            cs = e.carState
        elif w == "radarState":
            radar = e.radarState
        elif w == "carControl":
            cc = e.carControl
        elif w == "modelV2":
            m = e.modelV2
            has_lead = bool(radar is not None and getattr(radar.leadOne, "status", False))
            inputs = DetectorInputs(
                model_pos_x=list(m.position.x),
                v_ego=cs.vEgo if cs else 0.0,
                standstill=bool(cs.standstill) if cs else True,
                has_lead=has_lead,
                gas_pressed=bool(cs.gasPressed) if cs else False,
                cc_enabled=bool(cc.enabled) if cc else False,
            )
            frames.append(Frame(e.logMonoTime, inputs, getattr(m, "frameId", 0)))
    return frames


def load_segment(seg_dir):
    """세그먼트 디렉터리에서 (frames, qcamera_path) 반환."""
    rlogs = glob.glob(os.path.join(seg_dir, "rlog.zst")) or glob.glob(os.path.join(seg_dir, "rlog"))
    if not rlogs:
        raise FileNotFoundError(f"rlog 없음: {seg_dir}")
    frames = frames_from_events(_read_rlog(rlogs[0]))
    qcam = os.path.join(seg_dir, "qcamera.ts")
    return frames, (qcam if os.path.exists(qcam) else None)
