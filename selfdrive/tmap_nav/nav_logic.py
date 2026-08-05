"""tmap_nav 결정 로직 — cereal 비의존, PC에서 SIL 검증 가능.

폰(티맵) → Frida 브리지가 UDP 로 쏘는 JSON 을 받아 오파가 쓸 수 있는 형태로 거른다.
브리지 스키마와 게이팅 근거는 docs 회신 묶음 참조(remote_instruction/SDI_FREEZE_ANALYSIS.md).

핵심은 거르는 일이다. 브리지가 보내는 값은 세 가지 이유로 그대로 믿으면 안 된다.

1. 안내가 꺼져 있으면(`guiding=false`) 값이 없거나 낡았다.
2. SDI 는 엔진이 준 값(`sdiFrom == RGData.sdiInfo`)일 때만 유효하다.
   과거 브리지가 힙에서 지나간 이벤트를 주워오던 버그가 있었고(2026-08-05 수정),
   그 재발을 수신부에서도 막는다.
3. 채널마다 갱신 주기가 달라 패킷 나이(`dataAgeMs`) 하나로는 판단할 수 없다.
   SDI 채널 나이(`ageMs.sdi`)를 따로 본다.

거리(`nSdiDist`)는 콜백 주기(8~10초) 사이에 늙는다. 좌표는 안 늙으므로,
자차 위치를 알면 haversine 으로 다시 재는 쪽이 정확하다.
"""
import json
import math

LINK_TIMEOUT = 3.0        # 이 시간 동안 패킷이 없으면 링크 끊김으로 본다(브리지 하트비트 2Hz)
SDI_MAX_AGE_MS = 25000    # SDI 채널 최대 허용 나이. 콜백 주기 8~10초의 약 3배
ROAD_MAX_AGE_MS = 30000   # 도로/제한속도 채널 최대 허용 나이
SDI_SOURCE_OK = "RGData.sdiInfo"   # 엔진이 준 값. "none"/"heap" 은 쓰지 않는다

EARTH_R = 6371000.0


def haversine(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
  """두 좌표 사이 거리(m)."""
  dlat = math.radians(lat2 - lat1)
  dlon = math.radians(lon2 - lon1)
  a = (math.sin(dlat / 2) ** 2 +
       math.cos(math.radians(lat1)) * math.cos(math.radians(lat2)) * math.sin(dlon / 2) ** 2)
  return 2 * EARTH_R * math.asin(math.sqrt(a))


class TmapNavLogic:
  """수신 패킷을 누적해 매 주기 payload 를 만든다. 소켓·cereal 을 모르는 순수 로직."""

  def __init__(self):
    self.last_rx_mono = -1e9   # 마지막 패킷 수신 시각(monotonic)
    self.apilot: dict = {}     # 마지막으로 받은 apilot 딕셔너리
    self.rx_count = 0
    self.parse_errors = 0

  def on_packet(self, data: bytes, now_mono: float) -> bool:
    """데이터그램 하나를 흡수한다. 파싱 성공 시 True."""
    try:
      obj = json.loads(data.decode("utf-8"))
      apilot = obj["apilot"]
      if not isinstance(apilot, dict):
        raise TypeError("apilot is not a dict")
    except (ValueError, KeyError, TypeError, UnicodeDecodeError):
      self.parse_errors += 1
      return False

    self.apilot = apilot
    self.last_rx_mono = now_mono
    self.rx_count += 1
    return True

  def update(self, now_mono: float, lat: float, lon: float, have_fix: bool) -> dict:
    """현재 상태를 payload 로 만든다. 자차 위치를 주면 SDI 거리를 좌표로 다시 잰다."""
    link = (now_mono - self.last_rx_mono) < LINK_TIMEOUT
    a = self.apilot if link else {}

    guiding = bool(a.get("guiding"))
    ages = a.get("ageMs") or {}

    # --- 도로 제한속도 ---
    road_limit = 0
    road_name = ""
    # 나이 기본값은 -1 이다. 0 으로 두면 "값 없음"이 "가장 신선함"으로 통과해 버린다.
    road_age = _as_int(ages.get("road"), -1)
    limit_raw = _as_int(a.get("nRoadLimitSpeed"))
    if guiding and limit_raw > 0 and _age_ok(road_age, ROAD_MAX_AGE_MS):
      road_limit = limit_raw
      # roadNameGeneric=True 면 "일반도로" 같은 플레이스홀더다. 값은 두되 이름은 비운다.
      if not a.get("roadNameGeneric"):
        road_name = a.get("szPosRoadName") or ""

    # --- SDI(단속·경고 지점) ---
    sdi_src = a.get("sdiFrom")
    sdi_age = _as_int(ages.get("sdi"), -1)
    sdi_type = -1
    sdi_dist = 0.0
    sdi_limit = 0
    sdi_lat = sdi_lon = 0.0
    sdi_recalc = False

    sdi_usable = (guiding and sdi_src == SDI_SOURCE_OK and _age_ok(sdi_age, SDI_MAX_AGE_MS))
    if sdi_usable:
      t = _as_int(a.get("nSdiType"), -1)
      if t >= 0:
        sdi_type = t
        sdi_limit = _as_int(a.get("nSdiSpeedLimit"))
        sdi_dist = float(_as_int(a.get("nSdiDist")))
        sdi_lat = _as_float(a.get("vpSdiPointLat"))
        sdi_lon = _as_float(a.get("vpSdiPointLon"))
        # 좌표가 있으면 자차 위치로 다시 잰다 — 거리는 늙지만 좌표는 안 늙는다.
        if have_fix and sdi_lat != 0.0 and sdi_lon != 0.0:
          sdi_dist = haversine(lat, lon, sdi_lat, sdi_lon)
          sdi_recalc = True

    return {
      "lk": int(link),                    # 링크 살아있음
      "gd": int(guiding),                 # 안내 중
      "sl": road_limit,                   # 도로 제한속도(km/h), 0=없음
      "rn": road_name,                    # 도로명, ""=없음/플레이스홀더
      "st": sdi_type,                     # SDI 유형 코드, -1=없음
      "sd": round(sdi_dist, 1),           # SDI 지점까지 거리(m)
      "sx": sdi_limit,                    # SDI 제한속도(km/h), 0=없음
      "sy": round(sdi_lat, 7),            # SDI 지점 위도
      "sz": round(sdi_lon, 7),            # SDI 지점 경도
      "rc": int(sdi_recalc),              # 거리를 좌표로 다시 쟀는지
      "sa": sdi_age,                      # SDI 채널 나이(ms), -1=없음
      "ra": road_age,                     # 도로 채널 나이(ms), -1=없음
      "src": sdi_src or "",               # SDI 출처 원문(진단용)
      "n": self.rx_count,                 # 누적 수신 패킷 수
    }


def _as_int(v, default: int = 0) -> int:
  """None/문자열/실수가 섞여 오므로 방어적으로 정수화한다."""
  if v is None:
    return default
  try:
    return int(v)
  except (ValueError, TypeError):
    return default


def _as_float(v, default: float = 0.0) -> float:
  if v is None:
    return default
  try:
    return float(v)
  except (ValueError, TypeError):
    return default


def _age_ok(age_ms: int, limit_ms: int) -> bool:
  """나이가 없으면(-1/0 미만) 판단 불가로 보고 통과시키지 않는다."""
  return 0 <= age_ms <= limit_ms
