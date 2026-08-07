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

# --- 구간단속 -----------------------------------------------------------------
# 근거: docs remote_instruction/SECTION_ENFORCEMENT_ANALYSIS.md (경부 11.4km/110km/h 실측)
#
# nSdiType 과 nSdiBlockType 이 짝으로 온다. carrot 은 `nSdiBlockType in [2,3]` 으로
# 판정하는데, 실측 시작 접근은 **1** 이었다. 그대로 쓰면 진입을 통째로 놓친다.
SDI_BLOCK_START = 2       # SDI_SPEED_BLOCK_START_POS  구간 시점
SDI_BLOCK_END = 3         # SDI_SPEED_BLOCK_END_POS    구간 종점
SDI_BLOCK_MID = 4         # SDI_SPEED_BLOCK_MID_POS    구간 내 (현재 브리지가 미전송)

BLOCK_START = 1           # 시점 접근 — nSdiBlockDist = 구간 전체 길이
BLOCK_MID = 2             # 구간 내   — (현재 미전송)
BLOCK_END = 3             # 종점 접근 — nSdiBlockDist = 종점까지 남은 거리

# 시점을 이 거리 안까지 좁힌 뒤 신호가 끊기면 진입한 것으로 본다.
# 실측에서 857m 부터 보이다가 157m 를 마지막으로 사라졌다.
SECTION_ARM_DIST = 400.0
SECTION_OVERRUN = 1.2     # 구간 길이의 이 배를 넘게 달리면 이탈로 보고 상태를 버린다
SECTION_MAX_SEC = 3600.0  # 안전 타임아웃. 이 시간을 넘기면 무조건 해제
# 연속 fix 사이의 함의속도가 이보다 크면 순간이동으로 보고 누적하지 않는다.
# 고정 거리로 자르면 갱신 간격을 무시하게 된다 — 30초에 900m 는 정상 주행(108km/h)인데
# 거리만 보면 점프로 오판한다. 216km/h 를 넘는 이동은 물리적으로 주행이 아니다.
JUMP_MAX_MPS = 60.0

# 구간 상태
SEC_NONE = 0
SEC_APPROACH = 1          # 시점 접근 중
SEC_INSIDE = 2            # 구간 내부 (자체 추적)
SEC_EXIT = 3              # 종점 접근 중

# ★ 구간 채널은 `ageMs.sdi` 로 판정하면 안 된다 (2026-08-06 실측).
# 원격이 구간단속을 전방 점(point) SDI 와 별개 채널로 분리한 뒤로, `ageMs.sdi` 는
# 점 SDI 나이만 재고 구간 갱신에는 반응하지 않는다. 실제로 `nSdiBlockDist` 가
# 11,313 → 1,047 로 살아 움직이는 동안 `ageMs.sdi` 는 리셋 없이 518,878ms(8분 39초)
# 까지 선형 증가만 했다. 그 게이트를 태우면 완벽히 신선한 구간 정보를 전량 버린다.
# 그래서 구간 신선도는 **우리가 직접** 본다 — 필드가 실제로 변하는지로.
SECTION_STALE_SEC = 30.0

# 평균속도 0 은 산발 글리치가 아니라 3~6패킷(1~2초) 뭉치로 온다(1374건 중 36건, 9덩어리).
# 유효값 사이에 낀 단일 0 은 0건이라 "한 패킷 무시" 로는 못 거른다.
AVG_ZERO_HOLD_SEC = 3.0

# 진입 직후 ~30초는 표본 부족으로 요동한다(실측 46 → 71, 25km/h 점프). 그동안은 믿지 않는다.
AVG_WARMUP_SEC = 30.0

# 카운트다운이 이 값 이하로 내려온 뒤의 0 만 진짜 소진으로 본다.
# 351 에서 갑자기 0 이 오는 글리치가 있어, 그대로 받으면 위반 플래그가 오탐한다.
CD_ZERO_TRUST_SEC = 5

# 구간 안에서 카운트다운이 이보다 크게 되오르면 글리치로 본다.
# 실측(8/7): 14 -> 372(초기값) -> 13. 정상 진행은 패킷당 1~2초씩 감소한다.
CD_RISE_TOLERANCE = 5


def haversine(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
  """두 좌표 사이 거리(m)."""
  dlat = math.radians(lat2 - lat1)
  dlon = math.radians(lon2 - lon1)
  a = (math.sin(dlat / 2) ** 2 +
       math.cos(math.radians(lat1)) * math.cos(math.radians(lat2)) * math.sin(dlon / 2) ** 2)
  return 2 * EARTH_R * math.asin(math.sqrt(a))


class SectionTracker:
  """구간단속 상태를 스스로 붙든다.

  왜 필요한가: 브리지가 구간 내부에서 SDI 를 보내지 않는다. 실측(경부 11.4km)에서
  시점 통과 직후부터 종점 1km 전까지 **10분 동안 신호가 없었다.** 티맵 평균속도
  (`nSdiBlockAverageSpeed`)도 종점 1km 전에야 나오는데, 그때 이미 초과돼 있으면
  남은 1km 로는 되돌릴 수 없다.

  그래서 시점에서 받은 구간 길이·제한속도를 래치하고 자차 이동거리로 내부를 버틴다.
  브리지가 구간 내 전송(`nSdiType=4`)을 보완하면 이 추적은 교차검증용으로 남는다 —
  링크가 끊겨도 이미 진입한 구간은 끝까지 관리해야 하므로 어느 쪽이든 값어치가 있다.
  """

  def __init__(self):
    self.state = SEC_NONE
    self.length = 0.0        # 구간 전체 길이(m) — 시점에서 래치
    self.limit = 0           # 구간 제한속도(km/h)
    self.traveled = 0.0      # 진입 후 자체 누적 이동거리(m)
    self.remain = 0.0        # 종점까지 남은 거리(m)
    self.tmap_avg = -1       # 티맵이 준 구간 평균속도(km/h). -1 = 없음/무효
    self.countdown = -1      # nSdiBlockTime — 위반 판정 카운트다운(초). -1 = 없음
    self.risk = 0            # 1 = 현재 페이스면 카운트다운 소진 전 종점 도달 = 평균 초과
    self.fresh = 0           # 구간 필드가 실제로 갱신되고 있는가
    self.t_enter = 0.0
    self._arm_dist = 1e9     # 접근 중 시점까지 남은 거리
    self._last_fix = None
    self._last_fix_t = 0.0
    self._avg_valid = -1     # 마지막 유효 평균속도
    self._avg_zero_t = 0.0   # 0 이 연속되기 시작한 시각
    self._cd_valid = -1      # 마지막 유효 카운트다운
    self._cd_zero_t = 0.0
    self._sig = None         # 구간 필드 스냅샷(변화 감지용)
    self._sig_t = 0.0

  def step_distance(self, now: float, lat: float, lon: float, have_fix: bool) -> float:
    """연속 fix 사이 이동거리를 잰다. fix 가 끊기면 누적을 중단한다."""
    if not have_fix or (lat == 0.0 and lon == 0.0):
      self._last_fix = None
      return 0.0
    moved = 0.0
    if self._last_fix is not None:
      moved = haversine(self._last_fix[0], self._last_fix[1], lat, lon)
      dt = now - self._last_fix_t
      # 함의속도로 판정한다. 거리만 보면 갱신 간격이 긴 정상 주행을 점프로 오판한다.
      if dt <= 0.0 or moved / dt > JUMP_MAX_MPS:
        moved = 0.0
    self._last_fix = (lat, lon)
    self._last_fix_t = now
    return moved

  def update(self, now: float, moved: float, sdi_type: int, blk_type: int,
             blk_speed: int, blk_dist: float, blk_avg_raw: int,
             blk_time: int, blk_section: bool, sdi_dist: float) -> None:
    if self.state == SEC_INSIDE:
      self.traveled += moved
      self.remain = max(0.0, self.length - self.traveled)

    self._update_freshness(now, blk_type, blk_dist, blk_section)
    self._update_avg(now, blk_avg_raw)

    # nSdiBlockTime 은 위반 판정 카운트다운이다(실측: 초기값 372 = 11393m ÷ 110km/h,
    # 이후 1초에 1씩 실시간 감소, 0 도달 후 0 유지). 0 이 되기 전에 종점을 통과하면
    # 평균속도 위반이다. 평균속도를 안 쓰고도 판정이 되고, 단조 감소라 다루기 쉽다.
    self._update_countdown(now, blk_time)
    self._update_risk(now)

    approaching_start = (blk_type == BLOCK_START or sdi_type == SDI_BLOCK_START)
    in_middle = (blk_type == BLOCK_MID or sdi_type == SDI_BLOCK_MID or
                 (blk_section and blk_type == 0 and sdi_type < 0))
    approaching_end = (blk_type == BLOCK_END or sdi_type == SDI_BLOCK_END)

    if approaching_start:
      # nSdiBlockDist 는 여기서 **구간 전체 길이**다(실측 11393 고정).
      if blk_dist > 0:
        self.length = blk_dist
      if blk_speed > 0:
        self.limit = blk_speed
      self._arm_dist = sdi_dist if sdi_dist > 0 else self._arm_dist
      self.state = SEC_APPROACH
      return

    if in_middle:
      # 브리지가 구간 내 전송을 보완하면 여기로 들어온다. 자체 추적보다 우선한다.
      if self.state != SEC_INSIDE:
        self._enter(now)
      if blk_dist > 0:
        self.remain = blk_dist
        self.traveled = max(0.0, self.length - blk_dist)
      if blk_speed > 0:
        self.limit = blk_speed
      return

    if approaching_end:
      # nSdiBlockDist 는 여기서 **종점까지 남은 거리**다(실측 989 → 40).
      if self.state == SEC_NONE:
        self._enter(now)     # 시점을 놓쳤어도 종점 신호로 구간을 인지한다
      self.state = SEC_EXIT
      self.remain = blk_dist if blk_dist > 0 else sdi_dist
      if blk_speed > 0:
        self.limit = blk_speed
      return

    # --- 구간 신호가 없는 구간 ---
    if self.state == SEC_APPROACH:
      # 시점을 충분히 좁힌 뒤 신호가 끊겼다 = 통과해 진입한 것이다.
      # 실측: 857m 부터 보이다가 157m 를 마지막으로 사라졌다.
      if self._arm_dist <= SECTION_ARM_DIST:
        self._enter(now)
      else:
        self.reset()         # 멀리서 스쳤을 뿐이면 버린다
    elif self.state == SEC_EXIT:
      self.reset()           # 종점 신호가 사라졌다 = 구간을 벗어났다
    elif self.state == SEC_INSIDE:
      over = self.length > 0 and self.traveled > self.length * SECTION_OVERRUN
      if over or (now - self.t_enter) > SECTION_MAX_SEC:
        self.reset()         # 이탈했거나 너무 오래됐다. 붙들고 있으면 위험하다

  def _update_freshness(self, now: float, blk_type: int, blk_dist: float,
                        blk_section: bool) -> None:
    """구간 필드가 실제로 갱신되는지 **우리가 직접** 본다.

    `ageMs.sdi` 는 점 SDI 나이만 재므로 구간 판정에 쓸 수 없다(상수 주석 참조).
    대신 필드 스냅샷이 바뀌는지를 보고, 안 바뀐 채 오래되면 낡은 것으로 본다.
    """
    sig = (blk_type, round(blk_dist), bool(blk_section))
    if sig != self._sig:
      self._sig = sig
      self._sig_t = now
    has_signal = blk_type in (BLOCK_START, BLOCK_MID, BLOCK_END) or bool(blk_section)
    self.fresh = int(has_signal and (now - self._sig_t) <= SECTION_STALE_SEC)

  def _update_risk(self, now: float) -> None:
    """구간 평균속도 위반 위험을 본다.

    ★ 방향을 헷갈리기 쉽다. `nSdiBlockTime` 은 **제한속도로 갔을 때 걸리는 최소 시간**이다
    (실측 372초 = 11,393m ÷ 110km/h). 따라서

      - 카운트다운이 0 이 되기 **전에** 종점에 닿으면 → 그만큼 빨리 간 것 = **위반**
      - 구간 안에서 0 에 도달하면 → 최소 시간을 채운 것 = **준수**

    구간 안에서 0 을 위반으로 읽으면 정반대가 된다.
    여기서는 현재 페이스로 종점까지 걸릴 시간을 카운트다운과 비교해 미리 경고한다.
    """
    if self.state not in (SEC_INSIDE, SEC_EXIT) or self.countdown <= 0:
      self.risk = 0
      return
    speed_kph = self._own_speed_kph(now)
    if speed_kph <= 0 or self.remain <= 0:
      self.risk = 0
      return
    eta = self.remain / (speed_kph / 3.6)
    # 남은 거리를 지금 페이스로 달리면 카운트다운보다 먼저 도착한다 = 평균 초과
    self.risk = int(eta < self.countdown)

  def _own_speed_kph(self, now: float) -> float:
    dt = now - self.t_enter
    if dt < AVG_WARMUP_SEC or self.traveled <= 0.0:
      return 0.0
    return self.traveled / dt * 3.6

  def _update_countdown(self, now: float, raw: int) -> None:
    """위반 카운트다운. 값 0(소진)과 필드 부재(-1)를 구분하되 글리치 0 은 거른다.

    `nSdiBlockTime` 도 평균속도와 같은 뭉치 0 글리치를 갖는다(실측: 351 에서 갑자기 0,
    다음 패킷에 다시 351). 그대로 받으면 **위반 플래그가 오탐한다** — 오탐하면 안 되는
    신호다. 진짜 소진은 작은 값을 거쳐 오므로, 직전 유효값이 충분히 작을 때만 0 을 믿는다.
    """
    if raw > 0:
      # 구간 안에서 카운트다운은 감소만 한다. 위로 튀면 글리치다.
      # 실측(8/7 17:59:19): 14 -> 372(초기값) -> 13. 평균속도 0 글리치와 같은 순간에 온다.
      # 그대로 받으면 "시간이 많이 남았다"로 오판해 위반 위험이 오탐한다.
      if (self.state in (SEC_INSIDE, SEC_EXIT) and self._cd_valid > 0
          and raw > self._cd_valid + CD_RISE_TOLERANCE):
        self.countdown = self._cd_valid
        return
      self._cd_valid = raw
      self._cd_zero_t = 0.0
      self.countdown = raw
      return
    if raw < 0:
      self.countdown = -1        # 필드 자체가 없다
      return

    # raw == 0
    if self._cd_zero_t == 0.0:
      self._cd_zero_t = now
    genuine = (0 <= self._cd_valid <= CD_ZERO_TRUST_SEC) or \
              (now - self._cd_zero_t) > AVG_ZERO_HOLD_SEC
    self.countdown = 0 if genuine else self._cd_valid

  def _update_avg(self, now: float, raw: int) -> None:
    """평균속도의 0 뭉치와 진입 직후 요동을 거른다."""
    if raw > 0:
      self._avg_valid = raw
      self._avg_zero_t = 0.0
    else:
      # 0 은 3~6패킷(1~2초) 뭉치로 온다. 짧으면 직전 유효값을 유지하고 길어지면 버린다.
      if self._avg_zero_t == 0.0:
        self._avg_zero_t = now
      elif (now - self._avg_zero_t) > AVG_ZERO_HOLD_SEC:
        self._avg_valid = -1

    if self.state in (SEC_INSIDE, SEC_EXIT) and (now - self.t_enter) < AVG_WARMUP_SEC:
      self.tmap_avg = -1        # 진입 직후는 표본이 모자라 요동한다(46 → 71 실측)
    else:
      self.tmap_avg = self._avg_valid

  def own_avg_kph(self, now: float) -> int:
    """자체 추적 평균속도(km/h). 판단 불가면 -1."""
    if self.state not in (SEC_INSIDE, SEC_EXIT):
      return -1
    dt = now - self.t_enter
    # 창이 짧으면 표본이 모자라 터무니없는 값이 나온다(실측 진입 직후 249km/h).
    # 티맵 평균속도와 같은 기준으로 워밍업을 준다.
    if dt < AVG_WARMUP_SEC or self.traveled <= 0.0:
      return -1
    return int(round(self.traveled / dt * 3.6))

  def _enter(self, now: float) -> None:
    self.state = SEC_INSIDE
    self.traveled = 0.0
    self.remain = self.length
    self.t_enter = now

  def reset(self) -> None:
    self.state = SEC_NONE
    self.length = 0.0
    self.limit = 0
    self.traveled = 0.0
    self.remain = 0.0
    self.tmap_avg = -1
    self.countdown = -1
    self.risk = 0
    self.fresh = 0
    self._arm_dist = 1e9
    self._avg_valid = -1
    self._avg_zero_t = 0.0
    self._cd_valid = -1
    self._cd_zero_t = 0.0
    self._sig = None


class TmapNavLogic:
  """수신 패킷을 누적해 매 주기 payload 를 만든다. 소켓·cereal 을 모르는 순수 로직."""

  def __init__(self):
    self.last_rx_mono = -1e9   # 마지막 패킷 수신 시각(monotonic)
    self.apilot: dict = {}     # 마지막으로 받은 apilot 딕셔너리
    self.rx_count = 0
    self.parse_errors = 0
    self.section = SectionTracker()

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

    # --- 구간단속 ---
    # 자차 이동거리는 SDI 유효성과 무관하게 항상 누적한다. 구간 내부에서는 SDI 가
    # 아예 안 오므로, 그때도 거리를 세고 있어야 한다.
    moved = self.section.step_distance(now_mono, lat, lon, have_fix and link)

    # ★ 구간 채널에는 `ageMs.sdi` 게이트를 태우지 않는다.
    # 원격이 구간단속을 점 SDI 와 별개 채널로 분리한 뒤로 그 나이는 구간 갱신에
    # 반응하지 않는다. 태우면 살아 있는 구간 정보를 진입 수십 초 뒤부터 전량 버린다.
    # 대신 출처(sdiFrom)만 확인하고, 신선도는 SectionTracker 가 필드 변화로 직접 본다.
    section_usable = guiding and sdi_src == SDI_SOURCE_OK
    if not link:
      self.section.reset()   # 링크가 끊기면 붙들고 있던 구간 상태도 버린다
    elif section_usable:
      self.section.update(
        now_mono, moved,
        _as_int(a.get("nSdiType"), -1),
        _as_int(a.get("nSdiBlockType")),
        _as_int(a.get("nSdiBlockSpeed")),
        float(_as_int(a.get("nSdiBlockDist"))),
        _as_int(a.get("nSdiBlockAverageSpeed")),
        _as_int(a.get("nSdiBlockTime"), -1),
        bool(a.get("bSdiBlockSection")),
        float(_as_int(a.get("nSdiDist"))),
      )
    else:
      # 안내가 꺼졌거나 출처를 못 믿는 구간 — 내부 추적만 계속 돌린다
      self.section.update(now_mono, moved, -1, 0, 0, 0.0, 0, -1, False, 0.0)

    sec = self.section

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

      # --- 구간단속 ---
      "bs": sec.state,                    # 0=없음 1=시점접근 2=구간내 3=종점접근
      "bl": sec.limit,                    # 구간 제한속도(km/h), 0=없음
      "bn": round(sec.length, 1),         # 구간 전체 길이(m)
      "br": round(sec.remain, 1),         # 종점까지 남은 거리(m)
      "bt": round(sec.traveled, 1),       # 진입 후 자체 누적 이동거리(m)
      "ba": sec.tmap_avg,                 # 티맵 구간 평균속도(km/h), -1=없음/무효
      "bo": sec.own_avg_kph(now_mono),    # 자체 추적 평균속도(km/h), -1=판단불가
      "bk": sec.countdown,                # 위반 카운트다운(초), -1=없음. 0=소진
      "bv": sec.risk,                     # 1=현재 페이스면 평균속도 초과(위반 위험)
      "bf": sec.fresh,                    # 1=구간 필드가 실제로 갱신되고 있다
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
