#!/usr/bin/env python3
"""nav_logic SIL 테스트 — 실제 수신 패킷을 픽스처로 쓴다.

픽스처는 2026-08-05 원격 브리지가 실제로 보낸 패킷에서 가져왔다
(logs/rx_apilot_*.jsonl, remote_instruction/rx_live_180s_20260805.txt).
기기에 올리기 전에 게이팅이 의도대로 도는지 PC 에서 확인하는 것이 목적이다.

실행: python -m selfdrive.tmap_nav.tests.test_nav_logic
"""
import json
import unittest

from openpilot.selfdrive.tmap_nav.nav_logic import TmapNavLogic, haversine, LINK_TIMEOUT

# 안내 꺼짐 하트비트 — 신빌드는 ageMs/sdiFrom 키를 null 로 실어 보낸다
HEARTBEAT = {
  "active": 1,
  "apilot": {"type": "tmap-frida", "guiding": False, "sdiFrom": None,
             "ageMs": {"road": None, "sdi": None, "sdiPlus": None, "tbt": None},
             "dataAgeMs": 1311074},
}

# 안내 중 — 노인보호구역 129m 전방. vpCurrentPos 가 실제로 채워진 드문 표본
GUIDING = {
  "active": 1,
  "apilot": {
    "type": "tmap-frida", "guiding": True, "eRgStatus": 1,
    "nRoadLimitSpeed": 30, "szPosRoadName": "광교산로125번길", "roadNameGeneric": False,
    "nSdiType": 68, "sdiTypeName": "SDI_ELDER_START",
    "nSdiSpeedLimit": 30, "nSdiDist": 129,
    "vpSdiPointLat": 37.29935506136474, "vpSdiPointLon": 127.02775908435295,
    "nTBTTurnType": 13, "nTBTDist": 0, "szTBTMainText": "교차로",
    "sdiFrom": "RGData.sdiInfo",
    "ageMs": {"road": 1200, "sdi": 1200, "sdiPlus": 1200, "tbt": 1200},
    "dataAgeMs": 0,
  },
}


def pkt(obj) -> bytes:
  return json.dumps(obj).encode()


def variant(**over):
  """GUIDING 을 복사해 apilot 필드 일부만 바꾼다."""
  o = json.loads(json.dumps(GUIDING))
  o["apilot"].update(over)
  return o


class TestNavLogic(unittest.TestCase):

  def setUp(self):
    self.lg = TmapNavLogic()

  def test_no_packet_means_link_down(self):
    out = self.lg.update(100.0, 0, 0, False)
    self.assertEqual(out["lk"], 0)
    self.assertEqual(out["st"], -1)
    self.assertEqual(out["sl"], 0)

  def test_link_times_out(self):
    self.lg.on_packet(pkt(GUIDING), 10.0)
    self.assertEqual(self.lg.update(10.0 + LINK_TIMEOUT - 0.1, 0, 0, False)["lk"], 1)
    self.assertEqual(self.lg.update(10.0 + LINK_TIMEOUT + 0.1, 0, 0, False)["lk"], 0)

  def test_heartbeat_yields_nothing_usable(self):
    self.lg.on_packet(pkt(HEARTBEAT), 10.0)
    out = self.lg.update(10.0, 0, 0, False)
    self.assertEqual(out["lk"], 1)      # 링크는 살아 있다
    self.assertEqual(out["gd"], 0)      # 안내는 꺼져 있다
    self.assertEqual(out["sl"], 0)
    self.assertEqual(out["st"], -1)

  def test_guiding_packet_is_accepted(self):
    self.lg.on_packet(pkt(GUIDING), 10.0)
    out = self.lg.update(10.0, 0, 0, False)
    self.assertEqual(out["gd"], 1)
    self.assertEqual(out["sl"], 30)
    self.assertEqual(out["rn"], "광교산로125번길")
    self.assertEqual(out["st"], 68)
    self.assertEqual(out["sx"], 30)
    self.assertAlmostEqual(out["sd"], 129.0)
    self.assertEqual(out["rc"], 0)      # fix 없으면 재계산 안 함

  # --- 동결 버그 회귀 방어 ---------------------------------------------------

  def test_sdi_from_heap_is_rejected(self):
    """힙에서 주워온 값은 쓰지 않는다. 2026-08-05 이전 브리지의 동결 버그가 이것이었다."""
    self.lg.on_packet(pkt(variant(sdiFrom="heap")), 10.0)
    out = self.lg.update(10.0, 0, 0, False)
    self.assertEqual(out["st"], -1)
    self.assertEqual(out["sl"], 30)     # 도로 제한속도는 별개 채널이라 살아 있다

  def test_sdi_from_none_is_rejected(self):
    """전방 이벤트 없음. 낡은 값으로 메우지 않는다."""
    self.lg.on_packet(pkt(variant(sdiFrom="none")), 10.0)
    self.assertEqual(self.lg.update(10.0, 0, 0, False)["st"], -1)

  def test_stale_sdi_channel_is_rejected(self):
    """채널 나이가 임계를 넘으면 버린다 — 패킷 자체는 신선해도 그렇다."""
    old = variant(ageMs={"road": 1200, "sdi": 172370, "sdiPlus": 1200, "tbt": 1200})
    self.lg.on_packet(pkt(old), 10.0)
    out = self.lg.update(10.0, 0, 0, False)
    self.assertEqual(out["st"], -1)     # SDI 는 버리고
    self.assertEqual(out["sl"], 30)     # 도로는 살린다

  def test_missing_age_is_rejected(self):
    """나이를 모르면 신선도를 판단할 수 없으므로 통과시키지 않는다."""
    self.lg.on_packet(pkt(variant(ageMs={"road": None, "sdi": None})), 10.0)
    out = self.lg.update(10.0, 0, 0, False)
    self.assertEqual(out["st"], -1)
    self.assertEqual(out["sl"], 0)

  # --- 도로명 플레이스홀더 ---------------------------------------------------

  def test_generic_road_name_is_blanked_but_limit_kept(self):
    self.lg.on_packet(pkt(variant(szPosRoadName="일반도로", roadNameGeneric=True)), 10.0)
    out = self.lg.update(10.0, 0, 0, False)
    self.assertEqual(out["rn"], "")
    self.assertEqual(out["sl"], 30)

  def test_zero_limit_is_rejected(self):
    self.lg.on_packet(pkt(variant(nRoadLimitSpeed=0, szPosRoadName=None)), 10.0)
    out = self.lg.update(10.0, 0, 0, False)
    self.assertEqual(out["sl"], 0)
    self.assertEqual(out["rn"], "")

  # --- 좌표 재계산 -----------------------------------------------------------

  def test_distance_recomputed_from_coordinates(self):
    """거리는 콜백 사이에 늙지만 좌표는 안 늙는다. fix 가 있으면 다시 잰다."""
    self.lg.on_packet(pkt(GUIDING), 10.0)
    # 지점에서 정북으로 약 200m 떨어진 위치
    lat = 37.29935506136474 + 200.0 / 111320.0
    out = self.lg.update(10.0, lat, 127.02775908435295, True)
    self.assertEqual(out["rc"], 1)
    self.assertAlmostEqual(out["sd"], 200.0, delta=2.0)   # 129 가 아니라 실제 거리
    self.assertNotAlmostEqual(out["sd"], 129.0, delta=1.0)

  # --- 견고성 ----------------------------------------------------------------

  def test_garbage_is_ignored(self):
    for bad in (b"", b"not json", b"{}", b'{"apilot": 3}', b'\xff\xfe'):
      self.assertFalse(self.lg.on_packet(bad, 10.0))
    self.assertEqual(self.lg.parse_errors, 5)
    self.assertEqual(self.lg.update(10.0, 0, 0, False)["lk"], 0)

  def test_last_packet_wins(self):
    self.lg.on_packet(pkt(GUIDING), 10.0)
    self.lg.on_packet(pkt(HEARTBEAT), 10.1)
    self.assertEqual(self.lg.update(10.1, 0, 0, False)["gd"], 0)

  def test_haversine_sanity(self):
    # 위도 1도 = 약 111km
    self.assertAlmostEqual(haversine(37.0, 127.0, 38.0, 127.0), 111195, delta=200)
    self.assertEqual(haversine(37.0, 127.0, 37.0, 127.0), 0.0)


# ---------------------------------------------------------------------------
# 구간단속 — 값은 2026-08-06 경부고속도로 실측(11,393m / 110km/h)에서 가져왔다.
# 근거: docs remote_instruction/SECTION_ENFORCEMENT_ANALYSIS.md
# ---------------------------------------------------------------------------

def sec_pkt(**over):
  """구간단속 관련 필드를 갖춘 guiding 패킷."""
  return variant(nSdiSpeedLimit=0, **over)


class TestSectionTracker(unittest.TestCase):

  def setUp(self):
    self.lg = TmapNavLogic()

  def feed(self, t, lat=0.0, lon=0.0, fix=False, **over):
    self.lg.on_packet(pkt(sec_pkt(**over)), t)
    return self.lg.update(t, lat, lon, fix)

  def test_start_approach_latches_length_and_limit(self):
    """시점 접근에서 nSdiBlockDist 는 구간 전체 길이다(실측 11393 고정)."""
    out = self.feed(10.0, nSdiType=2, sdiTypeName="SDI_SPEED_BLOCK_START_POS",
                    nSdiDist=857, nSdiBlockType=1, nSdiBlockSpeed=110,
                    nSdiBlockDist=11393, nSdiBlockAverageSpeed=0)
    self.assertEqual(out["bs"], 1)        # 시점 접근
    self.assertEqual(out["bl"], 110)      # 구간 제한속도 래치
    self.assertAlmostEqual(out["bn"], 11393.0)

  def test_entry_detected_when_start_signal_disappears_up_close(self):
    """실측 패턴: 857m 부터 보이다가 157m 를 마지막으로 신호가 사라진다 = 진입."""
    self.feed(10.0, nSdiType=2, nSdiDist=857, nSdiBlockType=1,
              nSdiBlockSpeed=110, nSdiBlockDist=11393)
    self.feed(11.0, nSdiType=2, nSdiDist=157, nSdiBlockType=1,
              nSdiBlockSpeed=110, nSdiBlockDist=11393)
    out = self.feed(12.0, nSdiType=None, nSdiBlockType=0, nSdiBlockDist=0)
    self.assertEqual(out["bs"], 2)        # 구간 내부
    self.assertEqual(out["bl"], 110)      # 래치된 제한속도가 남아 있다
    self.assertAlmostEqual(out["bn"], 11393.0)

  def test_distant_start_then_gone_is_discarded(self):
    """멀리서 스쳤을 뿐이면 진입으로 보지 않는다."""
    self.feed(10.0, nSdiType=2, nSdiDist=3000, nSdiBlockType=1,
              nSdiBlockSpeed=110, nSdiBlockDist=11393)
    out = self.feed(11.0, nSdiType=None, nSdiBlockType=0)
    self.assertEqual(out["bs"], 0)

  def test_inside_accumulates_distance_without_any_sdi(self):
    """구간 내부 10분간 SDI 가 안 온다. 그동안 자체 이동거리로 버텨야 한다."""
    self.feed(10.0, nSdiType=2, nSdiDist=157, nSdiBlockType=1,
              nSdiBlockSpeed=110, nSdiBlockDist=11393)
    lat, lon = 37.25084, 127.10393       # 실제 시점 좌표
    out = self.feed(11.0, lat, lon, True, nSdiType=None, nSdiBlockType=0)
    self.assertEqual(out["bs"], 2)
    # 정북으로 약 900m 이동
    out = self.feed(41.0, lat + 900.0 / 111320.0, lon, True,
                    nSdiType=None, nSdiBlockType=0)
    self.assertAlmostEqual(out["bt"], 900.0, delta=10.0)
    self.assertAlmostEqual(out["br"], 11393.0 - 900.0, delta=10.0)
    self.assertGreater(out["bo"], 0)      # 자체 평균속도가 산출된다

  def test_gps_jump_is_not_accumulated(self):
    """경로 재생 되감기 같은 순간이동을 거리로 세면 안 된다."""
    self.feed(10.0, nSdiType=2, nSdiDist=157, nSdiBlockType=1,
              nSdiBlockSpeed=110, nSdiBlockDist=11393)
    self.feed(11.0, 37.25, 127.10, True, nSdiType=None, nSdiBlockType=0)
    out = self.feed(12.0, 37.35, 127.10, True, nSdiType=None, nSdiBlockType=0)
    self.assertAlmostEqual(out["bt"], 0.0, delta=1.0)   # 11km 점프는 무시

  def test_end_approach_distance_is_remaining_not_length(self):
    """종점 접근에서 nSdiBlockDist 는 종점까지 남은 거리다(실측 989 → 40)."""
    out = self.feed(10.0, nSdiType=3, sdiTypeName="SDI_SPEED_BLOCK_END_POS",
                    nSdiDist=989, nSdiBlockType=3, nSdiBlockSpeed=110,
                    nSdiBlockDist=989, nSdiBlockAverageSpeed=62)
    self.assertEqual(out["bs"], 3)
    self.assertAlmostEqual(out["br"], 989.0)
    self.assertEqual(out["ba"], 62)

  def test_zero_average_speed_is_rejected(self):
    """실측에서 62 -> 0 -> 62 로 튄다. 0 을 정지로 오독하면 안 된다."""
    out = self.feed(10.0, nSdiType=3, nSdiDist=665, nSdiBlockType=3,
                    nSdiBlockSpeed=110, nSdiBlockDist=665, nSdiBlockAverageSpeed=0)
    self.assertEqual(out["ba"], -1)

  def test_block_type_1_is_recognised(self):
    """carrot 은 nSdiBlockType in [2,3] 으로 거른다. 실측 시점 접근은 1 이라 놓친다."""
    out = self.feed(10.0, nSdiType=2, nSdiDist=857, nSdiBlockType=1,
                    nSdiBlockSpeed=110, nSdiBlockDist=11393)
    self.assertNotEqual(out["bs"], 0)

  def test_mid_type_when_bridge_starts_sending_it(self):
    """브리지가 nSdiType=4 를 보내주면 자체 추적보다 그 값을 우선한다."""
    out = self.feed(10.0, nSdiType=4, sdiTypeName="SDI_SPEED_BLOCK_MID_POS",
                    nSdiDist=0, nSdiBlockType=2, nSdiBlockSpeed=110,
                    nSdiBlockDist=9532)
    self.assertEqual(out["bs"], 2)
    self.assertAlmostEqual(out["br"], 9532.0)

  def test_overrun_releases_state(self):
    """구간 길이를 크게 넘겨 달리면 이탈로 보고 버린다. 붙들고 있으면 위험하다."""
    self.feed(10.0, nSdiType=2, nSdiDist=157, nSdiBlockType=1,
              nSdiBlockSpeed=110, nSdiBlockDist=500)
    lat, lon = 37.25, 127.10
    self.feed(11.0, lat, lon, True, nSdiType=None, nSdiBlockType=0)
    t = 12.0
    for i in range(1, 8):                 # 4초에 100m = 90km/h 로 전진
      t += 4.0
      out = self.feed(t, lat + i * 100.0 / 111320.0, lon, True,
                      nSdiType=None, nSdiBlockType=0)
    self.assertEqual(out["bs"], 0)        # 700m > 500m x 1.2 → 이탈

  def test_link_loss_clears_section(self):
    self.feed(10.0, nSdiType=2, nSdiDist=157, nSdiBlockType=1,
              nSdiBlockSpeed=110, nSdiBlockDist=11393)
    self.feed(11.0, 37.25, 127.10, True, nSdiType=None, nSdiBlockType=0)
    out = self.lg.update(11.0 + LINK_TIMEOUT + 1.0, 0, 0, False)
    self.assertEqual(out["bs"], 0)
    self.assertEqual(out["lk"], 0)


if __name__ == "__main__":
  unittest.main()
