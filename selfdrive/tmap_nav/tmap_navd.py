#!/usr/bin/env python3
"""tmap_navd — 폰(티맵) 내비 데이터 UDP 수신 데몬 (얇은 런타임 글루).

폰의 Frida 브리지가 쏘는 JSON 을 UDP 로 받아 게이팅한 뒤 customReservedRawData1 로 발행한다.
speed_camera_warnd 와 같은 구조다 — 소켓·cereal 은 여기, 판단은 nav_logic.py(순수, PC 검증 가능).

브리지는 발견 절차 없이 본체 주소로 직접 쏜다(tailnet 은 브로드캐스트를 라우팅하지 않음).
따라서 carrot 의 UDP 7705 브로드캐스트 비콘은 이식하지 않는다 — 그냥 열고 받는다.

수신 포트는 7706(carrot 세대). 브리지가 7707/7706 동시 송신하므로 한쪽만 받으면 된다.
"""
import json
import socket

from cereal import messaging
from openpilot.common.params import Params
from openpilot.common.realtime import Ratekeeper, config_realtime_process
from openpilot.common.swaglog import cloudlog
from openpilot.selfdrive.tmap_nav.nav_logic import TmapNavLogic

RATE_HZ = 5.0          # 발행 주기. 브리지 송신은 2Hz 이므로 충분하다
PARAM_REFRESH = 1.0
RECV_PORT = 7706
RECV_BUF = 8192        # 완전 페이로드가 약 1.4KB. carrot 은 4096 이었으나 여유를 둔다
DRAIN_MAX = 20         # 한 주기에 흡수할 최대 데이터그램 수(백로그 폭주 방지)


def main():
  config_realtime_process([0, 1, 2, 3], priority=5)

  sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
  sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
  sock.bind(("0.0.0.0", RECV_PORT))
  sock.setblocking(False)
  cloudlog.info(f"tmap_navd: listening udp {RECV_PORT}")

  logic = TmapNavLogic()
  params = Params()
  sm = messaging.SubMaster(['liveLocationKalman'])
  pm = messaging.PubMaster(['customReservedRawData1'])
  rk = Ratekeeper(RATE_HZ, print_delay_threshold=None)

  enabled = False
  param_timer = 0.0
  last_peer = None

  while True:
    sm.update(0)

    param_timer += 1.0 / RATE_HZ
    if param_timer >= PARAM_REFRESH:
      param_timer = 0.0
      try:
        enabled = params.get_bool("TmapNavEnabled")
      except (ValueError, TypeError):
        enabled = False

    # 소켓에 쌓인 것을 전부 비운다 — 마지막 것이 가장 최신이다.
    for _ in range(DRAIN_MAX):
      try:
        data, peer = sock.recvfrom(RECV_BUF)
      except BlockingIOError:
        break
      except OSError:
        break
      if peer != last_peer:
        cloudlog.info(f"tmap_navd: rx from {peer[0]}:{peer[1]}")
        last_peer = peer
      logic.on_packet(data, rk.frame / RATE_HZ)

    llk = sm['liveLocationKalman']
    pg = llk.positionGeodetic
    have_fix = sm.alive['liveLocationKalman'] and llk.gpsOK and pg.valid
    lat, lon = (pg.value[0], pg.value[1]) if pg.valid else (0.0, 0.0)

    payload = logic.update(rk.frame / RATE_HZ, lat, lon, have_fix)
    if not enabled:
      # 토글이 꺼져 있으면 수신은 계속하되 소비자에게는 아무것도 주지 않는다.
      payload = {"lk": payload["lk"], "gd": 0, "sl": 0, "rn": "", "st": -1,
                 "sd": 0.0, "sx": 0, "sy": 0.0, "sz": 0.0, "rc": 0,
                 "sa": -1, "ra": -1, "src": "", "n": payload["n"],
                 "bs": 0, "bl": 0, "bn": 0.0, "br": 0.0, "bt": 0.0,
                 "ba": -1, "bo": -1, "bk": -1, "bv": 0, "bf": 0}

    # customReservedRawData1 은 Data(raw bytes) 필드 → new_message 의 init(service) 가 안 먹힘.
    # service=None 으로 빈 Event 생성 후 Data 필드에 직접 대입. (speed_camera_warnd 와 동일)
    msg = messaging.new_message(None, valid=True)
    msg.customReservedRawData1 = json.dumps(payload).encode()
    pm.send('customReservedRawData1', msg)

    rk.keep_time()


if __name__ == "__main__":
  main()
