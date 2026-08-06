#!/usr/bin/env python3
"""tmap_nav 릴레이 — 본체 PC 가 받은 브리지 패킷을 LAN 의 기기로 넘겨준다.

왜 필요한가: 폰 브리지는 본체 PC 의 tailnet 주소로 쏘는데, comma 기기는 LAN 에만 있다.
tailnet 은 브로드캐스트를 라우팅하지 않고 기기는 tailnet 에 올라와 있지 않으므로,
둘 사이를 이어줄 것이 없으면 데이터가 기기에 닿지 않는다.
합의된 "본체 중계" 구조를 그대로 구현한 것이다.

    폰(티맵+Frida) --tailnet--> 본체 PC :7707 --LAN--> 기기 :7706 (tmap_navd)

기기를 tailnet 에 올리기로 하면 이 릴레이는 통째로 필요 없어진다.

사용:
  python -m tools.tmap_nav.relay --to 192.168.44.13
  python -m tools.tmap_nav.relay --to 192.168.44.13 --tee rx.jsonl

포트 7707 은 한 프로세스만 실질적으로 받는다. 이 릴레이를 돌리는 동안에는
다른 캡처 도구를 같은 포트에 띄우지 말 것 — 그래서 --tee 를 붙여 두었다.
"""
import argparse
import socket
import sys
import time

LISTEN_PORT = 7707     # 브리지가 본체로 쏘는 포트
DEVICE_PORT = 7706     # 기기 tmap_navd 가 바인드하는 포트(carrot 세대)
REPORT_EVERY = 10.0


def main():
  ap = argparse.ArgumentParser()
  ap.add_argument("--to", required=True, help="기기 IP (예: 192.168.44.13)")
  ap.add_argument("--to-port", type=int, default=DEVICE_PORT)
  ap.add_argument("--listen-port", type=int, default=LISTEN_PORT)
  ap.add_argument("--tee", help="받은 패킷을 이 jsonl 로도 기록한다")
  args = ap.parse_args()

  rx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
  rx.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
  rx.bind(("0.0.0.0", args.listen_port))
  rx.settimeout(1.0)

  tx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
  target = (args.to, args.to_port)

  tee = open(args.tee, "a", encoding="utf-8") if args.tee else None
  print(f"relay: 0.0.0.0:{args.listen_port} -> {target[0]}:{target[1]}"
        + (f"  (tee {args.tee})" if tee else ""), flush=True)

  n = sent = fail = 0
  last_report = time.time()
  last_n = 0

  try:
    while True:
      try:
        data, peer = rx.recvfrom(65535)
      except socket.timeout:
        data = None
      except OSError as e:
        print(f"relay: recv error {e}", file=sys.stderr, flush=True)
        continue

      if data:
        n += 1
        try:
          tx.sendto(data, target)
          sent += 1
        except OSError as e:
          fail += 1
          if fail <= 3:
            print(f"relay: send error {e}", file=sys.stderr, flush=True)
        if tee:
          tee.write(f'{{"t": {time.time():.3f}, "peer": "{peer[0]}:{peer[1]}", '
                    f'"raw": {data.decode("utf-8", "replace")}}}\n')
          tee.flush()

      now = time.time()
      if now - last_report >= REPORT_EVERY:
        rate = (n - last_n) / (now - last_report)
        print(f"relay: rx={n} sent={sent} fail={fail}  {rate:.1f}/s", flush=True)
        last_report, last_n = now, n
  except KeyboardInterrupt:
    print(f"\nrelay: 종료. rx={n} sent={sent} fail={fail}")
  finally:
    if tee:
      tee.close()


if __name__ == "__main__":
  main()
