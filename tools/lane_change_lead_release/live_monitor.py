#!/usr/bin/env python3
"""실시간 라이브 모니터 — 기기에서 모델이 '지금' 감지하는 전방/인접 차량 신호를 출력.

기기에서 실행:
    PYTHONPATH=/data/openpilot /usr/local/venv/bin/python -u \
        tools/lane_change_lead_release/live_monitor.py [duration_s]

읽기 전용(cereal 구독, 제어 무관). 주행 중 운전자가 직접 보지 말 것 —
동승자가 읽거나, 파일로 기록 후(`... > /tmp/mon.txt`) 정차 시 검토.

컬럼: Lbsm/Rbsm = 좌/우 후방 사각지대(테슬라 DAS), leads = leadsV3 중 prob>0.5 슬롯 수,
      leadOne d/v = 내 앞차 거리(m)/속도(km/h), leadTwo = 앞차의 앞차 유무,
      blink = 깜빡이, LCstate = 차선변경 상태.
"""
import sys
import time
from cereal import messaging


def main(dur: float):
    sm = messaging.SubMaster(["modelV2", "carState", "radarState"])
    end = time.monotonic() + dur
    t0 = time.monotonic()
    last = 0.0
    print("t(s) | Lbsm Rbsm | leads | leadOne d/v | lead2 | blink | LCstate", flush=True)
    while time.monotonic() < end:
        sm.update(200)
        now = time.monotonic()
        if now - last < 0.3:
            continue
        last = now
        cs, md, rs = sm["carState"], sm["modelV2"], sm["radarState"]
        n = sum(1 for L in md.leadsV3 if L.prob > 0.5)
        lo, lt = rs.leadOne, rs.leadTwo
        blink = (("L" if cs.leftBlinker else "") + ("R" if cs.rightBlinker else "")) or "-"
        lcd = str(md.meta.laneChangeState).split(".")[-1]
        print("%4.1f |  %d    %d  |   %d   | %s %2.0f/%2.0f | %s | %3s | %s" % (
            now - t0, cs.leftBlindspot, cs.rightBlindspot, n,
            "Y" if lo.status else "N",
            lo.dRel if lo.status else -1,
            (lo.vLead * 3.6) if lo.status else -1,
            "Y" if lt.status else "N", blink, lcd), flush=True)


if __name__ == "__main__":
    main(int(sys.argv[1]) if len(sys.argv) > 1 else 60)
