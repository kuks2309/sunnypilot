#!/usr/bin/env python3
"""전국무인교통단속카메라표준데이터 전량 다운로드 (공공데이터포털 OpenAPI).

data.go.kr 표준데이터 공통 API(`tn_pubr_public_unmanned_traffic_camera_api`)를
totalCount 기준으로 페이지네이션하여 전국 카메라를 한 번에 받아 CSV/JSON으로 저장한다.

표준 라이브러리만 사용한다(requests 등 외부 의존성 없음) — 기기/PC 어디서나 바로 실행.

사용 전 준비
------------
1. https://www.data.go.kr/data/15028200/openapi.do 에서 "활용신청"(자동승인)
2. 마이페이지 → 발급받은 "일반 인증키(Decoding)" 복사
3. 아래 둘 중 하나로 키 전달:
   - 환경변수:  export DATA_GO_KR_KEY="발급받은_디코딩_키"   (Windows PS: $env:DATA_GO_KR_KEY="...")
   - 인자:      --service-key "발급받은_디코딩_키"

예시
----
    python download_cameras.py                         # 전국 전량 → speed_cameras.csv
    python download_cameras.py -o cams.csv --json      # CSV + JSON 동시 저장
    python download_cameras.py --service-key "키..."   # 환경변수 대신 인자로 키 전달

주의
----
- 개발계정 기본 트래픽 1,000건/일. numOfRows=1000이면 전국(~3만건)도 수십 콜이라 여유.
- 표준데이터 갱신은 반기 단위 → 'referenceDate'(데이터기준일자) 컬럼으로 최신성 확인.
- CSV는 기본 utf-8-sig(BOM)로 저장해 Excel에서 한글이 깨지지 않게 한다.
  openpilot 런타임에서 직접 파싱할 거면 --no-bom 으로 순수 utf-8 저장.
"""
from __future__ import annotations

import argparse
import csv
import json
import os
import sys
import time
import urllib.error
import urllib.parse
import urllib.request

API_URL = "https://api.data.go.kr/openapi/tn_pubr_public_unmanned_traffic_camera_api"


def fetch_page(service_key: str, page_no: int, num_of_rows: int, timeout: int = 30) -> dict:
    """한 페이지를 받아 response.body 딕셔너리를 반환."""
    params = {
        "serviceKey": service_key,  # 디코딩 키 → urlencode가 안전하게 인코딩
        "pageNo": page_no,
        "numOfRows": num_of_rows,
        "type": "json",
    }
    url = f"{API_URL}?{urllib.parse.urlencode(params)}"
    req = urllib.request.Request(url, headers={"User-Agent": "openpilot-speedcam/1.0"})
    with urllib.request.urlopen(req, timeout=timeout) as resp:
        raw = resp.read().decode("utf-8")

    try:
        data = json.loads(raw)
    except json.JSONDecodeError:
        # 키 오류 등은 XML 에러로 돌아오는 경우가 있음 → 원문을 그대로 보여준다.
        raise SystemExit(f"[에러] JSON 파싱 실패. 서버 응답 원문:\n{raw[:800]}")

    header = data.get("response", {}).get("header", {})
    code = header.get("resultCode")
    if code not in ("00", "0", None):
        raise SystemExit(f"[에러] API resultCode={code} msg={header.get('resultMsg')}")
    return data.get("response", {}).get("body", {})


def collect_all(service_key: str, num_of_rows: int) -> list[dict]:
    """totalCount 기준으로 전 페이지를 순회해 전체 item 리스트 반환."""
    body = fetch_page(service_key, 1, num_of_rows)
    total = int(body.get("totalCount", 0))
    items = list(_as_items(body))
    pages = (total + num_of_rows - 1) // num_of_rows
    print(f"  totalCount={total}, numOfRows={num_of_rows}, pages={pages}")

    for page in range(2, pages + 1):
        body = fetch_page(service_key, page, num_of_rows)
        items.extend(_as_items(body))
        print(f"  page {page}/{pages} … 누적 {len(items)}건")
        time.sleep(0.1)  # 서버 부하 완화

    if total and len(items) != total:
        print(f"  [경고] 수집 {len(items)} != totalCount {total} (중복/누락 가능)")
    return items


def _as_items(body: dict) -> list[dict]:
    """body.items 정규화 — list 또는 단일 dict 모두 처리."""
    items = body.get("items", [])
    if isinstance(items, dict):
        # {"item": [...]} 형태로 한 번 더 감싸는 변종 방어
        items = items.get("item", items)
    if isinstance(items, dict):
        return [items]
    return items or []


def write_csv(items: list[dict], path: str, bom: bool = True) -> None:
    if not items:
        raise SystemExit("[에러] 저장할 데이터가 없습니다.")
    # 모든 행의 키 합집합 → 컬럼 누락 방지(필드명 바뀌어도 안전)
    fields: list[str] = []
    for row in items:
        for k in row:
            if k not in fields:
                fields.append(k)
    encoding = "utf-8-sig" if bom else "utf-8"
    with open(path, "w", newline="", encoding=encoding) as f:
        writer = csv.DictWriter(f, fieldnames=fields, extrasaction="ignore")
        writer.writeheader()
        writer.writerows(items)
    print(f"[완료] CSV 저장: {path} ({len(items)}건, {len(fields)}컬럼, {encoding})")


def write_json(items: list[dict], path: str) -> None:
    with open(path, "w", encoding="utf-8") as f:
        json.dump(items, f, ensure_ascii=False, indent=2)
    print(f"[완료] JSON 저장: {path} ({len(items)}건)")


def main() -> int:
    ap = argparse.ArgumentParser(description="전국 무인교통단속카메라 전량 다운로드")
    ap.add_argument("-o", "--output", default="speed_cameras.csv", help="CSV 출력 경로")
    ap.add_argument("--json", action="store_true", help="동일 이름으로 JSON도 저장")
    ap.add_argument("-n", "--num-rows", type=int, default=1000, help="페이지당 건수(기본 1000)")
    ap.add_argument("--service-key", default=os.environ.get("DATA_GO_KR_KEY"),
                    help="data.go.kr 디코딩 인증키(미지정 시 환경변수 DATA_GO_KR_KEY)")
    ap.add_argument("--no-bom", action="store_true", help="CSV를 BOM 없는 순수 utf-8로 저장")
    args = ap.parse_args()

    if not args.service_key:
        print("[에러] 인증키가 없습니다. --service-key 또는 환경변수 DATA_GO_KR_KEY 설정.",
              file=sys.stderr)
        return 2

    print("전국 무인교통단속카메라 수집 시작 …")
    try:
        items = collect_all(args.service_key, args.num_rows)
    except urllib.error.HTTPError as e:
        raise SystemExit(f"[에러] HTTP {e.code}: {e.reason}\n{e.read().decode('utf-8','ignore')[:500]}")
    except urllib.error.URLError as e:
        raise SystemExit(f"[에러] 네트워크: {e.reason}")

    write_csv(items, args.output, bom=not args.no_bom)
    if args.json:
        write_json(items, os.path.splitext(args.output)[0] + ".json")
    return 0


if __name__ == "__main__":
    sys.exit(main())
