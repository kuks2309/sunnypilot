# 단속카메라 거리경고 (Korea speed camera warning)

전국 무인교통단속카메라(고정식 과속·구간단속)에 접근하면 **engage(개입) 여부와 무관하게**
화면 알림 + 소리 경고를 낸다. 데이터는 공공데이터포털 전국무인교통단속카메라표준데이터.

## 구성요소
| 파일 | 역할 |
|---|---|
| `selfdrive/speed_camera/speed_cameras.bin` | compact 좌표 DB(카메라당 10B). `tools/speed_camera/build_db.py` 산출물 |
| `selfdrive/speed_camera/camera_db.py` | bin 로더 + 격자 인덱스 + 전방 최근접 질의(haversine) |
| `selfdrive/speed_camera/speed_camera_warnd.py` | 데몬. GNSS 5Hz 구독 → 2단계 판정 → `customReservedRawData0` 발행 |
| `selfdrive/selfdrived/events.py` `speed_camera_alert` | 메시지 → `EventName.speedCameraWarning`(ET.PERMANENT) 동적 alert |
| `selfdrive/selfdrived/selfdrived.py` | `customReservedRawData0` 구독 → stage>0 시 이벤트 add |
| `selfdrive/ui/.../settings/visuals.py` | 켜기 토글 + 경고음 선택기(설정 GUI) |
| `system/manager/process_config.py` | `speed_camera_warnd` 프로세스(only_onroad) 등록 |

## 데이터 흐름
```
gpsLocationExternal(GNSS) ─5Hz─▶ speed_camera_warnd ─customReservedRawData0─▶ selfdrived
                                       │ (43k DB 격자검색, 무거운 일은 여기서만)        │ EventName.speedCameraWarning
                                       ▼                                              ▼ (ET.PERMANENT)
                                 stage 0/1/2, snd, chime                    soundd(소리)+alert_renderer(화면)
```
- **stage 1**(사전): 전방 ~600m. **stage 2**(감속): 초과속 + 편안한 감속 시작거리 진입.
- **chime**: 단계 진입 시에만 ~1.2초 소리(wav 반복특성과 무관한 단발).
- engage 무관: `state.py` 의 `current_alert_types`가 비engage에도 `ET.PERMANENT` 포함 + soundd는 engage 검사 없음.

## engage 중 자동 감속 (선택)
경고와 별개로, **openpilot 종방향 engage 중**에만 카메라 제한속도+5km/h까지 자동 감속한다.
- 데몬이 `dl`(목표 km/h)·`dd`(거리 m)를 같은 `customReservedRawData0` 에 실어 발행.
- `speed_limit_resolver.py` 가 이를 읽어 **fail-safe min**(기존 제한보다 낮을 때만)으로 적용 → 기존 **SLA(Speed Limit Assist)** 감속곡선·튜닝 그대로 재사용. **속도를 올리지 못함**.
- engage 게이트는 SLA가 자동 처리(`long_enabled`) → 비engage엔 감속 영향 0(경고만).
- **구간단속**: 전방 최근접이 SECTION_END(구간 내부 의미) → 제한속도 유지(시점~종점).
- 점 카메라: `decel_trigger_dist`(a=2.0) 안으로 진입 + 초과속일 때만 감속 시작.

## 설정 (설정 → Visuals)
- **Speed Camera Warning (Korea)**: 경고 기능 on/off (`SpeedCameraWarnEnabled`)
- **Speed Camera Warning Sound**: Mute / Soft / Immediate / Prompt (`SpeedCameraWarnSound`)
- **Speed Camera: Slow Down (when engaged)**: engage 중 자동감속 on/off (`SpeedCameraDecelEnabled`, 기본 off)
- **Speed Camera Slow Down: Start Margin**: 감속 시작 여유 거리 0/15/30/50/80 m (`SpeedCameraDecelMargin`, 기본 15m). 클수록 더 일찍 시작. 감속 강도(=차간거리 COMFORT_BRAKE 2.0)는 불변.

## 데이터 갱신 (반기마다)
```
# PC에서:
python tools/speed_camera/download_cameras.py --service-key <data.go.kr 디코딩키>   # speed_cameras.csv
python tools/speed_camera/build_db.py                                              # → speed_cameras.bin
# bin 커밋 후 기기에 배포(재빌드)
```

## 빌드 (기기)
cereal `EventName` 추가가 있어 **cereal 재생성 필요**:
```
scons -j$(nproc)          # 또는 최소: cereal 재빌드 후 전체
```

## SIL 검증 (PC, 기기 불필요)
결정 로직은 `warn_logic.py`(cereal 비의존)로 분리되어 PC에서 합성 주행 폐루프로 검증한다:
```
python tools/speed_camera/sil_test.py
```
실제 DB(`speed_cameras.bin`) + 실제 로직으로 ① 경고만(비engage→감속0) ② 경고+감속(engage→제한+5 수렴)
③ 구간단속(종점까지 제한 유지)을 자동 검정(PASS/FAIL). 데몬은 이 로직의 얇은 글루.

## 온디바이스 검증
1. `params` 로 `SpeedCameraWarnEnabled=1` 확인(또는 설정 GUI 토글).
2. 단속카메라 ~600m 전방 접근 시 화면 "단속카메라 50 / 600m" + 선택한 소리(단발).
3. 제한속도 초과로 접근 시 "감속! 제한 50" (주황) 경고.
4. **engage 안 한 상태**에서도 위가 동일하게 뜨는지 확인(핵심 요구사항).
5. 로그: `speed_camera_warnd` 프로세스 alive, `customReservedRawData0` 발행 확인.

> 주의: 무거운 DB 검색은 데몬(5Hz)에서만 수행하므로 안전 루프에 영향 없음.
> 기기에서 별도 무거운 분석 병행 금지(commIssue로 engage 끊김 사례 있음).
