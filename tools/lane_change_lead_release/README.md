# lane_change_lead_release

차선 변경 중 **목표 차선이 비었을 때** 원래 차선의 느린 lead를 **보수적으로 조기 해제**하는
결정 로직(`decider`) + 오프라인 검증 하네스.

문제·설계 배경: [docs/superpowers/specs/2026-06-12-lane-change-lead-release-design.md](../../docs/superpowers/specs/2026-06-12-lane-change-lead-release-design.md)
구현 계획: [docs/superpowers/plans/2026-06-12-lane-change-lead-release.md](../../docs/superpowers/plans/2026-06-12-lane-change-lead-release.md)

## 구성

| 파일 | 역할 | openpilot 의존 |
|------|------|----------------|
| `decider.py` | **순수 결정 로직**(실차 이식 대상). 5-AND 보수 게이트 + hold 타이머 + 즉시 복원. | ✗ (numpy만) |
| `extractor.py` | rlog → `decider.Frame` 스트림 (`LogReader`) | ✓ |
| `replay.py` | baseline(항상 lead 유지) vs patched 비교, Δt·속도 추정 리포트 | ✓ |
| `probe_livetracks.py` | `liveTracks` 가용성/스키마 확인(선행 de-risk) | ✓ |
| `tests/test_decider.py` | 순수 단위 테스트 (14개) | ✗ |

## 실행

**단위 테스트 (어디서나 — numpy+pytest):**
```bash
python3 -m pytest tools/lane_change_lead_release/tests/ -v
```

**로그 검증 (리눅스 PC, openpilot env 필요):**
```bash
# 0) liveTracks 가용성 먼저 확인
PYTHONPATH=$PWD:$PWD/openpilot uv run --project openpilot python \
  tools/lane_change_lead_release/probe_livetracks.py logs/00000010--668530db29--75/rlog.zst
# 1) baseline vs patched
PYTHONPATH=$PWD:$PWD/openpilot uv run --project openpilot python \
  tools/lane_change_lead_release/replay.py logs/00000010--668530db29--75/rlog.zst
```

## 검증 합격 기준 & 결과

| 로그 | 기대 | 합격 조건 | 결과 |
|------|------|-----------|------|
| 단위 테스트 | — | 전부 통과 | ✅ **14 passed** (Windows/Linux 무관) |
| liveTracks 가용성 | 레이더 트랙 존재 | count>0, `.points` 스키마 | ✅ **1200개, `.points` 확인**(레이더 존재, vision-only 아님) |
| seg75 (느린 lead, 빈 목표차선) | release 켜짐 | starting 중 발동, Δt ≥ 2.0s | ✅ **t_release=38.28s, t_off=43.22s, Δt=4.94s**, 추정 91→109 km/h |
| seg74 (lead 없음) | 변화 없음 | `t_release=None` (게이트2 `no_blocking_lead`) | ✅ **t_release=None** (부작용 0) |
| 신규 캡처 (목표차선에 차) | release 꺼짐 | 전 구간 `t_release=None` (안전회귀) | ⏳ 캡처+실행 대기 |

> 검증 환경: comma 3X 기기(`/usr/local/venv` openpilot env)에서 저장 rlog 대상 오프라인 실행. 2026-06-12.
> 안전회귀(목표차선 점유) 1건만 캡처 후 실행하면 A단계 합격 완료.

## 한계 & 후속 (코드 리뷰 deferred 포함)

- **속도는 근사**: `replay`의 patched 속도는 `get_max_accel` 적분 추정. 실차 반영 전 **B단계 `process_replay`** 로 실제 `planV` 확정 필수.
- **liveTracks 의존**: 목표차선 점유 판정은 레이더 `liveTracks` 필요. `probe_livetracks.py`로 먼저 확인 — 비어 있으면(vision-only) 감지 소스를 `blindspot`+`leadsV3`로 **재설계**해야 함(spec §9).
- **차선 라인 붕괴 fail-open (리뷰 M8, 미해결)**: `target_lane_in_lane_prob`의 `half_w=max(0.1,…)` 클램프는 차선 라인이 소실/붕괴(inner≈outer)하면 band가 0.1m로 좁아져 **점유를 과소검출**(fail-open)할 수 있음. 현재는 차선 라인 유효성이 상위에서 게이트된다고 **가정**. 실차 이식 시 저신뢰 차선에서는 release를 억제(fail-closed)하도록 보강 검토.
- **우방향 실로그 통합 테스트**: 합성 단위테스트(`test_release_right_direction_when_target_empty`)는 있음. 실주행 우측 차선 변경 로그 검증은 로그 확보 후.
- **direction 미상 fail-closed (리뷰 C1, 해결됨)**: `laneChangeStarting`인데 방향이 `left`/`right`가 아니면 목표차선 검사 불가 → `no_direction`으로 **해제 금지**. 회귀 테스트 포함.

## 안전 원칙

모든 게이트는 AND. 기본값 = "해제 안 함"(현 동작 유지). release 시에도 lead 완전 무시 금지(B단계에서 `v_cruise` 캡 + 소프트 갭). 트랙/사각지대 감지 시 즉시 복원. 실차 주행 테스트는 사용자 책임.
