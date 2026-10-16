# sim/v1 — B1 공용 v1 모델 위 함태훈 몫 (week05)

| 스크립트 | 하는 일 | 결과 |
|---|---|---|
| `v1_common.py` | 모델 로더(한글 경로 대응) · 공통 도우미 · 그림 스타일. 기본 모델 = `b1_snapshot/v1`, `B1_V1` 환경변수로 박종진 원본 | – |
| `v1_actuator_table.py` | 액추에이터 파라미터표 (kp · 컴파일 kv · I_eff · Mᵢᵢ 범위 · ζ_hold · 계단 응답 · 포화 · 처짐) | `results/actuator_table.md` · .csv |
| `v1_validate_123.py` | 검증 ① 질량 · ② 정지 유지(보상 3방식) · ③ 중력 토크(4방법 검산, 전역 최적화, 페이로드별) | `results/validate_123.md` · .json |
| `v1_armature_study.py` | armature 유무 계단 응답 · 적분 안정 한계 (구술 Q2) | `results/armature_study.md` · .csv |
| `v1_base_velocity.py` | 공통 실습(화): 베이스 x · y · yaw 속도 명령 vs qvel, 시정수 이론 대조, 로봇 기준 명령 R(yaw) 변환 | `results/base_velocity.md` · .csv |
| `view_v1.py` | 노트북 뷰어: `sliders`(Control 슬라이더, 중력보상 B) · `sag`(수평 처짐 4.8°) · `gravcomp`(0°) · `mirror`(양팔 대칭) | 화면 캡처 → `assets/screenshots/week05/week05_laptop_*.png` |
| `laptop_check.py` | 노트북 재현: 검증 · 액추에이터표 · 미러 점검을 `results_laptop/` 에 다시 계산해 클라우드 값과 비교 | `results_laptop/laptop_compare.md` |
| `v1_mirror_actuator_check.py` | 팀 연계: 미러 규칙표로 생성 팔 액추에이터 부호 · 범위 · B2 부호표 교차 | `results/mirror_actuator_check.md` · .csv |

- `handoff/` — 박종진에게 넘기는 것: 근거를 채운 `actuator_params.json`(값 동일, 빌드 결과 동일), `build_v1_gravcomp_option.patch`(기본 꺼짐)
- `b1_snapshot/` — 박종진 347a0ca 의 mjcf · meshes · actuator_params · assumptions 사본 (읽기 전용)
- 실행: 저장소 루트에서 `python sim/v1/<스크립트>.py` (Windows 노트북은 `py`). 실행 로그 = `results/*_run_log.txt`
- 설명: [../../notes/10_v1_mjcf_actuator_sensor_validation.md](../../notes/10_v1_mjcf_actuator_sensor_validation.md)
