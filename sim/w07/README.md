# sim/w07 — 7주차 함태훈 몫: 게인 통일 · 짐 질량 중력보상 · 베이스 + 팔 협조 resolved-rate · 베이스 가감속 토크 예산

| 파일 | 하는 일 | 결과 |
|---|---|---|
| `w07_common.py` | 모델 로더(한글 경로 대응) · 메모리 편집(끝점 짐, 짐 gravcomp 비율 = 아는 / 모르는 / 틀린 질량, 중력보상 제거) · `set_payload`(잡기 · 놓기 런타임) | – |
| `wb_jac.py` | **전신 자코비안(3×7) · 가중치 · 가중 DLS · 영공간(관절 한계 회피)** — 박종진 `wb_ik`(인터페이스 v2) 공유 전 임시 구현, 받으면 4 함수 교체 | – |
| `w07_gain_payload.py` | 검증 ② 처짐 재기록(보상 끔 / 켬 × week05 / 통일 게인, 이론 대조) · 짐 0 / 3 kg 계단응답(302 자세) · 3 kg 게인 스케줄 · 중력보상 6 방식(모르는 짐 · 아는 짐 · 질량 틀림 2 · 적분 · 추정) | `results/gain_payload.{md,json}`, `week07_step_0_3kg.png` · `week07_payload_comp.png` |
| `w07_wb_rrc.py` | resolved-rate(CLIK) 직선 추종: 경로 A 전방 z 0.75 m · 경로 B 측방 z 0.90 m × 팔만 / 전신 W 1 · 10 · 100 (+ 3 kg) | `results/wb_rrc.{md,json}`, `wb_log_*.csv`, `week07_wb_line.png` · `week07_wb_line.gif` |
| `w07_base_torque.py` | 베이스 가감속 → 팔 관절 추가 토크 τ = M_ab·q̈_b + c(q, q̇): 3,000 자세 추정 · 시뮬 확인 12 건 · 토크 예산표 갱신 | `results/base_torque.{md,json}`, `torque_budget_w07.csv`, `week07_base_accel_torque.png` |
| `proposal/build_v1_joint_gains.diff` | 박종진 `b1/models/v1` 에 넣을 게인 통일 패치(actuator_params.json `joint_gains` + build_v1.py 9 줄 + validate_model.py 표 제목) | 빌드 후 b1.xml 액추에이터 8 줄만 바뀜, URDF 변화 0 |

실행 (저장소 루트, 합계 약 2분): `python sim/w07/w07_gain_payload.py` → `python sim/w07/w07_wb_rrc.py [--gif]` → `python sim/w07/w07_base_torque.py`
(Windows 노트북은 `py`. 리눅스 헤드리스 GIF 는 `MUJOCO_GL=egl`.) 실행 로그 = `results/run_log.txt`

## 노션 7주차 ↔ 이 저장소

| 노션 (함태훈) | 여기 | 비고 |
|---|---|---|
| 월: 게인 `actuator_params.json` 한 벌로 통일 | `b1_snapshot/v1/actuator_params.json` `joint_gains` + `proposal/*.diff` | 박종진 합의 · 빌드 반영 대기 |
| 월: 통일 게인으로 검증 5종 재실행 + 검증 ② 처짐 재기록(gravcomp 끔 / 켬) | `b1_snapshot/v1/results/validate_model.md`(박종진 스크립트, 패치 빌드), `gain_payload.md` 1절 | 5종 통과, 앞으로 수평 보상 끔 4.83 → 3.63° |
| 월: 짐 0 · 3 kg 오버슈트 · 정착시간 표 | `gain_payload.md` 2절 | 3 kg ζ 0.75 → 0.53 |
| 개별: 짐 질량 반영 중력보상(알려진 / 모르는 짐) | `gain_payload.md` 3절 | 적분 · 추정의 위험 포함 |
| 수 · 개별: resolved-rate 제어기 + 직선 추종, 그래프 + GIF | `w07_wb_rrc.py`, `wb_rrc.md`, `week07_wb_line.*` | 노션 예시 0.6 m 는 도달 불가(아래) → 0.75 m |
| 개별: 협조 동작 중 토크 예산 갱신(베이스 가감속) | `w07_base_torque.py`, `base_torque.md`, `torque_budget_w07.csv` | 가속 1 m/s² 가정 |
| 팀 연계: 박종진 `wb_ik` 자코비안 · 가중치 그대로 사용 | `wb_jac.py` (임시, 인터페이스 v2 제안) | 공유받으면 교체 후 재실행 |

- `b1_snapshot/` — 박종진 d6d6304 `b1/models/v1`(10/27, 10/29 이후 변경 없음 확인 11/04) + 게인 통일 패치로 다시 빌드한 사본(mjcf · meshes · inputs · actuator_params · 스크립트 4개 · 결과 4개). `B1_V1` 환경변수로 원본을 가리킬 수 있음
- 노션 예시 「테이블 높이 0.6 m」: 바닥 기준이면 팔 최저 도달 0.606 m(박종진 workspace.md) 아래이고 베이스는 높이를 못 바꿔 전신으로도 불가 → 6주차 pick_point 높이 0.75 m 로 진행
- 설명: [../../notes/12_wholebody_rrc_payload_comp.md](../../notes/12_wholebody_rrc_payload_comp.md)
