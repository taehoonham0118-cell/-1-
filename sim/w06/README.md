# sim/w06 — 6주차(PDR) 함태훈 몫: kp · kv 튜닝 · 집기 궤적 추종 · 토크 예산표 · Isaac 액추에이터표

| 스크립트 | 하는 일 | 결과 |
|---|---|---|
| `w06_common.py` | 모델 로더(한글 경로 대응) · 메모리 편집(case a2_doc/cad_ri85 · 끝점 짐 · 고정 kp/kv · 중력보상 제거) · 5차 다항식 · 그림 스타일 | – |
| `w06_kpkv_tune.py` | Mᵢᵢ 범위(armature 포함) → R1 1 kg 처짐 ≤ 1° 로 kp 하한 → R3(오버슈트 ≤ 5 %, 정착 ≤ 0.3 s, Mᵢᵢ 최소 · 중앙 · 최대 × 2 case)를 만족하는 최소 kp · 최적 고정 kv | `results/kpkv_tune.md` · .json, `assets/screenshots/week06/week06_kpkv_step.png` |
| `w06_pick_scenario.py` | 박종진 `ik.py`(v1) + 5차 궤적 집기(대기 → 접근 → 수직 하강 → 잡기 → 상승 → 복귀), 짐 0 / 0.5 / 1 / 3 kg, 짐 · 속도 피드포워드, kp×2, 베이스 동시 명령, week05 게인, cad_ri85 / 박종진 최악 자세 뻗기 / 가감속 민감도 / 이론 검산 (`--gif` 로 GIF) | `results/pick_scenario.md` · .json · `pick_log_*.csv`, `week06_pick_tracking.png` · `week06_pick_payload.png` · `week06_pick.gif` |
| `w06_torque_budget.py` | 노션 양식 토크 예산표(정격 · 피크 · 정적 최악 · 집기 · 뻗기 · 여유율 · 판정, 두 case) + Isaac Lab `ImplicitActuatorCfg` 대응표 | `results/torque_budget.md` · .csv · .json, `docs/actuator_params.md` |

실행 (저장소 루트, 순서대로 · 합계 약 1분): `python sim/w06/w06_kpkv_tune.py` → `python sim/w06/w06_pick_scenario.py [--gif]` → `python sim/w06/w06_torque_budget.py`
(Windows 노트북은 `py`. GIF 는 화면 있는 PC 에서 그대로, 리눅스 헤드리스는 `MUJOCO_GL=glfw xvfb-run -a`.) 실행 로그 = `results/run_log.txt`

## 노션 6주차 ↔ 이 저장소

| 노션 (함태훈) | 여기 | 비고 |
|---|---|---|
| 궤적 생성 · 추종 스크립트, 그래프 · 영상 | `w06_pick_scenario.py`, `week06_pick_tracking.png`, `week06_pick.gif` | IK = 박종진 `b1/models/v1/scripts/ik.py` (인터페이스 v1, 사본 `b1_snapshot/v1/scripts/`) |
| 관절별 kp · kv 튜닝 (오버슈트 · 정착시간) | `w06_kpkv_tune.py`, `results/kpkv_tune.md` | 고정 kv(= Isaac damping) |
| 페이로드별 토크 예산표 | `results/torque_budget.md` | 0 / 0.5 / 1.0 kg + 3 kg(PDR 최악) |
| `docs/actuator_params.md` (Isaac Lab) | `sim/w06/docs/actuator_params.md` | 박종진 `b1/models/v1/isaac/b1_v1_cfg.py` 팔 게인을 이 값으로 |
| 팀 연계: 작업공간 「토크 최대 도달 자세」 → 예산표 최악 조건 | 박종진 `results/workspace_max_torque.json` → 정적 최악 열 + 그 자세로 뻗기 동작(동적) 열 | |
| 공통 수: 집기 시나리오 + 페이로드 0.5 · 1.0 kg + 베이스 동시 명령 | `w06_pick_scenario.py` 실행 p05 · p1 · p1_base | |

- `b1_snapshot/` — 박종진 d6d6304(10/27) `b1/models/v1` 의 mjcf · meshes · inputs · actuator_params · ik.py · v1_mjio.py · 결과 json 4개 사본(읽기 전용). `B1_V1` 환경변수로 박종진 저장소 원본을 가리킬 수 있음
- 공용 모델 빌드 재현: 박종진 저장소에서 `python b1/models/v1/scripts/build_v1.py` → 파일 변화 0, `validate_model.py` 5종 통과 (10/28 확인)
- 설명: [../../notes/11_trajectory_tracking_torque_budget.md](../../notes/11_trajectory_tracking_torque_budget.md)
