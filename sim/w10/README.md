# sim/w10 — 함태훈 week10: 토크 결정 자료 · 단일 설정 · 회귀 테스트

| 스크립트 | 하는 일 | 결과 |
|---|---|---|
| `w10_common.py` | 집기 scene(박종진 pick_env, 그리퍼 0.9 kg) 위 왼팔 자세별 토크 성분(중력 · 짐 · 베이스 가속), 값은 `config/params.yaml` | – |
| `w10_torque_judge.py` | (11/20) 팀장 11/19 회신 반영: 실제 정격 ≥ 요구 정격(필요 × 1.2) 판정표 | `results/torque_judge_w10.md` · .csv |
| `w10_torque_decision.py` | **토크 결정 자료**: 여유율 = (정격 − 필요)/필요, case 2 × 짐 0 ~ 3 kg, m_max(선형식 vs 스윕), r_max, A팀 양식 결정표, 그림 | `results/torque_decision_w10.md` · .csv · .json, `reach_limit_w10.json`, `week10_torque_decision.png` |
| `w10_scenario_sweep.py` | 집기 시나리오(동적) 짐 0.2 · 1 · 2 · 3 kg × seed 10개 구동 토크 · 성공 (~8 분) | `results/scenario_sweep.json` · `_run_log.txt` |
| `w10_pipeline_demo.py` | 갱신 파이프라인 시연: 기본 · case 변경 · 팔꿈치 질량 +10 % | `results/pipeline_demo.md` |
| `w10_check_margin.py` | 팀 연계: 박종진 비교표 최소 여유율 검산(h5 · npz 팔 qpos) | 화면 출력 |

- 같이 보는 것: `config/params.yaml`(단일 원본) · `config/params.py`, `tests/`(pytest 15개), `tools/rebuild_and_test.py`(재빌드 + 테스트 한 번에 → `build/regression_summary.md`)
- 실행: 저장소 루트에서 `python sim/w10/<스크립트>.py` (Windows `py`). 박종진 저장소 경로 = `PJJ` 환경변수(기본 `../PJJ0337`, 31f40ad 에서 확인). 필요 패키지: mujoco 3.15.0 · numpy · matplotlib · pyyaml · pytest · trimesh
