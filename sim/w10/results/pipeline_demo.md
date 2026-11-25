# 모델 갱신 파이프라인 시연 (자동 생성 — w10_pipeline_demo.py)

명령 한 번: `python tools/rebuild_and_test.py --params <yaml>` = 박종진 build_v1.py 재빌드(MJCF 생성) + pytest 15개(검증 5종 · 토크 10)

| 시연 | 바꾼 것 | b1.xml | pytest | 걸린 테스트 |
|---|---|---|---|---|
| A_default | 없음 (params.yaml 그대로, a2_doc) | b4a314ad5c (저장소 원본과 같음) | 15 passed in 1.82s | — |
| B_case_cad_ri85 | params.yaml `shoulder_case: cad_ri85` 한 줄 | 60b145857c (저장소 원본과 다름 — case/입력 변경) | 15 passed in 1.89s | — |
| C_elbow_mass_+10pct | 팔꿈치 링크 질량 2.268 → 2.494 kg (+10 %, 관성 ×1.1) | 8f6dc374e3 (저장소 원본과 다름 — case/입력 변경) | 5 failed, 10 passed in 1.86s | test_torque.py::test_need_regression, test_torque.py::test_mmax_regression_and_linear[TQ-a2_doc], test_torque.py::test_mmax_regression_and_linear[RI85-cad_ri85], test_validation5.py::test_1_mass, test_validation5.py::test_3_gravity_torque |

- B: case 를 바꾸면 MJCF 의 어깨 forcerange · armature 가 RI85 값으로 바뀌고(md5 다름), 토크 판정은 정격 72.6 으로 다시 계산돼도 3 kg 에서 통과(최소 여유 49.1 %)
- C: 질량이 바뀌면 ① 질량 합계 회귀 · ③ 중력 토크 · 필요 토크 · m_max 회귀가 실패 → 「바뀐 숫자」를 바로 알려 줌. 의도한 변경(A팀 새 BOM)이면 regression 값을 새로 고정하고 로그에 이유를 적는다
- 원본 저장소(박종진)는 건드리지 않고 build_demo/ 사본에서만 빌드
