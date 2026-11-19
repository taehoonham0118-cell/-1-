# 10주차 기여 로그 — 함태훈 (B1) · 작성 중

- 기간: 2026. 11. 23. (월) ~ 11. 26. (목) 마감 — 커리큘럼 10주차는 노션 확인 후 채움
- 이전 주 태그: `week09` = **ed160f2** (11/18)
- 박종진 week09 = **4d81c70** (11/18, 함태훈 ed160f2 반영: BC × 함태훈 랜덤화 재현 200/200 일치(함태훈 대조 확인), env spec v0.2 §6, expert.py 주석 16 mm, 가정값 표 v0.5 마찰 max)

## 1. 수행 기록

| # | 날짜 | 수행 내용 | 산출물 |
|---|---|---|---|
| 1 | 11/19 | **데이터 명세 v1.1** (박종진 요청, DART 재수집 전): 선택 필드 `action_label`(T, 8) f64 = 학습 라벨(전문가 명령), 선택 속성 `dart`(action_label 있으면 필수) · `randomization`(랜덤화 수집 시 p · source). 재생 기준은 `action` 그대로, spec 문자열 `b1-dataset-1.0` 유지. `check` 에 선택 필드 검사 · `train_label(f)` 추가 → 샘플 3개 통과, 박종진 `collect_dart.py`(99d60da) 실제 파일 1개 통과, 잘못된 예(f32 · 그리퍼 0.09 · dart 없음) 3건 검출 | [docs/dataset_spec.md](../docs/dataset_spec.md) 7절, `sim/w08/b1_dataset.py` |
| 2 | 11/19 | 박종진 재현 대조: 박종진 `eval_taehoon_repro.json`(mujoco 3.14 · torch) vs 함태훈 `bc_eval_pjj.json`(3.15 · numpy) — 4조건 × 50 seed 성공/실패 **200/200 일치** 확인 | 박종진 4d81c70 `policies/results/eval_taehoon.md` |
| 3 | 11/20 | **팀장 11/19 회신 반영 — 토크 판정 하나로**: 「PDR 52 N·m 기준 여유 %」 삭제, 판정 = 실제 정격 ≥ 요구 정격(필요 × 1.2). 8주차 필요 토크 그대로 → 어깨 요구 정격 pitch **58.43** · roll **56.05** N·m, a2_doc(TQ 118.8) · cad_ri85(RI85 72.6) **둘 다 OK**(가장 빠듯 = RI85 pitch, 요구 대비 +24.3 %, 정격/필요 49.1 % = 박종진 torque_budget_b1 과 같음). 8주차 「52 N·m → 최대 짐 2.14 kg」 대신 실제 정격 기준 최대 짐: RI85 pitch **4.88 kg** · TQ pitch **11.04 kg**(3,000 자세 스윕 = 손 검산 4.89 · 11.04), 3 kg 필요 토크 재계산 48.69 · 46.71 = 8주차 일치 | `sim/w10/w10_torque_judge.py`, `sim/w10/results/torque_judge_w10.md` · `.csv` |
| 4 | 11/20 | 시스템 식별 계획서 v0.2: 52 N·m 표현 2곳을 요구 정격 기준으로 정정 | `docs/sysid_plan.md` |

## 6-1. 팀장 회신 · 결정 (11/19)

| 항목 | 내용 | 반영 |
|---|---|---|
| A팀 회신안 · 52 N·m 기준 | 52 는 구동기 사양이 아니라 6주차 필요 42.9 × 1.2. 필요 48.7 → 요구 58.4, 실제 정격으로 판정 → TQ · RI85 둘 다 통과, 짐 3 kg 그대로. 「PDR 52 여유 %」 열 삭제. A팀 회신(7 · 8주차 합본)은 팀장이 발송 | 함태훈 `torque_judge_w10.md`, 박종진 torque_budget_b1(31f40ad · 3c5676c) |
| 어깨 구동기 | 미확정 → A2 사양서 case(a2_doc) 계속, 새 기준으로 두 case 모두 통과 | 유지 |
| 손목 2축 예측 | 모델 4-DOF 유지(A1). CDR 설계 제안 근거로, 토크 여유는 실제 정격(TQ / RI85) 기준 — 박종진 담당 | 박종진 31f40ad `b1/docs/cdr/wrist_2axis_proposal.md` |
