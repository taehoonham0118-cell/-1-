# 6주차 기여 로그 — 함태훈 (B1)

- 기간: 2026. 10. 26. (월) ~ 10. 29. (목) 마감 (금: 구술 · 평가 피드백 반영)
- 게이트: **PDR (가중 2배)** — 기획서 통과 기준 「액추에이터 토크 여유 20 % 이상 확인」
- 저장소 태그: `week06` — 10/29 최종 제출 커밋에 찍음. 이후 수정은 week07
- 기준 커리큘럼: 노션 「B1 5·6주차 학습 커리큘럼」 §2 (6주차) — 함태훈 = 궤적 생성 · 추종, kp · kv 튜닝, 페이로드별 토크 예산표, Isaac Lab 액추에이터 파라미터표
- 공용 모델: 박종진 d6d6304 (10/27: A2 사양 case a2_doc · 중력보상 B 기본 on · 메시 visual/collision 분리 · 작업공간 · DLS IK v1). 사본 `sim/w06/b1_snapshot/`, 빌드 재현 확인(파일 변화 0, 검증 5종 통과)
- 학습 노트: [notes/11](../notes/11_trajectory_tracking_torque_budget.md) · 코드: [sim/w06/](../sim/w06/README.md) · 주간 보고서: [reports/학부생_주간활동보고서_6주차_함태훈_22212334_261029.docx](../reports/학부생_주간활동보고서_6주차_함태훈_22212334_261029.docx)

## 1. 이번 주 계획 (커리큘럼 6주차 + 5주차 보고서 다음 주 목표)

| 출처 | 계획 |
|---|---|
| 커리큘럼 개별 | 궤적 생성 · 추종 스크립트, 관절별 kp · kv 튜닝(오버슈트 · 정착시간), 페이로드별 토크 예산표, Isaac Lab 액추에이터 파라미터표(`ImplicitActuatorCfg`) |
| 커리큘럼 팀 연계 | 박종진 작업공간 결과의 「토크가 가장 큰 도달 자세」를 예산표 최악 조건으로 |
| 커리큘럼 공통 수 | 집기 시나리오(대기 → 접근 → 하강 → 상승 → 복귀) IK + 궤적, 페이로드 0.5 · 1.0 kg, 베이스 속도 동시 명령, 추종 오차 · 토크 그래프, GIF |
| 커리큘럼 구술 | ① 페이로드에 따른 추종 오차 증가 이유와 kp 증가 · 중력 보상 피드포워드 장단점 ② 시뮬 최대 토크 < 정격이어도 실제 모터에서 문제가 되는 이유 |
| 5주차 인계(박종진 합의) | A2 armature · forcerange 모델 반영(박종진 빌드) 확인, gravcomp 빌드 반영 확인, convex 분해 비교 |

## 2. 실제 수행 · 산출물

| # | 수행 내용 | 완료율 | 산출물 |
|---|---|---|---|
| 1 | **박종진 6주차 반영분 확인**: `actuator_params.json` a2.case = a2_doc → 어깨 armature 0.219 · forcerange 381.7, 팔꿈치 0.098 · 181.5, gravcomp on, jointactuatorfrc 센서 8개 — 빌드 재현(파일 변화 0) · 검증 5종 통과 · 3 kg pitch 42.918 N·m 내 계산과 일치 | 100 % | `sim/w06/b1_snapshot/`, notes/11 |
| 2 | **kp · kv 튜닝** (armature 반영 모델, 고정 kv = Isaac damping 방식): Mᵢᵢ 범위 5,000 자세(pitch 0.268 ~ 1.068, 3.98배) → R1(1 kg 처짐 ≤ 1°) · R3(OS ≤ 5 %, 정착 ≤ 0.3 s, 3 자세 × 2 case) → pitch 400 · 31.0 / roll 350 · 27.5 / yaw 300 · 15.0 / elbow 300 · 11.0. 최악 OS 1.5 %, 정착 0.288 s (week05 기본 0.362 s 미통과) | 100 % | [w06_kpkv_tune.py](../sim/w06/w06_kpkv_tune.py), [kpkv_tune.md](../sim/w06/results/kpkv_tune.md), [week06_kpkv_step.png](../assets/screenshots/week06/week06_kpkv_step.png) |
| 3 | **집기 시나리오** (박종진 ik.py v1 + 5차 궤적 + 직선 하강 IK 21 점): 짐 0 / 0.5 / 1 / 3 kg, 짐 · 속도 피드포워드, kp×2, 베이스 동시, week05 게인, cad_ri85 — 14 실행. 1 kg 끝점 오차 15.4 mm(잡은 뒤 처짐 5.5 mm), 최고 속도 1.90 rad/s. 이론 검산: 속도 지연 kv·q̇/kp 1.12 · 3.98° = 시뮬 1.09 · 3.94°, 처짐 ‖JK⁻¹Jᵀmg‖ 5.55 = 5.49 mm | 100 % | [w06_pick_scenario.py](../sim/w06/w06_pick_scenario.py), [pick_scenario.md](../sim/w06/results/pick_scenario.md), [tracking](../assets/screenshots/week06/week06_pick_tracking.png) · [payload](../assets/screenshots/week06/week06_pick_payload.png) · [GIF](../assets/screenshots/week06/week06_pick.gif), pick_log_*.csv |
| 4 | **팀 연계 — 최악 조건**: 박종진 `workspace_max_torque.json` 관절별 최악 자세로 실제 뻗기 동작(1.5 s 5차) → 동적 최대 = 정적 + 2.4 % (pitch 3 kg 43.95 N·m). 가감속 민감도 T 2.0 ~ 0.6 s: 43.35 → 57.19 N·m, T ≤ 0.75 s 는 속도 한계 3.0 rad/s 밖 | 100 % | 같은 스크립트, pick_scenario.json `reach` · `sens` |
| 5 | **토크 예산표 (노션 양식, A팀 피드백)**: 두 case × 4관절 × 0 / 0.5 / 1 / 3 kg. 최소 여유율 3 kg a2_doc pitch **170 %** · cad_ri85 **66 %** → 모두 OK. PDR 52 기준은 정적 21.2 % 통과, 동적 포함 시 18.3 %(주의) | 100 % | [w06_torque_budget.py](../sim/w06/w06_torque_budget.py), [torque_budget.md](../sim/w06/results/torque_budget.md) · .csv |
| 6 | **Isaac Lab 액추에이터 파라미터표**: stiffness · damping · effort_limit_sim · velocity_limit_sim · armature · friction, MuJoCo ↔ Isaac 대응 · 주의(수동 감쇠 0.5, actuatorgravcomp 없음, 속도 목표) | 100 % (실습 PC 확인 ⚠ 2건) | [docs/actuator_params.md](../sim/w06/docs/actuator_params.md) |
| 7 | 학습 정리: dampratio vs 고정 kv, 과감쇠 정착, 최소 저크, 속도 지연 · 정적 처짐, 토크 예산 정의, Manager-Based 대응 | 100 % | [notes/11](../notes/11_trajectory_tracking_torque_budget.md) |
| 8 | (5주차 인계) convex 분해 비교 | 0 % | 7주차로 넘김 — 이번 주는 PDR 토크 예산 우선 |

### 실행 환경 · 명령 (재현용)

| 항목 | 값 |
|---|---|
| 계산 | 클라우드 Linux, Python 3.13, mujoco 3.15.0, scipy · matplotlib · imageio |
| 모델 | 박종진 d6d6304 `build_v1.py` 빌드 사본, dt 0.002 s · implicitfast, A2 case a2_doc(기본) · cad_ri85(메모리 편집) |

```bash
pip install mujoco==3.15.0 scipy matplotlib imageio
python sim/w06/w06_kpkv_tune.py          # ~6 s
python sim/w06/w06_pick_scenario.py      # ~7 s (--gif 면 ~40 s)
python sim/w06/w06_torque_budget.py
# 박종진 원본으로: set B1_V1=<박종진 저장소>\b1\models\v1
```

### 핵심 수치 (조건 병기)

| 항목 | 값 | 조건 / 근거 |
|---|---|---|
| week06 게인 | pitch 400 · 31.0 / roll 350 · 27.5 / yaw 300 · 15.0 / elbow 300 · 11.0 | 고정 kv, kpkv_tune.md |
| 계단 0.2 rad 최악 | OS 1.5 · 1.5 · 1.3 · 1.5 %, 정착 0.288 · 0.232 · 0.172 · 0.094 s | Mᵢᵢ 최소 · 중앙 · 최대 × 2 case, 중력보상 B |
| 집기 추종 (1 kg) | 관절 최대 pitch 1.09° · elbow 3.94°, 끝점 15.4 mm, 처짐 5.5 mm | 모르는 짐, 속도 FF 없음 |
| 추종 개선 (1 kg 끝점) | kp×2 처짐 2.8 mm · 짐 FF 0.0 mm · 속도 FF 5.8 mm · 짐 + 속도 FF 2.0 mm | pick_scenario.md |
| 집기 최대 토크 | 0 / 0.5 / 1 / 3 kg pitch 2.32 / 3.62 / 5.14 / 11.55 · elbow 3.36 / 4.71 / 6.05 / 11.34 N·m | 테이블 높이 pick_point |
| 최악 자세 뻗기 | pitch 3 kg 43.95 (정적 42.92, +2.4 %), roll 40.52 (39.77) N·m | 1.5 s 5차, 박종진 최악 자세 |
| 최소 여유율 (3 kg) | a2_doc 170 % · cad_ri85 66 % (정격 대비) / PDR 52 대비 동적 18.3 % | torque_budget.md |
| IK (박종진) | 작업공간 안 8/8, 수렴률 위치만 99.6 % | 박종진 ik_results.md (6주차 핵심 수치) |

## 3. 문제 · 해결 과정

| 증상 | 원인 | 해결 |
|---|---|---|
| 고정 kv 를 ζ = 1(가장 무거운 자세)로 잡으니 pitch 정착 0.43 s | 가벼운 자세(Mᵢᵢ 0.147)가 ζ 2.7 과감쇠 → 느린 극이 정착을 정함 | kv 를 ζ(Mmax) 0.5 ~ 1.2 범위로 탐색, 만족하는 kv 없으면 kp 50 씩 올림 → ζ 0.75 · kp 400 |
| 짐 0 → 3 kg 인데 관절 최대 오차가 거의 그대로 | 오차의 대부분이 짐과 무관한 속도 지연 kv·q̇/kp | 정적 처짐(잡은 뒤 유지 구간)을 따로 기록, 속도 피드포워드 실험 추가 |
| 속도 FF 실행에서 elbow 토크가 23.7 N·m 로 튐 | 토크를 qfrc_actuator 만 기록 — FF 는 qfrc_applied 로 들어감 | 구동 토크 = qfrc_actuator + qfrc_applied 로 수정 → 6.07 N·m |
| 3 kg 유지 토크가 이론(IK 자세 qfrc_bias)보다 10 % 작음 | 16 mm 처지며 자세가 바뀜 | 처진 자세에서 qfrc_bias 다시 계산 → 시뮬과 일치(잡고 2.5 s 뒤 9.00 = 9.00 N·m, 0.3 s 시점은 아직 정착 중 8.84) |
| dampratio 로 짐을 붙이면 kv 가 바뀜 | dampratio 는 컴파일 시 관성(짐 포함)으로 kv 계산 | 게인은 고정 kp · kv 로 지정(`w06_common.edit_b1(gains=…)`) |

## 4. B1 업무와의 연결

- 토크 예산표 = PDR A팀 피드백: 3 kg 까지 실제 정격 대비 OK, 다만 PDR 52 기준은 동적 최악에서 주의 → 동작 시간 ≥ 1.5 s · 관절 속도 ≤ 3 rad/s 조건을 같이 넘김
- `actuator_params.md` = 실습 PC 첫날 박종진 `b1_v1_cfg.py` 팔 게인 교체 값
- 집기 시나리오 = 박종진 IK 모듈 인터페이스 v1 실사용 확인(연속 q_init, 21 점 모두 수렴)

## 5. 구술 대비 (10/29 목) — 상세: PC 6주차_작업/구술_답변_복붙용_6주차.md · notes/11

- Q1: 오차 = 속도 지연 kv·q̇/kp + 모르는 짐 정적 처짐 K⁻¹Jᵀmg (5.55 이론 = 5.49 mm 시뮬). kp×2 → 절반(대역폭 ↑, 백래시 · 잡음 · 접촉 충격 불리), 짐 FF → 0(질량을 알아야 함, 틀리면 잔차)
- Q2: 마찰(모델 0) · 백래시 0.002 rad(끝점 ≈ 1.2 mm) · 열(연속 정격 = RMS, 뻗고 유지하면 계속 토크) · 가감속(1.0 s 에 +7.5 %, 0.6 s 에 +33 %) · 기어 관성 · 효율 누락

## 6. 다음 주 목표 (7주차)

1. 팀장 어깨 구동기 확정 → case 고정 재실행(kp · kv 는 두 case 공통이라 그대로), 5 · 6주차 평가 피드백 반영
2. convex 분해 비교(5주차 인계), 손목 2축 추가 시 토크 예산 · 작업공간 변화 예측(박종진 IK 와)
3. 실습 PC 배정 시 `actuator_params.md` 로 Isaac Sim 임포트 · 검증 ② ③ · 집기 1개 재현

## 6-1. 팀장 회신 · 결정

| 항목 | 내용 | 반영 |
|---|---|---|
| 6주차 진행 (10/27 카톡) | 노션 내용 최대한 진행, 모르는 부분은 빼고 진행. 출장으로 답변 늦을 수 있음 | 노션 6주차 항목 모두 진행 |
| 어깨 구동기 TQ ↔ RI85 (박종진 문의 10/27) | 회신 대기 | a2_doc 기본 + cad_ri85 병기 (두 case 모두 OK) |
| 5주차 평가표 | 미수령 (10/28 기준) | 수령 시 week07 반영 |

## 7. 제출 · 릴리스 기록

| 일시 | 내용 |
|---|---|
| 2026-10-28 | 6주차 작업 · 업로드 |
| 2026-10-29 | 최종 제출 커밋 = `week06` 태그, 보고서 카톡 제출 · 구술 |
