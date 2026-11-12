# 접촉 파라미터 분석 · seed 1080 발산 원인 (week08, 함태훈) — 자동 생성 `w08_contact_report.py`

대상: 박종진 집기 시나리오 `scenarios/`(PJJ0337/-1- week08 = da55741), 임시 그리퍼 · 접촉 값 그대로에서 **하나씩만** 바꿈. 기준 = solref (0.01, 1) · solimp (0.95, 0.99, 0.001) · condim 4 · μ 1.0 · elliptic · impratio 10 · dt 1 ms · 손가락 0.2 kg · 파지력 40 N

## 1. seed 1080 재현 · 경과 (`w08_seed1080_diag.py`, 그림 `assets/screenshots/week08/week08_seed1080_diag.png`)

- 재현됨: dt 1 ms 에서 11.48 s BADQVEL (박종진 보고와 같음). GRASP 10.1 s 에 손가락 4점 접촉 · 관통 0.28 mm · 법선력 21 N(손가락당 상한 40 N 의 절반 — 4점에 나뉨)으로 정상
- LIFT 시작 0.2 s 뒤(10.86 s, 물체가 테이블에서 떨어지는 순간)부터 **물체 각속도가 0.1 → 1 → 17 rad/s 로 지수적으로 커지고** 관통이 0.3 → 1.9 mm 로 튀며 접촉 수가 4 → 2 → 1 로 깜빡임 → 발산. 미끄러짐(물체가 내려감)이 아니라 **수치 진동이 에너지를 키운 것**
- 같은 seed · 같은 값에서 dt 만 0.5 ms 로 줄이면 각속도 ≤ 0.13 rad/s · 관통 0.28 mm 그대로 유지 → 성공(목표 오차 정상)

## 2. 무거운 물체(≥ 2 kg) 38건 × 15설정 (`w08_contact_sweep.py`)

| 설정 (기준에서 하나만 바꿈) | 성공 | 원기둥 | 실패 분류 | 성공 시 관통 중앙 · 최대 mm | seed 1080 |
|---|---|---|---|---|---|
| 기준(박종진 임시값) | 36/38 | 21/22 | sim_unstable 1, ik_fail 1 | 0.28 · 0.45 | sim_unstable |
| solref 0.005 | 26/38 | 17/22 | sim_unstable 11, ik_fail 1 | 0.08 · 0.15 | sim_unstable |
| solref 0.02 | 34/38 | 20/22 | drop 2, sim_unstable 1, ik_fail 1 | 0.59 · 0.76 | sim_unstable |
| solref 0.04 | 33/38 | 20/22 | drop 3, sim_unstable 1, ik_fail 1 | 0.99 · 1.58 | sim_unstable |
| dampratio 0.7 | 35/38 | 21/22 | drop 2, ik_fail 1 | 0.15 · 0.27 | 성공 |
| dampratio 2 | 36/38 | 21/22 | sim_unstable 1, ik_fail 1 | 0.58 · 0.77 | sim_unstable |
| solimp width 5 mm | 35/38 | 20/22 | sim_unstable 1, ik_fail 1, tipped 1 | 0.31 · 0.62 | sim_unstable |
| solimp 0.9·0.95 | 35/38 | 21/22 | sim_unstable 2, ik_fail 1 | 0.47 · 0.71 | sim_unstable |
| solimp 0.99·0.999 | 28/38 | 20/22 | sim_unstable 5, drop 3, ik_fail 1, grasp_miss 1 | 0.07 · 0.55 | sim_unstable |
| impratio 1 | 24/38 | 16/22 | sim_unstable 12, tipped 1, ik_fail 1 | 0.28 · 0.63 | sim_unstable |
| impratio 3 | 33/38 | 21/22 | sim_unstable 4, ik_fail 1 | 0.28 · 0.65 | sim_unstable |
| condim 3 | 36/38 | 21/22 | sim_unstable 1, ik_fail 1 | 0.28 · 0.47 | sim_unstable |
| condim 6 | 36/38 | 21/22 | sim_unstable 1, ik_fail 1 | 0.28 · 0.45 | sim_unstable |
| dt 0.5 ms | 37/38 | 22/22 | ik_fail 1 | 0.28 · 0.54 | 성공 |
| solref 0.02 + width 5 mm | 33/38 | 21/22 | sim_unstable 3, ik_fail 1, drop 1 | 1.13 · 2.68 | sim_unstable |

- ik_fail 1건(seed 1146 상자 2.27 kg, 계획 단계 IK 실패)은 접촉과 무관 — 모든 설정에서 같음

## 3. 짐 3.0 kg 고정 20회 (seed 1000 ~ 1019, 최악 질량) (`w08_contact_3kg.py`)

| 설정 | 성공 | 원기둥 | 상자 | 실패 분류 | 계산 시간 s/에피소드 |
|---|---|---|---|---|---|
| 기준(박종진 임시값) | 9/20 | 2/12 | 7/8 | drop 2, sim_unstable 9 | 0.91 |
| dt 0.5 ms | 19/20 | 12/12 | 7/8 | drop 1 | 1.74 |
| dampratio 0.7 | 10/20 | 5/12 | 5/8 | drop 2, sim_unstable 7, slip 1 | 0.97 |
| dt 0.5 ms + dampratio 0.7 | 17/20 | 11/12 | 6/8 | drop 1, sim_unstable 2 | 1.73 |

## 4. 손가락 관통 · 미끄러짐 격자 (박종진 `gripper_slip.trial` 장치, 원기둥 3 kg · μ 1.0 · F 40 N, 들기 0.10 m + 2 s 유지) (`w08_contact_grid.py`)

| solref 시간상수 \ dampratio | 0.7 | 1.0 | 2.0 |
|---|---|---|---|
| 0.005 | 0.03 mm · 유지 중 0.43 mm | 0.06 mm · 유지 중 0.43 mm | 0.22 mm · 유지 중 0.40 mm |
| 0.01 | 0.11 mm · 유지 중 0.85 mm | 0.22 mm · 유지 중 0.80 mm | 0.52 mm · 유지 중 0.50 mm |
| 0.02 | 0.36 mm · 유지 중 1.40 mm | 0.52 mm · 유지 중 1.03 mm | 0.84 mm · 유지 중 0.48 mm |
| 0.04 | 0.66 mm · 유지 중 1.51 mm | 0.84 mm · 유지 중 1.05 mm | 2.77 mm · 유지 중 0.92 mm |

| solimp (dmin, dmax, width) — solref (0.01, 1) | 관통 mm | 유지 중 미끄러짐 mm |
|---|---|---|
| (0.9, 0.95, 0.001) | 0.39 | 1.43 |
| (0.95, 0.99, 0.001) | 0.22 | 0.80 |
| (0.99, 0.999, 0.001) | 0.04 | 0.20 |
| (0.95, 0.99, 0.005) | 0.23 | 0.86 |

- 96조합 모두 유지(낙하 0). 손가락 단독 장치에서는 dt 1 · 0.5 ms 관통이 같음(최대 차 0.00 mm) → 정적 관통은 solref · solimp 만이 정하고, 발산은 **팔 · 베이스와 묶인 전체 계의 적분 문제**
- 이론 검산(MuJoCo 정상상태): 관통 r ≈ (1 − d)/d · a₀ / k, k = 1/(d_max² · tc² · ζ²) → **r ∝ tc² · ζ²**, d 가 클수록(solimp 1 에 가까울수록) 작아짐. 표: tc 0.01 → 0.02 에서 0.22 → 0.52 mm(×2.4, width 1 mm 안에서 d 가 같이 변해 ×4 보다 작음), ζ 1 → 2 에서 0.22 → 0.52 mm, dmin · dmax 0.95 · 0.99 → 0.99 · 0.999 에서 0.22 → 0.04 mm

## 5. 원인 표 (seed 1080 · 3 kg 원기둥 발산)

| 후보 원인 | 바꾼 값 | 결과 | 판정 |
|---|---|---|---|
| 접촉이 너무 부드러워 관통 → 튕김 | solref 0.02 · 0.04, width 5 mm | 관통 2 ~ 4배, 낙하 2 ~ 3건 늘고 1080 여전히 발산 | 원인 아님(악화) |
| 접촉이 너무 딱딱해 강성 과대 | solref 0.005, solimp 0.99 · 0.999 | 발산 1 → 11 · 5건 | **방향은 맞음**(강성 ↑ → 발산 ↑) |
| 감쇠 부족 | dampratio 2 | 변화 없음(1080 발산) | 원인 아님 |
| 감쇠 과대 | dampratio 0.7 | 1080 성공, 3 kg 20회는 9 → 10/20 | 우연(일부만) |
| 마찰 원뿔 · 비틀림 마찰 | impratio 1 · 3, condim 3 · 6 | impratio ↓ 발산 12 · 4건, condim 영향 0 | 원인 아님(elliptic + impratio 10 유지) |
| **적분 간격 대비 접촉 강성** | **dt 1 → 0.5 ms** | **38건 37/38(ik_fail 1), 3 kg 20회 9 → 19/20(원기둥 2 → 12/12), seed 1000 ~ 1099 100/100** | **주원인** |

- 해석: 3 kg 물체가 0.2 kg 손가락 두 개(질량비 15) 사이에 끼고, 손가락은 equality(대칭) · 서보 kp 3000 · 팔 · 베이스와 직렬로 묶임. 물체가 테이블에서 떨어지는 순간 지지가 손가락 접촉만 남고, 이 접촉 강성(tc 10 ms) · 마찰 강성(impratio 10)과 가벼운 손가락이 만드는 빠른 모드를 1 ms 반암시적 적분(implicitfast 는 접촉을 암시적으로 다루지 않음)이 감쇠시키지 못해 진동이 커짐. 박종진이 한 조치(손가락 0.05 → 0.2 kg, 닫는 속도 제한, dt 2 → 1 ms)는 모두 이 모드를 느리게 · 작게 만드는 쪽이라 같은 원인을 줄인 것
- 비용: 계산 시간 ×1.9 (100 에피소드 수집 ~100 s → ~3 분)

## 6. B1 그리퍼 · 접촉 모델 v1.0 (박종진 `pick_env.GRIPPER · CONTACT · OPTION` 교체용)

| 항목 | v1.0 값 | 근거 · 상태 |
|---|---|---|
| 손가락 | stroke 55 mm/손가락, box 16 × 30 × 60 mm, **0.2 kg**, kp 3000, damping 8, armature 0.01 | 박종진 임시값 유지(가정값). 질량 0.2 kg 미만은 발산 쪽 |
| 파지력 | 손가락당 40 N (forcerange) | 3 kg · μ 1.0 이론 최소 14.7 N 의 2.7배, μ 0.6 이면 24.5 N 의 1.6배 — 그리퍼 사양 오면 교체 |
| 마찰 | 손가락 · 물체 1.0 (비틀림 0.005, 구름 0.0001), 테이블 0.8 | 가정값. 9주차 랜덤화 범위 0.6 ~ 1.0 후보 |
| condim | 4 | 3 · 6 과 결과 같음, 비틀림 마찰로 물체 회전 억제 |
| solref | (0.01, 1.0) | 관통 0.22 ~ 0.28 mm, 0.005 는 발산 ↑, 0.02 이상은 관통 · 낙하 ↑ |
| solimp | (0.95, 0.99, 0.001) | 0.99 · 0.999 는 발산 ↑, 0.9 · 0.95 는 관통 ×1.8 |
| option | **timestep 0.0005**, cone elliptic, impratio 10, multiccd | **변경점은 dt 하나** → `expert.NSUB` 20 → 40 (제어 50 Hz 유지) |

- 확인: 박종진 수집 seed 1000 ~ 1099 를 v1.0(dt 0.5 ms)으로 다시 실행 → **100/100 성공**(기준 99/100, seed 1080 포함)
