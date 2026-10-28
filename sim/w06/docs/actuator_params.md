# actuator_params.md — B1 v1 팔 관절 ↔ Isaac Lab `ImplicitActuatorCfg` (week06, 함태훈)

실습 PC 배정 첫날 `b1/models/v1/isaac/b1_v1_cfg.py`(박종진 초안)의 팔 게인을 이 표 값으로 바꾼다. 값 근거 = `sim/w06/results/kpkv_tune.md` · `torque_budget.md`, A2 사양 = `inputs/a2_actuators.json`.

## 1. 관절별 값 (case a2_doc 기본, 오른팔 = 같은 값)

| 관절 (`{left,right}_…_joint`) | 기종 | stiffness = kp (N·m/rad) | damping = kv (N·m·s/rad) | effort_limit_sim (N·m) | velocity_limit_sim (rad/s) | armature (kg·m²) | friction | 근거 |
|---|---|---|---|---|---|---|---|---|
| shoulder_pitch | TQ | 400 | 31.0 | 381.7 (피크) | 3.0 | 0.219 | 0.0 | R1 1 kg 처짐 ≤ 1° · R3 계단 OS ≤ 5 % · 정착 ≤ 0.3 s |
| shoulder_roll | TQ | 350 | 27.5 | 381.7 (피크) | 3.0 | 0.219 | 0.0 | R1 1 kg 처짐 ≤ 1° · R3 계단 OS ≤ 5 % · 정착 ≤ 0.3 s |
| shoulder_yaw | TQ | 300 | 15.0 | 381.7 (피크) | 3.0 | 0.219 | 0.0 | R1 1 kg 처짐 ≤ 1° · R3 계단 OS ≤ 5 % · 정착 ≤ 0.3 s |
| elbow | RI85 | 300 | 11.0 | 181.5 (피크) | 3.0 | 0.098 | 0.0 | R1 1 kg 처짐 ≤ 1° · R3 계단 OS ≤ 5 % · 정착 ≤ 0.3 s |

case cad_ri85(8개 RI85)로 확정되면: 어깨 3관절 effort_limit_sim 181.5 · armature 0.098, **kp · kv 는 그대로**(kv 탐색에 두 case 의 Mᵢᵢ 범위를 모두 넣었음 → `kpkv_tune.md` 3절).

## 2. MuJoCo ↔ Isaac Lab 대응 · 주의

| MuJoCo (b1.xml) | Isaac Lab | 비고 |
|---|---|---|
| `<position kp>` | `stiffness` | 같은 단위(N·m/rad). Isaac Lab 은 rad 기준 값을 PhysX 로 변환 |
| `<position kv>` (week06 고정값) | `damping` | week05 는 dampratio 1(컴파일 시 qpos0 Mᵢᵢ 로 계산) → Isaac 에는 그런 옵션이 없어 고정 kv 로 튜닝함 |
| joint `damping 0.5` (수동 감쇠) | `damping` 에 더하거나 joint 속성 | ⚠ PhysX 관절에는 드라이브와 별도의 점성 감쇠가 없음 → 0.5 를 damping 에 더해 쓰는 것이 같은 효과(실습 PC 에서 확인) |
| `forcerange` · `actuatorfrcrange` (피크) | `effort_limit_sim` | 정격(TQ 118.8 · RI85 72.6)은 토크 예산 판정용, 시뮬 상한은 피크 |
| URDF `velocity 3.0` | `velocity_limit_sim` | TQ 무부하 star-serial 3.5 rad/s 보다 작음(A2). 집기 · 뻗기 동작 최고 1.96 rad/s |
| joint `armature` | `armature` | = J_rotor × i² (로터만, 기어열 관성 TBD) |
| `frictionloss 0` | `friction` | A2 마찰값 받으면 갱신 |
| 중력보상 B (body `gravcomp=1` + `actuatorgravcomp`) | **없음** → 피드포워드 | ⚠ Isaac Lab 에는 actuatorgravcomp 이 없음. `disable_gravity=True` 는 토크 예산에 중력이 빠짐 → 관절 중력 항을 effort 피드포워드로 넣는 방식 권장(`b1_v1_cfg.py` 도 disable_gravity=False). 실습 PC 에서 API 확인 |
| position 목표 + 속도 피드포워드(`qfrc_applied = kv·q̇_ref`) | joint position target + **velocity target** | 속도 목표를 같이 주면 kv 지연(kv·q̇/kp)이 사라짐: 집기 1 kg 끝점 오차 15.4 → 5.8 mm(짐 FF 까지 2.0 mm) |

## 3. 베이스 (참고, 박종진 값 유지)

| 관절 | stiffness | damping | effort_limit_sim |
|---|---|---|---|
| base_x · base_y | 0 | 200 | 2000 |
| base_yaw | 0 | 50 | 500 |

