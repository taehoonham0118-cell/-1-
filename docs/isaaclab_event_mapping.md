# 도메인 랜덤화 → Isaac Lab 매핑표 (week09, 함태훈)

- 입력: `sim/w09/randomize.py` 의 파라미터 17개(물리 14 · 수치 3), 범위 근거 = 가정값 표 v0.4 (박종진 `b1/models/assumptions-v0.md`)
- 대상: Isaac Lab **Manager-Based** 환경의 `EventCfg`(랜덤화) — 박종진 `isaaclab_env_spec.md` 의 Event 절에 이 표를 그대로 넣는다
- 함수 이름 · 인자 = Isaac Lab `isaaclab.envs.mdp.events` 소스 확인(2026-11-18, main 9d9eff5). 2.x 와 이름 · 인자 같음. 실습 PC 의 설치 버전(2.3.2 예정)에서 다시 확인
- mode: `startup` = 환경 만들 때 1번(env 마다 다른 값, 4096 env 면 분포가 env 사이에 퍼짐) · `reset` = 에피소드마다 · `interval` = 일정 시간마다. 질량 · 재질은 PhysX 에서 `startup` 권장(소스 주석: CPU 텐서로 씀)

| MuJoCo 파라미터 (randomize.py) | 공칭 → 범위 (×1) | 민감도 (×1 / ×2 성공) | Isaac Lab 대응 | mode | 비고 |
|---|---|---|---|---|---|
| `table_dz` 테이블 높이 오차 | 0 → ±10 mm | 40 / **0** (+20 mm) · **Top 1** | `mdp.reset_root_state_uniform`(asset = 테이블, `pose_range={"z": (-0.01, 0.01)}`) + 물체도 같은 z 만큼 (사용자 정의 EventTerm 으로 묶음) | reset | 정책 하강 높이는 공칭 0.70 m 기준이어야 의미가 있음 → 관측에 테이블 높이를 넣지 않음 |
| `mu_pad_obj` 손가락 · 물체 마찰 | 1.0 → 0.5 ~ 1.2 | 39 / **15** (μ 0.1) · **Top 2** | `mdp.randomize_rigid_body_material`(`static_friction_range`, `dynamic_friction_range`, `restitution_range=(0,0)`, `num_buckets=64`, asset = 손가락 링크 + 물체) | startup | **결합 규칙 차이**: MuJoCo 기본 = 두 geom 중 max, PhysX 기본 = average → 손가락 · 물체 둘 다 같은 범위로 바꾸거나 combine mode 를 max 로 맞춤 |
| `obj_bias` 물체 위치 측정 편향 | 0 → 0 ~ 10 mm | 39 / 38 (20 mm), 목표 오차 +2.12 mm · **Top 3** | Event 아님 → `ObservationTermCfg(noise=NoiseModelWithAdditiveBiasCfg(bias_noise_cfg=UniformNoiseCfg(-0.01, 0.01)))` (bias 는 reset 마다 뽑음) | reset(편향) | 카메라 미정. 지금 정책은 참값 관측 |
| `grip_force` 손가락당 힘 상한 | 40 → 28 ~ 48 N | 40 / 39 (16 N) | 내장 Event 없음 → 사용자 정의 EventTerm: `asset.write_joint_effort_limit_to_sim(...)` (손가락 관절) | startup | A팀 그리퍼 사양 수령 시 범위 축소 |
| `finger_mass` 손가락 질량 | 0.2 → 0.1 ~ 0.3 kg | 40 / 40 | `mdp.randomize_rigid_body_mass`(손가락, `operation="abs"`, `recompute_inertia=True`) | startup | |
| `obj_mass_x` 물체 질량 배율 | ×1 → 0.8 ~ 1.2 | 40 / 39 (×1.4) | `mdp.randomize_rigid_body_mass`(물체, `operation="scale"`) — 과제 질량 0.2 ~ 3 kg 자체는 `abs`·`log_uniform` | startup | 물체 1개 = env 하나에 고정, 다양성은 env 수로 |
| `arm_mass_x` 팔 링크 질량 | ×1 → 0.9 ~ 1.1 | 40 / 40, 토크 pitch 25.1 ~ 29.2 N·m | `mdp.randomize_rigid_body_mass`(팔 링크, `operation="scale"`) | startup | MuJoCo 쪽은 중력보상 = 공칭 질량(gravcomp 1/s)으로 모델 오차를 재현. Isaac 은 중력보상을 제어기(공칭 모델)에서 계산하면 같은 효과 |
| `base_mass_x` 하체 · 몸통 질량 | ×1 → 0.8 ~ 1.2 | 40 / 40, 목표 오차 +1.09 mm (×0.8) | `mdp.randomize_rigid_body_mass`(base · torso, `scale`) | startup | |
| `armature_x` 반사 관성 | ×1 → 0.8 ~ 1.5 | 40 / 40 | `mdp.randomize_joint_parameters`(`armature_distribution_params=(0.8,1.5)`, `operation="scale"`) | startup | |
| `frictionloss` 관절 쿨롱 마찰 | 0 → 0 ~ 2 N·m | 40 / 39 (4 N·m) | `mdp.randomize_joint_parameters`(`friction_distribution_params=(0,2)`, `operation="abs"`) | startup | PhysX joint friction 은 무차원 계수인 버전이 있음 → 실습 PC 에서 단위 확인 |
| `damping_x` 관절 점성 감쇠 | ×1 → 0.5 ~ 2 | 40 / 40 | `mdp.randomize_actuator_gains`(`damping_distribution_params`, `scale`) — PhysX 관절에는 수동 감쇠가 따로 없어 drive damping 에 합침 | startup | |
| `kp_x` 토크 상수(kp · kv 함께) | ×1 → 0.9 ~ 1.1 | 40 / 40 | `mdp.randomize_actuator_gains`(`stiffness_distribution_params=(0.9,1.1)`, `damping_distribution_params=(0.9,1.1)`, `operation="scale"`) | startup | 실제로는 Kt 오차 = 두 게인 같이 → 같은 배율을 쓰려면 사용자 정의 |
| `base_kv_x` 베이스 속도 게인 | ×1 → 0.8 ~ 1.2 | 40 / 38 (×0.6) | `mdp.randomize_actuator_gains`(베이스 관절 3개, `damping_distribution_params`, `scale`) | startup | 베이스 = Isaac 사본 prismatic · prismatic · revolute(velocity drive) |
| `latency` 명령 지연 | 0 → 0 ~ 20 ms | 40 / 40 (40 ms) | Event 아님 → `DelayedPDActuatorCfg(min_delay=0, max_delay=k)`(물리 스텝 단위, k = 20 ms / dt) | – | ArticulationCfg actuators 에서 지정 |
| 물체 초기 위치 · yaw (과제) | S17 깊이 0.06 ~ 0.22 m · y 0 ~ 0.35 · yaw ±45° | (과제 분포) | `mdp.reset_root_state_uniform`(물체, `pose_range={"x":…, "y":…, "yaw":…}`) | reset | `pick_env.sample_task` 와 같은 분포 |
| `dt` 시뮬 간격 | 0.5 ms → 0.25 ~ 1 ms | 40 / 40 | `SimulationCfg(dt=…)`, `decimation` = 제어 주기 / dt | – | 랜덤화 대상 아님(엔진 설정). MuJoCo 1 ms 발산(8주차)은 PhysX 에서 따로 확인 |
| `solref_tc` 접촉 시간상수 | 0.01 → 0.005 ~ 0.02 | 40 / 38 (0.003) | 대응 없음 — PhysX 는 강체 접촉 + `contact_offset` · `rest_offset`, 필요 시 compliant contact(`compliant_contact_stiffness` · `damping`) | – | MuJoCo 전용 수치 설정 |
| `impratio` 마찰 / 법선 임피던스 비 | 10 → 3 ~ 10 | 40 / 39 (1) | 대응 없음 — PhysX `solver_position_iteration_count` · `friction_type` | – | 〃 |

## 쓰는 법 (예시 — 박종진 env spec 의 EventCfg 에 들어갈 모양)

```python
@configclass
class EventCfg:
    # Top 1 ~ 3 (민감도 순) — 범위 = 가정값 표 v0.4 근거 ×1
    table_height = EventTerm(func=b1_events.reset_table_and_object_z, mode="reset",
                             params={"dz_range": (-0.01, 0.01)})            # 사용자 정의(테이블 + 물체 같이)
    pad_friction = EventTerm(func=mdp.randomize_rigid_body_material, mode="startup",
                             params={"asset_cfg": SceneEntityCfg("robot", body_names=["left_finger_.*"]),
                                     "static_friction_range": (0.5, 1.2), "dynamic_friction_range": (0.5, 1.2),
                                     "restitution_range": (0.0, 0.0), "num_buckets": 64})
    obj_friction = EventTerm(func=mdp.randomize_rigid_body_material, mode="startup",
                             params={"asset_cfg": SceneEntityCfg("object"), "static_friction_range": (0.5, 1.2),
                                     "dynamic_friction_range": (0.5, 1.2), "restitution_range": (0.0, 0.0), "num_buckets": 64})
    # 나머지
    arm_mass = EventTerm(func=mdp.randomize_rigid_body_mass, mode="startup",
                         params={"asset_cfg": SceneEntityCfg("robot", body_names=["left_shoulder_.*", "left_elbow_link"]),
                                 "mass_distribution_params": (0.9, 1.1), "operation": "scale", "recompute_inertia": True})
    joint_armature = EventTerm(func=mdp.randomize_joint_parameters, mode="startup",
                               params={"asset_cfg": SceneEntityCfg("robot", joint_names=["left_.*_joint"]),
                                       "armature_distribution_params": (0.8, 1.5), "operation": "scale"})
    joint_friction = EventTerm(func=mdp.randomize_joint_parameters, mode="startup",   # 한 항에 operation 하나 → 둘로 나눔
                               params={"asset_cfg": SceneEntityCfg("robot", joint_names=["left_.*_joint"]),
                                       "friction_distribution_params": (0.0, 2.0), "operation": "abs"})
    gains = EventTerm(func=mdp.randomize_actuator_gains, mode="startup",
                      params={"asset_cfg": SceneEntityCfg("robot", joint_names=["left_shoulder_.*", "left_elbow_joint"]),
                              "stiffness_distribution_params": (0.9, 1.1), "damping_distribution_params": (0.9, 1.1), "operation": "scale"})
    object_pose = EventTerm(func=mdp.reset_root_state_uniform, mode="reset",
                            params={"asset_cfg": SceneEntityCfg("object"), "velocity_range": {},
                                    "pose_range": {"x": (0.06, 0.22), "y": (0.0, 0.35), "yaw": (-0.785, 0.785)}})
```

- 함수 위치: `isaaclab.envs.mdp` (= `import isaaclab.envs.mdp as mdp`). 사용자 정의 함수 2개(`reset_table_and_object_z`, 손가락 effort limit)는 `func(env, env_ids, **params)` 형태로 작성
- 확인 소스: Isaac Lab `source/isaaclab/isaaclab/envs/mdp/events.py`(randomize_rigid_body_material · randomize_rigid_body_mass · randomize_joint_parameters · randomize_actuator_gains · reset_root_state_uniform), `actuators/actuator_pd_cfg.py`(DelayedPDActuatorCfg), `utils/noise/noise_cfg.py`(NoiseModelWithAdditiveBiasCfg)

## 박종진 `b1/docs/isaaclab_env_spec.md` v0.1 §6 「함태훈 randomize.py」 열에 넣을 값 (99d60da 기준 행 순서)

| §6 행 (박종진) | 함태훈 randomize.py 열 |
|---|---|
| `mj_resetData` + 팔 영점 · 그리퍼 벌림 | — (공칭 그대로) |
| 과제 물체 종류 | — |
| 물체 위치 · yaw | — (과제 분포 S17 그대로) + **물체 위치 측정 편향 0 ~ 10 mm**는 Event 아님 → obs `NoiseModelWithAdditiveBiasCfg` (Top 3, 25 mm 에서 31/40) |
| 물체 질량 0.2 ~ 3 kg | 과제 질량 × **0.8 ~ 1.2** (`obj_mass_x`, ×2 에서 39/40) |
| 물체 관성 | 질량 비례(균일 밀도) 그대로 |
| 마찰 ×0.7 ~ 1.3 (손가락 · 물체 · 테이블 각각) | 손가락 · 물체 **같이 μ 0.5 ~ 1.2** (`mu_pad_obj`, Top 2, 경계 m ≤ 2μF/g). MuJoCo 결합 = max 라 테이블 μ 는 물체 μ 보다 작으면 안 쓰임 → PhysX 에선 combine mode max 로 맞추거나 셋 같은 범위 |
| 파지력 30 ~ 50 N | **28 ~ 48 N** (`grip_force`, ×2 16 N 에서 39/40) |
| 팔 armature ×0.8 ~ 1.2, damping ×0.5 ~ 2 | armature **×0.8 ~ 1.5**(기어열 미포함 → 위쪽 넓게), damping ×0.5 ~ 2, **쿨롱 마찰 0 ~ 2 N·m** 추가(`randomize_joint_parameters(friction_distribution_params=(0,2), operation="abs")`) |
| 팔 kp ×0.9 ~ 1.1 | kp · **kv 같이** ×0.9 ~ 1.1 (토크 상수 오차) |
| 베이스 하체 질량 ×0.85 ~ 1.15 | **하체 · 상판 · 몸통** ×0.8 ~ 1.2 (`base_mass_x`) + **팔 링크 질량 ×0.9 ~ 1.1** 추가(중력보상은 공칭 질량 기준) |
| 베이스 kv ×0.8 ~ 1.2 | 같음 (×2 0.6 에서 38/40) |
| (센서 노이즈 · 제어 지연) | 편향 0 ~ 10 mm(위), **지연 0 ~ 20 ms** → `DelayedPDActuatorCfg(min_delay=0, max_delay=40)`(dt 0.5 ms 기준 스텝) |
| (추가 행 제안) 테이블 높이 | **±10 mm**, `reset_root_state_uniform`(테이블) + 물체 z 같이 — **Top 1** (+12 mm 15/40 · +14 mm 0/40) |
| (추가 행 제안) 손가락 질량 | 0.1 ~ 0.3 kg (`randomize_rigid_body_mass`, abs) |

- MuJoCo 쪽 같은 표본: `sim/w09/randomize.py` `apply(m, seed)` = 박종진 `rand_dyn.apply` 와 같은 모양(모델 값만), 테이블 · 편향 · 지연 · 물체 질량은 `sim/w09/bc_eval_pjj.py` 감싸기
