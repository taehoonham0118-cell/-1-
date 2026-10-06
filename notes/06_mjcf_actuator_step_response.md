# MJCF 구동(actuator) · 계단 응답 실험 — kp · kv · armature (week03, 함태훈 · B1)

- 모델: [sim/mjcf/scene.xml](../sim/mjcf/scene.xml) (= [two_link_arm_v2.xml](../sim/mjcf/two_link_arm_v2.xml) include)
- 스크립트: [sim/scripts/step_response.py](../sim/scripts/step_response.py) → [step_results.csv](../sim/scripts/step_results.csv), 그래프 3장 `assets/screenshots/week03/week03_step_{kp,kv,armature}.png`
- 근거: MuJoCo XML Reference `actuator/position`, `joint/armature`, `option/integrator`; Isaac Sim 5.1 Articulation/Drive 문서

## 1. MuJoCo 액추에이터 3종 (전부 `<general>` 의 축약형)

| 태그 | 힘 식 (관절 토크) | 내부 표현 | 언제 쓰나 |
|---|---|---|---|
| `<motor gear="g">` | τ = g · ctrl | gain = g, bias = 0 | 토크 제어 (B2 저수준 제어, WBC 출력) |
| `<position kp kv>` | τ = kp·(ctrl − q) − kv·q̇ | gainprm[0]=kp, biasprm=[0, −kp, −kv] | 위치 PD (관절 서보, Isaac Drive 와 동일 형태) |
| `<velocity kv>` | τ = kv·(ctrl − q̇) | gain = kv, biasprm=[0, 0, −kv] | 속도 제어 (바퀴) |

- 공통 속성: `gear`(출력 배율·축 선택), `ctrlrange`(입력 범위, position 은 `inheritrange="1"` 로 joint range 상속), `forcerange`(출력 토크 클립), `ctrllimited/forcelimited`.
- `dampratio="1"` 을 주면 kv 를 임계감쇠(= 2·√(kp·I_eff))로 **자동 계산** (G1 menagerie 가 이 방식: `kp=500 dampratio=1`).
- 관절 쪽 손실 항: `damping`(점성, −b·q̇), `frictionloss`(쿨롱, 일정 크기), `armature`(로터 반영 관성, 질량행렬 대각에 더해짐).
- 적분기: `Euler` 도 joint damping 은 암시적으로 적분하지만, `implicitfast` 는 그 외 속도 의존 항까지 암시적 처리 → 큰 kv·armature 에서도 안정. 이번 모델(감쇠만 있음)에선 두 적분기의 자유낙하 궤적 차이 0.

## 2. 실험 설정

- 초기 q = [0, 0] (팔 아래로 늘어짐), t = 0 에 어깨 목표 **0.5 rad 계단**, 팔꿈치 목표 0. 2 s, timestep 1 ms, implicitfast, forcerange ±25 N·m.
- 해석용 수치: 어깨 기준 관성 **I ≈ 0.24 kg·m²** (2·0.0075 + 1·0.15² + 1·0.45²), 목표 자세 중력토크 τ_g = 0.6·9.81·sin(0.49) ≈ **2.8 N·m**.
- 이론: ω_n = √(kp / (I + armature)), 임계감쇠 kv_c = 2·√(kp·(I + armature)), 정상상태 오차 e_ss ≈ τ_g / kp (P 제어만 있고 중력보상이 없어서 남음).

## 3. 결과 (`step_results.csv`)

| 그룹 | kp | kv | armature | 정착(±2 %) | 정상값 [rad] | e_ss [rad] | 오버슈트 | 상승 10→90 % | 정착시간 | 이론 ω_n [rad/s] / kv_c |
|---|---|---|---|---|---|---|---|---|---|---|
| kp | 20 | 0 | 0 | ✗ 진동 지속 | 0.361 (진동 중심) | 0.139 | 98 % | 0.118 s | – | 9.1 / 4.4 |
| kp | 50 | 0 | 0 | ✗ | 0.485 | 0.015 | 73 % | 0.087 s | – | 14.4 / 6.9 |
| kp | 200 | 0 | 0 | ✗ | 0.440 | 0.060 | 99 % | 0.066 s | – | 28.9 / 13.9 |
| kv | 200 | 5 | 0 | ○ | 0.486 | **0.0137** | 24 % | 0.069 s | 0.416 s | 28.9 / 13.9 |
| kv | 200 | 14 (≈kv_c) | 0 | ○ | 0.486 | 0.0137 | **0 %** | 0.121 s | **0.212 s** | 28.9 / 13.9 |
| armature | 200 | 5 | 0 | ○ | 0.486 | 0.0137 | 24.0 % | 0.069 s | 0.416 s | 28.9 / 13.9 |
| armature | 200 | 5 | 0.01 (G1 값) | ○ | 0.486 | 0.0137 | 24.8 % | 0.070 s | 0.427 s | 28.3 / 14.1 |
| armature | 200 | 5 | 0.1 | ○ | 0.486 | 0.0137 | **30.6 %** | 0.079 s | **0.513 s** | 24.3 / 16.5 |

(kv=0 그룹의 "정상값" 은 1~2 s 평균 = 진동 중심. 감쇠가 joint damping 0.1 뿐이라 2 s 안에 정착하지 않음.)

### 해석

1. **kp (강성)** — 크면 빠르고(ω_n ∝ √kp: 9.1 → 28.9 rad/s, 상승 0.118 → 0.066 s) 중력 처짐이 줄지만(e_ss = τ_g/kp: 0.139 → 0.014 rad), 감쇠 없이는 그대로 진동한다. kp=200 은 처음부터 토크가 **±25 N·m 포화**(bang-bang) → 상승 시간이 kp=50 과 큰 차이 없음. 강성만 올려도 토크 한계에 막힌다.
2. **kv (감쇠)** — kp=200 에서 kv=5 → 오버슈트 24 %, 정착 0.42 s; kv=14(≈임계감쇠 13.9) → 오버슈트 0, 정착 0.21 s. **이론값 kv_c = 2√(kp·I) 가 그대로 맞음** → 우리 로봇도 각 관절 유효관성 I 를 알면 kv 를 바로 잡을 수 있다 (`dampratio="1"` 이 이 계산을 대신함).
3. **armature (로터 반영 관성)** — 유효관성 I+armature 가 커져 ω_n 이 28.9 → 24.3 rad/s 로 떨어지고, 같은 kv 에서 감쇠비 ζ = kv/(2√(kp·(I+a))) 가 0.36 → 0.30 으로 줄어 오버슈트 24 → 31 %, 정착 0.42 → 0.51 s. 정상상태 오차는 **변하지 않음**(관성은 정적 평형에 무관). G1 수준 0.01 은 0.24 에 비해 4 % 라 영향 미미하지만, 손목처럼 링크 관성이 작은 관절(전완 0.03 kg·m² 수준)에선 같은 0.01 이 30 % 가 된다.
4. 정상상태 오차 0.0137 rad 검산: τ_g(0.486) = 0.6·9.81·sin(0.486) = 2.75 N·m, /200 = 0.0138 rad ✓. → 중력보상(B3 WBC)이 없으면 PD 만으로는 처짐이 남는다는 근거 수치.

## 4. 구술 대비 (10/8 목)

**Q1. position 액추에이터의 kp · kv 는 물리적으로 무엇이고, Isaac Sim Drive 의 stiffness · damping 과 어떻게 대응되는가?**

- kp 는 **관절에 붙은 비틀림 스프링 상수** [N·m/rad], kv 는 **점성 댐퍼 계수** [N·m·s/rad]. 토크 τ = kp·(q_target − q) − kv·q̇ 로, 목표 위치에 스프링-댐퍼로 끌어당기는 PD 제어기.
- 2차계로 보면 ω_n = √(kp/I), ζ = kv / (2√(kp·I)). 실험: I = 0.24 kg·m², kp = 200 → ω_n = 28.9 rad/s, 임계감쇠 kv_c = 13.9 → kv = 14 에서 오버슈트 0 %, 정착 0.21 s 로 이론과 일치.
- 중력보상이 없으면 정상상태 오차 e = τ_g/kp (실험 2.75/200 = 0.0137 rad) 가 남는다.
- Isaac Sim(PhysX) 의 Joint Drive 는 τ = stiffness·(target_pos − q) + damping·(target_vel − q̇), max_force 로 클립. 따라서 **stiffness = kp, damping = kv, max_force = forcerange** 로 1:1 대응. URDF Importer 는 drive 값을 사용자가 지정해야 하므로(URDF 엔 없음) MJCF 에서 튜닝한 kp·kv 를 그대로 넘긴다. 단 Isaac 은 PhysX 의 implicit 적분이라 같은 값이어도 큰 timestep 에서 더 안정적이고, MuJoCo 쪽은 `implicitfast` 를 써야 비슷해진다.

**Q2. armature 는 무엇이고, 감속기가 달린 관절에서 왜 중요한가?**

- MuJoCo `joint armature` = 그 자유도의 **질량행렬 대각에 더해지는 상수 관성** [kg·m²]. 물리적으로는 **모터 로터 + 감속기 입력측 관성을 출력축으로 환산한 반영 관성(reflected inertia) = J_rotor · N²** (N = 감속비). 로터는 출력축보다 N 배 빨리 돌아 운동에너지가 ½·J·(N·q̇)² 이므로 N² 이 붙는다.
- 감속기가 있으면 N² 때문에 작은 로터 관성도 링크 관성과 같은 자릿수가 된다. 예: 로터 J = 1×10⁻⁴ kg·m², N = 10 → 0.01 kg·m² (G1 menagerie 값과 같음). 전완(0.3 m·1 kg)의 팔꿈치 기준 관성 mL²/3 = 0.03 이므로 **30 %** 추가. 토크 100 Nm 급 어깨 구동기(N 이 더 큼)는 더 크다.
- 빠뜨리면 (1) 시뮬 관절이 실제보다 가볍게 반응해 ω_n 이 과대평가되고 kp·kv 튜닝이 실기에서 틀어진다(Sim-to-Real 갭), (2) 실험처럼 같은 kv 에서 감쇠비가 떨어져 오버슈트가 커진다(0.1 추가 시 24 → 31 %), (3) 관성이 작은 자유도에서 수치 안정성이 나빠진다(armature 는 질량행렬 조건수를 좋게 하는 정규화 효과도 있음).
- 우리 로봇: RI85 구동기의 로터 관성·감속비를 A2 에서 받아 `armature = J_rotor·N²` 로 넣는다 → ICD 요청 항목.

## 5. B1 업무 연결

- 우리 로봇 MJCF 의 액추에이터 기본값은 `<default class>` 에 `position kp kv(또는 dampratio) forcerange` 로 두고, 관절별 유효관성에 맞춰 kv_c = 2√(kp·I) 로 초기 튜닝 → B2 에 전달.
- `armature`·`frictionloss` 는 URDF 에 없으므로 구동기 사양표(A2/BOM)에서 받아 MJCF 에서 추가. Isaac Sim 으로 갈 땐 같은 값을 Drive stiffness/damping, joint armature 로 옮긴다.
- 토크 포화(±25 N·m)가 응답을 지배한 것처럼, 우리 로봇도 `forcerange` 를 구동기 정격·피크 토크로 정확히 넣어야 WBC 시뮬 검증이 의미 있다.
