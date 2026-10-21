# 10. v1 MJCF 덧붙이기 · 액추에이터 근거 · 검증 ①②③ (week05, 함태훈 · B1)

- 커리큘럼: 노션 「B1 5·6주차 학습 커리큘럼」 §1 (5주차) — 함태훈 개별 과제 = MJCF 덧붙이기(`default class` 별 액추에이터 kp · kv · forcerange, armature, damping, 센서) + 검증 ① · ② · ③
- 모델: 박종진 `build_v1.py` 가 만든 B1 공용 v1 (`b1.xml` · `scene.xml`, 박종진 커밋 347a0ca → 652b89e). **URDF 가 원본**, MJCF 는 빌드가 생성 → 함태훈 값은 `actuator_params.json` 한 곳에만 두고 빌드가 다시 읽는다 (재생성해도 유지)
- 재현용 사본: [sim/v1/b1_snapshot/](../sim/v1/b1_snapshot/) (환경변수 `B1_V1` 로 박종진 원본을 가리킬 수 있음)
- 실행 (저장소 루트, 클라우드 Linux · Python 3.13 · mujoco 3.15.0 · scipy 1.18 · matplotlib 3.11):
  ```
  pip install mujoco==3.15.0 trimesh scipy matplotlib
  python sim/scripts/rebase_v1_links.py <박종진 저장소>/b1/models/v1/urdf/b1_v1.urdf   # v1 링크 원점 = 관절점 (메시 · 관성)
  python sim/v1/v1_actuator_table.py          # 액추에이터 파라미터표 + 관절별 계단 응답
  python sim/v1/v1_validate_123.py            # 검증 ① · ② · ③ (약 35 s)
  python sim/v1/v1_armature_study.py          # armature 유무 계단 응답 · 수치 안정성 (구술 Q2)
  python sim/v1/v1_mirror_actuator_check.py   # 팀 연계: 미러 규칙표로 생성 팔 액추에이터 부호 · 범위
  ```
  Windows 노트북: `python` 대신 `py` (스토어 별칭). 한글 경로는 `v1_common.load()` 가 파일을 메모리로 읽어 넘기므로 그대로 됨

## 0. 이번 주에 바뀐 전제 (팀장 10/16 카톡 · 4주차 평가표)

| 항목 | 결정 | 이 노트에 미친 영향 |
|---|---|---|
| 손목 | A1 확인: 손목 없음, **팔당 4-DOF** 확정 (기획서 6-DOF 불일치는 팀장이 상위 보고) | 커리큘럼의 「손목 class」는 없음 → class = shoulder(3) · elbow(1) |
| 베이스 | C3 평면 3-DOF 채택, URDF `planar` 1개 유지 (Isaac 이식 시 3관절 분해) | 베이스 = velocity 액추에이터 3개 (박종진) |
| 축 부호 | 안 A(거울) 승인, 반전은 B2 드라이버 계층 한 곳에서만 | MJCF gear 는 전부 +1 (§6 점검 4) |
| PDR 토크 여유 | **정격 기준**: 정격 ≥ 정적 필요 토크 × 1.2 (10/16 46 N·m → **10/19 52 N·m**, 그리퍼 box 가정 유지). effort = 피크(요청서 §3) | 검증 ③ 판정 = 최악 \|τg\| × 1.2 ≤ 46 |
| 공용 가정값 | 하체 30 + 상판 3.69 kg(별도 링크, 윗면 0.40 m) · 몸통 8 kg + RI85 고정측 · 어깨 1.20 m · effort 100 · velocity 3.0 · armature 0 · damping 0.5 · kp 300 · dampratio 1 · 밀도 3종 | 두 사람 모델 총질량 일치(74.09 ≈ 74.10 kg, 박종진 확인). v1 = 75.10 kg (그리퍼 0.5 kg × 2 추가, S14) |

## 1. v1 링크 원점 = 관절점 (4주차 인계 1순위)

- 4주차 문제: 함태훈 링크 원점(CAD 축 위 임의 점)과 B1 골격 관절점(RI85 출력 디스크 중심)이 **축 방향으로 17.8~22.15 mm** 다름 (축선 수직거리는 ≤ 1.73 mm) → 박종진이 `merge_taehoon.py` 로 평행이동해서 씀
- 이번 주: [rebase_v1_links.py](../sim/scripts/rebase_v1_links.py) 가 두 모델의 `cad_frame` 기준 링크 프레임을 비교 → 회전 차 2e-6° (평행이동만) 확인 → COM · 메시를 dp 만큼 옮김. COM 기준 관성은 그대로 (평행이동에 불변)

| 링크 | 축 방향 원점 차 [mm] | 축선 수직 [mm] |
|---|---|---|
| left_shoulder_pitch | −22.14 | 0.00 |
| left_shoulder_roll | 19.00 | 0.04 |
| left_shoulder_yaw | 22.15 | 1.66 |
| left_elbow | 17.79 | 1.73 |
| torso | (0.01 mm) | – |

- 결과 [sim/robot_v1/link_inertials_v1.json](../sim/robot_v1/link_inertials_v1.json) 을 박종진 `build_v1.py` 입력으로 넣으면 팔 링크 URDF 가 **글자 하나 다르지 않음**, 검증 결과 동일. 몸통 COM 만 0.003 mm · ixz 1e-5 차이 — 박종진 몸통 변환이 cad_frame 좌표의 dp 를 몸통 프레임 COM 에 그대로 더함(축 순서 차이, 0.01 mm 라 영향 없음)
- → 이제 함태훈 출력물을 **변환 없이** 바로 쓸 수 있음. 오른팔은 빌드가 왼팔을 거울로 생성 (안 A)

## 2. MJCF 덧붙이기 — 무엇이 어디에 들어가나

### 2.1 파일 구조와 「include 로 못 넣는 것」

- 3주차 합의 3단 구조 = `robot.xml`(변환본, 수정 금지) → `robot_actuated.xml`(include + 액추에이터 · 센서) → `scene.xml`(include + 바닥 · 조명 · option)
- v1 에서 바뀐 점(박종진 체크리스트 0-3절 제안): **site 는 body 안에 있어야** 해서 include 로 덧붙일 수 없다 → 빌드가 변환본 `b1_urdf.xml` 을 읽어 site · class · 액추에이터 · 센서를 넣은 `b1.xml` 을 **생성**. 변환본은 그대로 두고, 덧붙일 값은 `actuator_params.json` 에만 → 함태훈 동의 (사람이 손으로 고치는 MJCF 가 없으므로 3단 구조의 목적 = 「URDF 재생성해도 덧붙이기 유지」 가 더 확실히 지켜짐)

### 2.2 `default class`

```xml
<default>
  <default class="shoulder">
    <joint armature="0.0" damping="0.5" frictionloss="0.0"/>
    <position kp="300.0" dampratio="1.0" forcerange="-100 100" inheritrange="1"/>
  </default>
  <default class="elbow"> … </default>
  <default class="base"><velocity kv="200" forcerange="-2000 2000"/></default>
</default>
```

- class 는 「이 그룹의 기본값」. `<joint class="shoulder">` · `<position class="shoulder">` 처럼 요소가 class 를 고르면 그 값이 들어가고, 요소에 직접 쓴 속성이 class 보다 우선. 빌드는 변환본 joint 의 damping · armature 를 지우고 class 값을 쓰게 함 → 관절 그룹별로 한 줄만 바꾸면 된다
- 우리 팔은 4-DOF 라 class 2개(shoulder = pitch · roll · yaw, elbow). RI85 가 8개 모두 같은 구동기라 지금은 값도 같지만, 사양이 오면 감속비 · 로터 관성이 다를 수 있어 그룹을 나눠 둠

### 2.3 position 액추에이터 — 힘 식과 kv 가 정해지는 방식

- MuJoCo 액추에이터 힘 = `gain · ctrl + bias`. position 은 gainprm = (kp), biasprm = (0, −kp, −kv) → **τ = kp(ctrl − q) − kv · q̇** (모델에서 읽은 값: kp 300, biasprm[2] = −kv)
- `dampratio` 를 주면 kv 를 컴파일 때 계산: **kv = 2ζ√(kp · I_eff), I_eff = 1/(M⁻¹)ᵢᵢ at qpos0** — 「다른 모든 자유도(베이스 포함)가 자유롭게 움직일 때」의 반사관성. 스크립트에서 assert 로 확인 (오차 < 1e-6)
  - shoulder_pitch: Mᵢᵢ = 0.847 인데 I_eff = 0.351 (팔꿈치 축이 영점에서 pitch 축과 평행 + 베이스가 밀려서 관성이 작게 보임) → kv 20.53
  - 그런데 실제 운용에서는 다른 관절이 서보로 붙잡혀 있어 관성 ≈ Mᵢᵢ → **실효 감쇠비 ζ_hold = kv/(2√(kp·Mᵢᵢ)) = 0.64** (dampratio 1 이라고 써도 임계감쇠가 아님). 계단 0.2 rad 오버슈트 3.6 % (2차계 예측 7.1 %, 감쇠 0.5 · 결합 때문에 더 작음)
- `ctrlrange` + `inheritrange="1"` → 목표 범위 = 관절 범위. `forcerange` = 액추에이터 힘 한계 (±100 = effort 피크, C4) → 오차가 100/300 rad = **19.1°** 를 넘으면 포화
- URDF `<limit effort>` 는 joint `actuatorfrcrange` 로도 들어감 (±100) — 한 관절에 걸린 액추에이터 힘 합계 한계. `<limit velocity>`(3.0) 는 MuJoCo 에서 버려짐 (2주차 확인) → 속도 제한은 6주차 궤적에서 직접 확인해야 함
- 베이스 = velocity 액추에이터: τ = kv(ctrl − q̇). 박종진 시험 시정수 0.378 s = M/kv = 75.1/200 = 0.375 s

### 2.4 armature · damping · frictionloss — 각각 식의 어디로 가나

| 속성 | 식에서 위치 | 의미 | v1 값 |
|---|---|---|---|
| armature | 질량행렬 대각 M ← M + diag(armature) | 감속기 앞 회전자 관성을 출력축으로 옮긴 값 **J_rotor · N²** (N = 감속비). 링크와 같이 가속되지만 중력 · 원심 · 결합은 없음 | 0 (RI85 사양 A2 대기) |
| damping | qfrc_passive = −damping · q̇ | 점성 마찰. **qfrc_bias 에 안 들어감** (§4) | 0.5 N·m·s/rad (C4) |
| frictionloss | 제약 솔버의 마찰 제약 (쿨롱) | 정지 마찰 · 백래시 근사에 쓰는 값 | 0 |

### 2.5 센서 (v1 = 30개)

| 센서 | 붙는 곳 | 값 | 주의 |
|---|---|---|---|
| jointpos · jointvel | 관절 | q, q̇ | – |
| actuatorfrc | 액추에이터 | `actuator_force` = kp(ctrl−q) − kv q̇ (포화 후) | **actuatorgravcomp 보상분은 안 들어감** → 토크 예산용은 `jointactuatorfrc` (§5.2) |
| gyro · accelerometer | site `base_imu` (상판 중심) | 각속도 · 가속도 (센서 프레임) | 정지 상태 가속도계 = (0, 0, **+9.81**) — 중력의 반대 방향(비관성 프레임 반력)이 나옴 (확인값 −1.2e-5, 3e-14, 9.81) |
| framepos · framequat | site `{side}_ee` (그리퍼 box 끝, tool0 에서 0.10 m) | 끝점 위치 · 자세 | 영점 left_ee (0.0002, 0.2862, 0.6050) m — 어깨 1.20 m − 팔 0.595 m |

## 3. 액추에이터 파라미터표 (커리큘럼 산출물) — [sim/v1/results/actuator_table.md](../sim/v1/results/actuator_table.md)

| joint (양팔 같음) | kp | kv (dampratio 1) | Mᵢᵢ qpos0 [자세 범위] | ω_n | ζ_hold [자세 범위] | 계단 OS · 정착 | 수평 처짐 τg/kp |
|---|---|---|---|---|---|---|---|
| shoulder_pitch | 300 | 20.53 | 0.847 [0.049~0.849] | 18.8 | 0.64 [0.64~2.68] | 3.6 % · 0.32 s | 4.85° (τg 25.40) |
| shoulder_roll | 300 | 28.06 | 0.737 [0.478~0.737] | 20.2 | 0.94 [0.94~1.17] | 0.1 % · 0.27 s | 0 (옆 벌림 4.37°) |
| shoulder_yaw | 300 | 4.25 | 0.0153 [0.0153~0.0704] | 140 | 0.99 [0.46~0.99] | 0 % · 0.04 s | 0.05° |
| elbow | 300 | 6.24 | 0.0605 [일정] | 70.4 | 0.73 | 1.6 % · 0.06 s | 0.57° |

- 근거: kp 300 · dampratio 1 · effort 100 · armature 0 · damping 0.5 = **B1 공용 C4 · C5 (팀장 10/16)** — 값은 바꾸지 않고 근거 · 검증을 붙임. 근거 문구를 채운 [handoff/actuator_params.json](../sim/v1/handoff/actuator_params.json) 은 박종진 빌드에 넣어도 b1.xml 이 바이트 단위로 같음
- 읽는 법: ① pitch 는 자세에 따라 관성이 **17배** 변함 → 6주차에 kv 를 고정값으로 튜닝하면 ζ 0.64~2.68 (dampratio 가 컴파일 1회 계산이라 같은 문제) ② kp 300 만으로 수평 처짐 0.5° 를 맞추려면 kp ≈ 2,900 필요 → 중력보상이 답 (§5.2) ③ 포화 19.1° — 큰 계단(예: 0.5 rad)은 처음에 forcerange 에 걸림

## 4. 구술 Q1 — `qfrc_bias` 에는 무엇이 들어 있고, 왜 qvel = 0 에서 읽으면 중력 토크인가

- 운동방정식 **M(q)q̈ + c(q, q̇) + g(q) = τ_act + τ_passive + Jᵀf_contact**. MuJoCo `qfrc_bias` = **c(q, q̇) + g(q)** = 「가속도 0 일 때 RNE(역동역학)로 구한 일반화 힘」 = 코리올리 · 원심 + 중력 (+ 회전 프레임 항). 감쇠 · 스프링은 `qfrc_passive`, 액추에이터는 `qfrc_actuator`, 접촉은 `qfrc_constraint` 로 따로
- c(q, q̇) 는 q̇ 에 2차 → **q̇ = 0 이면 0** → qfrc_bias = g(q) = 그 자세를 버티는 데 필요한 토크 (부호: 중력을 상쇄하는 방향 = 모터가 내야 하는 토크)
- 검산 4가지 (무작위 200 자세, [validate_123.md](../sim/v1/results/validate_123.md) ③):
  1. `mj_rne(m, d, 0, out)` (가속 0) — 차 0.0
  2. 자코비안 전치합 **τ = −Σᵢ J_com,iᵀ (mᵢ g)** (`mj_jacBodyCom`) — 차 8e-15 N·m
  3. 외적 합 **τⱼ = −aⱼ · Σ_{i ∈ 바깥 링크} (cᵢ − pⱼ) × (mᵢ g)** (관절축 aⱼ, 앵커 pⱼ) — 차 8e-15 N·m
  4. **손계산** (팔 앞으로 수평, FK 없이): 영점에서 팔이 수직이므로 COM 마다 「어깨축 아래 깊이」 = 수평 팔 길이. Σ mᵢ g dᵢ = **25.399 N·m** vs MuJoCo 25.4035 (차 0.005 = 더미 프레임 1 g) / 3 kg(ee, 0.595 m) 42.910 vs 42.914
- q̇ ≠ 0 실험: 자세 (−90°, 0.3, 0.4, 0.8 rad), 팔 4관절 3 rad/s → pitch 중력 −23.24 vs bias −28.54 N·m (**코리올리 · 원심 −5.31**, 23 %). 감쇠 0.5 × 3 = 1.5 N·m 은 bias 가 아니라 qfrc_passive → 「움직이는 중에 bias 를 중력으로 쓰면 틀림」
- 이 값을 쓰는 곳: 검증 ③(정격 비교), ② 중력보상 피드포워드, 6주차 토크 예산표의 「정적 최악 자세」 열

## 5. 검증 ① · ② · ③ (함태훈 독립 계산) — [sim/v1/results/validate_123.md](../sim/v1/results/validate_123.md)

| 검증 | 기준 | 결과 | 박종진 공용 계산과 대조 |
|---|---|---|---|
| ① 질량 | ±5 % | 75.0965 vs 기준 75.0936 kg (+0.004 % = 더미 프레임 2.9 g), 링크 13개 ≤ 0.0001 %, **주관성 모멘트 ≤ 0.0002 %** | 같음 (75.097 / +0.004 %). 기준을 함태훈 CAD 질량표에서 직접 다시 만듦 + 관성까지 비교 |
| ② 정지 유지 | 5 s · 0.5° | 보상 없음 영점 0.045° 통과 · 앞 수평 4.83° (이론 4.85°) 미통과 / 목표 오프셋 보상: **roll 상한 자세 4.33° 미통과** / gravcomp 보상: 4자세 0° 통과, 떨림 RMS ≤ 3e-11 rad/s, 베이스 밀림 0 | 박종진은 영점 · 앞 수평 2자세 → 같은 수치. **관절 한계 자세에서 오프셋 보상이 실패하는 것을 새로 찾음** |
| ③ 중력 토크 | \|τ\| × 1.2 ≤ 52 (10/19) | 3 kg(ee): pitch **42.92** (필요 정격 51.5) · roll **39.77** (47.7) → **통과** (여유 21.2 · 30.8 %), yaw 11.85 · elbow 11.78 통과 | pitch 같음, roll 38.89 → 39.77 (+2.3 %) → 박종진 13ca395 국소 최적화로 39.77 일치 |

### 5.1 ③ 최악 자세 토크표 (관절 범위 안 전역 최적화, 페이로드 = ee site)

| 관절 | 0 kg | 0.5 kg | 1.0 kg | 3 kg | 3 kg @플랜지 | 필요 정격 (×1.2) | 기준 52 여유 % (46 일 때) |
|---|---|---|---|---|---|---|---|
| shoulder_pitch | 25.41 | 28.32 | 31.24 | 42.92 | 39.98 | 51.5 | 21.2 (7.2) |
| shoulder_roll | 22.86 | 25.68 | 28.50 | 39.77 | 36.83 | 47.7 | 30.8 (15.7) |
| shoulder_yaw | 3.05 | 4.52 | 5.98 | 11.85 | 8.91 | 14.2 | 339 (288) |
| elbow | 2.99 | 4.45 | 5.92 | 11.78 | 8.84 | 14.1 | 341 (290) |

- 그림: [week05_gravity_torque.png](../assets/screenshots/week05/week05_gravity_torque.png). 페이로드 1 kg 마다 pitch +5.84 N·m (= 9.81 × 수평 도달 0.595 m, 표의 0.5 kg 간격 = 2.92)
- 판정 이력: 10/16 기준 46 에서는 3 kg 어깨 2관절 미통과(원인 = 그리퍼 0.5 kg +2.9 N·m · 작용점 ee +0.10 m +2.9 N·m) → 팀장 10/19: 그리퍼 BOM 없음, **box 가정 유지하고 기준을 52 N·m 로** (42.92 × 1.2 = 51.5 올림) → **통과**
- **A2 사양서 (10/21)**: 어깨 = TQ ILM-E85x30 정격 118.8 · 팔꿈치 = RI85 정격 72.6 N·m → 기준 52 를 실제 구동기가 충족, 3 kg 대비 여유 pitch 177 % — [a2_spec_impact.md](../sim/v1/results/a2_spec_impact.md). 단 CAD 는 8개 모두 RI85 로 그려져 있음(TQ 모터 822 g = RI85 의 2배) → 어깨 질량이 바뀌면 τ 재계산

### 5.2 ② 중력보상 두 방식 — 함태훈 발견 (10/17)

| | 방식 A: 목표 오프셋 | 방식 B: MuJoCo gravcomp |
|---|---|---|
| 구현 | ctrl = q* + qfrc_bias / kp (매 스텝) | 팔 body `gravcomp="1"` + joint `actuatorgravcomp="true"` |
| 원리 | position 서보가 처짐 τg/kp 만큼 미리 더 감 | 보상 토크를 qfrc_actuator 에 직접 더함 (실제 「모터가 내는 피드포워드」) |
| 한계 | **ctrl 이 ctrlrange(= 관절 범위)에 잘림** → 관절 한계 근처에서 보상 사라짐 (roll 1.5 rad 에서 4.33°) | 합계가 joint actuatorfrcrange(±100)로 잘림 = 실제 모터 한계와 같은 의미. `actuatorfrc` 센서엔 안 보임 → `jointactuatorfrc` 필요 |
| 결과 | 4자세 중 3 통과 | 4자세 전부 0° |

- 박종진 빌드에 넣을 수 있게 옵션 패치를 만들어 둠: [handoff/build_v1_gravcomp_option.patch](../sim/v1/handoff/build_v1_gravcomp_option.patch) — `actuator_params.json` 에 `"gravcomp": {"enable": true}` 일 때만 켜지고(기본 꺼짐 → 기존 b1.xml 변화 없음 확인), `{side}_{joint}_jfrc` 센서 8개 추가. roll 1.5 rad 유지 오차 3e-18°, jfrc 22.86 N·m (actuatorfrc 0)
- 실제 로봇에서는 B 가 맞는 그림: B3 WBC 가 중력 토크를 피드포워드로 내고 position 루프는 오차만 담당 (3주차 e_ss = τg/kp 결론과 같은 이야기)

## 6. 구술 Q2 — armature 를 넣었을 때와 뺐을 때 — [sim/v1/results/armature_study.md](../sim/v1/results/armature_study.md)

### 6.1 계단 응답 (shoulder_pitch 0.2 rad, 다른 관절 유지, 중력보상)

| armature [kg·m²] | dampratio 1: kv · OS · 정착 | kv 고정 20.53: OS · 정착 | ω_n = √(kp/(Mᵢᵢ+arm)) |
|---|---|---|---|
| 0 | 20.53 · 3.6 % · 0.32 s | 3.6 % · 0.32 s | 18.8 |
| 0.1 | 23.29 · 2.6 % · 0.32 s | 5.2 % · 0.34 s | 17.8 |
| 0.5 | 31.99 · 1.0 % · 0.27 s | **11.5 %** · 0.39 s | 14.9 |

- 그림: [week05_armature_step.png](../assets/screenshots/week05/week05_armature_step.png)
- **kv 를 고정**하면 armature 가 관성을 키워 ζ = kv/(2√(kp·I)) 가 내려감 → 오버슈트 증가 (0.5 에서 3.6 → 11.5 %), ω_n 감소로 느려짐. 3주차 2링크(kv 고정 5): armature 0 → 0.1 에서 ω_n 28.9 → 24.3, 오버슈트 24 → 30.6 %, 정착 0.42 → 0.51 s, e_ss 불변 — **같은 방향**
- **dampratio** 면 컴파일 때 kv 가 √I 에 비례해 커져 ζ 유지(오히려 약간 증가) → 오버슈트 감소, 상승 0.116 → 0.176 s 로 느려짐만 남음. 즉 armature 값이 들어오면 dampratio 로 쓴 class 는 자동으로 맞춰지지만 kv 고정값은 **반드시 재튜닝**
- 정상상태 오차(중력 처짐)는 armature 와 무관 (정적 문제엔 관성이 안 들어감)

### 6.2 수치 안정성 (가장 가벼운 shoulder_yaw, Mᵢᵢ 0.0153 kg·m², armature 를 팔 8관절에 동시 적용)

| armature | ω_n | Euler 예측 한계 dt | Euler 시뮬 안정 최대 dt | implicitfast 시뮬 안정 최대 dt |
|---|---|---|---|---|
| 0 | 140 | 0.0054 | 0.006 | 0.03 |
| 0.01 | 109 | 0.0071 | 0.007 | 0.04 |
| 0.05 | 68 | 0.0117 | 0.012 | 0.05 |

- 예측 = 1-DOF 반암시적 Euler(v⁺ = v + dt(−kx − cv)/I, x⁺ = x + dt v⁺) 행렬의 스펙트럼 반지름 < 1 조건 (k = kp, c = kv + damping). 감쇠를 빼면 2/ω_n 이지만 kv 항이 명시적이라 한계가 더 낮음 (0.0054 < 0.0143) — 시뮬과 1 그리드 안에서 일치
- **armature 가 없으면 가벼운 관절의 ω_n 이 커서(140 rad/s) 명시적 적분 안정 한계가 작아진다**. 감속기 관절에 armature 를 넣는 것은 물리적으로 맞을 뿐 아니라 수치적으로도 유리. 우리 v1 은 implicitfast(속도 항 암시적) + dt 0.002 라 armature 0 이어도 여유 15배
- 사양 예시 감각(값 아님): J_rotor 1e-4 kg·m² × N² (50² = 2500) = 0.25 kg·m² → yaw 링크 관성(0.015)의 **17배**. 실제 RI85 값이 오면 원위 관절 동특성은 armature 가 지배할 수 있음 → A2 요청 항목 J_rotor · N 우선순위 높음

## 7. 팀 연계 — 미러 규칙표로 생성 팔 점검 — [sim/v1/results/mirror_actuator_check.md](../sim/v1/results/mirror_actuator_check.md)

- B1 v1 은 기준 = 왼팔, 오른팔 = 스크립트 생성 → 커리큘럼의 「좌측 팔 점검」을 **생성 팔(right)** 로 바꿔 수행. 17항목 전부 OK
  1. 정적: class · kp · kv · forcerange · ctrlrange(= 관절 범위 = 공용값) · gear(+1) · damping · armature · actuatorfrcrange 좌우 동일, 축 = 유사벡터 규칙 (pitch 유지, roll · yaw · elbow 반전)
  2. 중력: 같은 관절각 300 자세 좌우 qfrc_bias 차 1.8e-14 N·m — **부호까지 같음** → 토크 예산표 · 제어기 게인을 한 팔 기준으로 써도 됨
  3. 동작: 각 액추에이터 +0.3 rad → ee 변위 x · z 같음, y 반대 (예: roll + = 바깥 벌림 ±169.5 mm, elbow + = 앞으로 굽힘 +88.4 mm, pitch + = 뒤로 −175.9 mm)
  4. **B2 부호표 교차 확인**: 함태훈 CAD 축(고정측 → 출력측, joint_axes_cad.csv)을 CAD 자세(yaw = +π/2)의 URDF 축과 내적 → 「반대」 = left yaw, right pitch · roll · elbow — 박종진 joint-origin-check-v0 §2 와 8/8 일치. 각도차 1.37° = A1 편차
- 결론: MJCF 쪽 gear 는 전부 +1 이 맞음 (부호 반전은 B2 드라이버 한 곳)

## 7-1. 노트북 재현 · 뷰어 캡처 (10/17)

- `py sim/v1/laptop_check.py` (Windows 11 · Python 3.14.7 · MuJoCo 3.15.0): 질량 · 손계산 · ② · ③ · kv 14개 수치 클라우드와 차 0, 미러 17/17 → [results_laptop/laptop_compare.md](../sim/v1/results_laptop/laptop_compare.md)
- `py sim/v1/view_v1.py`: [Control 슬라이더](../assets/screenshots/week05/week05_laptop_sliders.png) (11개, 중력보상 B 라 놓은 자세 그대로 유지) · [수평 처짐 4.832°](../assets/screenshots/week05/week05_laptop_sag.png) (ee z 1.1469 m) · [중력보상 0.000°](../assets/screenshots/week05/week05_laptop_gravcomp.png) (ee z 1.200 m = 어깨 높이), 좌우 ee y ±0.2866

## 7-2. 공통 실습(화) — 베이스 속도 응답 — [results/base_velocity.md](../sim/v1/results/base_velocity.md)

- velocity 액추에이터 τ = kv(ctrl − q̇) → 1차 지연, 시정수 = M/kv: x · y 0.380 s (이론 75.10/200 = 0.3755), yaw 0.082 s (3.915/50 = 0.0783). 축 사이 섞임 ≤ 2e-4 → 홀로노믹 (x · y · yaw 독립)
- 로봇 기준 「앞으로 0.5 m/s」 를 R(yaw)로 월드 명령으로 바꿔 0.5 rad/s 로 돌면, 돌아가는 명령을 τ 만큼 늦게 따라가 옆 미끄러짐 v·ωτ/(1+(ωτ)²) = 0.091 m/s (시뮬 0.091) — 회전 중 직진에는 kv ↑ 또는 ωτ 만큼 앞선 각도로 보정 필요 (6주차 팔 + 베이스 동시 명령에 연결)
- 그림: [week05_base_velocity.png](../assets/screenshots/week05/week05_base_velocity.png)

## 8. B1 업무와의 연결

- 액추에이터 파라미터표 = 6주차 Isaac Lab `ImplicitActuatorCfg` (stiffness = kp, damping = kv, effort_limit = forcerange, velocity_limit, armature) 로 옮길 값의 출처. **kv 는 dampratio 로부터 계산된 숫자(20.53 등)를 옮겨야** 함 — Isaac 에는 dampratio 가 없음
- 검증 ③ · §5.1 표 = 6주차 토크 예산표의 「정적 최악 자세」 열, PDR(6주차, 가중 2배) A팀 피드백 자료 (기준 52 N·m, A2 실제 정격으로 충족 — 6주차 여유율도 52 기준)
- 중력보상 방식 B · jointactuatorfrc = 6주차 궤적 추종 · 페이로드별 토크 기록의 전제

## 9. 한계 · 다음 단계

- BOM 미수령 · A2 사양 일부 TBD: ① 질량 기준이 CAD + 가정값 ② armature 는 공용 모델 0 — A2 값(로터만 0.219 · 0.098)은 영향 분석만, 기어열 관성 TBD ③ 그리퍼 0.5 kg box 가정 유지(팀장 10/19) ④ 어깨 구동기 TQ ↔ CAD RI85 불일치 확인 필요
- 관절 범위(충돌 스윕)는 convex hull 기준이라 보수적 (4주차 Q2) — 오목 링크 convex 분해 비교는 6주차로
- 6주차: 궤적 추종 · kp/kv 튜닝(오버슈트 · 정착 기준, 자세별 관성 변화 고려) · 페이로드별 토크 예산표 · `actuator_params.md`(Isaac 대응)
