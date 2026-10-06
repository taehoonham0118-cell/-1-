# URDF ↔ MJCF 속성 대응표 — 2링크 팔 기준 (week03, 함태훈 · B1)

- 기준 파일: [sim/urdf/two_link_arm.urdf](../sim/urdf/two_link_arm.urdf) ↔ [sim/mjcf/two_link_arm_v2.xml](../sim/mjcf/two_link_arm_v2.xml)
- 변환 확인: URDF 를 MuJoCo 로 읽고 `mj_saveLastXML` 로 저장한 결과 = [sim/mjcf/two_link_arm_from_urdf.xml](../sim/mjcf/two_link_arm_from_urdf.xml)
- 근거: MuJoCo XML Reference / Modeling(URDF extensions) 문서, ROS URDF XML 스펙. 환경: mujoco 3.15 (노트북 3.14 동일 결과)
- 쓰임: 4주차 우리 로봇 URDF 작성 시 "URDF 에 넣을 것 / MJCF 에서만 넣을 것" 구분. 박종진 「MuJoCo 로딩 체크리스트」에 반영.

## 1. 구조 · 단위

| 항목 | URDF | MJCF | 비고 |
|---|---|---|---|
| 루트 | `<robot name>` | `<mujoco model>` | |
| 트리 표현 | `<link>` 와 `<joint parent/child>` 를 **따로** 나열 (그래프) | `<body>` 안에 자식 `<body>` 를 **중첩**, joint 는 자식 body 안 (트리) | MuJoCo 는 URDF 의 parent/child 를 따라가 트리로 재구성 |
| 단위 | m · kg · rad 고정 | m · kg, 각도는 `<compiler angle="degree|radian">` (기본 degree) | URDF 로드 시 자동 radian. MJCF 직접 쓸 때 `angle="radian"` 명시 |
| 좌표계 | joint `origin` = 부모 링크 프레임 기준 자식 프레임 | body `pos quat/euler` = 부모 body 프레임 기준 | 2주차 구술 Q1 과 동일 원리 |
| 월드 고정 루트 | 첫 링크(부모 joint 없음) = 고정 | `<worldbody>` 직속 body 에 joint 없음 = 고정, `<freejoint/>` = 자유 | URDF 로드 시 루트 링크는 기본 `fusestatic` 으로 world 에 **흡수**됨 (아래 §4) |

## 2. 링크 ↔ body

| 정보 | URDF `<link>` | MJCF `<body>` | 변환 결과 (from_urdf.xml) |
|---|---|---|---|
| 이름 | `name` | `name` | 그대로 |
| 시각 형상 | `<visual><origin/><geometry/><material/>` | `<geom contype="0" conaffinity="0" group="2">` (class="visual") | URDF 로드 시 기본 `discardvisual="true"` → **visual 전부 버려짐** (geom 2개만 남음). 보이게 하려면 `<mujoco><compiler discardvisual="false"/></mujoco>` |
| 충돌 형상 | `<collision><origin/><geometry/>` | `<geom>` (class="collision", group 3) | 실린더 `size="r half_len"` — MJCF 는 **반길이** (0.15 = URDF length 0.3 의 절반) |
| 질량 | `<inertial><mass value/>` | `<inertial mass/>` 또는 geom `mass`/`density` | URDF 에 inertial 이 없으면 MuJoCo 가 geom 밀도(1000 kg/m³)로 계산 (2주차: 0.377 kg) |
| 무게중심 | `<inertial><origin xyz/>` | `<inertial pos/>` | 둘 다 **링크(body) 프레임 기준** |
| 관성 텐서 | `<inertia ixx iyy izz ixy ixz iyz>` (COM 기준, 링크 프레임 축) | `diaginertia` + `quat`(주축) 또는 `fullinertia` | MuJoCo 가 고유값 분해해 `diaginertia` 로 저장. 삼각부등식 위반(A+B<C)이면 **컴파일 오류** → `balanceinertia="true"` 로 3값 평균 처리 (실험: 0.02/0.0075/0.0002 → 0.00923×3) |
| 재질·색 | `<material><color rgba/>` | `rgba` 또는 `<asset><material>` | |
| 메시 | `<mesh filename="package://..." scale>` | `<asset><mesh file scale/>` + `<compiler meshdir>` | 4주차: STL 경로를 `<mujoco><compiler meshdir="../meshes"/></mujoco>` 로 지정 |

## 3. 관절 ↔ joint · actuator

| 정보 | URDF `<joint>` | MJCF `<joint>` / `<actuator>` | 변환 결과 |
|---|---|---|---|
| 종류 | `type="revolute"` / `continuous` / `prismatic` / `fixed` / `floating` | `type="hinge"` / `hinge`(range 없음) / `slide` / joint 없음(고정 body) / `free` | revolute·continuous 둘 다 hinge, revolute 만 `range` |
| 위치·자세 | `<origin xyz rpy>` (부모 링크 기준) | 자식 `<body pos quat>` 에 들어가고 joint `pos` 는 0 | `upper_arm pos="0 0 -0.025"` 로 확인 |
| 회전축 | `<axis xyz>` (joint/자식 프레임 기준) | `axis` (body 프레임 기준) | 그대로 `axis="0 1 0"` |
| 가동 범위 | `<limit lower upper>` | `range="lower upper"` (+ `limited`, `autolimits`) | 그대로 |
| 토크 한계 | `<limit effort>` | joint `actuatorfrcrange` / actuator `forcerange` | `actuatorfrcrange="-25 25"` 로 변환됨 (joint 쪽 한계). 액추에이터 `forcerange` 와는 별개 |
| 속도 한계 | `<limit velocity>` | 직접 대응 **없음** (MuJoCo 는 관절 속도 한계를 강제하지 않음) | 변환 시 사라짐 → 제어기(B2)나 `velocity` 액추에이터 `ctrlrange` 로 대신 |
| 점성 감쇠 | `<dynamics damping>` | joint `damping` | `damping="0.1"` 그대로 |
| 쿨롱 마찰 | `<dynamics friction>` | joint `frictionloss` | 0 이면 생략됨. 값 있으면 `frictionloss` 로 |
| **로터 반영 관성** | **없음** | joint `armature` (= J_rotor · N²) | URDF 에는 없는 정보 → MJCF 에서 추가해야 함 (G1: 0.01) |
| **액추에이터** | `<transmission>` (ROS 전용, MuJoCo **무시**) | `<actuator><position kp kv/>` · `<motor gear/>` · `<velocity kv/>` | 변환 시 `nu=0`. MJCF 에서 직접 추가 |
| 액추에이터 입력 범위 | 없음 | `ctrlrange` (position 은 `inheritrange="1"` 로 joint range 상속) | |
| 기어비 | 없음 | actuator `gear` (ctrl → 토크 배율) | motor 의 토크 = gear × ctrl |
| 센서 | 없음 (ROS `<gazebo>` 확장) | `<sensor><jointpos/><jointvel/><actuatorfrc/>` | |
| 자기 충돌 제어 | 없음 (ROS 는 SRDF 로) | geom `contype`/`conaffinity`, `<contact><exclude body1 body2/>` | MuJoCo 는 부모-자식 body 쌍은 기본 충돌 제외 (단, world 에 고정된 부모는 예외 — 2주차 문제) |

## 4. `mj_saveLastXML` 변환에서 확인한 것 (two_link_arm.urdf → two_link_arm_from_urdf.xml)

| 관찰 | 원인 | 대응 |
|---|---|---|
| body 3개 (`world, upper_arm, forearm`) — `base_link` 사라짐, 그 `<inertial>`(5 kg) 도 사라짐 | URDF 로드 시 `fusestatic` 기본 **true** → 관절 없는 고정 링크를 부모(world)에 합침 | 베이스 질량이 필요하면(베이스가 움직이면) `<mujoco><compiler fusestatic="false"/></mujoco>` 또는 floating joint 부여 (실험: `fusestatic=false` → body 4개) |
| geom 2개만 (collision 실린더), 베이스 박스 없음 | `discardvisual` 기본 **true** (URDF 한정) | `discardvisual="false"` → geom 5개, visual 은 group 1 로 들어옴 |
| `nu = 0`, 뷰어에 Control 패널 없음 | `<transmission>` 무시 | MJCF 에서 `<actuator>` 추가 (v2 파일) |
| `<limit effort="25">` → `actuatorfrcrange="-25 25"` | 토크 한계가 joint 속성으로 변환 | 액추에이터 쪽 `forcerange` 는 별도 지정 |
| `<limit velocity="5">` 사라짐 | MuJoCo 에 대응 속성 없음 | 제어기에서 처리 |
| `<option>`, `<default>`, 조명·카메라 없음 | URDF 엔 그런 개념이 없음 | scene.xml 에서 include 로 보강 |

## 5. 결론 — 4주차 우리 로봇 모델에 적용

1. **URDF 가 단일 진실 공급원**: 기하(메시) · 질량 · COM · 관성 · joint origin/axis · range · effort 는 URDF 에 넣는다 (A팀 CAD/BOM 에서 옴).
2. **MJCF 에서만 넣는 것**: `armature`, `frictionloss`, `<actuator>`(kp·kv·gear·ctrlrange·forcerange), `<sensor>`, `<option>`(timestep·integrator), `<default>`, 충돌 필터, scene. → URDF 를 `mj_saveLastXML` 로 변환한 뒤 **include 로 덧붙이는** 구조로 간다 (팔 파일 + scene 파일).
3. URDF 안 `<mujoco><compiler meshdir discardvisual="false" fusestatic balanceinertia/></mujoco>` 확장을 ICD 규약에 넣자고 제안 (박종진 체크리스트 항목).
4. Isaac Sim 으로 갈 때도 같은 URDF 를 쓰고, `kp → Drive stiffness`, `kv → Drive damping`, `armature → (Isaac) joint armature` 로 옮긴다 (notes/06 §4).
