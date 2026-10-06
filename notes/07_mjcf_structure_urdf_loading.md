# MJCF 구성 심화 · URDF 로딩 옵션 (week03, 함태훈 · B1)

노션 3주차(10/6 개정) "USD 항목 → MuJoCo 대응" 을 실제 파일로 확인한 기록. 근거: MuJoCo XML Reference(compiler · option · default · include), Modeling 문서(URDF extensions), Python `mujoco.MjSpec`.

## 1. MJCF 전체 구성 (USD 의 Stage · Prim · Attribute 에 해당)

| 요소 | 역할 | 이번 파일에서 쓴 값 |
|---|---|---|
| `<compiler>` | 파싱 규칙: `angle`(degree/radian), `meshdir`/`texturedir`, `autolimits`, `inertiafromgeom`, `balanceinertia`, `fusestatic`, `discardvisual` | `angle="radian" autolimits="true"` |
| `<option>` | 물리 옵션: `timestep`, `integrator`(Euler/RK4/implicit/implicitfast), `gravity`, `cone`, `solver` | `timestep=0.001 integrator=implicitfast gravity="0 0 -9.81"` |
| `<default>` | 속성 기본값 트리. `<default class="X">` 안의 요소 속성을 같은 class 요소가 상속, body `childclass` 로 자식 전체에 적용 | `class="arm"` (joint·geom·position), 하위 `visual`/`collision` |
| `<asset>` | mesh · texture · material · hfield | scene.xml 의 체커 바닥 |
| `<worldbody>` | body 트리 (USD 의 Xform 계층). body 안에 inertial · joint · geom · site · camera · light | base_link → upper_arm → forearm |
| `<actuator>` / `<sensor>` / `<contact>` / `<equality>` | 구동 · 센서 · 충돌 제외 · 구속 | position ×2, jointpos·jointvel·actuatorfrc |
| `<include file>` | 다른 XML 의 **최상위 자식 요소들**을 그 자리에 삽입 (USD sublayer/reference 와 비슷). 포함되는 파일도 `<mujoco>` 루트를 가진 완전한 파일이어야 함 | scene.xml → two_link_arm_v2.xml |

## 2. 모델 재사용 3가지 (USD 합성 ↔ MuJoCo)

| USD | MuJoCo | 이번 실습 |
|---|---|---|
| sublayer (레이어 겹치기) | `<include file="...">` | `scene.xml` 이 팔 파일을 include → 팔 파일은 단독으로도, scene 안에서도 열림. menagerie `scene.xml`/`g1.xml` 과 같은 구조 |
| reference (다른 파일의 prim 을 가져와 재사용) | Python **`MjSpec.attach`** — 다른 spec 의 body 를 내 spec 의 `frame`/`site` 에 붙임, `prefix` 로 이름 충돌 방지 | [attach_arm_to_base.py](../sim/scripts/attach_arm_to_base.py): 같은 팔 파일을 `left_`/`right_` 로 2번 붙여 베이스+몸통+팔 2개 = [arm_on_base.xml](../sim/mjcf/arm_on_base.xml) (body 9, joint 5, actuator 4, 총 49.7 kg) |
| variant (옵션 전환) | `<default class>` 전환, 또는 mjSpec 에서 Python 분기 | class="visual"/"collision" 로 형상 역할 전환 |

- `mj_saveLastXML(path, model)` : 마지막으로 컴파일된 모델을 MJCF 로 저장 → URDF → MJCF 변환 수단. (`MjSpec.to_xml()` 도 같은 용도, 편집 가능)
- attach 시 default class 도 `left_main/left_arm` 식으로 prefix 가 붙어 복제됨 (출력 XML 로 확인).

## 3. 베이스 고정 vs 자유 (USD ArticulationRoot · RigidBody 에 해당)

| 표현 | MJCF | 결과 | 쓰임 |
|---|---|---|---|
| 고정 베이스 | `<worldbody>` 직속 body 에 joint 없음 | nq = 관절 수. 월드 고정 body 는 **자식과 부모-자식 충돌 제외가 안 됨** (2주차 문제 → 베이스 collision 끔) | 팔 단독 시험, 벤치 |
| 자유 베이스 | body 안 `<freejoint/>` (또는 `<joint type="free">`) | nq += 7 (pos 3 + quat 4), nv += 6. qpos[0:7] 이 베이스 자세 | 홀로노믹 베이스 + 상체 전신 (arm_on_base.xml) |
| 홀로노믹 베이스 구동 | (a) freejoint + 베이스에 `<motor>` 로 x·y·yaw 힘/토크 직접 인가 (옴니휠 추상화), (b) `slide`×2 + `hinge` 3-DOF 평면 관절 + velocity 액추에이터, (c) 바퀴 4개 각각 hinge + 접촉 (가장 사실적, 무거움) | B4 자율주행 시뮬은 (a)/(b), 하체 실측 후 (c) 검토 | 박종진 구술 ① 과 연결 |

## 4. MuJoCo 의 URDF 로딩 옵션 (Isaac Sim URDF Importer 옵션에 해당)

URDF 파일 안에 `<robot>` 자식으로 `<mujoco><compiler .../></mujoco>` 확장을 넣는다. 아래는 two_link_arm.urdf 로 실제 실험한 결과 (mujoco 3.15).

| 옵션 | URDF 로드 시 기본값 | 효과 | 실험 결과 | Isaac Importer 대응 |
|---|---|---|---|---|
| `fusestatic` | **true** (MJCF 는 false) | 관절 없는 고정 body 를 부모에 합침 — body 수 ↓, 그 inertial 은 부모(world 면 소실)로 | 기본: body 3 (`base_link` 소실), false: body 4 | "Merge fixed joints" |
| `discardvisual` | **true** (MJCF 는 false) | `<visual>` 형상 버리고 `<collision>` 만 geom 으로 | 기본: geom 2, false: geom 5 (visual 은 group 1) | (Isaac 은 visual·collision 모두 가져옴) |
| `balanceinertia` | false | 관성 삼각부등식(A+B ≥ C) 위반 시 3 대각값을 평균으로 | 위반 값 → 컴파일 오류 "inertia must satisfy A + B >= C"; true → 0.00923×3 | (Isaac 은 경고 후 로드) |
| `strippath` / `meshdir` | – | 메시 경로 처리 | 4주차 STL 에 사용 | "Mesh path" |
| 고정 / 자유 베이스 | 루트 링크 고정 | URDF 에 `floating` joint 를 넣으면 free joint 로 변환 (실험: nq 2 → 9) | | "Fix base link" |
| 자기 충돌 | 부모-자식 body 쌍 기본 제외, 그 외는 충돌 | 추가 제외는 `contype/conaffinity` 비트마스크 또는 `<contact><exclude>` | world 고정 부모는 예외 | "Self collision" 체크박스 |
| `<transmission>` | 무시 | 액추에이터 없음 (nu=0) | | Importer 는 Drive 를 생성 (stiffness/damping 지정) |

### 언제 `fusestatic` 을 꺼야 하나
- 고정 링크의 **질량·관성이 동역학에 필요할 때** (예: 베이스를 나중에 freejoint 로 바꿀 계획, 몸통에 고정된 배터리 링크 질량), 고정 링크의 **이름으로 센서·site 를 붙일 때**, 고정 링크 사이 **충돌을 따로 다룰 때**. 반대로 고정 브래킷·커버가 많은 CAD 기반 URDF(우리 로봇 46 부품)는 켜 두는 쪽이 body 수를 줄여 빠르다.

## 5. 체크리스트 (박종진 「우리 로봇 URDF → MuJoCo 로딩 체크리스트」에 제안할 항목)

1. URDF 안 `<mujoco><compiler meshdir="../meshes" discardvisual="false" balanceinertia="true"/></mujoco>` 유무
2. 루트 링크 고정/자유 결정 (홀로노믹 베이스 = freejoint + 평면 구동 추상화 중 택1) — `fusestatic` 와 함께 결정
3. 모든 링크 `<inertial>` 존재 + 삼각부등식 만족 (없으면 밀도 1000 kg/m³ 자동 → 틀린 질량)
4. 실린더/캡슐 `size` 는 **반길이**, 메시 scale 은 mm→m `0.001`
5. 관절 `<limit effort>` → `actuatorfrcrange`, 액추에이터 `forcerange` 는 별도. `<limit velocity>` 는 소실됨을 인지
6. `<transmission>` 은 무시되므로 `<actuator>` 를 MJCF(include 파일)에서 추가, `armature`·`frictionloss` 도 거기서
7. 월드 고정 베이스와 첫 링크의 충돌 (contype 0 또는 `<exclude>`)
8. 로드 후 `mj_saveLastXML` 로 저장해 body·geom·joint 수를 URDF 와 대조
