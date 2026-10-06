# 3주차 기여 로그 — 함태훈 (B1)

- 기간: 2026. 10. 5. (월) ~ 10. 8. (목) — 10/9(금) 한글날 휴일로 **목요일 마감**
- 게이트: 해당 없음 (가중 1)
- 저장소 태그: `week03`
- 기준 커리큘럼: 노션「B1 2·3·4주차 학습 커리큘럼 — MuJoCo · URDF · CAD→모델 (GPU 없이, USD는 실습 PC 배정 후)」**10/6 개정판** — 3주차 USD·Isaac Sim 실습은 실습 PC 배정 주로 연기, 같은 개념을 MuJoCo 로 학습
- 주간 보고서: [reports/학부생_주간활동보고서_3주차_함태훈_22212334_261008.docx](../reports/학부생_주간활동보고서_3주차_함태훈_22212334_261008.docx)

> 2주차 로그 §6 의 "다음 주 목표"(USD 작성·URDF↔USD 대응표)는 10/6 커리큘럼 개정으로 **URDF↔MJCF 대응표 + 액추에이터 계단 응답** 으로 교체됨. USD 실습은 하지 않았으므로 부록 없음(커리큘럼 지시대로 생략).

## 1. 이번 주 계획 (개정 커리큘럼 3주차 + 함태훈 개별 과제)

| 요일 | 공통 학습 · 실습 | 개별 과제 (함태훈) |
|---|---|---|
| 화 | MJCF 구성 심화(compiler · option · default · include). 2주차 URDF 를 MuJoCo 로 읽어 `mj_saveLastXML` 로 MJCF 저장, 변환 결과 확인 | 2링크 팔 MJCF 를 default class · include 로 정리(팔 + scene 분리) |
| 수 | 구동: position kp · kv, gear, ctrlrange · forcerange, armature(J_rotor·N²), damping. `mj_step` 루프에서 `data.ctrl` 입력 · `qpos` 기록 | position 액추에이터 추가, kp · kv · armature 를 바꿔 **계단 응답 그래프 3장** |
| 목 | URDF 로딩 옵션(fusestatic, 고정/자유 베이스, 자기 충돌), 보고서 · Git week03 태그 · 구술 | **URDF ↔ MJCF 속성 대응표**, CAD → URDF → MJCF 파이프라인 그림, 박종진 체크리스트로 내 파일 점검 |

## 2. 실제 수행 · 산출물

| # | 수행 내용 | 완료율 | 산출물 |
|---|---|---|---|
| 1 | 2주차 URDF 를 MuJoCo 로 읽어 `mj_saveLastXML` 저장. 변환에서 베이스 링크 소실(fusestatic), visual 소실(discardvisual), `effort`→`actuatorfrcrange`, `velocity` 소실, `<transmission>` 무시(nu=0) 확인 | 100 % | [sim/mjcf/two_link_arm_from_urdf.xml](../sim/mjcf/two_link_arm_from_urdf.xml), [notes/05 §4](../notes/05_urdf_mjcf_mapping.md) |
| 2 | MJCF v2: `<default class="arm">`(joint · geom · position 기본값) + `visual`/`collision` 하위 class, `<include>` 로 팔 파일과 scene 파일 분리, position 액추에이터(kp 20, forcerange ±25, `inheritrange`) · 센서 3개 추가. v1 과 자유낙하 궤적 차이 **0** 확인 | 100 % | [sim/mjcf/two_link_arm_v2.xml](../sim/mjcf/two_link_arm_v2.xml), [sim/mjcf/scene.xml](../sim/mjcf/scene.xml), [week03_scene_v2.png](../assets/screenshots/week03/week03_scene_v2.png) |
| 3 | 계단 응답 실험 스크립트: 어깨 0 → 0.5 rad 계단, kp(20/50/200) · kv(0/5/14) · armature(0/0.01/0.1) 3그룹 9조건, 정상상태 오차 · 오버슈트 · 상승 · 정착시간 · 토크 피크 자동 산출, 그래프 3장 | 100 % | [sim/scripts/step_response.py](../sim/scripts/step_response.py), [step_results.csv](../sim/scripts/step_results.csv), [week03_step_kp.png](../assets/screenshots/week03/week03_step_kp.png) · [kv](../assets/screenshots/week03/week03_step_kv.png) · [armature](../assets/screenshots/week03/week03_step_armature.png), [notes/06](../notes/06_mjcf_actuator_step_response.md) |
| 4 | URDF ↔ MJCF 속성 대응표 (구조 · 링크 · 관절 · **URDF 에 없는 actuator · armature · frictionloss · sensor · 충돌 필터** 포함) + 4주차 적용 결론 | 100 % | [notes/05_urdf_mjcf_mapping.md](../notes/05_urdf_mjcf_mapping.md) |
| 5 | URDF 로딩 옵션 실험: `fusestatic`(body 3 ↔ 4), `discardvisual`(geom 2 ↔ 5), `balanceinertia`(오류 ↔ 평균 0.00923), floating 루트(nq 2 → 9) + 베이스 고정/자유/홀로노믹 표현 정리 + 박종진 체크리스트 제안 8항목 | 100 % | [notes/07_mjcf_structure_urdf_loading.md](../notes/07_mjcf_structure_urdf_loading.md) |
| 6 | `MjSpec.attach` 연습: 같은 팔 파일을 `left_`/`right_` prefix 로 임시 베이스(상판 0.575×0.472 m, freejoint) + 몸통 box 에 2개 부착 → body 9 · joint 5 · actuator 4 · 49.7 kg 모델 생성 (USD reference 에 대응) | 100 % | [sim/scripts/attach_arm_to_base.py](../sim/scripts/attach_arm_to_base.py), [sim/mjcf/arm_on_base.xml](../sim/mjcf/arm_on_base.xml), [week03_arm_on_base.png](../assets/screenshots/week03/week03_arm_on_base.png) |
| 7 | CAD → URDF → MJCF(MuJoCo) 파이프라인 그림 1장 (실습 PC 배정 후 Isaac Sim 경로는 점선) | 100 % | [docs/pipeline_cad_urdf_mjcf.png](../docs/pipeline_cad_urdf_mjcf.png) (.dot · .svg 동봉) |
| 8 | 팀 연계: 박종진 체크리스트로 내 URDF · MJCF 점검 | 대기 | 박종진 초안 공유 후 점검 결과를 notes/07 §5 에 추가 예정 |

### 실행 환경 · 명령 (재현용)

| 항목 | 값 |
|---|---|
| OS / PC | Windows 11 노트북 (Intel Arc 140V, NVIDIA GPU 없음) |
| Python / mujoco | 3.14 / 3.14.0 (결과 수치는 3.15 로도 동일 확인) |
| 추가 패키지 | numpy, matplotlib (그래프), graphviz `dot` (파이프라인 그림, 선택) |

```bash
pip install mujoco numpy matplotlib
python -m mujoco.viewer --mjcf=sim/mjcf/scene.xml            # include 된 팔 + 바닥. Control 패널에서 kp 목표 조작
python -m mujoco.viewer --mjcf=sim/mjcf/two_link_arm_v2.xml   # 팔 파일 단독으로도 열림
python sim/scripts/step_response.py                           # → step_results.csv + 그래프 3장
python sim/scripts/attach_arm_to_base.py                      # → sim/mjcf/arm_on_base.xml
python -m mujoco.viewer --mjcf=sim/mjcf/arm_on_base.xml
python -c "import mujoco; m=mujoco.MjModel.from_xml_path('sim/urdf/two_link_arm.urdf'); mujoco.mj_saveLastXML('sim/mjcf/two_link_arm_from_urdf.xml', m)"
dot -Tpng -Gdpi=150 docs/pipeline_cad_urdf_mjcf.dot -o docs/pipeline_cad_urdf_mjcf.png
```

### 핵심 수치

| 항목 | 값 | 근거 / 조건 |
|---|---|---|
| 어깨 기준 팔 관성 I | 0.24 kg·m² | 2·0.0075 + 1·0.15² + 1·0.45² (팔꿈치 0°) |
| 고유진동수 ω_n = √(kp/I) | kp 20 / 50 / 200 → 9.1 / 14.4 / 28.9 rad/s | 그래프 (1) 진동 주기와 일치 |
| 임계감쇠 kv_c = 2√(kp·I) | kp=200 → 13.9 N·m·s/rad | kv=14 에서 오버슈트 0 %, 정착 0.212 s (kv=5: 24 %, 0.416 s) |
| 정상상태 오차 e_ss = τ_g/kp | kp 20 → 0.139 rad, kp 200 → 0.0137 rad | τ_g = 0.6·9.81·sin(0.486) = 2.75 N·m. 중력보상 없는 PD 의 한계 |
| 토크 포화 | kp ≥ 50 에서 ±25 N·m 포화 | kp=200 상승시간 0.066 s 가 kp=50(0.087 s) 대비 큰 개선 없음 |
| armature 효과 | 0 → 0.1 kg·m²: ω_n 28.9 → 24.3, 오버슈트 24 → 31 %, 정착 0.42 → 0.51 s, e_ss 불변 | armature = J_rotor·N² (예: 1e-4 × 10² = 0.01 = G1 값) |
| URDF→MJCF 변환 | body 3(기본) / 4(fusestatic=false), geom 2 / 5(discardvisual=false), nu 0 | two_link_arm.urdf, mujoco 3.15 |
| balanceinertia | 0.02 / 0.0075 / 0.0002 → 0.00923 ×3 | 삼각부등식 위반 관성에 적용 시 |
| attach 모델 | body 9, joint 5 (free 1 + hinge 4), actuator 4, 49.7 kg | arm_on_base.xml |

## 3. 문제 · 해결 과정

| 증상 | 원인 | 해결 |
|---|---|---|
| 커리큘럼이 10/6 에 USD → MuJoCo 심화로 바뀜 | 실습 PC(RTX) 미배정, Isaac Importer 가 URDF 에서 USD 를 자동 생성하므로 USD 수작업 불필요 | 2주차 로그의 다음 주 목표를 개정판에 맞춰 교체, USD 부록 없음 |
| 계단 응답 kv=0 조건에서 "정상값" 이 조건마다 달라 보임 | 감쇠가 joint damping 0.1 뿐이라 2 s 안에 정착하지 않음 → 구간 평균이 진동 위상에 좌우됨 | 1~2 s 평균(진동 중심)으로 바꾸고 `settled` 플래그 추가, 표에 "진동 지속" 명시 |
| kp 를 50 → 200 으로 올려도 상승 시간이 거의 같음 | 액추에이터 `forcerange` ±25 N·m 포화 (kp·0.5 rad = 100 N·m 요구) | 토크 그래프를 같이 그려 포화 구간 확인. 우리 로봇은 forcerange 를 구동기 정격으로 정확히 입력해야 함 |
| attach 한 팔이 베이스 상판을 관통 | 어깨 장착점 y=±0.2 m 가 상판 반폭 0.236 m 안쪽 | 장착점을 ±0.3 m 로 이동 |
| `mj_saveLastXML` 결과에 베이스 링크 · 5 kg 관성이 없음 | URDF 로드 시 `fusestatic` 기본 true | 베이스를 움직일 땐 `<mujoco><compiler fusestatic="false"/></mujoco>` 또는 floating joint |
| 노트북에 Git 미설치 | 2주차와 동일 | 웹 업로드 + Releases 태그 (docs/GIT_RULES.md §2) |

## 4. B1 업무와의 연결

- **URDF 단일 진실 공급원 + MJCF include 덧붙이기** 구조 확정: 기하 · 질량 · 관성 · joint origin/axis/limit 은 URDF(A팀 CAD/BOM), actuator · armature · frictionloss · sensor · option · 충돌 필터는 MJCF 에서. 4주차 우리 로봇 v0 URDF 를 이 구조로 만든다.
- 계단 응답 수치는 B2(관절 제어) 에 kp·kv 초기값 근거로, 정상상태 오차 e=τ_g/kp 는 B3(WBC 중력보상 필요성) 근거로 전달.
- `armature = J_rotor·N²`, `forcerange` = 구동기 정격/피크 토크 → A2/BOM 에 RI85 로터 관성 · 감속비 · 토크 요청 (ICD 항목).
- 홀로노믹 베이스 MJCF 표현 3안(freejoint+평면 motor / slide·slide·hinge / 바퀴 4개 접촉) 정리 → 팀장 결정 요청.

## 5. 구술 대비 (10/8 목, 2문항) — 상세: notes/06 §4

- **Q1. position 액추에이터 kp·kv 의 물리적 의미, Isaac Sim Drive stiffness·damping 과의 대응**
  kp = 관절 비틀림 스프링 상수 [N·m/rad], kv = 점성 댐퍼 [N·m·s/rad], τ = kp(q_t − q) − kv·q̇ 인 PD. ω_n = √(kp/I), kv_c = 2√(kp·I) — 실험 I=0.24, kp=200 → 28.9 rad/s, kv_c=13.9 → kv=14 에서 오버슈트 0 %. 중력보상 없으면 e_ss = τ_g/kp = 0.0137 rad 잔류. Isaac Drive 는 τ = stiffness(q_t − q) + damping(q̇_t − q̇), max_force 클립 → **stiffness=kp, damping=kv, max_force=forcerange** 로 1:1.
- **Q2. armature 의 의미와 감속기 관절에서 중요한 이유**
  질량행렬 대각에 더하는 상수 관성 = 로터 관성을 출력축으로 환산한 반영 관성 **J_rotor·N²** (로터가 N 배 빨리 돌아 운동에너지 ½J(Nq̇)²). 예 1e-4 × 10² = 0.01 kg·m² (G1 값) = 전완 관성 0.03 의 30 %. 빠뜨리면 ω_n 과대평가 → kp·kv 튜닝이 실기에서 틀어짐(Sim-to-Real), 같은 kv 에서 감쇠비 ↓(실험 24 → 31 % 오버슈트), 작은 관성 자유도 수치 안정성 ↓.
- 예상 추가: "fusestatic 이 뭐냐"(박종진 ②) → 관절 없는 고정 body 를 부모에 합침, 질량·이름이 필요하면 끔. "kp 를 더 올리면?" → forcerange 포화, 이산 시간 안정 한계(timestep 1 ms 에서 kp·dt²/I ≪ 1 이어야).

## 6. 다음 주 목표 (커리큘럼 4주차, 10/12 ~ 10/15 목 마감)

1. FreeCAD 1.0 설치, 「양팔로봇 최종버전.stp」· 「BASE 3.STEP」 내려받기 (원본은 커밋 금지 — `.gitignore` 에 `*.step *.stp bom*` 추가 완료)
2. 링크 분할표(ICD 이름 규칙 `{side}_{부위}_{운동}`) → **STEP → 링크별 visual/collision STL 변환 절차서** (FreeCAD 매크로 또는 수동 절차)
3. 링크별 질량 · COM · 관성 표 (FreeCAD 밀도 · trimesh · BOM 3자 비교) → URDF `<inertial>` 블록 생성, 평행축 정리 검산
4. 박종진 골격 URDF 의 joint origin 이 CAD 관절 축과 맞는지 검증표, 합친 v0 URDF 를 MuJoCo 로 열어 캡처
5. 구술: 평행축 정리 · URDF inertial 기준 / visual·collision 분리 이유와 convex hull 한계
