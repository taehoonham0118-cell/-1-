# 08. STEP → 링크별 visual/collision STL · 질량 · COM · 관성 → URDF `<inertial>` 변환 절차서 (week04, 함태훈 · B1)

| 항목 | 내용 |
|---|---|
| 상태 | **v1 (2026-10-14)** — 절차와 스크립트를 ① 손계산이 가능한 테스트 STEP, ② Unitree G1 실제 메시로 먼저 검증(10/13)한 뒤, ③ 우리 로봇 STEP(「양팔로봇 최종버전.stp」 10/13 · 「BASE 3.STEP」 10/14)에 그대로 실행했다 → 결과는 [notes/09](09_our_robot_link_split_v0.md). ④ 노트북 FreeCAD 1.0.2 매크로로 같은 값이 나오는지 확인(§4-4). |
| 도구 | OpenCascade(`pip install cadquery-ocp`, FreeCAD가 내부에서 쓰는 CAD 엔진) · trimesh 5.1 · MuJoCo 3.14/3.15. FreeCAD 1.0은 눈으로 확인·캡처용 (§4-4 매크로) |
| 스크립트 | [step_to_links.py](../sim/scripts/step_to_links.py) (본 절차) · [make_test_step.py](../sim/scripts/make_test_step.py) · [verify_test_step.py](../sim/scripts/verify_test_step.py) · [g1_mesh_vs_official.py](../sim/scripts/g1_mesh_vs_official.py) · [check_inertial_roundtrip.py](../sim/scripts/check_inertial_roundtrip.py) · [hull_contact_demo.py](../sim/scripts/hull_contact_demo.py) · [freecad_link_export.FCMacro](../sim/scripts/freecad_link_export.FCMacro) |
| 근거 | ROS URDF XML `link/inertial` · MuJoCo XML Reference `asset/mesh`, `compiler`(inertiafromgeom · balanceinertia) · MuJoCo Computation 「Collision detection」(mesh = convex hull) · OpenCascade `GProp_GProps::MatrixOfInertia`(COM 기준) |
| 쓰임 | 박종진 골격 URDF(link · joint · limit)에 이 절차로 만든 메시와 `<inertial>`을 넣어 v0 URDF 완성 → 3단 구조(`robot.xml` → `robot_actuated.xml` → `scene.xml`)로 MuJoCo 검증 |

---

## 0. 한눈에 보기

| 단계 | 요일 | 하는 일 | 명령 / 도구 | 산출물 |
|---|---|---|---|---|
| ① 부품 목록 | 월 | STEP 조립 트리 → 부품별 부피 · 질량(알루미늄 가정) · 크기 | `step_to_links.py --inventory` | `parts.csv` |
| ② 링크 분할표 | 월 | 「서로 상대 운동이 없는 부품 묶음 = 링크」로 부품을 배정, 링크 원점 지정 | 손으로 JSON 작성 | `linkmap.json` |
| ③ 메시 | 화 | 링크별 visual STL(세밀) · collision STL(convex hull), mm → m | `step_to_links.py --linkmap` | `meshes/*_visual.stl`, `*_collision.stl` |
| ④ 질량 · 관성 | 수 | 링크별 질량 · COM · COM 기준 관성 → `<inertial>` 블록, trimesh · BOM과 3자 비교 | 같은 실행에서 함께 | `link_mass_props.csv`, `inertial_blocks.xml` |
| ⑤ 검증 | 수~금 | URDF를 MuJoCo로 로드: 총질량 · 관절축 관성(질량행렬) · 낙하 · 충돌 | `verify_*.py` 방식 | 로드 캡처, 수치 |

---

## 1. 규칙 (먼저 정할 것)

| 항목 | 규칙 | 이유 · 근거 |
|---|---|---|
| 링크 | 서로 상대 운동이 없는 부품의 묶음. 볼트 · 커버 · 브래킷은 붙어 있는 링크에 합친다 | URDF link = 강체 1개 |
| 구동기(RI85) | 하우징 · 고정자 쪽 = 볼트로 고정된 링크, 출력 플랜지 쪽 = 다음 링크. 내부 기어 · 베어링은 메시에서 빼거나 단순화하되 **질량은 포함** | 노션 4주차 주의사항. 회전자 관성은 링크 관성이 아니라 MJCF `armature = J_rotor·N²` 로 (3주차 notes/06) — 장착 방향은 A2 확인 필요 |
| 링크 원점 | 부모 관절의 회전축 위 한 점, 축은 가능하면 x/y/z 중 하나와 나란히 | 박종진 ICD 초안 3.3 · URDF joint origin = 부모 링크 프레임 기준 |
| 영점 자세 | 팔을 아래로 곧게 내린 자세 = 모든 관절 0 rad | ICD 초안 3.2 |
| 이름 | `{side}_{부위}_{운동}_link` / `_joint`, 소문자 · 밑줄 | ICD 초안 4절, 노션 4주차 월 |
| 단위 | STEP mm → STL · URDF m (× 0.001), 밀도 kg/m³ | URDF · MuJoCo 모두 SI |
| 관성 기준 | URDF `<inertial>`: `origin xyz` = 링크 원점 → COM, `inertia` = **COM 기준** 텐서(축 = origin rpy, 보통 0 → 링크 축) | ROS URDF 스펙. 링크 원점 기준 값을 넣으면 평행축 항이 두 번 들어감 (§4-2) |

---

## 2. 월 — 부품 목록 → 링크 분할표

```bash
python sim/scripts/step_to_links.py --step "<STEP 경로>" --inventory sim/out/parts.csv
```

- `parts.csv` 열: 부품 경로(조립품/부품 이름) · 솔리드 수 · 부피 cm³ · 알루미늄 가정 질량 · 바운딩 박스 mm.
- 부품마다 어느 링크에 넣을지 정해 `linkmap.json` 을 쓴다 (부품 이름에 맞는 정규식, 링크 원점, 밀도, BOM 질량):

```json
{"density_default": 2700,
 "links": [
  {"name": "left_shoulder_pitch_link", "parts": ["어깨 브래킷1", "RI85.*:1$"], "origin_mm": [0, 0, 0], "rpy": [0, 0, 0],
   "density_by_part": {"planet|sun|ring|bearing": 7850}, "bom_mass_kg": null}
 ]}
```

- 실행하면 「두 링크에 중복 배정된 부품」과 「어디에도 안 들어간 부품」을 출력한다 → 0개가 될 때까지 고친다.

## 3. 화 — visual · collision 메시

```bash
python sim/scripts/step_to_links.py --step "<STEP 경로>" --linkmap sim/linkmap.json --out sim/out --lin_defl_mm 0.05 --ang_defl 0.1
```

| 메시 | 만드는 법 | 확인 |
|---|---|---|
| visual | B-rep을 삼각형으로 분할(현 높이 허용 0.05 mm, 각도 0.1 rad) → × 0.001 | `watertight=True`(닫힌 솔리드), 면 수 |
| collision | visual의 convex hull (면 수 상한 200, 줄일 수 없으면 그대로) | hull 부피 / 실제 부피 비 (`hull_vol_ratio`) — 1.3 넘으면 오목 → §5-4 |

- 분할 허용 오차의 영향(테스트 원통 r 20 mm): 각도 0.3 rad → 메시 관성 오차 0.17 %, 0.1 rad → 0.04 %.
- MuJoCo는 mesh geom 충돌을 **convex hull로** 계산한다 → collision STL을 따로 hull로 만들어 두면 Isaac Sim에서도 같은 형상을 쓴다.

## 4. 수 — 질량 · COM · 관성 → `<inertial>`

### 4-1. 계산
- 같은 실행에서 OpenCascade가 솔리드별 부피 적분 → 밀도 가중 합 → 링크 프레임 기준 질량 · COM · **COM 기준** 관성을 낸다.
- `link_mass_props.csv`: OpenCascade 값, trimesh(메시) 값, BOM 질량을 한 줄에 → **3자 비교**. `inertial_blocks.xml` 을 URDF 각 `<link>` 에 붙인다.

### 4-2. 평행축 정리 (링크 원점 기준 ↔ COM 기준)

  I_O = I_C + m (|d|² E − d dᵀ),  d = 원점 → COM 벡터 (E = 3×3 단위행렬)

- URDF는 I_C를 요구한다. CAD가 링크 원점(관절) 기준 값을 주면 **빼서** 넣는다: I_C = I_O − m(|d|²E − d dᵀ).
- 테스트 전완 원통(0.611 kg, r 20 mm, 길이 180 mm, COM이 팔꿈치에서 0.09 m): Iyy_C = 0.00171003 → Iyy_O = 0.00171003 + 0.610726 × 0.09² = 0.00665691 kg·m² (**3.9배**). MuJoCo 질량행렬 M[elbow, elbow] = 0.00665691로 정확히 같음 (§5-1).

### 4-3. FreeCAD 값을 쓸 때 단위 주의
- FreeCAD `Shape.Volume` · `CenterOfMass` · `MatrixOfInertia`는 **밀도 1, mm 단위**이고 MatrixOfInertia는 **COM 기준 [mm⁵]** (OpenCascade와 같은 함수, 10/13 OpenCascade로 확인: 10×20×30 mm 상자 Ixx = 650 000 = V(b²+c²)/12).
- kg·m² 로: × 밀도[kg/mm³] (= kg/m³ × 1e-9) × 1e-6 (mm² → m²).

### 4-4. 노트북 FreeCAD로 확인할 때
1. FreeCAD 1.0 → 파일 → 열기 → STEP → 트리에서 한 링크의 부품을 Ctrl+클릭으로 선택
2. 매크로 → `sim/scripts/freecad_link_export.FCMacro` 실행 → 링크 이름 · 원점(mm) · 밀도 입력
3. 리포트 뷰의 `<inertial>` 이 `inertial_blocks.xml` 과 같은지 비교 (같은 CAD 엔진이라 일치해야 함)
4. **10/14 노트북 FreeCAD 1.0.2 로 검증 완료**: 상판 3.6901 kg · 팔꿈치 Al 3부품 1.65145 kg, 관성 6성분이 스크립트와 6자리까지 일치 (notes/09 §7, 캡처 `assets/screenshots/week04/week04_freecad_*.png`)
5. 주의 ① STEP 을 열고 저장하지 않은 문서는 파일 경로가 비어 있음 → 매크로가 결과를 먼저 출력하고 STL 저장 폴더는 선택 창으로 받음 ② 이 STEP 의 한글 부품 이름이 트리에 `\X2\C804…\X0\`(UTF-16 코드)로 보임 → `freecad_select_parts.FCMacro`(이름 코드 + 위치)로 부품 자동 선택 ③ 파이썬 콘솔에 여러 줄을 붙여 넣으면 for 문 뒤 빈 줄이 빠져 SyntaxError → 매크로 파일로 실행

---

## 5. 검증 결과 (2026-10-13, 클라우드 Linux · mujoco 3.15.0 · trimesh 5.1.1 · OCP 7.8.1)

### 5-1. 테스트 STEP (손계산 정답이 있는 조립품) — `make_test_step.py` → `step_to_links.py` → `verify_test_step.py`
상완 = 상자 40×40×200 mm + 볼트 원통(r 5, 40 mm), 전완 = 원통 r 20 × 180 mm, 알루미늄 2700 kg/m³, STEP AP214 · mm.

| 확인 | 결과 |
|---|---|
| 부품 3개 → 링크 2개 배정, 중복 · 누락 0 | OK |
| 질량 (OpenCascade vs 손계산) | 상완 0.872482 / 0.872482 kg, 전완 0.610726 / 0.610726 kg (오차 < 1e-13 %) |
| COM | 상완 (0.000389, 0, −0.1) m — 볼트 때문에 +x 0.39 mm 이동, 손계산과 차이 1.2e-4 mm |
| COM 기준 관성 상대오차 | 9e-8 (상완), 2e-7 (전완) |
| 메시(trimesh) 관성 vs OpenCascade | 0.000 % (상자), 0.043 % (원통 — 분할 오차) |
| 평행축 정리 vs 링크 원점 기준 직접 적분 | 상완 Ixx_O 0.01172013 / 0.01172009, 전완 0.00665691 / 0.00665413 (0.04 %) |
| MuJoCo 로드 (URDF + 생성 메시 + inertial) | body 4, joint 2, 총질량 1.483208 kg = 손계산. M[elbow,elbow] 0.00665691 = Iyy_C + m·0.09², M[shoulder,shoulder] 0.06480686 / 손계산 0.06480683 |

### 5-2. G1 실제 메시 vs G1 공식 inertial — `g1_mesh_vs_official.py`
G1 공식 값(g1.xml) = 우리 로봇의 「BOM · 실측」 자리, 메시를 균일 밀도로 적분한 값 = 「CAD · trimesh」 자리. 밀도는 공식 질량에 맞춘 ρ_eff = m / V.

| 링크 | 메시 닫힘 | 부피 cm³ | 공식 질량 kg | 알루미늄 가정 kg | ρ_eff kg/m³ | COM 차 mm | 관성 오차 % |
|---|---|---|---|---|---|---|---|
| left_shoulder_pitch | ✗ | 285.68 | 0.718 | 0.771 | 2 513 | 2.7 | 23.1 |
| left_shoulder_roll | ✗ | 172.00 | 0.643 | 0.464 | 3 738 | 14.4 | 69.4 |
| left_shoulder_yaw | ✗ | 414.27 | 0.734 | 1.119 | 1 772 | 22.4 | 28.5 |
| left_elbow | ✗ | 214.49 | 0.600 | 0.579 | 2 797 | 7.2 | 49.8 |
| left_wrist_roll | ✓ | 11.91 | 0.085 | 0.032 | 7 175 | 1.7 | 21.4 |
| **left_wrist_pitch** | ✓ | 208.14 | 0.484 | 0.562 | 2 326 | **0.8** | **5.2** |
| left_wrist_yaw (+손) | ✓ | 172.02 | 0.255 | 0.465 | 1 480 | 21.0 | 27.6 |
| waist_yaw | ✗ | 145.59 | 0.214 | 0.393 | 1 470 | 5.3 | 9.3 |
| waist_roll | ✓ | 6.94 | 0.086 | 0.019 | 12 400 | 0.0 | 24.6 |

해석
1. 닫힌 메시 + 재질이 고른 링크(wrist_pitch, ρ_eff 2 326 ≈ 알루미늄)는 COM 0.8 mm · 관성 5 % 로 맞는다 → 절차 자체는 맞다.
2. 9개 중 5개는 **열린(watertight 아님) visual 껍데기 메시**라 부피 적분 자체가 무의미하다. 우리 로봇은 STEP 솔리드에서 바로 계산하므로 이 문제가 없다(§5-1에서 전부 닫힘).
3. ρ_eff가 1 470 ~ 12 400 kg/m³ 로 흩어진다 = 안에 모터 · 강철 기어처럼 무거운 부품이 몰려 있으면 균일 밀도 가정이 질량 분포를 틀리게 만든다(COM 최대 22 mm, 관성 최대 69 %). → 우리 로봇은 부품별 밀도(`density_by_part`)를 주고, **BOM 질량과 대조**해 차이가 크면 BOM · A4 실측값으로 질량을 맞춘다(3자 비교의 이유).

### 5-3. MJCF(quat + diaginertia) ↔ URDF(전체 텐서) 왕복 — `check_inertial_roundtrip.py`
G1 공식 값을 URDF식 전체 텐서(I = R·diag·Rᵀ)로 바꿔 MuJoCo에 다시 넣으면 9개 링크 모두 질량 · COM 동일, 관성 상대오차 최대 2.6e-6 (6자리 반올림 수준). → URDF에는 rpy 0 + 비대각 성분으로 넣으면 되고, MuJoCo가 알아서 주축으로 분해한다.

### 5-4. convex hull 충돌의 한계 — hull 부피 비 · 틈 · MuJoCo 낙하
| 링크 | hull 부피 / 실제 | hull 표면 → 실제 표면 최대 틈 |
|---|---|---|
| left_shoulder_roll | 3.20 배 | 30.9 mm |
| left_elbow | 1.49 배 | 26.9 mm |
| left_wrist_roll | 10.7 배 | 22.1 mm |
| left_wrist_pitch | 1.12 배 | 4.7 mm |

- MuJoCo 낙하 시험(`hull_contact_demo.py`, 반지름 4 mm 공, 수직 slide 관절): G1 팔꿈치 링크 위에 떨어뜨리면 **실제 표면보다 28.8 mm 위(hull 표면)에서 멈춤**, 접촉 1개.
- 대처: 오목한 링크는 ① 원시 도형(G1도 어깨 pitch · roll 충돌은 원통으로 둠) ② convex 분해(V-HACD · CoACD로 hull 여러 개) ③ 자기 충돌이 문제면 `<contact><exclude>`.

---

## 6. 자주 틀리는 것 (점검표)

| # | 실수 | 증상 | 막는 법 |
|---|---|---|---|
| 1 | STL을 mm 그대로 | 로봇이 1000배 | 스크립트가 × 0.001, URDF `scale` 따로 쓰지 않기 |
| 2 | 링크 원점 기준 관성을 그대로 `<inertia>`에 | 관절 관성이 m·d² 만큼 두 번 → 느린 응답 | §4-2 식으로 COM 기준 변환 |
| 3 | `<inertial>` 누락 | MuJoCo가 geom 부피 × 1000 kg/m³로 자동 계산 (2주차: 0.377 kg) | 모든 움직이는 링크 inertial (체크리스트 A1) |
| 4 | 열린 메시로 부피 적분 | 엉뚱한 질량 · COM | `watertight=True` 확인, STEP 솔리드로 계산 |
| 5 | 관성 삼각부등식 위반 | MuJoCo 컴파일 오류 | 스크립트 `triangle_ineq` 열 (체크리스트 A3) |
| 6 | 오목 링크 충돌을 메시 하나로 | 가짜 접촉 (§5-4, 28.8 mm) | 원시 도형 · convex 분해 |
| 7 | 구동기 내부 부품 누락 | 링크 질량 과소 | 메시에서는 빼도 질량에는 포함, BOM 대조 |

## 7. 진행 상태 (10/14)
1. ✅ `--inventory` → 링크 분할표 `sim/robot/linkmap_arm.json` (박종진 ICD 이름 규칙)
2. ✅ `--linkmap` 실행 → 메시 · `inertial_blocks.xml` → `build_v0_urdf.py` → `robot.xml`(mj_saveLastXML, 수정 금지) → `robot_actuated.xml` → `scene.xml`
3. ✅ 박종진 골격 joint origin vs CAD 관절 축 검증표 (`check_skeleton_vs_cad.py`, notes/09 §9)
4. ◐ v0 자세 유지 · 초기 접촉 확인 완료, 관절 범위 · 자기 충돌 · BOM 3자 비교는 BOM · RI85 사양 수령 후
