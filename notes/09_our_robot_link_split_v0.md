# 09. 우리 로봇 팔 STEP → 링크 분할 · 질량 · 관성 · v0 URDF (week04, 함태훈 · B1)

- 입력: 「양팔로봇 최종버전.stp」 (팀장 10/13 카톡, Inventor 2026, AP214, mm). 원본은 저장소 밖(`모델링파일/`)
- 실행 순서 (저장소 루트, STEP 경로만 바꿔서):
  ```
  python sim/scripts/step_to_links.py --step <stp> --inventory sim/robot/parts.csv      # 부품 389개 목록 (저장소엔 안 올림)
  python sim/scripts/find_joint_axes.py --step <stp> --out sim/robot/joint_axes_cad.csv  # RI85 8개 축 · 접촉 부품
  python sim/scripts/make_linkmap_arm.py                                                 # 링크 분할표 → sim/robot/linkmap_arm.json
  python sim/scripts/step_to_links.py --step <stp> --linkmap sim/robot/linkmap_arm.json --out sim/robot --lin_defl_mm 0.5 --ang_defl 0.5
  python sim/scripts/build_v0_urdf.py                                                    # → sim/robot/our_robot_v0.urdf
  python sim/scripts/verify_robot_v0.py                                                  # → robot.xml · robot_actuated.xml · scene.xml · 점검 · 캡처
  ```
- 환경: cadquery-ocp 7.8.1.1, trimesh 5.1.1, mujoco 3.15.0 (10/13 클라우드). 전체 1분 + 1분 30초

## 1. STEP 구조 (부품 목록 · 축 찾기 결과)

| 항목 | 값 | 비고 |
|---|---|---|
| 부품 인스턴스 | 389개, 부품 종류 39 | 노션 미리보기 「46종 · 103 인스턴스」는 하위 조립을 세는 기준이 다름 |
| 전체 체적 | 10.118 L | 노션 미리보기 「약 10.1 L」 과 일치 |
| 전부 알루미늄(2700)일 때 | 27.3 kg | 노션 「약 27 kg」 과 일치 |
| RI85 구동기 | 8개 = 팔당 4개 | 출력부 원통 R 52 mm 의 축 = 관절 축 |
| 팔 자유도 | **팔당 4-DOF** (어깨 pitch · roll · yaw, 팔꿈치) | 2주차 계획 「6-DOF 팔」 과 다름 — 손목 2축이 STEP 에 없음, 끝은 그리퍼 플랜지 |
| 몸통 | 없음 | 두 어깨 RI85 고정부 사이 150 mm (몸통 폭 후보) |

## 2. 링크 분할표 (`sim/robot/linkmap_arm.json`) — 왼팔 (오른팔은 같은 구성, 「(반대)」 · 인스턴스 2)

| 링크 (ICD 이름) | STEP 부품 | 움직이는 관절 · 축(CAD) |
|---|---|---|
| `torso_link` | RI85:1 · RI85:6 고정부 (+ URDF 에서 임시 box) | – |
| `left_shoulder_pitch_link` | 어깨 브래킷1 · 2, 모터드라이버 예상:1, 베어링 6810:1, RI85:1 출력부, RI85:2 고정부 | J1 = RI85:1, CAD +x |
| `left_shoulder_roll_link` | C브래킷1 · 2(수정본), 상완원통 일체형:2, 모터드라이버:3, RI85:2 출력부, RI85:3 고정부 | J2 = RI85:2, CAD +z |
| `left_shoulder_yaw_link` | 상완 칼라1 · 2, 베어링 6810:2, RI85:3 출력부, RI85:4 고정부 | J3 = RI85:3, CAD −y (1.37° 기울어짐) |
| `left_elbow_link` | 전완요크1 · 2, 그리퍼 플랜지, 모터드라이버:5, RI85:4 출력부 | J4 = RI85:4, CAD +z (1.37° 기울어짐) |

- 분할 근거: 부품 사이 최소 거리 0 mm = 맞닿음 (`find_joint_axes.py` · 접촉 확인). 389개 부품 모두 정확히 한 링크에 배정(중복 · 누락 0)
- **RI85 를 둘로 나눔**: 출력과 같이 도는 부품(output-v1 · ring2 기어 · cover plate1 — output 과 0 mm 접촉) → 자식 링크, 나머지(하우징 · 모터 · 1단 링기어 · 캐리어 · 유성기어 · 베어링) → 부모 링크. 캐리어 · 유성기어는 중간 속도로 돌지만 질량이 작아 v0 는 부모. 회전 관성은 armature(= J_rotor·N²)로 따로 넣을 값 (RI85 ICD 요청 중)
- visual 메시에서 RI85 내부(기어 · 베어링 · 모터 · 축)는 뺌 (`mesh_exclude`) — 노션 「액추에이터 내부는 하나의 강체로 묶고 메시에서는 제외」. 질량 · 관성에는 포함
- 링크 프레임: 원점 = 관절 축 위 점, 축 방향 = 로봇 좌표(REP-103). CAD → 로봇: x = CAD +z, y = CAD +x, z = CAD +y (rpy −π/2, −π/2, 0)

## 3. 질량 · COM · 관성 (`link_mass_props.csv`, `inertial_blocks.xml`) — COM 기준, 링크 축 방향 = URDF `<inertial>` 기준

| 링크 | 질량 [kg] (재질 가정) | 전부 Al [kg] | BOM | COM [mm] | Ixx · Iyy · Izz [×10⁻³ kg·m²] | trimesh 차 (질량 · 관성) | hull/실제 부피 |
|---|---|---|---|---|---|---|---|
| torso (RI85 고정부 2개) | 4.063 | 2.967 | 미수령 | (0.1, 0, 0) | 74.53 · 6.13 · 74.54 | 0.65 % · 0.86 % | 6.08 (두 어깨 사이를 메움) |
| L shoulder_pitch | 4.396 | 3.737 | 미수령 | (0.2, 105.0, −11.3) | 16.66 · 11.41 · 14.77 | 1.19 % · 2.77 % | 2.59 |
| L shoulder_roll | 4.170 | 3.543 | 미수령 | (−19.2, 0.2, −95.9) | 20.82 · 24.72 · 12.29 | 0.26 % · 0.73 % | 3.84 |
| L shoulder_yaw | 3.336 | 2.677 | 미수령 | (−3.8, 0.0, −99.3) | 9.84 · 10.05 · 6.42 | 0.70 % · 1.14 % | 2.95 |
| L elbow | 2.268 | 2.189 | 미수령 | (−21.5, −0.1, −79.8) | 14.49 · 17.89 · 8.21 | 0.18 % · 0.31 % | 4.34 |
| **왼팔 합** | **14.170** | 12.146 | | | | | |
| 오른팔 합 | 14.231 | | | | | | |

- 밀도 가정: 브래킷 · 하우징 · 출력부 알루미늄 2700, 기어 · 축 · 키 · 베어링 · 모터 강 7850, 엔코더 기판 1850 kg/m³. **BOM 수령 시 bom_mass_kg 열로 3자 비교 · 밀도 보정**
- 「FreeCAD」 열 = OpenCascade(FreeCAD 내부 엔진, 같은 MatrixOfInertia 규약) 값. 테스트 STEP 에서 손계산과 2e-7 이내로 확인(notes/08)
- trimesh 차이는 메시에 들어간 부품끼리 비교. 메시는 부품 여러 개가 맞닿아 watertight 아님(공유면) → 부피 적분 0.2~1.3 % 차
- 관성 삼각부등식 9개 링크 모두 만족 → `balanceinertia="false"`(팀 결정 B6) 그대로 컴파일 통과

## 4. 관절 원점 (CAD) — 박종진 골격과 비교할 기준표 (`joint_axes_cad.csv`, `joint_origin_check.csv`)

| joint | 부모 → 자식 origin [m] (부모 링크 프레임) | axis (자식 프레임) |
|---|---|---|
| left_shoulder_pitch | (0, 0.1476, 0) — 몸통 중심 기준 | (0, 1, 0) |
| left_shoulder_roll | (0.019, 0.130, −0.020) | (1, 0, 0.0002) |
| left_shoulder_yaw | (−0.0207, 0.00001, −0.1602) | (0.0239, −0.0009, 0.9997) |
| left_elbow | (0.0178, 0.0001, −0.1155) | (0.9997, 0, −0.0239) |
| right_shoulder_yaw | (−0.0206, **0.0071**, −0.1597) | (0.0239, −0.0006, 0.9997) |

- MuJoCo 로드 후 관절 앵커(xanchor) = CAD 관절점 월드 변환값, 최대 차 0.0 mm (8개 관절)
- 박종진 골격 URDF 수령 시: 위 origin 과 골격 origin 차이를 같은 표에 열 추가 (노션 팀 연계 과제)

## 5. STEP 에서 찾은 확인 필요 사항 (팀장 · A팀)

| # | 내용 | 수치 | 영향 |
|---|---|---|---|
| 1 | 팔당 RI85 4개 → **4-DOF** (손목 없음) | 8개 / 2팔 | 2주차 「6-DOF 팔」 가정 · 박종진 골격 관절 수 |
| 2 | 어깨 yaw · 팔꿈치 축이 CAD 축에서 **1.37°** 기울어짐 (양팔 같음) | axis (0.0009, −0.9997, −0.0239) | 설계 의도인지 조립 구속 문제인지 — 의도 아니면 0° 로 보정 |
| 3 | **오른팔** 상완원통 이하가 왼팔 대칭 위치보다 x 로 **+7.1 mm** 어긋남 | 오른 J3 x −388.62 vs 대칭 −395.75 mm | 조립 구속 누락 의심 → v0 는 CAD 그대로, 확인 후 수정 |
| 4 | RI85:2 · :4 · :8 에 encoder cover 없음 (나머지 5개는 있음) | 22.6 cm³ (Al 0.061 kg) | 왼 · 오른 어깨 pitch 링크 질량 0.061 kg 차이의 원인 |
| 5 | 「모터드라이버 예상」 6개 = 55×16×80 mm 상자(임시), 팔당 3개(J1 용 없음) | 개당 Al 0.19 kg | BOM 실제 질량 · 장착 위치 |
| 6 | 구동기 내부 모터(MB039) 형상이 거의 비어 있음 | 2.9 cm³ | 구동기 질량이 실제보다 작게 나올 수 있음 → BOM 질량으로 보정 |
| 7 | 전방 · 좌우 방향 STEP 에 표시 없음 | 가정: 전방 = CAD +z (드라이버가 −z 쪽) | 좌우 팔 이름이 바뀔 수 있음 |

## 6. v0 URDF (`our_robot_v0.urdf` → `robot.xml` → `robot_actuated.xml` → `scene.xml`)

> **10/14 팀장 회신 반영으로 다시 만듦** (§10): REP 103 + `cad_frame` 1개, 공칭값(축 직교 · 오른팔 거울), 영점 = 팔 내림 · 팔꿈치 전방 굽힘(yaw origin rpy ∓π/2). 아래 수치 중 자세 유지 · 중력 토크는 10/14 공칭 v0 기준, 10/13 CAD 그대로 판은 커밋 43fc7ae.

- 구성: world —planar→ base_link(상판 STEP 메시 + 아래 하체 임시 box) —fixed→ torso_link(어깨 RI85 고정부 + 임시 box) → 양팔 4링크. 관절 11개(베이스 _TX · _TY · _RZ + 팔 8), 질량 67.464 kg
- MuJoCo 3.15 로드: 초기 자세 접촉 0개, 관성 경고 없음. position 액추에이터(kp 200 · kv 10, 가정)로 2 s 유지 시 최대 0.056° (10/13 CAD 판 0.145°)
- 중력 토크: 왼팔을 앞으로 수평(pitch −π/2)으로 들 때 어깨 pitch **22.725 Nm** = 팔 14.170 kg × 9.81 × COM 수평거리 0.1635 m. 팔 내림 자세 0.095 Nm → RI85 정격 토크 확인 필요(ICD)
- 캡처: `assets/screenshots/week04/week04_v0_front.png` · `_iso` · `_pose`(visual) · `_iso_collision`(convex hull 충돌 형상)

### 가정값 표 (build_v0_urdf.py `ASSUME`, 확정 시 교체)

| 값 | v0 | 확정 근거 |
|---|---|---|
| 상판 | ✅ BASE 3.STEP 메시 (10/14 수령): 575 × 472 × 12 mm, 1366.7 cm³, Al 가정 3.690 kg, trimesh 차 0.03 % | 재질 · 질량 BOM |
| 상판의 전방 방향 · 몸통 장착 위치 | 상판 긴 변(575) = 전방, 몸통은 상판 중심 | 하체 · 몸통 CAD |
| 바닥 → 상판 윗면 | 0.30 m | 하체 실측 |
| 하체 질량 | 30 kg | 하체 실측 · BOM |
| 바닥 → 어깨 pitch 축 | 0.80 m | 몸통 CAD |
| 몸통 box | 125 × 150 × 550 mm, 5 kg | 몸통 CAD · BOM |
| 관절 범위 · effort · velocity | ±1.57 rad, 50 Nm, 3 rad/s | 박종진 구동기 사양표 · RI85 ICD |
| kp · kv · armature | 200 · 10 · 0 | RI85 J_rotor · N (E6, 0 허용 — 팀장 10/8) |
| 베이스 | planar 3-DOF | 팀장 「확인 후 연락」(10/13) |

## 7. FreeCAD 1.0.2 직접 확인 (노트북, 10/14) — 3자 비교의 FreeCAD 열

`freecad_link_export.FCMacro` 를 노트북 FreeCAD 1.0.2 에서 처음 실행 (BASE 3.STEP, 밀도 2700, 원점 0,0,0). 캡처: `assets/screenshots/week04/week04_freecad_base_plate.png`

| 상판 base_plate | FreeCAD 1.0.2 (매크로) | step_to_links.py (OpenCascade, 클라우드) | trimesh (메시) | BOM |
|---|---|---|---|---|
| 부피 [cm³] | 1366.703 | 1366.7 | – | – |
| 질량 [kg] | 3.6901 | 3.690097 | 차 0.034 % | 미수령 |
| COM [m] (상판 중심 기준) | (2.9e-6, 2.9e-6, −3.33e-5) | (3e-6, 3e-6, −3.3e-5) | – | – |
| ixx · iyy · izz [kg·m²] | 0.0710614 · 0.11366 · 0.184633 | 0.0710614 · 0.1136601 · 0.1846328 | 차 0.044 % | – |
| ixy [kg·m²] | 2.68e-8 | 2.7e-8 | – | – |

→ FreeCAD 와 스크립트가 표시 자릿수(6자리)까지 같음: 같은 엔진 · 같은 규약(COM 기준, 밀도 1 mm⁵ → kg·m²) 확인. 매크로 수정 1건: 저장 안 한 문서는 FileName 이 비어 `C:\Users\links_out` 권한 오류 → 결과 먼저 출력 + 저장 폴더 선택 창.

| 왼팔 팔꿈치 링크 Al 부품 3개 (전완요크1 · 2, 그리퍼 플랜지), 원점 = J4 (159.57, −295.61, 16.09) mm, CAD 축 | FreeCAD 1.0.2 | step_to_links.py |
|---|---|---|
| 부피 [cm³] | 611.648 | 611.648 |
| 질량 [kg] | 1.65145 | 1.651450 |
| COM [m] | (−9.49e-5, −0.105411, −0.0225703) | (−9.49e-5, −0.105411, −0.0225703) |
| ixx · iyy · izz [kg·m²] | 0.0115097 · 0.00592792 · 0.00955885 | 0.0115097 · 0.00592792 · 0.00955885 |
| ixy · ixz · iyz [kg·m²] | −5.0253e-6 · −2.287e-7 · −1.73776e-4 | −5.0253e-6 · −2.287e-7 · −1.73777e-4 |

→ 부품 여러 개(솔리드 3개)를 합칠 때 쓰는 평행축 정리 합산도 FreeCAD 와 일치. 캡처 `assets/screenshots/week04/week04_freecad_left_elbow_al.png`.
FreeCAD 는 이 STEP 의 한글 이름을 `\X2\…\X0\`(UTF-16 코드)로 표시 → 부품 선택은 `freecad_select_parts.FCMacro`(이름 코드 + 위치)로 자동화.

## 8. 팀 연계 — 박종진 관절 표와 대조 (10/14, `check_joint_table_vs_pjj.py` → `sim/robot/joint_check_vs_pjj.csv`)

박종진 `b1/models/link-split-v0.md` §3 (커밋 0dfa6e8, 10/13 18:34) 의 관절 점 · 축(출력 디스크 중심)을 함태훈 CAD 축(출력부 R52 원통 축)과 비교. 골격 URDF 는 아직 없어 **관절 표 기준 v0**.

| 관절 | 박종진 점 → 함태훈 축까지 수직거리 | 축 위 거리 (원점 잡는 위치 차) | 축 방향 차 |
|---|---|---|---|
| 8개 관절 | 최대 **0.064 mm** (박종진 값 소수 1자리 반올림 수준) | 22.1~22.2 mm (디스크 중심 vs 원통 기준면) | 최대 **0.024°** |

- 결론: 두 사람이 따로 뽑은 관절 축이 같은 직선. 원점 위치 차 22 mm 는 축 위 이동이라 URDF 관절은 같음(어느 점을 원점으로 할지만 통일하면 됨 → 박종진 「출력 디스크 중심」 채택 제안)
- 링크 분할도 9개 링크 구성 일치. 차이 2건: ① 모터드라이버:5 · 6 — 박종진은 elbow, 함태훈 v0 는 yaw 링크 → 접촉 거리 재확인(전완요크2 와 0 mm, 상완 칼라2 와 7 mm) 결과 **박종진이 맞아 elbow 로 수정** ② RI85 출력측 — 박종진은 output-v1 만, 함태훈은 output-v1 + ring2 기어 + cover plate1(output 과 0 mm 접촉, 17.3 cm³) — 질량 차 약 0.13 kg/관절, 확인 후 통일
- 체적 합: 박종진 10.22 L vs 함태훈 10.118 L(팀장 미리보기 10.1 L) — 0.1 L 차이 원인 대조 필요(중복 집계 여부)
- 남은 것: 박종진 골격 URDF 수령 후 같은 표에 joint origin(부모 프레임 기준) 열 추가. 박종진 골격은 축 이상화(D1) · 거울 대칭(D2) 예정이라 그 차이(1.37°, 7.1 mm)는 「의도된 차이」로 표시

## 9. 팀 연계 — 박종진 골격 URDF joint origin 검증표 (10/14, `check_skeleton_vs_cad.py` → `sim/robot/joint_check_skeleton.csv`)

박종진 `b1/models/v0/urdf/b1_humanoid_skeleton.urdf` (커밋 e292b37, 10/13 19:26) 와 함태훈 CAD v0 를 q = 0 으로 MuJoCo 에 올려, 관절 앵커 · 축을 몸통(torso_link) 프레임에서 비교.

| 관절 | 골격 앵커 → CAD 축 수직거리 | 축 각도 차 | 축 부호 | 판정 |
|---|---|---|---|---|
| L / R shoulder_pitch | 0.00 / 0.00 mm | 0.00° | 같음 / 같음 | 일치 |
| L / R shoulder_roll | 0.05 / 0.05 mm | 0.01° | 같음 / **반대** | 위치 일치, 부호 규약 차 |
| L / R shoulder_yaw | 2.19 / 7.51 mm | 1.37° | **반대** / 같음 | 골격이 축 정렬(D1) · 좌우 거울(D2) 한 의도된 차이 |
| L / R elbow | 0.88 / 7.38 mm | 1.37° | 같음 / **반대** | 〃 |

- 축 위 거리(원점 선택 차) 16~23 mm 는 같은 관절이라 무관
- 결론: 위치는 CAD 와 맞고, 차이는 ① 1.37° · 7 mm 를 골격이 보정한 것(팀장 확인 대상) ② **축 부호 규약 3건** — 골격은 오른팔 roll · elbow 를 거울(−x), 왼 yaw 를 −z 로 둠. 함태훈 v0 는 양팔 같은 방향(+x, +z) → 관절각 양(+) 방향이 반대가 되므로 B2 제어 전 통일 필요
- 질량 비교: 골격 v0 왼팔 pitch 4.610 · roll 4.383 · yaw 3.550 · elbow 2.1425 kg vs 함태훈 4.396 · 4.170 · 3.336 · 2.268 kg. elbow 차 0.1255 kg = 함태훈이 출력측에 넣은 ring2 기어(강 15.24 cm³ = 0.1197 kg) + cover plate1(Al 2.08 cm³ = 0.0056 kg) → 출력측 부품 범위만 통일하면 맞음. 나머지 링크 차 0.21 kg 은 같은 방식으로 대조 예정

## 10. 팀장 회신(10/14) 반영 — 공칭 v0 (`build_v0_urdf.py` 머리말 규칙 ①~④)

| 규칙 | 반영 |
|---|---|
| ① REP 103 + CAD → base 고정 변환 하나 | `cad_frame` (torso_link 아래 고정, xyz (0, 0.11814, 0) m, rpy (π/2, 0, π/2)). CAD 점 p[mm] → cad_frame 기준 p/1000 |
| ② 공칭값 | 축 = 단위축, 왼팔 joint origin 에서 1 mm 미만 성분(축 기울기 · 조립 오차)은 0, 오른팔 origin = 왼팔 y 거울 |
| ③ 영점 | shoulder_yaw origin rpy (0, 0, −π/2) left · (+π/2) right → 팔 내림에서 elbow + = 전방. 확인: 양팔 elbow +0.5 rad → 팔꿈치 링크 COM x −1.7 → +36.5 mm (left) · −1.6 → +36.7 mm (right) |
| ④ 축 부호 | 왼팔 +y · +x · +z · +x, 오른팔 +y · −x · −z · −x (거울, ICD v0 3.4) |

**편차 기록 (joint 원점 검증표, `joint_origin_check.csv`)** — 공칭 URDF 를 CAD 자세(양팔 yaw q = +π/2)로 두고 CAD 축과 비교

| 관절 | left: 축 직선까지 · 축 각도 | right |
|---|---|---|
| shoulder_pitch | 0.00 mm · 0.00° | 0.00 mm · 0.00° |
| shoulder_roll | 0.00 mm · 0.01° | 0.00 mm · 0.01° |
| shoulder_yaw | 0.01 mm · 1.37° | **7.12 mm** · 1.37° |
| elbow | 0.11 mm · 1.37° | **7.22 mm** · 1.37° |

**박종진 골격(0129b3c 이후)과 대조** (`joint_check_nominal_vs_skeleton.csv`, q = 0, 몸통 프레임): 8개 관절 축 방향 · 부호 모두 일치(0.00°), 앵커 수직거리 pitch 0.00 · roll 0.05 · yaw 1.66 · elbow 1.73 mm(양팔 같음) — 공칭 위치를 어느 점 기준으로 반올림했는지의 차이, 한 벌로 합칠 때 하나로 통일.

질량 · 관성 · 메시는 링크 프레임 기준이라 바뀌지 않음(§3 표 그대로). 왼 팔꿈치 평행축: 공칭 축 +x 기준 I_C 0.01449 → 0.02893 kg·m² (COM 축에서 79.8 mm) = MuJoCo M[elbow] 0.028927.
