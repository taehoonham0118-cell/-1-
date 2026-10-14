# 4주차 기여 로그 — 함태훈 (B1)

- 기간: 2026. 10. 12. (월) ~ 10. 15. (목) 마감 (금: 구술 · 평가 피드백 반영)
- 게이트: 해당 없음 (가중 1)
- 저장소 태그: `week04` — **10/15 최종 제출 커밋에** 찍음 (3주차 피드백 F1). 제출 후 수정하면 릴리스를 다시 만들고 이 로그 §7 에 기록
- 기준 커리큘럼: 노션「B1 2·3·4주차 학습 커리큘럼」 4주차 — 우리 로봇 CAD(STEP) · BOM → URDF / MJCF (GPU 없이)
- 입력 파일: 「양팔로봇 최종버전.stp」(팀장 10/13 카톡) · 「BASE 3.STEP」(10/14) — **원본은 저장소에 올리지 않음** (`.gitignore` `*.step *.stp *.STEP bom*`). BOM 미수령
- 주간 보고서: [reports/학부생_주간활동보고서_4주차_함태훈_22212334_261015.docx](../reports/학부생_주간활동보고서_4주차_함태훈_22212334_261015.docx)

## 1. 이번 주 계획 (커리큘럼 4주차 + 함태훈 개별 과제)

| 요일 | 공통 학습 · 실습 | 개별 과제 (함태훈) |
|---|---|---|
| 월 | STEP 어셈블리 구조 → 링크 분할표 (ICD `{side}_{부위}_{운동}`) | STEP → 링크별 visual / collision STL 변환 절차서 |
| 화 | CAD → 메시: 링크별 visual(세밀) · collision(convex hull), trimesh 검사 | 〃 |
| 수 | 질량 · 관성: 링크 1개 inertial(FreeCAD · trimesh · BOM 3자 비교), v0 URDF(베이스 + 몸통 + 팔) MuJoCo 캡처 | 링크별 질량 · COM · 관성 표 → URDF inertial 블록. 팀 연계: 박종진 골격 joint origin 검증표 |
| 목 | 보고서 · `week04` 태그 · 구술 | 구술 ① 평행축 정리 · URDF inertial 기준 ② visual · collision 분리, convex hull 한계 |

## 2. 실제 수행 · 산출물

| # | 수행 내용 | 완료율 | 산출물 |
|---|---|---|---|
| 1 | (10/13, STEP 수령 전) 변환 도구 · 절차서 v0 를 손계산 정답이 있는 테스트 STEP 으로 먼저 검증: OpenCascade 질량 · 관성 = 손계산 (상대오차 ≤ 2e-7), MuJoCo 질량행렬 M[elbow] 0.00665691 = I_C + m·d² | 100 % | [notes/08](../notes/08_step_to_urdf_inertial_procedure.md), [sim/scripts/step_to_links.py](../sim/scripts/step_to_links.py), [make_test_step.py](../sim/scripts/make_test_step.py) · [verify_test_step.py](../sim/scripts/verify_test_step.py), [week04_run_log.txt](../sim/scripts/week04_run_log.txt) |
| 2 | (10/13) G1 실제 메시로 메시 관성 · convex hull 한계 실험 (메시 9개 중 5개 열림, hull 1.49~10.7배, 낙하 공 28.8 mm 위 정지), URDF ↔ MJCF 관성 왕복 2.6e-6 | 100 % | [g1_mesh_vs_official.py](../sim/scripts/g1_mesh_vs_official.py) (+csv), [hull_contact_demo.py](../sim/scripts/hull_contact_demo.py), [check_inertial_roundtrip.py](../sim/scripts/check_inertial_roundtrip.py) |
| 3 | **링크 분할표 (월)**: 팔 STEP 부품 389개 → 9개 링크(몸통 + 팔 4 × 2), 중복 · 누락 0. RI85 축 = 출력부 R52 원통 축, 링크 배정 = 부품 사이 거리 0 mm 접촉. RI85 는 출력측(output · ring2 · cover plate1)/고정측으로 분할 | 100 % (BOM 열 대기) | [notes/09 §1~2](../notes/09_our_robot_link_split_v0.md), [find_joint_axes.py](../sim/scripts/find_joint_axes.py), [make_linkmap_arm.py](../sim/scripts/make_linkmap_arm.py), [sim/robot/linkmap_arm.json](../sim/robot/linkmap_arm.json), [joint_axes_cad.csv](../sim/robot/joint_axes_cad.csv) |
| 4 | **링크별 STL (화)**: 9개 링크 + 상판 visual(1.1만~5.6만 면, 구동기 내부 기어 · 베어링 제외) · collision(hull 270~390 면) | 100 % | [sim/robot/meshes/](../sim/robot/meshes/) (20개) |
| 5 | **질량 · COM · 관성 표 → inertial 블록 (수)**: 밀도 Al 2700 / 강 7850 / PCB 1850 가정. OpenCascade · trimesh · **FreeCAD 1.0.2(노트북)** 3자 중 2자+FreeCAD 비교, BOM 열 대기 | 90 % (BOM) | [link_mass_props.csv](../sim/robot/link_mass_props.csv), [inertial_blocks.xml](../sim/robot/inertial_blocks.xml), [freecad_link_export.FCMacro](../sim/scripts/freecad_link_export.FCMacro), 캡처 [상판](../assets/screenshots/week04/week04_freecad_base_plate.png) · [팔꿈치](../assets/screenshots/week04/week04_freecad_left_elbow_al.png) |
| 6 | **v0 URDF → MuJoCo (수)**: world —planar→ base(상판 STEP 메시 + 임시 box) —fixed→ torso(어깨 RI85 고정부 + 임시 box) → 양팔 4링크. 3단 구조 robot.xml(mj_saveLastXML, 수정 금지) → robot_actuated.xml → scene.xml | 100 % (가정값 표 포함) | [build_v0_urdf.py](../sim/scripts/build_v0_urdf.py), [our_robot_v0.urdf](../sim/robot/our_robot_v0.urdf), [robot.xml](../sim/robot/robot.xml) · [robot_actuated.xml](../sim/robot/robot_actuated.xml) · [scene.xml](../sim/robot/scene.xml), 캡처 [iso](../assets/screenshots/week04/week04_v0_iso.png) · [pose](../assets/screenshots/week04/week04_v0_pose.png) · [collision](../assets/screenshots/week04/week04_v0_iso_collision.png), **노트북 MuJoCo 뷰어** [기본 자세](../assets/screenshots/week04/week04_v0_viewer_laptop.png) · [Control 로 왼팔 pitch −1.21 · elbow 0.801 유지](../assets/screenshots/week04/week04_v0_viewer_laptop_pose.png) |
| 7 | v0 점검 (금 과제 일부 선행): 초기 접촉 0, 2 s 자세 유지 최대 0.056°, 영점 확인(양팔 elbow +0.5 rad → 팔꿈치 COM +x 38 mm, 앞으로 굽힘), 공칭 vs CAD 편차표, 중력 토크 | 70 % (관절 범위는 가정) | [verify_robot_v0.py](../sim/scripts/verify_robot_v0.py), [verify_robot_v0_log.txt](../sim/robot/verify_robot_v0_log.txt), [joint_origin_check.csv](../sim/robot/joint_origin_check.csv) |
| 8 | **팀 연계**: ① 박종진 관절 표(0dfa6e8) ↔ CAD 축: 수직거리 ≤ 0.064 mm, 축 ≤ 0.024°, 분할 차이 1건 반영(모터드라이버 5 · 6 → elbow) ② **골격 URDF(e292b37) joint origin 검증표**: pitch · roll 앵커 ≤ 0.05 mm · 0.01° 일치, yaw · elbow 는 골격의 축 정렬 1.37° · 거울 대칭 7.4 mm(의도된 차이), 축 부호 규약 3건 차이 → 통일 요청 | 100 % | [check_joint_table_vs_pjj.py](../sim/scripts/check_joint_table_vs_pjj.py) · [check_skeleton_vs_cad.py](../sim/scripts/check_skeleton_vs_cad.py), [joint_check_vs_pjj.csv](../sim/robot/joint_check_vs_pjj.csv) · [joint_check_skeleton.csv](../sim/robot/joint_check_skeleton.csv), [notes/09 §8~9](../notes/09_our_robot_link_split_v0.md) |
| 9 | (팀장 회신 10/8) 박종진 RI85 ICD 요청서 v0 검토 — 보완 3건(다단 유성기어 반사관성, 기어 효율 · 백래시, 질량 항목 근거) | 50 % (전달 · 공동 제출 전) | b1/docs/icd/actuator-spec-request.md (박종진 저장소) |

### 실행 환경 · 명령 (재현용)

| 항목 | 값 |
|---|---|
| 계산 | 클라우드 Linux, Python 3.13, cadquery-ocp 7.8.1.1, trimesh 5.1.1, mujoco 3.15.0 (노트북 Windows · Python 3.14 · cadquery-ocp 8.0.1.1 도 같은 결과) |
| 노트북 뷰어 | Windows 11, `py sim/scripts/view_v0.py` (한글 경로, `python` 은 스토어 별칭이라 `py` 사용) — 10/14 로드 · Run 상태 자세 유지 확인 |
| FreeCAD | 노트북 Windows 11, FreeCAD 1.0.2 (`winget install --id FreeCAD.FreeCAD --version 1.0.2 -e`) |

```bash
pip install cadquery-ocp trimesh scipy rtree mujoco pillow
STP="<양팔로봇 최종버전.stp>"          # 원본은 저장소 밖
python sim/scripts/find_joint_axes.py --step "$STP" --out sim/robot/joint_axes_cad.csv
python sim/scripts/make_linkmap_arm.py
python sim/scripts/step_to_links.py --step "$STP" --linkmap sim/robot/linkmap_arm.json --out sim/robot --lin_defl_mm 0.5 --ang_defl 0.5
python sim/scripts/step_to_links.py --step "<BASE 3.STEP>" --linkmap sim/robot/linkmap_base.json --out sim/robot/base --lin_defl_mm 0.5 --ang_defl 0.5
cp sim/robot/base/meshes/* sim/robot/meshes/
python sim/scripts/build_v0_urdf.py
python sim/scripts/verify_robot_v0.py          # → robot.xml · robot_actuated.xml · scene.xml · 점검 로그 · 캡처
python sim/scripts/check_joint_table_vs_pjj.py
python sim/scripts/check_skeleton_vs_cad.py --skeleton <박종진 저장소>/b1/models/v0/urdf/b1_humanoid_skeleton.urdf
python sim/scripts/view_v0.py                  # 뷰어 (한글 경로 대비). 영어 경로면 python -m mujoco.viewer --mjcf=sim/robot/scene.xml
# 테스트 STEP 검증: python sim/scripts/make_test_step.py && python sim/scripts/step_to_links.py --step sim/test_cad/test_arm.step --linkmap sim/test_cad/linkmap_test.json --out sim/test_cad/out && python sim/scripts/verify_test_step.py
```

### 핵심 수치 (조건 병기 — 3주차 피드백 F2)

| 항목 | 값 | 조건 / 근거 |
|---|---|---|
| STEP 전체 체적 · 전부 Al 질량 | 10.118 L · 27.3 kg | 팀장 미리보기 「약 10.1 L · 27 kg」 과 일치 |
| 팔 자유도 | 팔당 RI85 4개 → **4-DOF** | 계획 6-DOF 와 차이 (손목 없음) → 팀장 확인 요청 |
| 팔 1개 질량 | 왼 14.170 · 오른 14.231 kg | Al · 강 혼합 밀도 가정 (전부 Al 이면 12.15 kg). 차이 0.061 kg = RI85 encoder cover 누락(CAD) |
| 링크 질량 (왼팔) | pitch 4.396 · roll 4.170 · yaw 3.336 · elbow 2.268 kg | 같음 |
| 3자 비교 — 상판 | FreeCAD 3.6901 kg = 스크립트 3.690097, 관성 6자리 일치, trimesh 0.034 % | Al 2700, 상판 중심 기준 |
| 3자 비교 — 팔꿈치 Al 3부품 | FreeCAD 1.65145 kg, ixx 0.0115097 = 스크립트 동일 | 원점 J4, CAD 축 |
| trimesh vs OpenCascade (링크) | 질량 0.2~1.3 %, 관성 0.3~3.2 % | 부품이 맞닿아 메시 watertight 아님 |
| 평행축 정리 (왼 팔꿈치) | I_C 0.01449 → 축 기준 0.02893 kg·m² (2.0배) = MuJoCo M[elbow] 0.028927 | 공칭 축(+x), COM 이 축에서 79.8 mm, 2.268 kg |
| convex hull / 실제 부피 | pitch 2.59 · roll 3.84 · yaw 2.95 · elbow 4.34 · 몸통 6.08배 | collision 270~390 면 |
| 관절 축 기울기 · 비대칭 (CAD) | yaw · elbow 축 1.37°, 오른팔 상완 이하 7.1~7.2 mm | v0 URDF 는 공칭값, 편차는 `joint_origin_check.csv` (팀장 10/14) · A1 확인 요청 |
| v0 MuJoCo | body 11 · joint 11(planar _TX _TY _RZ + 8) · 67.464 kg, 초기 접촉 0 | mujoco 3.15, 베이스 30 kg · 몸통 box 5 kg 가정 |
| 자세 유지 | 2 s 최대 0.056° | position kp 200 · kv 10 (가정), armature 0 |
| 중력 토크 | 팔 수평(pitch −90°) 어깨 22.725 Nm = 14.170 × 9.81 × 0.1635 | 팔 내림 0.095 Nm, 공칭 v0 (영점 = 팔 내림 · 팔꿈치 전방) |
| 박종진 관절 표 대조 | 수직거리 ≤ 0.064 mm, 축 각도 ≤ 0.024° | 원점 위치 차 22 mm 는 축 위 이동 |
| 공칭 v0 vs 박종진 골격(0129b3c 이후) | 축 · 부호 8개 모두 일치(0.00°), 앵커 수직거리 ≤ 1.73 mm (공칭 위치 반올림 기준 차) | q = 0, `joint_check_nominal_vs_skeleton.csv` |
| 박종진 골격 URDF 검증 (10/13, CAD v0 vs e292b37) | pitch · roll ≤ 0.05 mm · 0.01°, yaw · elbow 1.37° · 오른팔 7.4~7.5 mm, 축 부호 반대 3건(L yaw, R roll, R elbow) | q = 0, 몸통 프레임 기준 |
| 골격 v0 vs 함태훈 질량 (왼 elbow) | 2.1425 vs 2.268 kg, 차 0.1255 kg = ring2 기어 + cover plate1 | 출력측 부품 범위 차 |
| E6 armature (F3) | **0 — RI85 사양 미수령, 팀장 10/8 0 허용** | ICD 요청 중 |

## 3. 문제 · 해결 과정

| 증상 | 원인 | 해결 |
|---|---|---|
| 노션 「상판.zip」 이 안 열림 | 첨부 파일이 아니라 노션이 글자를 `http://상판.zip` 링크로 바꾼 것 | 팔 STEP 은 팀장 카톡(10/13), 상판은 10/14 수령 |
| 메시 · 질량 계산 전 STEP 미수령 (10/13 오전) | 배포 지연 | 손계산 정답이 있는 테스트 STEP + G1 메시로 도구 · 절차 먼저 검증 |
| 링크 분할에서 베어링이 두 링크에 중복 배정 | 정규식 `/JIS B 1521 6810:1/` 이 RI85 내부 같은 이름 부품에도 맞음 | 최상위 인스턴스만 맞게 `^[^/]+/` 로 고정 → 중복 · 누락 0 |
| visual 메시가 링크당 10만 면 이상 | 0.05 mm 허용오차 + 구동기 내부 1,500 솔리드 | 내부 부품은 질량만 넣고 메시 제외(`mesh_exclude`), 허용오차 0.5 mm → 1.1만~5.6만 면 |
| FreeCAD 매크로 `PermissionError: C:\Users\links_out` | STEP 을 열고 저장 안 한 문서는 FileName 이 비어 있음 | 결과를 먼저 출력하고, 저장 폴더는 선택 창으로 받게 수정 |
| FreeCAD 트리에 부품 이름이 `\X2\C804…` 로 보임 | STEP 의 한글 이름을 UTF-16 코드로 표시 | 이름 코드 + 위치로 고르는 `freecad_select_parts.FCMacro` 작성 |
| 모터드라이버 5 · 6 배정이 박종진 분할표와 다름 | 함태훈은 근처 부품 거리만 보고 yaw 에 배정 | 접촉 재확인(전완요크2 0 mm) → elbow 로 수정, 링크 질량 갱신 |
| 클라우드 MuJoCo 캡처 오류 | EGL 없음, 오프스크린 크기 640 제한 | OSMesa 설치, scene.xml `offwidth 1280` |

## 4. B1 업무와의 연결

- 링크 질량 · 관성 · STL 을 박종진 골격 URDF 와 합치는 v0 를 먼저 만들어 둠 → 골격 수령 시 joint origin 검증표만 추가하면 됨
- 가정값 표(베이스 높이 0.30 m · 30 kg, 어깨 높이 0.80 m, 관절 ±1.57 rad · 50 Nm · 3 rad/s, kp 200 · kv 10)를 BOM · RI85 ICD · 하체 실측 수령 시 교체
- 팔 수평 22.7 Nm, 팔 관성 0.701 kg·m²(어깨 pitch 기준) → B2 관절 제어 · RI85 토크 사양 확인 근거
- STEP 확인 필요 7건(4-DOF, 축 1.37°, 오른팔 7.1 mm, encoder cover, 드라이버, 모터 형상, 전방 방향) → 팀장 · A1 전달

## 5. 구술 대비 (10/15 목, 2문항) — 상세: 구술 답변 파일 · notes/09

- **Q1. 평행축 정리와 URDF inertial 기준**: I_O = I_C + m(|d|²E − ddᵀ). URDF `<inertial>` 은 origin = 링크 원점 → COM, inertia = **COM 기준**. 우리 왼 팔꿈치 2.268 kg, COM 축에서 79.8 mm → 0.01449 → 0.02893 kg·m²(2.0배) = MuJoCo 질량행렬 0.028927. FreeCAD 1.0.2 와 6자리 일치
- **Q2. visual · collision 분리와 convex hull 한계**: 충돌은 매 스텝 계산 → 단순 형상(우리 팔 visual 1.1만~4.2만 면 → collision 270~320 면). MuJoCo 메시 충돌 = convex hull → 오목한 C브래킷 · 요크가 메워짐(hull/실제 2.6~4.3배), G1 팔꿈치 낙하 시험 28.8 mm 위 정지 → 원시 도형 · convex 분해 · exclude

## 6. 다음 주 목표 (5주차, 커리큘럼 공지 후 확정)

1. BOM · RI85 사양 수령 시 밀도 · 질량 보정, 3자 비교 BOM 열 · armature · effort 반영
2. 박종진과 RI85 출력측 부품 범위 · 축 부호 규약 통일, 메시 · 관성 한 벌로 합치기
3. A1 회신(1.37° · 7 mm 의도 여부, 손목 2축) · 정면(+z_CAD) 확정 반영
4. 금요일 과제 마무리: 관절 범위 · 자기 충돌 점검, 오목 링크 convex 분해 비교

## 6-1. 팀장 회신 (10/14, 박종진 공유 · 박종진 0129b3c) → **10/14 v0 에 반영 완료**

| 규칙 | 내용 | 10/13 v0 | 10/14 v0 (반영) |
|---|---|---|---|
| 좌표 | base_link = REP 103(+x 전방, +y 좌, +z 상), CAD → base 고정 변환 하나(`cad_frame`)만 둠. CAD 점 p[mm] → cad_frame 기준 p/1000 | 링크마다 rpy(−π/2, −π/2, 0)로 같은 변환을 직접 적용 | ✅ `cad_frame` (torso 아래 고정, xyz 0 0.11814 0, rpy π/2 0 π/2) |
| 영점 | CAD 자세가 아니라 「팔 내림 + 팔꿈치 전방 굽힘」 = 0. 90° 차이는 shoulder_yaw origin rpy (0, 0, ∓π/2)(left −, right +)로 흡수 → 양팔 elbow + = 앞으로 굽힘 | CAD 자세 = 0 (팔꿈치가 옆으로 굽힘) | ✅ yaw origin rpy (0,0,∓π/2), CAD 자세 = yaw q +π/2. elbow +0.5 rad → 팔꿈치 COM +38 mm 전방 |
| 공칭값 | URDF 는 축 직교 · 좌우 대칭 공칭값, 편차(1.37°, 7.4 mm)는 joint 원점 검증표에 기록 | CAD 측정값 그대로(축 1.37° 기울기 · 오른팔 7.1 mm 포함) | ✅ 공칭 축 · 오른팔 거울 위치, 편차는 `joint_origin_check.csv`(CAD 자세에서 1.37° · 7.1~7.2 mm) |
| 축 부호 | 오른팔 거울(roll · yaw · elbow 부호 반대), 구동기 출력 방향과 URDF 축이 반대인 관절은 B2 부호표로 | 양팔 같은 부호 | ✅ 오른팔 roll −x · yaw −z · elbow −x — 박종진 골격과 8개 관절 축 · 부호 일치 |

- 팀장이 「이번 주 보고서에 그대로」라고 해서 제출 전(10/14)에 v0 를 다시 만듦: `build_v0_urdf.py` · `verify_robot_v0.py` 재실행, 캡처 갱신. 10/13 CAD 그대로 판은 커밋 43fc7ae 에 남음.
- 1.37° · 7 mm 는 A1 전달 대상 — 박종진이 측정 부품 · 기준 그림으로 팀장께 전달(10/14).

## 7. 제출 · 릴리스 기록

| 일시 | 내용 |
|---|---|
| 2026-10-15 | `week04` 태그 · 릴리스 (최종 제출 커밋) — 업로드 후 커밋 해시 기입 |
| 정정 (3주차 F1) | week03 로그의 「10/7 점검」 은 실제 커밋 c441a5e(10/6 19:56 KST) 기준 **10/6** |
