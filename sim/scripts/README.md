# sim/scripts

직접 작성·수정한 실행 스크립트 보관. 파일 상단 주석에 **주차 · 목적 · 실행 명령**을 적는다.

```python
# week03 | Cartpole 보상 수정 실험
# 실행: ./isaaclab.sh -p <이 파일 경로> --num_envs 32
```

## week04 — 우리 로봇 STEP → URDF (실행 순서)

| 순서 | 스크립트 | 하는 일 | 출력 |
|---|---|---|---|
| 0 | `make_test_step.py` → `step_to_links.py` → `verify_test_step.py` | 손계산 정답이 있는 테스트 STEP 으로 도구 검증 | `sim/test_cad/out/` |
| 1 | `find_joint_axes.py --step <stp>` | RI85 8개 축 · 관절점 · 맞닿은 부품 | `sim/robot/joint_axes_cad.csv` |
| 2 | `make_linkmap_arm.py` | 링크 분할표 생성 (ICD 이름, 밀도 가정, 메시 제외 부품) | `sim/robot/linkmap_arm.json` |
| 3 | `step_to_links.py --step <stp> --linkmap ...` | 링크별 visual · collision STL, 질량 · COM · 관성, `<inertial>` | `sim/robot/meshes/`, `link_mass_props.csv`, `inertial_blocks.xml` |
| 4 | `build_v0_urdf.py` | 베이스(planar) + 몸통 + 양팔 v0 URDF, 가정값 표 `ASSUME` | `sim/robot/our_robot_v0.urdf` |
| 5 | `verify_robot_v0.py` | robot.xml(mj_saveLastXML) → robot_actuated.xml → scene.xml, 점검 · 캡처 | `sim/robot/*.xml`, `verify_robot_v0_log.txt`, `assets/screenshots/week04/` |
| 6 | `view_v0.py` | 한글 경로에서도 v0 를 MuJoCo 뷰어로 열기 | – |
| 7 | `check_joint_table_vs_pjj.py` · `check_skeleton_vs_cad.py` | 박종진 관절 표 · 골격 URDF 와 관절 축 대조 | `sim/robot/joint_check_*.csv` |
| FreeCAD | `freecad_select_parts.FCMacro` → `freecad_link_export.FCMacro` | 노트북 FreeCAD 1.0.2 로 같은 질량 특성 확인 | 리포트 뷰 `<inertial>` |
| 보조 | `mj_load.py`, `g1_mesh_vs_official.py`, `check_inertial_roundtrip.py`, `hull_contact_demo.py` | 한글 경로 로드, G1 메시 실험 | `week04_run_log.txt` |
