# week03 스크린샷 · 그래프

| 파일 | 내용 |
|---|---|
| week03_scene_v2.png | scene.xml (include 된 two_link_arm_v2.xml + 체커 바닥), 어깨 0.8 · 팔꿈치 0.6 rad 목표, 측면 카메라 (오프스크린 렌더) |
| week03_arm_on_base.png | attach_arm_to_base.py 로 만든 arm_on_base.xml — 임시 베이스 + 몸통 + 팔 2개 |
| week03_step_kp.png | 계단 응답 (1) kp = 20 / 50 / 200, kv = 0, armature = 0 — 위: 어깨 각, 아래: 액추에이터 토크 (±25 N·m 포화선) |
| week03_step_kv.png | 계단 응답 (2) kp = 200, kv = 0 / 5 / 14 (≈ 임계감쇠 13.9) |
| week03_step_armature.png | 계단 응답 (3) kp = 200, kv = 5, armature = 0 / 0.01 / 0.1 kg·m² |

노트북 뷰어 캡처(관절 드래그 등)는 `week03_viewer_*.jpg` 로 추가.
