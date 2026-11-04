# v1 모델 검증 5종 결과 (자동 생성 — validate_model.py)

모델: `mjcf/scene.xml` — nq 11 · nv 11 · nu 11 · nbody 18 · 센서 38, MuJoCo 3.17.0

| 검증 | 통과 기준 | 결과 | 판정 |
|---|---|---|---|
| ① 질량 합계 | 전체 · 링크별 ±5 % (기준 = 공용 가정값 + 함태훈 CAD 질량, BOM 대기) | 모델 75.097 kg vs 기준 75.094 kg (+0.004 %), 링크별 최대 0.000 % | **통과** |
| ② 정지 유지 | 5 s, 관절 오차 ≤ 0.5°, 발산 · 떨림 없음 | 영점 0.0393°. 앞으로 수평: 보상 없음 3.6303° → 보상 A 0.0° · B 0.0°. roll 상한: A 3.72°(ctrlrange 클리핑) → **B 0.0°** | **통과** (중력보상 B 필요) |
| ③ 중력 토크 vs 구동기 | 관절별 최악 |τg| × 1.2 ≤ 정격 (판정 기준 52 N·m = 그리퍼 가정 포함 최악 × 1.2, 팀장 10/19) | 3 kg: pitch 42.92 · roll 39.77 · yaw 11.85 · elbow 11.78 N·m | **통과** (PDR 기준) · A2 정격(어깨 TQ 118.8 / RI85 72.6, 10/19) 대비 여유 pitch 177 % (어깨도 RI85면 69 %) |
| ④ 관절 범위 sweep | 범위 안 자기 충돌 0건 | 3770자세(8관절 × 0.5°), 접촉 0건 | **통과** |
| ⑤ 좌우 대칭 | y 부호만 다르고 x · z 차 ≤ 1 mm | 504자세 최대 |Δx| 0.00e+00 · |Δz| 0.00e+00 · |y_L + y_R| 0.00e+00 mm | **통과** |
| 보조 · Isaac용 URDF | planar ↔ prismatic · prismatic · revolute 동등 | 질량 75.0965 vs 75.0985 kg(가상 링크 2 g), 50자세 최대 차 5.3e-15 | **통과** |

## ① 링크별 질량

| 링크 | 모델 kg | 기준 kg | 오차 % |
|---|---|---|---|
| base_link | 30.0 | 30.0 | 0.0 |
| base_plate_link | 3.69 | 3.69 | 0.0 |
| torso_link | 12.0634 | 12.0634 | 0.0 |
| left_shoulder_pitch_link | 4.3963 | 4.3963 | -0.0 |
| left_shoulder_roll_link | 4.1696 | 4.1696 | -0.0 |
| left_shoulder_yaw_link | 3.3364 | 3.3364 | -0.0 |
| left_elbow_link | 2.2677 | 2.2677 | -0.0 |
| left_gripper_link | 0.5 | 0.5 | 0.0 |
| right_shoulder_pitch_link | 4.3963 | 4.3963 | -0.0 |
| right_shoulder_roll_link | 4.1696 | 4.1696 | -0.0 |
| right_shoulder_yaw_link | 3.3364 | 3.3364 | -0.0 |
| right_elbow_link | 2.2677 | 2.2677 | -0.0 |
| right_gripper_link | 0.5 | 0.5 | 0.0 |

- 더미 프레임(odom · cad_frame · tool0 등) 0.0031 kg 포함. 오른팔 기준 = 왼팔 거울(공칭). 함태훈 CAD 오른팔 14.231 kg과 차 +0.061 kg (0.4 %)

## ② 정지 유지 (게인 = 빌드 모델 그대로: kp/kv shoulder_pitch 400/31 · shoulder_roll 350/27.5 · shoulder_yaw 300/15 · elbow 300/11)

| 자세 | 보상 | 최대 오차 ° | 5 s 후 오차 ° | 마지막 1 s 떨림 ° | ctrlrange 클리핑 | 접촉 |
|---|---|---|---|---|---|---|
| zero | 없음 | 0.0407 | 0.0393 | 0.0 | - | 0 |
| zero | A 목표 오프셋 | 0.0 | 0.0 | 0.0 | - | 0 |
| zero | B gravcomp | 0.0 | 0.0 | 0.0 | - | 0 |
| forward | 없음 | 3.7472 | 3.6303 | 0.0 | - | 0 |
| forward | A 목표 오프셋 | 0.0 | 0.0 | 0.0 | - | 0 |
| forward | B gravcomp | 0.0 | 0.0 | 0.0 | - | 0 |
| roll_upper | 없음 | 3.8236 | 3.72 | 0.0 | - | 0 |
| roll_upper | A 목표 오프셋 | 3.8236 | 3.72 | 0.0 | 예 | 0 |
| roll_upper | B gravcomp | 0.0 | 0.0 | 0.0 | - | 0 |

- A = 위치 목표에 τg / kp 오프셋(qfrc_bias, qvel ≈ 0). 목표가 ctrlrange(= 관절 범위, inheritrange)를 넘으면 잘려서 한계 근처 자세에서 처짐이 남음 (함태훈 발견 10/17: roll 상한 4.33°)
- B = 팔 body `gravcomp=1` + joint `actuatorgravcomp=true` — 보상 토크가 actuator 쪽(qfrc_actuator)으로 들어가 ctrlrange와 무관, 합은 actuatorfrcrange ±100으로 제한. build_v1.py `actuator_params.json` gravcomp.enable(제출본 off, week06 on) — 여기서는 b1.xml에 같은 속성을 넣은 모델을 따로 컴파일해 시험

## ③ 관절별 최악 중력 토크 (왼팔 15° 격자 전수 + 상위 6자세 국소 최적화(L-BFGS-B), 판정 기준 정격 52 N·m · 피크 100 N·m 가정)

| 관절 | 0 kg τ [N·m] (자세 °) | 3 kg τ [N·m] (자세 °) | 3 kg 15° 격자값 | 3 kg 정격 여유 % | 3 kg 피크 여유 % |
|---|---|---|---|---|---|
| shoulder_pitch | 25.41 ([-89.7, -0.6, -45.0, 0.5]) | 42.92 ([-89.6, -0.6, -45.0, 0.0]) | 42.91 | 21.2 | 133.0 |
| shoulder_roll | 22.86 ([-0.0, 85.9, -35.1, -1.6]) | 39.77 ([-0.0, 85.9, -55.1, -3.1]) | 38.89 | 30.8 | 151.5 |
| shoulder_yaw | 3.05 ([90.0, -15.0, 94.7, -89.3]) | 11.85 ([90.0, 30.0, 92.5, -89.8]) | 11.84 | 338.9 | 743.9 |
| elbow | 2.99 ([0.2, -0.3, -120.0, -89.6]) | 11.78 ([-0.1, -0.1, -45.0, 90.1]) | 11.78 | 341.3 | 748.6 |

- 팔 앞으로 수평(pitch −90°) 기준: 0 kg 25.403 · 3 kg 42.914 N·m — v0(그리퍼 없음, 페이로드 = 플랜지) 22.73 / 37.30 대비 증가분 = 그리퍼 box 0.5 kg(S14 가정) + 페이로드 작용점이 플랜지 → ee(+0.10 m)로 이동. 자세 = (pitch, roll, yaw, elbow)

## ④ 관절 범위 sweep

| joint | 범위 ° | 샘플 | 범위 안 접촉 | 한계 밖 첫 접촉까지 여유 ° (−/+) |
|---|---|---|---|---|
| left_shoulder_pitch_joint | -171.9 ~ 171.9 | 688 | 0 | None / None |
| left_shoulder_roll_joint | -16.0 ~ 85.9 | 204 | 0 | 2 / None |
| left_shoulder_yaw_joint | -143.2 ~ 143.2 | 573 | 0 | None / None |
| left_elbow_joint | -104.9 ~ 104.9 | 420 | 0 | 3 / 4 |
| right_shoulder_pitch_joint | -171.9 ~ 171.9 | 688 | 0 | None / None |
| right_shoulder_roll_joint | -16.0 ~ 85.9 | 204 | 0 | 2 / None |
| right_shoulder_yaw_joint | -143.2 ~ 143.2 | 573 | 0 | None / None |
| right_elbow_joint | -104.9 ~ 104.9 | 420 | 0 | 3 / 4 |

- 여유 None = 한계 밖으로 180°(또는 ±π)까지 접촉 없음. 충돌 판정 = collision 메시(convex hull) · box, 부모-자식 접촉은 MuJoCo 기본 제외, 추가 `exclude` · `contype` 조치 없음

## ⑤ 좌우 대칭
- 영점 left_ee [0.0, 0.2866, 0.605] · right_ee [0.0, -0.2866, 0.605] m
- 미러 규칙표: [../docs/mirror_rules.md](../docs/mirror_rules.md)
