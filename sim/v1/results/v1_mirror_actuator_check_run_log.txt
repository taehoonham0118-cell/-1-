# 팀 연계 — 미러 규칙표로 생성 팔(right) 액추에이터 부호 · 범위 점검 (자동 생성 — sim/v1/v1_mirror_actuator_check.py)

모델 v1 `scene.xml`, 미러 규칙표 = 박종진 `b1/models/v1/docs/mirror_rules.md`, MuJoCo 3.15.0. **종합: 전부 OK** (17/17)

## 1. 정적 (값 · 축 규칙)

| joint | left axis | right axis | 규칙 | 축 OK | right ctrlrange = 관절 범위 = 공용값 | 좌우 다른 값 | 판정 |
|---|---|---|---|---|---|---|---|
| shoulder_pitch | [0.0, 1.0, 0.0] | [0.0, 1.0, 0.0] | 유지 | True | [-3.0, 3.0] True | 없음 | OK |
| shoulder_roll | [1.0, 0.0, 0.0] | [-1.0, 0.0, 0.0] | 반전 | True | [-0.28, 1.5] True | 없음 | OK |
| shoulder_yaw | [0.0, 0.0, 1.0] | [0.0, 0.0, -1.0] | 반전 | True | [-2.5, 2.5] True | 없음 | OK |
| elbow | [1.0, 0.0, 0.0] | [-1.0, 0.0, 0.0] | 반전 | True | [-1.83, 1.83] True | 없음 | OK |

- 비교한 값: class kp · kv(컴파일) · forcerange · ctrlrange · gear · 관절 range · damping · armature · joint actuatorfrcrange

## 2. 중력 토크 부호

- 300 자세 max|τ_L − τ_R| = 1.8e-14 N·m → 같은 관절각에서 좌우 토크가 부호까지 같음 = 제어기 · 토크 예산표를 한 팔 기준으로 써도 됨

## 3. 동작 (영점에서 +0.3 rad, 1.5 s 후 ee 변위 mm)

| joint | left Δ(x, y, z) | right Δ(x, y, z) | 거울(x 같음 · y 반대 · z 같음) | + 방향 의미 | 판정 |
|---|---|---|---|---|---|
| shoulder_pitch | [-175.85, -0.0, 26.57] | [-175.85, 0.0, 26.57] | True | pitch + = 팔 뒤로 (ee x 감소) — y축 오른손 법칙 (True) | OK |
| shoulder_roll | [0.0, 169.53, 28.34] | [0.0, -169.53, 28.34] | True | roll + = 바깥으로 벌림 (ee |y| 증가) (True) | OK |
| shoulder_yaw | [-2.66, -0.4, -0.0] | [-2.66, 0.4, -0.0] | True | 영점에서 팔이 수직이라 ee 거의 안 움직임 (축 = 팔 길이 방향) (True) | OK |
| elbow | [88.35, -0.0, 13.35] | [88.35, 0.0, 13.35] | True | elbow + = 앞으로 굽힘 (ee x 증가) (True) | OK |

## 4. 구동기 +방향(고정측 → 출력측) vs URDF 축 — B2 부호표 교차 확인

| joint | STEP 구동기 | 함태훈 CAD 축 · URDF 축 내적 | 축 각도차 ° | 함태훈 | 박종진 §2 | 일치 |
|---|---|---|---|---|---|---|
| left_shoulder_pitch | RI85:1 | 1.0 | 0.0 | 같음 | 같음 | OK |
| left_shoulder_roll | RI85:2 | 1.0 | 0.0 | 같음 | 같음 | OK |
| left_shoulder_yaw | RI85:3 | -0.9997 | 1.4 | 반대 | 반대 | OK |
| left_elbow | RI85:4 | 0.9997 | 1.4 | 같음 | 같음 | OK |
| right_shoulder_pitch | RI85:6 | -1.0 | 0.0 | 반대 | 반대 | OK |
| right_shoulder_roll | RI85:5 | -1.0 | 0.0 | 반대 | 반대 | OK |
| right_shoulder_yaw | RI85:7 | 0.9997 | 1.4 | 같음 | 같음 | OK |
| right_elbow | RI85:8 | -0.9997 | 1.4 | 반대 | 반대 | OK |

- 「반대」 4관절(left yaw, right pitch · roll · elbow)은 B2 드라이버 계층 한 곳에서만 q · τ 부호 반전 (팀장 10/16). MJCF 액추에이터 gear 는 전부 +1 로 두는 것이 맞음 — 여기서 반전하면 두 번 뒤집힘
- 축 각도차 1.37° = A1 편차(yaw · elbow, 회신 대기). 공칭 축은 직교 유지
