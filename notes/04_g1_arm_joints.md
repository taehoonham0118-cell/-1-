# Unitree G1 (mujoco_menagerie) 어깨·팔꿈치 joint 정리 — week02

> 파일: `mujoco_menagerie/unitree_g1/g1.xml` · MuJoCo 3.14 로드 결과: body 31, joint 30, actuator 29, 총질량 33.34 kg
> 실행: `python -m mujoco.viewer --mjcf=mujoco_menagerie/unitree_g1/scene.xml`

| joint | axis (부모 body 기준) | range [rad] | range [deg] | actuatorfrcrange [N·m] | 액추에이터 |
|---|---|---|---|---|---|
| left_shoulder_pitch_joint | (0, 1, 0) | −3.0892 ~ 2.6704 | −177 ~ 153 | ±25 | position, kp=500, dampratio=1, gear=1 |
| left_shoulder_roll_joint | (1, 0, 0) | −1.5882 ~ 2.2515 | −91 ~ 129 | ±25 | 〃 |
| left_shoulder_yaw_joint | (0, 0, 1) | −2.618 ~ 2.618 | ±150 | ±25 | 〃 |
| left_elbow_joint | (0, 1, 0) | −1.0472 ~ 2.0944 | −60 ~ 120 | ±25 | 〃 |
| right_shoulder_pitch_joint | (0, 1, 0) | −3.0892 ~ 2.6704 | −177 ~ 153 | ±25 | 〃 |
| right_shoulder_roll_joint | (1, 0, 0) | −2.2515 ~ 1.5882 | −129 ~ 91 (좌우 대칭) | ±25 | 〃 |
| right_shoulder_yaw_joint | (0, 0, 1) | −2.618 ~ 2.618 | ±150 | ±25 | 〃 |
| right_elbow_joint | (0, 1, 0) | −1.0472 ~ 2.0944 | −60 ~ 120 | ±25 | 〃 |

- 기어비: `<motor gear>`가 아니라 `<position>` 액추에이터를 쓰며 gear 기본값 1 (= 관절 토크를 직접 지정). 힘 한계는 joint의 `actuatorfrcrange`로 건다.
- 공통 default(class `g1`): joint `armature=0.01`, `frictionloss=0.3` → 모터 회전자 관성과 쿨롱 마찰을 관절에 부여.
- 비교: 다리(무릎·고관절 roll)는 ±139 N·m, 발목 ±50 N·m → 팔(±25)보다 훨씬 큰 토크. 우리 로봇 목표(어깨 100 Nm급 / 팔꿈치 40 Nm급)는 G1 팔의 4배 수준.

## B1 활용
- 우리 로봇 MJCF/URDF에서 joint `range`·`actuatorfrcrange`는 A2(구동기 배분표)의 토크 사양과 ICD로 맞춰야 하는 값.
- `armature`·`frictionloss`는 Sim-to-Real 갭을 줄이는 파라미터 — 실측(다이나모) 후 채운다.
