# v1 액추에이터 파라미터표 (자동 생성 — sim/v1/v1_actuator_table.py)

모델: 박종진 build_v1.py 빌드 `b1.xml`(값 = actuator_params.json), MuJoCo 3.15.0, timestep 0.002 s, integrator implicitfast. 왼팔 기준(오른팔 = 거울, 값 동일)

| joint | class | kp [N·m/rad] | dampratio → kv 컴파일값 [N·m·s/rad] | I_eff = 1/(M⁻¹)ᵢᵢ | Mᵢᵢ(qpos0) [범위] | ω_n=√(kp/Mᵢᵢ) | ζ_hold [범위] | OS 예측/시뮬 % | 정착 2 % [s] | forcerange (포화 오차) | ctrlrange [rad] | armature · damping · frictionloss |
|---|---|---|---|---|---|---|---|---|---|---|---|---|
| `{side}_shoulder_pitch_joint` | shoulder | 300 | 1 → 20.531 | 0.35128 | 0.84663 [0.04908~0.84928] | 18.8 | 0.644 [0.64~2.68] | 7.1 / 3.6 | 0.316 | ±100 N·m (19.1°) | -3~3 | 0 · 0.5 · 0 |
| `{side}_shoulder_roll_joint` | shoulder | 300 | 1 → 28.058 | 0.65606 | 0.73697 [0.47803~0.73693] | 20.2 | 0.944 [0.94~1.17] | 0.0 / 0.1 | 0.274 | ±100 N·m (19.1°) | -0.28~1.5 | 0 · 0.5 · 0 |
| `{side}_shoulder_yaw_joint` | shoulder | 300 | 1 → 4.251 | 0.01506 | 0.0153 [0.01529~0.07039] | 140.0 | 0.992 [0.46~0.99] | 0.0 / 0.0 | 0.044 | ±100 N·m (19.1°) | -2.5~2.5 | 0 · 0.5 · 0 |
| `{side}_elbow_joint` | elbow | 300 | 1 → 6.244 | 0.03249 | 0.06046 [0.06046~0.06046] | 70.4 | 0.733 [0.73~0.73] | 3.4 / 1.6 | 0.062 | ±100 N·m (19.1°) | -1.83~1.83 | 0 · 0.5 · 0 |

| joint | 수평 자세 τg [N·m] | 처짐 τg/kp [°] | 0.5° 기준 필요 kp | json ↔ 모델 일치 |
|---|---|---|---|---|
| `{side}_shoulder_pitch_joint` | 25.4 | 4.85 | 2911 | OK |
| `{side}_shoulder_roll_joint` | 0.0 | 0.0 | 0 | OK |
| `{side}_shoulder_yaw_joint` | 0.25 | 0.05 | 29 | OK |
| `{side}_elbow_joint` | 2.99 | 0.57 | 342 | OK |
