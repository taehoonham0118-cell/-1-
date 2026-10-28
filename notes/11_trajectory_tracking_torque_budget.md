# 11. 궤적 추종 · kp/kv 튜닝 · 토크 예산 · Isaac 액추에이터 (week06, 함태훈)

코드 · 결과: [sim/w06/](../sim/w06/README.md). 모델 = 박종진 d6d6304 v1 빌드(A2 case a2_doc, 중력보상 B on), 사본 `sim/w06/b1_snapshot/`.

## 1. kp · kv 를 왜 다시 정했나

- week05 값: kp 300, `dampratio 1` → MuJoCo 가 **컴파일 시 qpos0 자세**의 관성으로 kv 를 계산. 자세가 바뀌면 감쇠비가 달라지고, Isaac Lab `ImplicitActuatorCfg` 에는 dampratio 가 없어 **고정 kv** 숫자가 필요함
- A2 armature(어깨 TQ 0.219, 팔꿈치 RI85 0.098) 반영 후 Mᵢᵢ 범위: pitch 0.268 ~ 1.068 kg·m²(3.98배, armature 없을 때 17배) → armature 가 바닥을 올려 고정 kv 가 쓸 만해짐
- 규칙(함태훈 제안, 팀 기준 아님)
  - R1 kp ≥ Δτ_1kg / 1° — 모르는 짐 1 kg 이 붙어도 정적 처짐 1° 이하. Δτ = 박종진 작업공간 최악 자세 0 → 1 kg 차(pitch 5.84 · roll 5.63 · yaw · elbow 2.93 N·m), 하한 C5 300
  - R3 0.2 rad 계단: 오버슈트 ≤ 5 %, 2 % 정착 ≤ 0.30 s — Mᵢᵢ 최소 · 중앙 · 최대 자세 × 두 case(a2_doc · cad_ri85)
  - R3 를 만족하는 kv 가 없으면 kp 를 50 씩 올림, 있으면 최악 정착시간이 가장 짧은 kv
- 결과: **pitch 400 · 31.0 / roll 350 · 27.5 / yaw 300 · 15.0 / elbow 300 · 11.0** (kp N·m/rad · kv N·m·s/rad). 최악 오버슈트 1.5 · 1.5 · 1.3 · 1.5 %, 정착 0.288 · 0.232 · 0.172 · 0.094 s → 통과. week05 기본(kp 300 · dampratio)은 pitch 정착 0.362 s 로 미통과
- 해석: 2차계 정착 ≈ 4/(ζω_n). pitch 는 무거운 자세(1.068)에서 ω_n = √(400/1.068) = 19.4 rad/s, ζ 0.75 → 0.27 s. 가벼운 자세(0.147)는 ζ 2.02 과감쇠로 느린 극 ω(ζ − √(ζ²−1)) 이 정착을 정함 → kv 를 너무 키우면 가벼운 자세가, 너무 줄이면 무거운 자세가 걸림 → 그 사이 최적점이 ζ(Mmax) ≈ 0.75
- 이론 오버슈트 e^(−πζ/√(1−ζ²)): ζ 0.80 → 1.5 % = elbow 시뮬 1.5 %

## 2. 집기 시나리오 (커리큘럼 수)

- 박종진 IK `solve_ik`(위치만, 연속 q_init) → pick_point (0.2551, 0.2867, 0.7508) m(base 기준, 4-DOF 로 수직 접근 되는 점) 위 0.10 m 에서 직선 하강 · 상승을 21 점 IK(모두 수렴, ≤ 0.62 mm)
- 접근 · 복귀는 관절공간 5차 다항식(최소 저크: 시작 · 끝 속도 · 가속도 0), 하강 · 상승은 직선 경로 위를 5차 시간척도로
- 짐: 「잡기」 순간 끝점 질점 질량을 켬(`body_mass` 런타임 변경). 중력보상 대상이 아님 = 모르는 짐
- 결과(짐 1 kg): 관절 추종 오차 최대 pitch 1.09° · elbow 3.94°, 끝점 15.4 mm, 잡은 뒤 처짐 5.5 mm, 최대 토크 pitch 5.14 · elbow 6.05 N·m, 최고 속도 1.90 rad/s(≤ 3.0)
- **오차의 두 성분**
  1. 속도 지연 e ≈ kv·q̇/kp — position 액추에이터 τ = kp(q_ref − q) − kv·q̇ 에서 −kv·q̇ 가 절대 속도에 걸리므로 움직이는 동안 뒤처짐. 이론 pitch 1.12° · elbow 3.98° = 시뮬 1.09 · 3.94°. 짐과 무관
  2. 정적 처짐 Δq = K⁻¹Jᵀmg, Δp = JΔq — 이론 0.5 · 1 · 3 kg 2.77 · 5.55 · 16.65 mm = 시뮬 2.75 · 5.49 · 16.22 mm
- 줄이는 법(1 kg): kp×2 → 처짐 2.8 mm · 짐 질량 피드포워드 → 0.0 mm · 속도 피드포워드(kv·q̇_ref) → 전 구간 15.4 → 5.8 mm · 둘 다 → 2.0 mm
- 베이스 동시 명령(접근 중 전진 0.3 m/s, 복귀 중 회전 0.3 rad/s): 끝점 15.4 → 16.1 mm, yaw 토크 0.03 → 0.36 N·m — 영향 작음
- cad_ri85: 차이 ≤ 0.1 — 이 속도에서는 armature × q̈ 가 작음

## 3. 토크 예산표 (A팀 피드백, PDR)

- 열: 정격 / 피크 · 정적 최악(박종진 `workspace_max_torque.json` = 5주차 ③) · 집기 동작 최대 · 뻗기 동작 최대(정적 최악 자세로 1.5 s 뻗고 유지) · 여유율 = 정격 / 최대 − 1 · 판정
- 최소 여유율(3 kg): a2_doc pitch **170 %**(43.95 / 118.8), cad_ri85 pitch **66 %**(/ 72.6) → 두 case OK. 1 kg: 277 % · 131 %
- 집기(테이블 높이) 토크는 정적 최악의 27 % — 예산을 정하는 것은 팔을 뻗은 자세
- PDR 52 기준(정격 ≥ 최악 × 1.2): 정적은 통과(21.2 %), 동적까지 넣으면 pitch 3 kg 18.3 %(1.5 s) · 12.7 %(1.0 s, 속도 한계 근처) → **주의**. 0.6 s(속도 한계 밖)면 57.19 N·m 로 초과 → 동작 시간 · 속도 제한과 같이 정해야 한다는 근거

## 4. Isaac Lab 액추에이터 (`sim/w06/docs/actuator_params.md`)

- stiffness = kp, damping = kv(고정값), effort_limit_sim = 피크(381.7 / 181.5), velocity_limit_sim = 3.0, armature = 0.219 / 0.098, friction = 0
- 주의: PhysX 관절에는 드라이브 외 점성 감쇠가 없음 → MuJoCo joint damping 0.5 는 damping 에 더하는 것으로 맞출 것(실습 PC 확인). Isaac Lab 에는 `actuatorgravcomp` 가 없음 → 중력 토크 피드포워드로(disable_gravity 는 토크 예산에서 중력이 빠짐)

## 5. Isaac Lab Manager-Based 대응 (금요일 공통 준비)

| Manager | 이번 주 MuJoCo 코드 |
|---|---|
| Action | `d.ctrl[u] = q_ref` (관절 position 목표) + 선택 속도 피드포워드 |
| Observation | jointpos · jointvel · jointactuatorfrc 센서, `{side}_ee` framepos |
| Reward | 추종 오차 ‖q − q_ref‖, 끝점 오차, 토크 크기 |
| Termination | 시간 종료, 관절 한계 · 자기 충돌(접촉 수) |
| Event | 잡는 순간 짐 질량 변경(`body_mass`) = 질량 무작위화와 같은 위치 |
