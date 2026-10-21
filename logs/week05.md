# 5주차 기여 로그 — 함태훈 (B1)

- 기간: 2026. 10. 19. (월) ~ 10. 22. (목) 마감 (금: 구술 · 평가 피드백 반영). 작업은 10/17(토) 선행 — 팀장 10/16 「다음 주가 시험기간이지만 5주차 진행」
- 게이트: 해당 없음 (가중 1). 다음 주 6주차 = **PDR (가중 2배)**
- 저장소 태그: `week05` — **10/22 최종 제출 커밋에** 찍음. 제출 후 수정하면 릴리스를 다시 만들고 §7 에 기록
- 기준 커리큘럼: 노션 「B1 5·6주차 학습 커리큘럼 — 우리 로봇 MJCF 완성 · 검증 · 기구학 · 제어 (GPU 없이)」 §1 (5주차)
- 공용 모델: 박종진 v1 (`b1/models/v1`, 커밋 347a0ca → 10/21 652b89e: 함태훈 링크 관성 입력 채택 · PDR 기준 52) — 양팔 + 평면 3-DOF 베이스 + 그리퍼 임시 box, URDF → MJCF → 덧붙이기 빌드. 함태훈은 그 위에 「MJCF 덧붙이기 값 · 근거」와 검증 ① · ② · ③ 담당 (사용자 결정 10/17: 박종진 v1 위에 내 몫)
- 학습 노트: [notes/10](../notes/10_v1_mjcf_actuator_sensor_validation.md) · 주간 보고서: [reports/학부생_주간활동보고서_5주차_함태훈_22212334_261022.docx](../reports/학부생_주간활동보고서_5주차_함태훈_22212334_261022.docx)

## 1. 이번 주 계획 (커리큘럼 5주차 + 4주차 평가표 합의)

| 출처 | 계획 |
|---|---|
| 커리큘럼 개별 (함태훈) | MJCF 덧붙이기: `default class` 별 액추에이터(kp · kv · forcerange) · armature(BOM 감속비 · 로터 관성) · damping · 센서. 검증 ① 질량 · ② 정지 유지 · ③ 중력 토크 vs 정격 토크 표. 산출물 = `b1.xml` · `scene.xml`, 액추에이터 파라미터표, 검증 ①②③ 결과 |
| 커리큘럼 팀 연계 | 박종진 미러 규칙표로 좌측 팔 액추에이터 부호 · 범위 점검 |
| 커리큘럼 구술 | ① `qfrc_bias` 에 무엇이 들어 있고 왜 qvel = 0 에서 중력 토크인가 ② armature 유무에 따른 계단 응답 · 수치 안정성 (3주차 결과와 연결) |
| 4주차 평가표 합의 | 1. BOM · RI85 사양 수령 시 반영(수령 전 밀도 3종 유지) 2. 두 URDF 가정값 통일 → 총질량 · 수평 토크 같은 숫자 확인 3. 관절 범위 · 자기 충돌 점검, 오목 링크 convex 분해 비교 |
| 4주차 인계 | v1 첫 작업: 링크 원점을 박종진 골격 관절점에 맞춰 변환 없이 쓰게 |
| 4주차 보완할 점 | ① 보고서 수치에 조건 병기 ② FreeCAD 대조 범위 명시 ③ 금요일 과제는 박종진 충돌 스윕 활용 |

## 2. 실제 수행 · 산출물

| # | 수행 내용 | 완료율 | 산출물 |
|---|---|---|---|
| 1 | **v1 링크 원점 = 관절점**: 두 모델 cad_frame 기준 회전 차 2e-6° 확인 → COM · 메시 평행이동(축 방향 −22.14 · 19.00 · 22.15 · 17.79 mm). 박종진 빌드에 넣으면 팔 URDF 글자 하나 차이 없음 · 검증 결과 동일 → 이제 변환 없이 사용 | 100 % | [sim/scripts/rebase_v1_links.py](../sim/scripts/rebase_v1_links.py), [sim/robot_v1/](../sim/robot_v1/) (link_inertials_v1.json · meshes 10 · rebase_v1_report.csv) |
| 2 | **액추에이터 파라미터표**: 관절별 kp · 컴파일 kv · I_eff · Mᵢᵢ 자세 범위 · ω_n · 실효 감쇠비 · 계단 응답 · 포화 오차 · 수평 처짐 · 필요 kp. `dampratio` → kv = 2ζ√(kp/(M⁻¹)ᵢᵢ) 규칙 assert 확인. 값(C4 · C5) 유지 + 근거 기입한 `actuator_params.json` (박종진 빌드에 넣어도 b1.xml 동일) | 100 % (armature = RI85 대기) | [sim/v1/v1_actuator_table.py](../sim/v1/v1_actuator_table.py), [results/actuator_table.md](../sim/v1/results/actuator_table.md) · .csv, [handoff/actuator_params.json](../sim/v1/handoff/actuator_params.json), [week05_step_per_joint.png](../assets/screenshots/week05/week05_step_per_joint.png) |
| 3 | **검증 ① · ② · ③ 독립 계산**: ① 기준을 함태훈 CAD 질량표에서 다시 만들고 주관성 모멘트까지 ② 4자세 × 보상 3방식 ③ qfrc_bias 4가지 검산 + 손계산, 최악 자세 전역 최적화, 페이로드 0 / 0.5 / 1 / 3 kg · 작용점 2곳 | 100 % | [sim/v1/v1_validate_123.py](../sim/v1/v1_validate_123.py), [results/validate_123.md](../sim/v1/results/validate_123.md) · .json, [week05_hold.png](../assets/screenshots/week05/week05_hold.png) · [week05_gravity_torque.png](../assets/screenshots/week05/week05_gravity_torque.png) |
| 4 | **중력보상 문제 발견 · 해결안**: 목표 오프셋 방식(ctrl = q* + τg/kp)이 ctrlrange(= 관절 범위)에 잘려 roll 상한 자세에서 4.33° 남음 → MuJoCo `gravcomp` + `actuatorgravcomp` 방식으로 4자세 0°. 박종진 빌드용 옵션 패치(기본 꺼짐, `jointactuatorfrc` 센서 8개 포함) | 100 % (빌드 반영은 박종진 합의) | [handoff/build_v1_gravcomp_option.patch](../sim/v1/handoff/build_v1_gravcomp_option.patch), notes/10 §5.2 |
| 5 | **armature 실험 (구술 Q2)**: armature 0 ~ 0.5 × kv 정책 2종 계단 응답, integrator × timestep 안정 한계 (예측식 = 시뮬) | 100 % | [sim/v1/v1_armature_study.py](../sim/v1/v1_armature_study.py), [results/armature_study.md](../sim/v1/results/armature_study.md), [week05_armature_step.png](../assets/screenshots/week05/week05_armature_step.png) |
| 6 | **팀 연계**: 미러 규칙표로 생성 팔(right) 액추에이터 점검 17항목 OK — 값 · 축 규칙, 중력 토크 부호, +명령 동작 의미, B2 부호표(함태훈 CAD 축으로 교차) 8/8 일치 | 100 % | [sim/v1/v1_mirror_actuator_check.py](../sim/v1/v1_mirror_actuator_check.py), [results/mirror_actuator_check.md](../sim/v1/results/mirror_actuator_check.md) |
| 7 | 학습 정리: default class, position 힘 식 · dampratio, armature · damping · frictionloss 위치, 센서(가속도계 +g), qfrc_bias 구성, 중력보상 2방식, 적분 안정성 | 100 % | [notes/10](../notes/10_v1_mjcf_actuator_sensor_validation.md) |
| 8 | 재현용 공용 모델 사본(박종진 347a0ca → 10/21 652b89e 로 갱신, 결과 수치 변화 없음 — 몸통 COM 0.003 mm 만) → 이 저장소만으로 sim/v1 스크립트 실행 | 100 % | [sim/v1/b1_snapshot/](../sim/v1/b1_snapshot/) |
| 8-1 | **노트북 재현 (10/17)**: Windows 11 · Python 3.14.7 · MuJoCo 3.15.0 에서 검증 · 액추에이터표 · 미러 점검 재실행 → 핵심 수치 14개 클라우드와 차 0, 미러 17/17 | 100 % | [sim/v1/laptop_check.py](../sim/v1/laptop_check.py), [results_laptop/laptop_compare.md](../sim/v1/results_laptop/laptop_compare.md) |
| 8-2 | **(커리큘럼 수) 노트북 뷰어 캡처 (10/17)**: Control 슬라이더 11개 동작(left_shoulder_pitch 1.32 · right_elbow 1.04 → 목표 자세 유지, 중력보상 B) / 양팔 앞 수평 보상 없음 4.832° 처짐(ee z 1.1469 m) / 중력보상 B 0.000°(ee z 1.200 m) — 좌우 ee y ±0.2866 대칭 | 100 % | [sim/v1/view_v1.py](../sim/v1/view_v1.py), [sliders](../assets/screenshots/week05/week05_laptop_sliders.png) · [sag](../assets/screenshots/week05/week05_laptop_sag.png) · [gravcomp](../assets/screenshots/week05/week05_laptop_gravcomp.png) |
| 8-3 | **(커리큘럼 화, 공통 실습) 베이스 속도 명령 vs qvel (10/17)**: x · y 시정수 0.380 s(이론 M/kv = 75.10/200 = 0.3755), yaw 0.082 s(3.915/50 = 0.0783), 축 섞임 ≤ 2e-4. 로봇 기준 전진 + 회전(R(yaw) 변환): 전진 0.482 · 옆 0.091 m/s = 이론 v/(1+(ωτ)²) 0.483 · v·ωτ/(1+(ωτ)²) 0.091 | 100 % | [sim/v1/v1_base_velocity.py](../sim/v1/v1_base_velocity.py), [results/base_velocity.md](../sim/v1/results/base_velocity.md), [week05_base_velocity.png](../assets/screenshots/week05/week05_base_velocity.png) |
| 9 | (4주차 합의 2) 가정값 통일 | 100 % | 박종진 0653da5 에서 공용값 적용 시 74.093 = 74.096 kg 확인 → v1 부터 **한 벌**(공용 빌드 + 함태훈 link_inertials) |
| 10 | (4주차 합의 3) 관절 범위 · 자기 충돌 / convex 분해 비교 | 50 % | 범위 = 박종진 충돌 스윕(공용 C3, 검증 ④ 3,770자세 0건) 채택. convex 분해 비교는 6주차로 |
| 11 | (4주차 합의 1) BOM · RI85 사양 반영 | 50 % | **A2 사양서 10/21 수령**(팀장 단톡): 어깨 = TQ ILM-E85x30(정격 118.8 · 피크 381.7 N·m, armature 0.219), 팔꿈치 = RI85(72.6 · 181.5, 0.098). 영향 분석 완료 — ③ 실제 정격 대비 여유 pitch 177 %, armature 넣으면 yaw ω_n 140 → 35.8 rad/s. 공용 모델 반영은 박종진 합의 후 6주차 (로터만 값 · CAD 는 8개 모두 RI85 로 그려짐 → 확인 필요). BOM(그리퍼)은 팀장도 없음 → box 가정 유지 — [v1_a2_spec_impact.py](../sim/v1/v1_a2_spec_impact.py), [results/a2_spec_impact.md](../sim/v1/results/a2_spec_impact.md) |
| 12 | **검증 5종 종합 (노션 목요일 제출 필수)** — ① +0.004 % · ② gravcomp(B) 0° · ③ 52 기준 여유 21.2 % (함태훈, B1 공용 `validate_model.py` 와 대조 일치) / ④ 8관절 × 0.5° = 3,770자세 범위 안 자기 충돌 0건(한계 밖 첫 접촉까지 roll −2° · elbow −3°/+4°) · ⑤ 504자세(대표 4 + 무작위 500) \|Δx\| · \|Δz\| · \|y_L + y_R\| 최대 0 mm (박종진, 미러 생성이라 정확히 0. CAD 오른팔 편차 1.37° · 7.4 mm 는 joint-origin-check 별도) → **5종 모두 통과** | 100 % | ①②③ [sim/v1/v1_validate_123.py](../sim/v1/v1_validate_123.py) / ④⑤ 박종진 `b1/models/v1/scripts/validate_model.py` · `results/validate_model.md` (박종진 확인 10/22). 노션 ↔ 저장소 대응표 = [sim/v1/README.md](../sim/v1/README.md) |

### 실행 환경 · 명령 (재현용)

| 항목 | 값 |
|---|---|
| 계산 | 클라우드 Linux, Python 3.13, mujoco 3.15.0, scipy 1.18.1, matplotlib 3.11.2, trimesh |
| 모델 | 박종진 `build_v1.py` 빌드 (커밋 652b89e, 10/17 노트북 재현 시점은 347a0ca — 수치 동일) — 사본 `sim/v1/b1_snapshot/`, scene.xml timestep 0.002 s · implicitfast |
| 노트북 | Windows 11 · Python 3.14.7 · MuJoCo 3.15.0 (10/17 3.14.0 → 3.15.0 업그레이드), `py` 로 실행, 한글 경로는 `v1_common.load()` 가 메모리 로드 — `py sim/v1/laptop_check.py` · `py sim/v1/view_v1.py sliders|sag|gravcomp` |

```bash
pip install mujoco==3.15.0 trimesh scipy matplotlib
python sim/scripts/rebase_v1_links.py <박종진 저장소>/b1/models/v1/urdf/b1_v1.urdf
python sim/v1/v1_actuator_table.py
python sim/v1/v1_validate_123.py          # 약 35 s (전역 최적화)
python sim/v1/v1_armature_study.py
python sim/v1/v1_mirror_actuator_check.py
# 박종진 원본으로 돌리려면: set B1_V1=<박종진 저장소>\b1\models\v1   (Linux: export B1_V1=...)
```

### 핵심 수치 (조건 병기 — 4주차 보완 ①)

| 항목 | 값 | 조건 / 근거 |
|---|---|---|
| v1 모델 | nq = nv = 11, nu 11, 센서 30, **75.0965 kg** | 공용 가정값 v0.2 + 함태훈 CAD 팔 · 몸통, 그리퍼 0.5 kg × 2 (S14) |
| ① 질량 | +0.004 % (더미 프레임 2.9 g), 링크 13개 ≤ 0.0001 %, 주관성 모멘트 ≤ 0.0002 % | 기준 = 함태훈 CAD 질량표 + 가정값 (BOM 미수령). CAD 실제 오른팔 14.231 vs 거울 14.170 kg (+0.43 %) |
| kv 컴파일값 | pitch 20.53 · roll 28.06 · yaw 4.25 · elbow 6.24 N·m·s/rad | kp 300 · dampratio 1, I_eff = 1/(M⁻¹)ᵢᵢ at qpos0 (베이스 포함 전 자유도 자유) |
| 실효 감쇠비 ζ_hold | pitch **0.64** · roll 0.94 · yaw 0.99 · elbow 0.73 | 다른 관절 유지 시 관성 Mᵢᵢ 기준. pitch Mᵢᵢ 자세 범위 0.049~0.849 (17배) → 고정 kv 면 ζ 0.64~2.68 |
| 계단 응답 0.2 rad | OS pitch 3.6 · roll 0.1 · yaw 0 · elbow 1.6 %, 정착(2 %) 0.32 · 0.27 · 0.04 · 0.06 s | 영점, 다른 관절 유지, 중력보상, 최대 토크 60 N·m (= kp × 0.2) |
| ② 정지 유지 | 영점 0.045°, 앞 수평 보상 없음 **4.83°** (이론 τg/kp 4.85°) → 보상 0°. roll 상한(1.5 rad) 오프셋 보상 4.33° → gravcomp 0° | 5 s, kp 300, 양팔 같은 자세, 베이스 명령 0 · 밀림 0 mm, 떨림 RMS ≤ 3e-11 rad/s |
| 0.5° 기준 필요 kp | pitch ≈ 2,911 N·m/rad (보상 없이) | 수평 τg 25.40 N·m ÷ 0.5° |
| qfrc_bias 검산 | mj_rne 0 · 자코비안 전치합 8e-15 · 외적 합 8e-15 N·m | 무작위 200 자세, qvel = 0 |
| 손계산 (앞 수평) | 0 kg **25.399** vs 25.4035 · 3 kg 42.910 vs 42.914 N·m | Σ mᵢ g × 어깨축 아래 깊이, 차 = 더미 1 g. ee 수평 도달 0.595 m |
| qvel ≠ 0 | pitch 중력 −23.24 vs bias −28.54 N·m (코리올리 · 원심 −5.31) | 자세 (−90°, 0.3, 0.4, 0.8 rad), 4관절 3 rad/s. 감쇠 1.5 N·m 은 qfrc_passive |
| ③ 최악 중력 토크 (3 kg, ee) | pitch **42.92** · roll **39.77** · yaw 11.85 · elbow 11.78 N·m → 필요 정격 51.5 · 47.7 N·m | 관절 범위 안 differential_evolution × 시드 3. **PDR 기준 52 N·m(팀장 10/19) 통과**, 여유 21.2 · 30.8 % (10/16 기준 46 이었을 때는 7.2 · 15.7 % 미통과) |
| ③ 실제 구동기 정격 (A2 10/21) | 어깨 TQ 118.8 · 팔꿈치 RI85 72.6 N·m → 3 kg 대비 여유 pitch 177 % · roll 199 % · elbow 516 % | 사양서 §1 · §2. CAD 는 어깨도 RI85 로 그려져 있어 확인 필요 |
| A2 armature 적용 시 | ω_n pitch 18.8 → 16.8 · yaw 140 → 35.8 · elbow 70.4 → 43.5 rad/s, ζ_hold pitch 0.64 → 0.83 | 어깨 0.219 · 팔꿈치 0.098 (로터만), dampratio 1 재계산 kv 20.5 → 29.8 |
| ③ 페이로드별 pitch | 0 · 0.5 · 1 · 3 kg = 25.41 · 28.32 · 31.24 · 42.92 N·m | 3 kg 까지 × 1.2 ≤ 52 통과. 작용점 플랜지면 3 kg 39.98 (−2.94) |
| armature (kv 고정 20.53) | 0 → 0.1 → 0.5 kg·m²: OS 3.6 → 5.2 → **11.5 %**, ω_n 18.8 → 17.8 → 14.9 rad/s | shoulder_pitch 계단. dampratio 1 이면 OS 3.6 → 2.6 → 1.0 % (kv 20.5 → 32.0 자동) |
| 적분 안정 한계 (Euler) | armature 0 · 0.01 · 0.05 → 0.006 · 0.007 · 0.012 s (예측 0.0054 · 0.0071 · 0.0117) | shoulder_yaw 계단, implicitfast 는 0.03 · 0.04 · 0.05 s. 현재 dt 0.002 |
| 베이스 속도 응답 | x · y τ 0.380 s (이론 0.3755) · yaw 0.082 s (0.0783), 회전하며 전진 시 옆 미끄러짐 0.091 m/s (이론 0.091) | kv 200 / 50, 0.5 m/s · 0.5 rad/s 계단, 팔 영점 + 중력보상 B |
| 노트북 재현 | 14개 수치 차 0 (Windows 11 · Py 3.14.7 · MuJoCo 3.15.0) | results_laptop/laptop_compare.md |
| 미러 점검 | 17/17 OK, 좌우 중력 토크 차 1.8e-14 N·m, B2 「반대」 4관절 박종진 표와 8/8 일치 | 생성 팔 = right |

## 3. 문제 · 해결 과정

| 증상 | 원인 | 해결 |
|---|---|---|
| 박종진이 10/16 에 v1 · 검증 5종을 모두 올려 둠 — 함태훈 몫이 중복될 위험 | 커리큘럼 공지 전 박종진 선행 | 사용자 결정(10/17): 박종진 빌드를 공용 모델로 두고, 함태훈은 값 · 근거와 ①②③ 을 **다른 방법**으로 재계산해 대조 |
| `dampratio 1` 인데 pitch 계단에 오버슈트 3.6 % | kv 를 I_eff = 1/(M⁻¹)ᵢᵢ(다른 자유도 자유)로 계산 → 붙잡힌 실제 관성 Mᵢᵢ 보다 작음 (0.351 vs 0.847) | ζ_hold = 0.64 로 표에 병기, 6주차 kv 튜닝 시 자세별 Mᵢᵢ 범위 고려 |
| roll 상한 자세에서 중력보상을 켜도 4.33° 처짐 | 보상을 position 목표에 더하는 방식이라 ctrl 이 ctrlrange(= 관절 범위, inheritrange)에 잘림 | body `gravcomp` + joint `actuatorgravcomp` 로 qfrc_actuator 에 직접 → 0°. 빌드 옵션 패치 전달 |
| actuatorgravcomp 켠 뒤 `actuatorfrc` 센서가 0 | 보상분은 actuator_force 가 아니라 qfrc_actuator 에만 들어감 | 토크 기록용 `jointactuatorfrc` 센서 추가 (패치에 포함) |
| Euler 안정 한계가 armature 와 상관없이 0.005 s 에서 멈춤 | armature 를 shoulder class 에만 넣어 elbow(kv 명시적, 2I/kv ≈ 0.01)가 먼저 발산 | 팔 8관절 모두 적용, 1-DOF 반암시적 Euler 스펙트럼 반지름으로 예측 → 시뮬과 1 그리드 일치 |
| 15° 격자 최악 토크가 roll 에서 2.3 % 작음 | 최악 자세(yaw ≈ −55°)가 격자 사이 | scipy differential_evolution 전역 최적화 (판정은 같음) |
| 손계산과 MuJoCo 가 0.005 N·m 다름 | tool0 등 더미 프레임 질량 1 g × 0.5 m | 차이 원인 기록 (오차 아님) |
| 박종진 몸통 COM 이 내 rebase 와 0.003 mm 다름 | 박종진 merge 가 cad_frame 좌표의 dp 를 몸통 프레임에 그대로 더함 | 영향 없음(0.01 mm), 박종진 공유 메시지에 포함 |
| 노트북 `laptop_check.py` 첫 실행 FileNotFoundError (`sim/robot/link_mass_props.csv`) | 5주차_작업 폴더에 주차 신규 파일만 넣어 4주차 CAD 질량표 · 축 표가 빠짐 | 두 파일 추가 → 재실행 「재현 일치」. 저장소에는 원래 있음 |

## 4. B1 업무와의 연결

- 액추에이터 파라미터표의 **컴파일 kv 숫자**가 6주차 Isaac Lab `ImplicitActuatorCfg`(stiffness · damping · effort_limit · armature) 로 옮길 값 — Isaac 에는 dampratio 가 없음
- ③ 표 = 6주차 토크 예산표 「정적 최악 자세」 열, PDR A팀 피드백: 3 kg 이면 어깨 정격 ≥ 51.5 N·m 필요 → 팀장 10/19 기준 52 로 확정, A2 사양 어깨 정격 118.8 N·m 로 충족
- gravcomp 방식 B · jointactuatorfrc 센서 = 6주차 궤적 추종 · 페이로드별 토크 기록의 전제
- B2 부호표 「반대」 4관절을 CAD 에서 독립 확인 → 드라이버 계층 반전 대상 확정 자료

## 5. 구술 대비 (10/22 목, 2문항) — 상세: 구술 답변 파일 · notes/10 §4 · §6

- **Q1. qfrc_bias**: = c(q, q̇) + g(q) (가속 0 RNE, 코리올리 · 원심 + 중력). c 는 q̇ 에 2차라 q̇ = 0 이면 0 → 중력 토크만 남음. 감쇠 · 액추에이터 · 접촉은 별도 배열. 앞 수평 손계산 25.399 = MuJoCo 25.4035 N·m, 4방법 8e-15 일치. 3 rad/s 에선 코리올리 −5.31 N·m(23 %) 섞임
- **Q2. armature**: M 대각에 J_rotor·N² 가 더해짐 → kv 고정이면 ζ↓ · ω_n↓ (pitch 0 → 0.5: OS 3.6 → 11.5 %, 3주차 24 → 30.6 % 와 같은 방향), dampratio 면 kv 자동 증가로 OS 오히려 감소 · 느려짐만. 정상상태 오차 불변. 안정성: 가벼운 관절 ω_n 이 내려가 명시적 적분 한계 dt 가 커짐 (Euler 0.006 → 0.012 s)

## 6. 다음 주 목표 (6주차 · PDR, 커리큘럼 §2)

1. 궤적 생성 · 추종 스크립트(5차 다항식, 집기 시나리오) + 관절별 kp · kv 튜닝(오버슈트 · 정착, 자세별 관성 범위 고려) — 박종진 IK 모듈 인터페이스 합의
2. 페이로드 0 / 0.5 / 1.0 kg 토크 예산표(정격 · 피크 · 여유율 · 판정), 박종진 작업공간 결과의 최대 토크 도달 자세를 최악 조건으로
3. Isaac Lab 액추에이터 파라미터표 `docs/actuator_params.md`, gravcomp 옵션 빌드 반영 합의 — 마감 10/29(목), week06

## 6-1. 팀장 회신 · 결정 (10/16 카톡 · 4주차 평가표) → 반영

| 항목 | 내용 | 반영 |
|---|---|---|
| 손목 | A1: 손목 없음, 4-DOF 확정 (상위 보고 · 최종 사양 확인 중) | class = shoulder · elbow 2개 |
| 베이스 | C3 평면 3-DOF, URDF planar 1개 | 공용 v1 (박종진) |
| 축 부호 | 안 A(거울) 승인, 반전은 드라이버 한 곳 | gear +1 확인 (팀 연계 점검 4) |
| 토크 여유 | 정격 ≥ 46 (정격 기준), effort = 피크 → **10/19 52 N·m** (그리퍼 box 가정 유지, 박종진 652b89e assumptions v0.3) | ③ 판정 기준 → 통과 |
| 그리퍼 (10/19) | BOM 정보 없음 → 0.5 kg · 80×80×100 mm · TCP +0.10 m box 가정 유지 | 변경 없음 |
| A2 사양 (10/21) | 팀장 단톡 `A2_Actuator_Specs_for_B1.pdf` | 영향 분석만 (§2 #11), 공용 모델 반영은 6주차 |
| 공용 가정값 | assumptions v0.1 (C1~C8) | 검증 ① 기준 |
| 대기 | A1 편차(1.37° · 7 mm), +z 정면, 어깨 구동기 TQ ↔ CAD RI85 불일치(어깨 링크 질량도 바뀔 수 있음 — 함태훈 · 박종진 팀장 문의 10/22) | 가정값 유지 |

## 7. 제출 · 릴리스 기록

| 일시 | 내용 |
|---|---|
| 2026-10-17 | 작업 선행 (토). 저장소 업로드(6150e12) · 박종진 공유. 노트북 재현 · 뷰어 캡처 3장 추가 업로드 |
| 2026-10-17 | 베이스 속도(공통 화) 업로드 13b01ad |
| 2026-10-21 | A2 사양 영향 분석 · ③ 기준 52 반영 업로드 2236a1a. 박종진 5주차 마지막 커밋 = 0b50a7a (A2 사양 수령 기록) |
| 2026-10-22 | 박종진 회신 반영(검증 5종 대응 · ④⑤ 수치) — **이 커밋이 최종 제출 커밋 = `week05` 태그**. 보고서 카톡 제출 · 구술. 이후 수정은 week06 |
