# B1 시연 데이터 형식 명세 v1.0 (`b1-dataset-1.0`)

- 작성: 함태훈 (week08, 2026-11-12) · 기준: 박종진 초안 v0.1 `datasets/dataset_spec_draft.md` (PJJ0337/-1- week08 = da55741)
- 원칙: 박종진 초안의 필드 이름 · 단위 · 순서는 **그대로 유지**(재수집 부담 최소). v1.0 은 ① 로봇 좌표 파생 필드 4개 ② 속성 5개 ③ 성공 시 `fail = ""` 만 추가 · 정리했다.
- 검사 · 변환 도구: `sim/w08/b1_dataset.py` — `check`(위반 목록), `upgrade`(초안 → v1.0, 원본 배열은 손대지 않음 → 재생 결과 불변)
- 재생 검증 결과: `sim/w08/results/replay_check.md`

## 1. 파일 · 시간 축

| 항목 | 규칙 |
|---|---|
| 단위 | 에피소드 1개 = HDF5 1개 `ep_<seed 5자리>.hdf5`, gzip 4 |
| 커밋 | 대용량 원본(`datasets/raw/`)은 커밋하지 않음(.gitignore). 저장소에는 **샘플 3개**(1개는 카메라 포함)만 |
| 시간 | 제어 주기 50 Hz (`control_hz`), 시뮬 dt 0.5 ms (`sim_dt` = 0.0005) → 주기당 40 스텝 (B1 그리퍼 · 접촉 v1.0, `sim/w08/results/contact_sweep.md`). 길이 T = 제어 주기 수 |
| 관측 · 행동 짝 | `obs/*[t]` = t번째 명령을 내기 **직전** 상태, `action[t]` = 그 뒤 0.02 s 동안 유지한 명령 |
| 좌표 · 단위 | world = MuJoCo 월드(+x 전방, +z 위, REP 103). 로봇 좌표 = 베이스 x · y · yaw 기준(높이는 world 와 같음). m · rad · s · kg, 쿼터니언 [w, x, y, z] |
| 형 | 재생 기준인 `obs/qpos` · `action` · `ctrl` · `time` · `final_qpos` 는 f64, 나머지 관측은 f32, 이미지 u8 |

## 2. 필드 (★ = v1.0 에서 추가)

| 경로 | 모양 · 형 | 필수 | 내용 |
|---|---|---|---|
| `obs/qpos` | (T, 27) f64 | ● | 베이스 TX · TY · RZ(3) · 왼팔 4 · 왼손가락 2 · 오른팔 4 · 상자 free 7 · 원기둥 free 7 |
| `obs/qvel` | (T, 25) f32 | ● | 같은 순서(free joint 6) |
| ★ `obs/base_pose` | (T, 3) f32 | ● | [x, y, yaw] = `qpos[:, 0:3]` (학습 입력용 사본) |
| `obs/ee_pos`, `obs/ee_quat` | (T, 3) · (T, 4) f32 | ● | 왼손 site `left_ee`(손바닥 아래면 중심), world. 파지 중심 = ee − 0.045 ẑ_ee |
| ★ `obs/ee_pos_base` | (T, 3) f32 | ● | `ee_pos` 를 로봇 좌표로 |
| `obs/obj_pos`, `obs/obj_quat` | (T, 3) · (T, 4) f32 | ● | 집는 물체 몸체 위치 · 자세, world |
| ★ `obs/obj_pos_base` | (T, 3) f32 | ● | `obj_pos` 를 로봇 좌표로 |
| `obs/gripper_width` | (T,) f32 | ● | 손가락 안쪽 면 사이 거리 [m] (0 ~ 0.11) |
| `obs/stage` | (T,) i8 | ● | 상태기계 단계 번호(`stage_names`). 분석 · 분할용 — **학습 입력에 넣지 않음** |
| `obs/images/overview` | (T, 96, 96, 3) u8 | 선택 | 고정 카메라 `overview` |
| `action` | (T, 8) f64 | ● | `[base_vx, base_vy, base_wz, left_shoulder_pitch, left_shoulder_roll, left_shoulder_yaw, left_elbow, gripper_opening]` — 베이스 = **world** 속도 명령, 팔 = **관절 위치 목표**(rad), 그리퍼 = 손가락 1개 벌림 목표(0 ~ 0.055 m). 재생의 기준(실제로 넣은 값) |
| ★ `action_base_body` | (T, 3) f32 | ● | 베이스 명령을 로봇 좌표로: `[vx_b, vy_b] = R(yaw_t)ᵀ [vx, vy]`, wz 그대로 (Isaac Lab · 실기 베이스 명령과 같은 좌표) |
| `ctrl` | (T, 13) f64 | ● | 실제 `d.ctrl` 전체(액추에이터 순서) |
| `time` | (T,) f64 | ● | 시뮬 시간, 간격 = 1/`control_hz` |
| `final_qpos` | (27,) f64 | ● | 마지막 명령 뒤 상태(재생 끝점 비교) |

## 3. 속성 (`f.attrs`)

| 키 | 내용 |
|---|---|
| `spec` | `"b1-dataset-1.0"` |
| `seed` · `kind` · `mass` · `obj_xy` · `obj_yaw` | `pick_env.sample_task(seed)` 와 같은 값(같은 seed = 같은 초기 상태) |
| `success` · `fail` · `fail_stage` | 성공이면 `fail = ""` · `fail_stage = ""` (★ 초안은 `"None"` 문자열). 실패 분류 10종 = 박종진 `expert.FAIL` |
| `goal_err_mm` · `t_total` · `events` | 목표 오차, 길이, 단계 전환 시각 |
| `control_hz` · `sim_dt` | 50 · 0.0005 (재생 시 주기당 스텝 = 1/(control_hz · sim_dt) = 40) |
| `model_hash` | 컴파일 모델 sha256 앞 12자리 — 다르면 재생 보장 안 함 |
| ★ `mujoco_version` | 재생 결정성은 같은 버전 · 같은 기기 · 단일 스레드에서만 보장 |
| `action_names` · ★ `action_body_names` · `stage_names` | JSON 문자열 |
| `params` | 그리퍼 · 접촉 · 테이블 · 목표 · 물체 · option · 무작위 범위 JSON |
| ★ `assumptions` | 가정값 항목 목록(그리퍼 · 접촉 · 테이블 · 물체 · 어깨 구동기 case) — 수치 인용 시 같이 적음 |
| ★ `source` | 수집 코드 저장소 · 커밋 · 스크립트 |

## 4. 재생(replay) 규칙 · 합격 기준

1. `m = pick_env.build(cam=True)`, `reset(m, d, sample_task(seed))`, 지문 `model_hash` 일치 확인
2. t = 0 … T−1: `apply_action(m, d, action[t])` → `mj_step` 40번 (= `expert.NSUB`) (`action_base_body` 는 학습용, 재생에는 쓰지 않음)
3. 매 t 에서 `d.qpos` vs `obs/qpos[t]`, 끝에서 `final_qpos` 비교
4. **합격 = 최대 절대 차 0**(같은 버전 · 기기) · 재생 성공 판정 = 저장 판정 · 지문 일치. 다른 기기 · 버전이면 차를 기록하고 성공 판정 일치로 대신함

## 5. action 형식 결정 (박종진과 9주차 BC 입력 전 합의할 것)

- v1.0 저장 형식 = **관절 목표**(재생이 비트 단위로 같음, 실기 서보에 그대로). 끝점 목표는 저장하지 않고 필요할 때 **FK 로 만든다**(같은 파일에서 결정적으로 계산 가능) — 근거 `sim/w08/results/action_format.md`
- BC 기본 입력(제안): `obs/qpos` 중 팔 4 + 손가락 1, `obs/qvel` 팔 4, `ee_pos_base`, `obj_pos_base`, `obj_quat`, `gripper_width` → 출력 `action[3:8]` + `action_base_body`

## 6. LeRobot 형식 대응 (9주차 로더에서)

| LeRobot | B1 v1.0 |
|---|---|
| `observation.state` | 위 BC 기본 입력을 이어 붙인 벡터 |
| `observation.images.overview` | `obs/images/overview` |
| `action` | `[action_base_body, action[3:8]]` |
| `episode_index` · `frame_index` · `timestamp` | 파일 순번 · t · `time` |
| `next.success` | 마지막 프레임 = `success` |

## 7. 판 이력

| 판 | 날짜 | 내용 |
|---|---|---|
| draft 0.1 | 11/12 | 박종진 초안(수집 100 · 재생 100/100) |
| **1.0** | 11/12 | 함태훈 확정: 초안 필드 유지 + 로봇 좌표 파생 4개 · 속성 5개 · 성공 시 fail 빈 문자열. 검사 · 변환 도구 `b1_dataset.py` |
| 1.0 (문서 정정) | 11/12 | 시간 항목 dt 1 ms · 20 스텝 → **0.5 ms · 40 스텝**(박종진 지적, 파일 `sim_dt` 속성은 이미 0.0005). 필드 · spec 문자열 변경 없음 |
