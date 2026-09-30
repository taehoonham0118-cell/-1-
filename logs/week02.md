# 2주차 기여 로그 — 함태훈 (B1)

- 기간: 2026. 9. 28. (월) ~ 10. 2. (금)
- 게이트: **SRR (×2)** — 가중치 2배 주차
- 저장소 태그: `week02`
- 기준 커리큘럼: 노션「B1 2·3주차 학습 커리큘럼 — MuJoCo · URDF · USD (GPU 없이)」(2026. 9. 28. 김이겸 공유)
- 주간 보고서: [reports/학부생_주간활동보고서_2주차_함태훈_22212334_261001.docx](../reports/학부생_주간활동보고서_2주차_함태훈_22212334_261001.docx)

> 1주차 평가 시 합의했던 "Isaac Sim 설치" 목표는 실습 PC(RTX GPU) 미배정으로 위 커리큘럼(GPU 없이 MuJoCo·URDF)으로 대체됨. 개인 노트북(Intel Arc 내장 GPU, Windows)에서 수행.

## 1. 이번 주 계획 (커리큘럼 2주차 + 함태훈 개별 과제)

| 요일 | 공통 학습 · 실습 | 개별 과제 (함태훈) |
|---|---|---|
| 월 | MuJoCo 설치, 뷰어 조작, Unitree G1 열어 관절 드래그 스크린샷 | |
| 화 | MJCF 구조, G1 어깨·팔꿈치 joint 축·범위·기어비 표 | |
| 수 | URDF 개념, 2링크 팔 URDF 손으로 작성 | 2링크 팔을 **URDF·MJCF 두 버전**으로 작성해 동작 비교 |
| 목 | URDF vs MJCF 차이, 작성한 URDF를 MuJoCo 뷰어로 열어 확인 | **관성값 변경 시 낙하 거동** 기록 |
| 금 | 주차 보고, Git week02 태그, 구술 점검 | SRR 기여: **기보유 하체 실측표** |

## 2. 실제 수행 · 산출물

| # | 수행 내용 | 완료율 | 산출물 |
|---|---|---|---|
| 1 | Python 3.14.7 + mujoco 3.14.0 · usd-core 26.8 · yourdfpy 설치. Git 미설치 상태라 `fetch_g1.py`로 menagerie `unitree_g1` 63파일 직접 다운로드. 뷰어에서 Pause 후 Joint 슬라이더로 left_shoulder_pitch −1.07 rad, right_shoulder_pitch 0.57 rad, right_shoulder_roll −0.79 rad 이동 | 100 % | [week02_g1_viewer.jpg](../assets/screenshots/week02/week02_g1_viewer.jpg), [sim/scripts/fetch_g1.py](../sim/scripts/fetch_g1.py) |
| 2 | G1 어깨 3축 · 팔꿈치 joint의 axis · range · 토크 한계 · 액추에이터 표 (`g1.xml` 파싱) | 100 % | [notes/04_g1_arm_joints.md](../notes/04_g1_arm_joints.md) |
| 3 | 2링크 팔 URDF · MJCF 작성 (링크 0.3 m · 1 kg × 2, 어깨 ±150°, 팔꿈치 −60°~120°, 감쇠 0.1 N·m·s/rad). 두 파일 모두 MuJoCo 뷰어에서 열어 shoulder 1.26 / elbow 0.853 rad 자세 확인 | 100 % | [sim/urdf/two_link_arm.urdf](../sim/urdf/two_link_arm.urdf), [sim/mjcf/two_link_arm.xml](../sim/mjcf/two_link_arm.xml), [mjcf 캡처](../assets/screenshots/week02/week02_two_link_arm_mjcf.jpg), [urdf 캡처](../assets/screenshots/week02/week02_two_link_arm_urdf.jpg) |
| 4 | URDF vs MJCF 동작 비교 + 관성 변경 실험 스크립트 작성·실행 (노트북에서 재현) | 100 % | [compare_urdf_mjcf.py](../sim/scripts/compare_urdf_mjcf.py), [results.csv](../sim/scripts/results.csv), [run_log.txt](../sim/scripts/run_log.txt) |
| 5 | 기보유 하체(홀로노믹 베이스) 실측표 양식 작성, 자료 폴더 CAD 렌더·주행 영상으로 4륜 옴니휠 구성 사전 확인 | 30 % (보류) | [logs/week02_base_measurement.md](week02_base_measurement.md) — **9/30 팀장 지시로 보류**: G1 시뮬 구조 이해 우선, CAD·BOM 완성 후 재개 |

### 실행 환경 (재현용)

| 항목 | 값 |
|---|---|
| OS / PC | Windows 11, Intel Core Ultra 7 258V, 32 GB, Intel Arc 140V (NVIDIA GPU 없음) |
| Python | 3.14.7 (python.org Python install manager) |
| mujoco / usd-core / yourdfpy | 3.14.0 / 26.8 / 0.0.60 |
| G1 모델 | mujoco_menagerie `unitree_g1` (main, 63 files) |

### 실행 명령

```bash
pip install mujoco usd-core yourdfpy
python sim/scripts/fetch_g1.py                                       # Git 없을 때 G1 폴더만 받기
python -m mujoco.viewer --mjcf=mujoco_menagerie/unitree_g1/scene.xml
python -m mujoco.viewer --mjcf=sim/mjcf/two_link_arm.xml
python -m mujoco.viewer --mjcf=sim/urdf/two_link_arm.urdf            # MuJoCo가 URDF를 직접 읽음
python sim/scripts/compare_urdf_mjcf.py                              # → results.csv
```

### 핵심 수치

**(1) URDF vs MJCF 동작 비교** — 어깨 90°(수평)에서 놓아 5 s 자유낙하, timestep 1 ms

| 항목 | URDF | MJCF |
|---|---|---|
| 최하점 첫 통과 시각 | 0.390 s | 0.390 s |
| 어깨 최대 각속도 | 6.10 rad/s | 6.10 rad/s |
| 5 s 관절각 최대 차이 | shoulder 1×10⁻¹⁵ rad, elbow 1×10⁻¹⁵ rad (수치 오차 수준 → **동일**) | |

- 해석 검산 (팔꿈치 잠금 단진자): 어깨 기준 관성 I = 2×0.0075 + 1×0.15² + 1×0.45² = **0.24 kg·m²**, 최대 중력토크 τ = 9.81×(0.15+0.45) = **5.886 N·m**, 무감쇠 최대 각속도 √(2τ/I) = **7.00 rad/s** (시뮬 6.53 rad/s — 감쇠 0.1 때문에 감소).
- 포맷 차이로 확인한 것: URDF 로드 시 `nu=0`(액추에이터 없음, MuJoCo는 `<transmission>` 무시) → 뷰어에 Control 패널 없음. 월드 고정 베이스 링크의 `<inertial>`은 무시됨(body 3개 vs MJCF 4개).

**(2) 관성 변경 실험** — MJCF, 팔꿈치 잠금(단진자), 어깨 90°에서 놓음

| 조건 | 링크 질량 [kg] | 첫 통과 [s] | 최대 각속도 [rad/s] | 4~5 s 잔류 진폭 [rad] |
|---|---|---|---|---|
| 기준 (m=1, I=0.0075) | 1.0 | 0.385 | 6.53 | 0.592 |
| 질량 ×5 + 관성 ×5 | 5.0 | 0.376 | 6.90 | 1.241 |
| 관성만 ×5 (m=1 유지) | 1.0 | 0.429 | 5.88 | 0.680 |
| 관성만 ×0.2 | 1.0 | 0.375 | 6.69 | 0.578 |
| `<inertial>` 생략 (geom 밀도 1000 kg/m³ 자동) | 0.377 | 0.405 | 5.85 | 0.140 |

- **관성만 키우면 느려진다** (τ/I 감소: 0.385 → 0.429 s, 6.53 → 5.88 rad/s).
- **질량과 관성을 같이 키우면 궤적은 거의 같다** (중력토크와 관성이 같이 5배 → 상쇄). 감쇠 0.1은 그대로라 상대적으로 약해져 잔류 진폭만 2배로 커짐(0.59 → 1.24 rad).
- `<inertial>` 생략 시 MuJoCo가 geom 부피×밀도로 0.377 kg을 자동 계산 → 우리 로봇 URDF에는 반드시 실측 질량·관성을 넣어야 함 (A4 실측값 → B1).

## 3. 문제 · 해결 과정

| 증상 | 원인 | 해결 |
|---|---|---|
| 배치 파일이 실행 직후 꺼짐 | `.bat` 안의 `chcp 65001`(UTF-8 전환) 후 한글 파싱 오류 | 배치를 영문 전용으로 작성, 진행 로그를 `setup_log.txt`에 남김 |
| `python` 명령을 못 찾음 | python.org 새 설치 방식(Python install manager)은 `%LocalAppData%\Python\bin`에 설치, 첫 실행 전 런타임 없음 | 해당 경로 탐색 + `py install default` 자동 실행 |
| Git 미설치로 `git clone` 불가 | Git 미설치 | `fetch_g1.py`로 raw.githubusercontent.com에서 `unitree_g1` 63파일만 직접 다운로드 |
| 2링크 팔이 t=0에 어깨에 큰 접촉력 (qacc −873 rad/s²) | MuJoCo는 **월드에 고정된 body는 자식과 부모-자식 충돌 제외가 적용되지 않음** → 베이스 박스와 위팔 실린더가 충돌 | 베이스 geom `contype=0 conaffinity=0`, URDF는 베이스 collision 제거 |
| 팔이 위로 올라가 관절 한계에 걸림 | 링크를 +Z(위) 방향으로 모델링해 q=0이 거꾸로 선 불안정 자세였음 | 링크를 −Z(아래로 늘어짐)로 재정의, q=0 = 안정 자세 |
| 첫 비교에서 URDF·MJCF 차이 0.36 rad | MJCF에만 바닥 plane과 위치 액추에이터(kp=20, 목표 0)가 있었음 | 바닥 제거, 비교 시 액추에이터 gain 0으로 → 차이 10⁻¹⁵ rad |
| 하체 실측표를 채울 수 없음 | 하체 CAD 공유 불가(전종욱 선배), 실측 일정 미정 | 9/30 팀장(김이겸)에게 문의 → "실제 로봇 전에 시뮬 구조부터 이해, MuJoCo·Isaac Sim에 G1을 먼저 넣어 학습. 로봇 CAD·BOM은 완성·구축 후 시뮬 구현 가능" → **실측 보류**, 양식만 유지 |

## 4. B1 업무와의 연결

- URDF `origin`/`axis`, `inertial` 작성법은 4주차 우리 로봇 URDF(A팀 CAD → URDF) 작성의 기본. 관성 실험 결과가 "실측 질량·관성 없이는 시뮬 거동을 믿을 수 없다"는 ICD 요청 근거가 됨.
- 월드 고정 베이스 충돌 문제 → 우리 로봇의 홀로노믹 베이스를 `fixed`로 둘지 `floating`으로 둘지 규약 필요 (팀장 질문 사항).
- G1 팔 토크 ±25 N·m vs 우리 목표 어깨 100 Nm급 → 우리 로봇 관절 `actuatorfrcrange`는 A2 구동기 배분표 기준으로 설정.

## 5. 구술 대비 (금요일 질문 2개)

- **Q1. URDF joint의 `origin`과 `axis`는 각각 무엇을 기준으로 하는가?**
  `origin`은 **부모 링크 프레임** 기준으로 자식 링크(=joint) 프레임의 위치·자세(xyz, rpy). `axis`는 **그 joint 프레임(자식 링크 프레임)** 기준 회전축 단위벡터. 예: 이번 팔꿈치는 `origin xyz="0 0 -0.3"`(위팔 끝), `axis="0 1 0"`(자식 프레임 Y축).
- **Q2. `revolute`와 `continuous`의 차이는?**
  둘 다 1축 회전이지만 `revolute`는 `<limit lower upper effort velocity>`가 **필수**(가동 범위 있음, 어깨 ±2.618 rad · effort 25 N·m), `continuous`는 각도 한계가 없는 무한 회전(바퀴·옴니휠)으로 limit에 각도가 없음. MuJoCo는 둘 다 hinge로 읽고 revolute만 `range`를 건다.
- 예상 추가: "관성 5배면 왜 느려지나" → 각가속도 = τ/I. 질량까지 5배면 τ도 5배라 상쇄.

## 6. 다음 주 목표 (커리큘럼 3주차)

1. usd-core로 큐브 `.usda` 생성 → 2링크 팔을 Xform 계층 USD로 작성, `RevoluteJoint` 2개 + `DriveAPI` 추가
2. 2주차 URDF ↔ USD 대응표 (질량·관성·joint origin/axis가 어느 prim 속성으로 가는지), 박종진 임포트 체크리스트로 내 URDF 점검
3. CAD → URDF → USD(Isaac Sim) / MJCF(MuJoCo) 파이프라인 그림 1장, 구술 준비(reference vs sublayer, inertial → USD 속성), Git `week03` 태그
4. G1(MuJoCo)으로 시뮬 구조 이해 계속 — 하체 실측표는 CAD·BOM 완성 후 재개 (9/30 팀장 지시)
