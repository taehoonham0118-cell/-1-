# 1주차 기여 로그 — 함태훈 (B1)

- 평가일: 2026. 9. 18. (1차 평가자 김이겸)
- 게이트: 해당 없음 (×1)
- 저장소 태그: 없음 — 1주차는 Git 저장소 개설 전으로 **개인 학습 노션 페이지를 기여 로그로 인정**받음. 이 파일은 노션 기록을 저장소로 이전한 것.

## 1. 이번 주 계획 (전주 합의)

1. Isaac Sim / Isaac Lab 개요 및 설치 절차 이해
2. Isaac Lab 기본 개념(Core Concepts) 학습
3. 기초 데모 내용 파악

## 2. 실제 수행 · 산출물

| # | 수행 내용 | 완료율 | 산출물 |
|---|---|---|---|
| 1 | 노션「캡스톤디자인」정독. 필요 환경(Ubuntu 24.04 · CUDA 12 · Isaac Sim 5.1 · Isaac Lab 2.3.2)과 설치 순서(Miniconda → Isaac Lab → Isaac Sim 연결 → 패키지 설치 → 테스트) 정리. 설치는 미착수 | 100 % | [notes/01_isaac_install.md](../notes/01_isaac_install.md) |
| 2 | 환경 설계 두 방식(Manager-Based / Direct), 액추에이터, 센서 5종(카메라·접촉·좌표변환·IMU·레이캐스터), 모션 생성기 역할 정리 | 100 % | [notes/02_isaaclab_core_concepts.md](../notes/02_isaaclab_core_concepts.md) |
| 3 | 환경 목록 출력, 로봇팔·이족보행·H1 보행 데모, Cartpole zero/random agent 목적·실행 명령 확인 | 100 % | [notes/03_isaaclab_demos.md](../notes/03_isaaclab_demos.md) |

## 3. 구술 평가 (질문 · 답변 요지)

- **Q1.** Isaac Sim과 Isaac Lab은 각각 무엇을 담당하며, 왜 Sim 5.1 / Lab 2.3.2 / CUDA 12 버전을 맞춰야 하는가?
  - A. Isaac Sim은 물리 시뮬레이션을, Isaac Lab은 강화학습 환경을 담당. 호환성 오류 없이 정상 작동하도록 버전을 맞춰야 함.
- **Q2.** Cartpole 데모에서 `--num_envs 32`의 의미와 zero_agent / random_agent 비교로 확인할 수 있는 것은?
  - A. 32개 환경을 병렬 실행. 두 에이전트 비교로 물리 엔진과 제어 시스템이 올바르게 작동하는지 확인.

## 4. 평가자 피드백 → 반영 계획

| 피드백 | 반영 |
|---|---|
| 개념마다「B1 업무(URDF/MJCF 모델, 시뮬 환경)에서 어디에 쓰이는가」한 줄씩 붙일 것 | `notes/02` 각 개념에 **B1 활용** 줄 추가 |
| 핵심 수치(토크식, 센서 대역폭 등)를 근거와 함께 기록 → 구술 척도 4 조건 | `notes/02`에 수치 기록란 추가, 2주차 실행 후 채움 |
| 산출물은 노션 링크·파일명 명시 | 이 저장소 파일 경로로 명시 |
| 2주차부터 Git 저장소 주차별 태그 필수 | `week02` 태그부터 적용 |
| 학습 순서: create_empty → Cartpole → URDF 임포트 | 2주차 목표에 반영 |

## 5. 다음 주 목표 (합의안) → [week02.md](week02.md)
