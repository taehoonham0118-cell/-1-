# 2주차 기여 로그 — 함태훈 (B1)

- 기간: 2026. 9. 19. ~ 2026. 9. 25. (실제 일정에 맞게 수정)
- 게이트: **SRR (×2)** — 가중치 2배 주차
- 저장소 태그: `week02`

> ⚠️ 작성 중. 아래 `[ ]`와 빈칸을 실제 수행 결과로 채운 뒤 `week02` 태그를 최종 커밋에 맞춘다.

## 1. 이번 주 계획 (1주차 평가 시 합의)

1. 실습 PC 배정 즉시 Isaac Sim 5.1 + Isaac Lab 2.3.2 설치 → `create_empty.py` 실행 확인 스크린샷 저장
2. Cartpole zero_agent / random_agent 직접 실행, 설치·실행 중 오류와 해결 과정 기록
3. (PC 미배정 시) Isaac Lab 공식 튜토리얼「Creating an empty scene」·「Interacting with an articulation」코드 정독 후 단계별 주석 문서화
4. Git 저장소 개인 폴더 생성, 노션 정리본을 md로 옮겨 `week02` 태그로 첫 기여 로그 업로드

※ 실습 PC(Ubuntu 24.04 · RTX GPU) 배정 시 1·2번 즉시 착수, 미배정 시 3번 우선

## 2. 실제 수행 · 산출물

| # | 수행 내용 | 완료율 | 산출물 |
|---|---|---|---|
| 1 | [ ] Isaac Sim 5.1 + Isaac Lab 2.3.2 설치, create_empty 실행 | % | `assets/screenshots/week02/week02_create_empty.png` |
| 2 | [ ] Cartpole zero/random agent 실행 | % | `assets/screenshots/week02/week02_cartpole_zero.png`, `..._random.png` |
| 3 | [ ] (PC 미배정 시) 튜토리얼 2종 주석 문서화 | % | `sim/scripts/` |
| 4 | [x] 저장소 구조 생성, 노션 1주차 정리본 md 이전 | 100 % | `notes/01~03`, `logs/week01.md` |

### 실행 환경 (재현용)

| 항목 | 값 |
|---|---|
| OS | Ubuntu 24.04 |
| GPU / 드라이버 | |
| CUDA | 12.x |
| Python (conda env) | 3.11 (`env_isaaclab`) |
| Isaac Sim | 5.1.0 |
| Isaac Lab | 2.3.2 |

### 실행 명령

```bash
./isaaclab.sh -p scripts/tutorials/00_sim/create_empty.py
./isaaclab.sh -p scripts/environments/zero_agent.py   --task Isaac-Cartpole-v0 --num_envs 32
./isaaclab.sh -p scripts/environments/random_agent.py --task Isaac-Cartpole-v0 --num_envs 32
```

### 핵심 수치

| 항목 | 값 | 조건 |
|---|---|---|
| Isaac Sim 첫 실행 로딩 시간 | | 셰이더 캐시 생성 전/후 |
| Cartpole 32 env 실행 시 FPS | | headless / GUI |

## 3. 문제 · 해결 과정

| 증상 (오류 메시지) | 원인 | 해결 |
|---|---|---|
| | | |

## 4. B1 업무와의 연결

- create_empty → Cartpole → **URDF 임포트** 순서의 첫 두 단계. 시뮬 환경이 정상 동작해야 이후 우리 로봇 URDF/MJCF 모델을 올릴 수 있음.

## 5. 다음 주 목표 (합의안)

1.
2.
