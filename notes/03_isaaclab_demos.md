# Isaac Lab 기초 데모 · 실행 명령

> 1주차 노션 정리본 이전. 모든 명령은 `IsaacLab/` 폴더에서 실행.

## 1. 환경 목록

```bash
./isaaclab.sh -p scripts/environments/list_envs.py
```
등록된 태스크 이름(예: `Isaac-Cartpole-v0`) 확인 → 이후 `--task` 인자로 사용.

## 2. 로봇 데모

| 데모 | 명령 | 확인할 것 |
|---|---|---|
| 로봇팔 | `./isaaclab.sh -p scripts/demos/arms.py` | 여러 로봇팔 에셋 로딩 |
| 이족보행 | `./isaaclab.sh -p scripts/demos/bipeds.py` | 이족 로봇 에셋 로딩 |
| H1 보행 | `./isaaclab.sh -p scripts/demos/h1_locomotion.py` | 학습된 정책으로 휴머노이드 보행 |

## 3. Cartpole zero / random agent

```bash
./isaaclab.sh -p scripts/environments/zero_agent.py   --task Isaac-Cartpole-v0 --num_envs 32
./isaaclab.sh -p scripts/environments/random_agent.py --task Isaac-Cartpole-v0 --num_envs 32
```

- `--num_envs 32`: 같은 환경 32개를 **GPU에서 병렬** 실행 → 학습 데이터 수집 속도가 환경 수만큼 늘어남.
- **zero_agent**: 행동 = 0 → 외력 없이 물리만 작동. 막대가 중력으로 쓰러지는지 = 물리 엔진 정상 확인.
- **random_agent**: 무작위 행동 → 행동이 관절에 전달되는지 = 행동 공간·액추에이터 연결 확인.
- 두 결과가 다르게 움직이면 "물리 + 제어 입력 경로"가 모두 정상.

## 4. 학습 순서 (대학원생 제공 예제 순서)

1. `create_empty.py` — 빈 장면
2. Cartpole — 환경·에이전트 구조
3. **URDF 임포트** — 우리 로봇 모델 올리기 (B1 핵심 업무)

## 5. B1 활용

- zero/random agent 점검은 **우리 로봇 모델을 새로 올릴 때마다** 첫 검증 절차로 재사용 가능 (모델이 물리적으로 안정한가 / 관절 명령이 먹는가).
