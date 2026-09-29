# Isaac Sim 5.1 + Isaac Lab 2.3.2 설치 절차

> 1주차 노션 정리본 이전. 실제 설치 시 겪은 오류·해결은 `logs/week02.md` §3에 기록.

## 1. 필요 환경

| 항목 | 버전 | 비고 |
|---|---|---|
| OS | Ubuntu 24.04 | |
| GPU | NVIDIA RTX 계열 | RT 코어 필요 (렌더링) |
| CUDA | 12.x | |
| Python | 3.11 | Isaac Sim 5.x 요구 버전 |
| Isaac Sim | 5.1.0 | 물리 엔진(PhysX) + 렌더링 + USD 장면 |
| Isaac Lab | 2.3.2 | Isaac Sim 위의 로봇 학습(RL/IL) 프레임워크 |

**버전을 맞추는 이유:** Isaac Lab은 Isaac Sim의 Python API를 직접 호출한다. Lab 릴리스마다 지원하는 Sim 버전이 정해져 있고, Sim 5.x는 Python 3.11로 빌드되어 있어 다른 조합이면 import 실패나 API 불일치가 생긴다.

## 2. 설치 순서 (팀 노션「개발환경 구축」기준 — Isaac Sim 바이너리 + 심볼릭 링크)

1. **Miniconda** 설치
   ```bash
   mkdir -p ~/miniconda3
   wget https://repo.anaconda.com/miniconda/Miniconda3-latest-Linux-x86_64.sh -O ~/miniconda3/miniconda.sh
   bash ~/miniconda3/miniconda.sh -b -u -p ~/miniconda3
   ~/miniconda3/bin/conda init bash
   conda config --set auto_activate_base false
   ```
2. **Isaac Sim 5.1 바이너리** 다운로드 후 압축 해제 (예: `/apps/nvidia/isaacsim/isaac-sim-standalone-5.1.0`)
3. **Isaac Lab 2.3.2** 클론 → Isaac Sim 폴더를 `_isaac_sim` 이름으로 **심볼릭 링크** (Lab이 Sim의 Python 환경·확장을 찾는 유일한 경로)
   ```bash
   git clone https://github.com/isaac-sim/IsaacLab.git -b v2.3.2
   cd /apps/nvidia/IsaacLab
   ln -s /apps/nvidia/isaacsim/isaac-sim-standalone-5.1.0 _isaac_sim
   ```
4. **conda 환경 생성 + 패키지 설치**
   ```bash
   ./isaaclab.sh --conda            # env_isaaclab 생성
   conda activate env_isaaclab
   ./isaaclab.sh --install
   ```
5. **테스트**
   ```bash
   ./isaaclab.sh -p scripts/tutorials/00_sim/create_empty.py
   ./isaaclab.sh -p scripts/reinforcement_learning/rsl_rl/train.py --task=Isaac-Ant-v0 --headless
   ```
   빈 장면 창이 뜨면 성공. 첫 실행은 셰이더/확장 캐시 생성으로 수 분 걸릴 수 있음.

- (대안) pip 방식: `pip install "isaacsim[all,extscache]==5.1.0" --extra-index-url https://pypi.nvidia.com` 후 `./isaaclab.sh --install` — 문서: https://isaac-sim.github.io/IsaacLab/v2.3.2/source/setup/installation/pip_installation.html
- ※ 2026-09-28 기준 실습 PC(RTX GPU) 미배정 → 2·3주차는 GPU 없이 MuJoCo·URDF·USD 커리큘럼으로 진행, Isaac Sim 설치는 PC 배정 후.

## 3. B1 활용

- 모든 B1 작업(URDF/MJCF 모델 임포트, 시뮬 환경 구성, 학습)의 전제 조건. 설치 버전은 팀원 간 동일해야 결과 재현 가능 → 보고서에 버전 표를 항상 첨부.

## 참고

- Isaac Lab 바이너리 설치 문서: https://isaac-sim.github.io/IsaacLab/v2.1.0/source/setup/installation/binaries_installation.html
- 팀 노션「캡스톤디자인 › 개발환경 구축」
