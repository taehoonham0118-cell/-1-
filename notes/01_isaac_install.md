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

## 2. 설치 순서

1. **Miniconda** 설치 → 가상환경 생성
   ```bash
   conda create -n env_isaaclab python=3.11
   conda activate env_isaaclab
   ```
2. **Isaac Sim** 설치 (pip 방식)
   ```bash
   pip install "isaacsim[all,extscache]==5.1.0" --extra-index-url https://pypi.nvidia.com
   ```
   - 바이너리(압축 해제) 방식으로 설치했다면 Isaac Lab 폴더에서 심볼릭 링크로 연결: `ln -s <isaac-sim 경로> _isaac_sim`
3. **PyTorch** (CUDA 12.8 빌드)
   ```bash
   pip install -U torch==2.7.0 torchvision==0.22.0 --index-url https://download.pytorch.org/whl/cu128
   ```
4. **Isaac Lab** 클론 및 패키지 설치
   ```bash
   git clone https://github.com/isaac-sim/IsaacLab.git --branch v2.3.2
   cd IsaacLab
   ./isaaclab.sh --install
   ```
5. **테스트**
   ```bash
   ./isaaclab.sh -p scripts/tutorials/00_sim/create_empty.py
   ```
   빈 장면 창이 뜨면 성공. 첫 실행은 셰이더/확장 캐시 생성으로 수 분 걸릴 수 있음.

## 3. B1 활용

- 모든 B1 작업(URDF/MJCF 모델 임포트, 시뮬 환경 구성, 학습)의 전제 조건. 설치 버전은 팀원 간 동일해야 결과 재현 가능 → 보고서에 버전 표를 항상 첨부.

## 참고

- Isaac Lab 설치 문서: https://isaac-sim.github.io/IsaacLab/main/source/setup/installation/pip_installation.html
