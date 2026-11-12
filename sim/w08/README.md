# sim/w08 — 그리퍼 · 접촉 모델 분석, 데이터 형식 v1.0 · 재생 검증, action 형식 비교, 파지 중 토크 예산 (week08, 함태훈)

시나리오 · 그리퍼 · 전문가 정책 · 수집 코드는 **박종진 PJJ0337/-1- `week08` (da55741) `scenarios/`** 를 그대로 쓴다(이 폴더 스크립트는 그 모듈의 `CONTACT` · `GRIPPER` · `OPTION` 값만 바꿔 실행). 모델 = 박종진 `b1/models/v1`(게인 통일 f061e4b 반영), 어깨 구동기 a2_doc.

| 파일 | 내용 | 결과 |
|---|---|---|
| `w08_seed1080_diag.py` | seed 1080(원기둥 2.98 kg) 들기 순간 접촉 · 관통 · 각속도 기록, dt 1 vs 0.5 ms | `results/seed1080_dt*.csv`, `assets/screenshots/week08/week08_seed1080_diag.png` |
| `w08_contact_sweep.py` | solref · solimp · impratio · condim · dt 15설정 × 무거운 물체(≥ 2 kg) 38건 | `results/contact_sweep.json` |
| `w08_contact_3kg.py` | 짐 3.0 kg 고정 20회 × 후보 4설정 | `results/contact_3kg.json` |
| `w08_contact_grid.py` | 박종진 `gripper_slip.trial` 장치로 solref × solimp × dt 96조합 관통 · 미끄러짐 | `results/contact_grid.json` |
| `w08_rerun100.py` | 박종진 수집 seed 1000 ~ 1099 를 권장 설정(dt 0.5 ms)으로 재실행 | `results/rerun100_dt05.json` (100/100) |
| `w08_contact_report.py` | 위 결과 정리 → 원인 표 · 그리퍼 · 접촉 v1.0 값 | **`results/contact_sweep.md`** |
| `b1_dataset.py` | 데이터 형식 v1.0 검사(`check`) · 초안 → v1.0 변환(`upgrade`) | 명세 = `docs/dataset_spec.md` |
| `w08_replay_check.py` | 박종진 샘플 3 · 변환본 3 · 새 수집 10 재생 검증 | `results/replay_check.md` |
| `w08_action_format.py` | action 관절 목표 vs 끝점 목표(IK 재생) 비교 10 에피소드 | `results/action_format.md` |
| `w08_grasp_torque.py` | 파지 중 토크 예산(3 kg @ 파지 중심 + 그리퍼 0.9 kg, 옆면 접근 실행 토크) | `results/grasp_torque.md`, `torque_budget_w08.csv` |

## 실행 (노트북 · CPU)

```bash
pip install mujoco==3.15.0 numpy scipy h5py matplotlib
git clone https://github.com/PJJ0337/-1-.git ../PJJ0337 && git -C ../PJJ0337 checkout week08   # 저장소 옆에 두면 기본 경로, 아니면 set PJJ=<경로>
python sim/w08/w08_seed1080_diag.py          # ~5 s
python sim/w08/w08_contact_sweep.py 1008,1027,1030,1052,1062,1077,1080,1081,1097,1107,1119,1129,1134,1135,1139,1146,1151,1154,1166,1169,1170,1175,1197,1202,1204,1208,1211,1216,1223,1233,1239,1247,1253,1257,1279,1282,1290,1299 all   # ~6 분
python sim/w08/w08_contact_3kg.py            # ~1 분
python sim/w08/w08_contact_grid.py           # ~40 s
python sim/w08/w08_rerun100.py               # ~1.5 분
python sim/w08/w08_contact_report.py         # 정리
python sim/w08/w08_replay_check.py 10        # ~40 s
python sim/w08/w08_action_format.py          # ~1 분
python sim/w08/w08_grasp_torque.py           # ~40 s (sim/w07/results 의 7주차 예산표를 읽음)
```

계산 환경: 클라우드 Linux, Python 3.13, mujoco 3.15.0 (박종진과 같은 버전 → 모델 지문 7016330767f5 일치), CPU 2개.
