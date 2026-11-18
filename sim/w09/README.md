# sim/w09 — 도메인 랜덤화 · 민감도, BC 평가 하네스 (week09, 함태훈)

시나리오 · 전문가 정책 · 성공 판정은 **박종진 PJJ0337/-1- 552614e `scenarios/`**(박종진 week08 최종, dt 0.5 ms)를 그대로 쓴다. 이 폴더 스크립트는 모델을 빌드한 뒤 **컴파일된 모델 값만** 바꿔 실행(생성 MJCF 는 손대지 않음).

| 파일 | 내용 | 결과 |
|---|---|---|
| `randomize.py` | 파라미터 17개(물리 14 · 수치 3) 범위 · 근거(가정값 표 v0.4), 모델 적용(`make_model`), 측정 편향 전문가(`BiasedExpert`), 명령 지연 버퍼, 모드 `oat`(하나씩 ×1) · `oat2`(×2 스트레스) · `joint`(동시 ×0 · 0.5 · 1 · 2) · `curve`(Top 3 세부) | `results/randomize_{oat,oat2,joint,curve_*}.json` |
| `randomize_report.py` | 민감도 순위 · Top 3 · 이론 검산 · 그림 | **`results/randomize.md`**, `results/sensitivity.json`, `assets/screenshots/week09/week09_sensitivity.png` · `week09_top3_curves.png` |
| `w09_clearance.py` | 공칭 파지 순간 손끝 ↔ 테이블 · 손바닥 ↔ 물체 윗면 여유 실측(테이블 높이 민감도 근거) | `results/clearance.json` (손끝 16 mm, 원기둥 11 mm, 상자 16 mm) |
| `bc_eval_pjj.py` | **팀 연계** — 박종진 BC(99d60da `bc_joint_dart_c25.pt`)를 이 랜덤화로 평가: 박종진 `eval_bc.run()` 그대로 + 물체 질량 · 측정 편향 · 지연 감싸기. 모드 기본(전문가 vs BC × 7조건, seed 2000 ~ 2049) · `oat`(BC 하나씩) | `results/bc_eval_pjj.json` · `bc_eval_pjj_oat.json` |
| `bc_eval_pjj_report.py` | 위 결과 → 실패 ↔ 민감 파라미터 표 | **`results/bc_eval_pjj.md`** |
| `pt_numpy.py` | PyTorch 없이 박종진 .pt 체크포인트를 numpy 로 읽어 같은 MLP 추론(클라우드에 torch 설치 불가) — 노트북에서는 박종진 `train_bc.Policy` 그대로 써도 됨 | 랜덤화 없음 18/50 = 박종진 결과와 같음 |
| `bc_baseline.py` | (부록) 박종진 BC 받기 전 만든 임시 기준 BC(MLP, scikit-learn) · 판정기 `Judge` | `results/bc_eval.md` 등 |

## 실행 (노트북 · CPU 2개 기준 시간)

```bash
pip install mujoco==3.15.0 numpy scipy h5py matplotlib scikit-learn
git clone https://github.com/PJJ0337/-1-.git ../PJJ0337 && git -C ../PJJ0337 checkout 552614e   # 저장소 옆, 아니면 set PJJ=<경로>
python sim/w09/randomize.py oat 40            # ~20 분 (1,320 에피소드)
python sim/w09/randomize.py oat2 40           # ~19 분
python sim/w09/randomize.py joint 100         # ~7 분 (400)
python sim/w09/randomize.py curve table_dz,mu_pad_obj,obj_bias 40   # ~16 분 (880)
python sim/w09/randomize_report.py            # 표 · 그림
python sim/w09/w09_clearance.py               # ~10 s
git -C ../PJJ0337 checkout 99d60da         # 박종진 BC 포함 커밋 (randomize 결과는 552614e 와 시나리오 같음)
python sim/w09/bc_eval_pjj.py 50             # ~13 분 (700 에피소드)
python sim/w09/bc_eval_pjj.py 25 joint_dart_c25 oat   # ~12 분 (650)
python sim/w09/bc_eval_pjj_report.py
python sim/w09/bc_baseline.py collect 100     # (부록) ~3 분 → results/bc_data.npz (15 MB, 커밋 안 함)
python sim/w09/bc_baseline.py train           # ~20 s
python sim/w09/bc_baseline.py eval 20 table_dz,mu_pad_obj,obj_bias   # ~8 분
```

계산 환경: 클라우드 Linux, Python 3.13, mujoco 3.15.0, scikit-learn, CPU 2개(Xeon 2.1 GHz). 전문가 에피소드 ≈ 2.5 s/개. 공칭 결과가 박종진 수집과 같음(seed 1000 t_total 28.64 s 일치, 100/100).

## 모델 적용 방식 (요약)

- 마찰: 손가락 패드 · 물체 geom 의 μ 를 **둘 다** 바꿈 — MuJoCo 는 두 geom 마찰 중 큰 값을 씀(그래서 지금 테이블 0.8 은 실제로 쓰이지 않고 물체 1.0 이 적용됨)
- 팔 링크 질량 ×s: `body_mass` · `body_inertia` ×s, `body_gravcomp` = 1/s → 중력보상은 공칭 질량만큼만(모델 오차 재현)
- 테이블 높이: 테이블 · 물체는 참값 높이, 전문가 하강 높이 `E.GRASP_Z` 는 공칭 0.70 m 기준 그대로
- 측정 편향: 전문가가 보는 물체 위치에만 고정 편향(방향 무작위), 성공 판정은 참값
- 지연: 시뮬 스텝 단위 명령 버퍼(20 ms = 40 스텝)
- 질량 · armature 변경 뒤 `mj_setConst` 로 상수 재계산
- 박종진 `rand_dyn.apply` 대체: `randomize.apply(m, seed, f=1.0)` (모델 값만, 같은 seed = 같은 표본)
