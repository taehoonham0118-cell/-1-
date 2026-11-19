# 10주차 기여 로그 — 함태훈 (B1) · 작성 중

- 기간: 2026. 11. 23. (월) ~ 11. 26. (목) 마감 — 커리큘럼 10주차는 노션 확인 후 채움
- 이전 주 태그: `week09` = **ed160f2** (11/18)
- 박종진 week09 = **4d81c70** (11/18, 함태훈 ed160f2 반영: BC × 함태훈 랜덤화 재현 200/200 일치(함태훈 대조 확인), env spec v0.2 §6, expert.py 주석 16 mm, 가정값 표 v0.5 마찰 max)

## 1. 수행 기록

| # | 날짜 | 수행 내용 | 산출물 |
|---|---|---|---|
| 1 | 11/19 | **데이터 명세 v1.1** (박종진 요청, DART 재수집 전): 선택 필드 `action_label`(T, 8) f64 = 학습 라벨(전문가 명령), 선택 속성 `dart`(action_label 있으면 필수) · `randomization`(랜덤화 수집 시 p · source). 재생 기준은 `action` 그대로, spec 문자열 `b1-dataset-1.0` 유지. `check` 에 선택 필드 검사 · `train_label(f)` 추가 → 샘플 3개 통과, 박종진 `collect_dart.py`(99d60da) 실제 파일 1개 통과, 잘못된 예(f32 · 그리퍼 0.09 · dart 없음) 3건 검출 | [docs/dataset_spec.md](../docs/dataset_spec.md) 7절, `sim/w08/b1_dataset.py` |
| 2 | 11/19 | 박종진 재현 대조: 박종진 `eval_taehoon_repro.json`(mujoco 3.14 · torch) vs 함태훈 `bc_eval_pjj.json`(3.15 · numpy) — 4조건 × 50 seed 성공/실패 **200/200 일치** 확인 | 박종진 4d81c70 `policies/results/eval_taehoon.md` |
