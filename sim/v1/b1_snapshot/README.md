# B1 v1 공용 모델 스냅샷 (재현용 사본)

- 출처: 박종진 저장소 https://github.com/PJJ0337/-1- 커밋 `652b89e` (2026-10-19, 함태훈 링크 관성 입력 · 근거본 actuator_params · PDR 기준 52 N·m 반영. 이전 사본 347a0ca) — `b1/models/v1/{mjcf,meshes,actuator_params.json}`, `b1/models/assumptions_b1.json`
- 이 폴더는 함태훈 `sim/v1/*.py` 를 이 저장소만으로 다시 돌리기 위한 **읽기 전용 사본**이다. 원본은 박종진 `build_v1.py` 가 만든다 (URDF 가 원본, MJCF 는 생성물 — 손으로 고치지 않음)
- 최신 원본으로 돌리려면: `set B1_V1=<박종진 저장소>\b1\models\v1` (Windows) / `export B1_V1=...` 후 스크립트 실행
- STEP · BOM 원본 없음 (STL 메시 · MJCF 만)
