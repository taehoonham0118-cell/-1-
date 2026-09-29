# 캡스톤디자인 — 함태훈 개인 기여 저장소

> 2026학년도 캡스톤디자인 · **홀로노믹 모바일 베이스 기반 전신 휴머노이드 로봇 개발**
> TEAM B / 서브팀 **B1 (모델링 · 시뮬레이션)** · 담당 대학원생 김이겸

| 항목 | 내용 |
|---|---|
| 이름 / 학번 | 함태훈 / 22212334 |
| 팀 / 서브팀 | TEAM B / B1 (모델링 · 시뮬레이션) |
| 주요 도구 | MuJoCo 3.14 · URDF/MJCF/USD · (PC 배정 후) Ubuntu 24.04 · CUDA 12 · Isaac Sim 5.1 · Isaac Lab 2.3.2 |
| 저장소 용도 | 주차별 **개인 기여 로그**, 주차 보고, 학습 정리, 시뮬레이션 코드·결과 보관 |

---

## 주차별 기록 (Weekly Index)

| 주차 | 게이트 | 핵심 목표 | 기여 로그 | 태그 |
|---|---|---|---|---|
| 1주 | – | Isaac Sim / Lab 개요·개념·데모 파악 | [week01](logs/week01.md) | – (저장소 개설 전, 노션으로 대체 인정) |
| 2주 | **SRR (×2)** | MuJoCo 설치 · G1 joint 표 · 2링크 팔 URDF/MJCF 비교 · 관성 실험 · 하체 실측표 | [week02](logs/week02.md) | `week02` |
| 3주 | – | USD 개념 · 2링크 팔 USD · URDF↔USD 대응표 · 파이프라인 그림 | | `week03` |

전체 누적표는 [`logs/contribution_log.md`](logs/contribution_log.md) 참고.

---

## 폴더 구조

```
.
├── README.md                  ← 지금 이 파일 (주차 인덱스)
├── logs/
│   ├── contribution_log.md    ← 누적 기여 로그 (매주 한 줄 추가)
│   ├── week01.md              ← 주차별 기여 로그 (주차별 1개)
│   ├── week02.md
│   └── week02_base_measurement.md  ← SRR 하체 실측표
├── reports/                   ← 제출한 주간활동보고서(docx) 사본
├── notes/                     ← 학습 정리
│   ├── 01_isaac_install.md    (Isaac Sim/Lab 설치, 노션 기준)
│   ├── 02_isaaclab_core_concepts.md
│   ├── 03_isaaclab_demos.md
│   └── 04_g1_arm_joints.md    (G1 어깨·팔꿈치 joint 표)
├── sim/
│   ├── urdf/                  ← URDF 모델 (two_link_arm.urdf)
│   ├── mjcf/                  ← MJCF 모델 (two_link_arm.xml)
│   ├── scripts/               ← 실험 스크립트·결과 (compare_urdf_mjcf.py, results.csv)
│   └── configs/               ← 환경/로봇 설정 파일
├── assets/screenshots/weekNN/ ← 실행 확인 스크린샷 (주차별 폴더)
└── docs/
    ├── GIT_RULES.md           ← 저장소 운영 규칙 (커밋·태그)
    └── templates/weekly_report_template.md
```

---

## 운영 규칙 요약 (상세: [docs/GIT_RULES.md](docs/GIT_RULES.md))

1. **매주 기여 로그 필수** — 로그가 없는 주차는 평가 항목 ④(문서화)가 **척도 1(3점)** 로 처리됨.
2. **주차별 태그** — 그 주 작업을 마감하면 `weekNN` 태그 (예: `week02`, `week10`).
3. **재현 가능하게** — 보고서에는 수치 · 파일 경로 · 실행 명령 · 태그를 함께 적는다 (척도 4 기준).
4. **스크린샷·로그는 `assets/screenshots/weekNN/`** 에, 파일명은 `weekNN_내용.jpg`(또는 .png).
