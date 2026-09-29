# 저장소 운영 규칙

평가표의 **④ 문서화 · 주차 보고(15점)** 채점 근거인 「개인 기여 로그 기록 · 주차 보고서 품질 · 공유 저장소 업로드(주차별 태그)」를 지키기 위한 규칙.

## 1. 평가표가 요구하는 것

| 척도 | 기준 (평가표 원문 요지) | 이 저장소에서 할 일 |
|---|---|---|
| 1 (3점) | 기여 로그·주차 보고 **미제출** | 절대 금지. 로그 없으면 자동 척도 1 |
| 3 (9점) | 기한 내 제출, 했는지는 확인 가능하나 **수치·파일 링크 미비** | – |
| 4 (12점) | 기한 내 제출, **수치·파일·저장소 태그가 연결되어 재현 가능** | 보고서에 명령어·수치·파일 경로·태그를 모두 적기 |
| 5 (15점) | **타인이 그대로 이어받아 작업 가능**, 절차서·BOM·ICD 갱신에 기여 | `notes/`에 절차서 수준으로 정리, 팀 문서 갱신 기여 기록 |

## 2. 매주 루틴 (주차 마감 전)

```bash
# 1) 이번 주 보고서 작성 (템플릿 복사)
cp docs/templates/weekly_report_template.md logs/week03.md

# 2) 스크린샷·결과 파일 정리
mkdir -p assets/screenshots/week03

# 3) 누적 로그(logs/contribution_log.md)에 한 줄 추가, README 주차 인덱스 갱신, 제출한 보고서 docx는 reports/ 에 사본

# 4) 커밋 & 푸시
git add .
git commit -m "week03: Cartpole 학습 실행 및 보상 곡선 기록"
git push

# 5) 주차 태그 (마감 시점에 1회)
git tag -a week03 -m "3주차 기여 로그"
git push origin week03
```

### git 없이 웹에서 할 때

1. 저장소 화면 → **Add file → Upload files** → 파일/폴더를 끌어다 놓기 → 커밋 메시지 입력 → **Commit changes**
   - 기존 파일 수정은 파일 열기 → 연필(✏️) 아이콘 → 수정 → Commit changes
2. 주차 태그: 오른쪽 **Releases → Create a new release** → *Choose a tag*에 `week03` 입력 → **Create new tag** → 제목 `3주차 기여 로그` → **Publish release**

## 3. 커밋 메시지

`weekNN: 무엇을 했는지 한 줄` 형식.

- `week02: Isaac Sim 5.1 + Isaac Lab 2.3.2 설치 및 create_empty 실행 확인`
- `week02: Cartpole zero/random agent 실행 로그 추가`
- `docs: 설치 오류 해결 과정 정리`

## 4. 태그 규칙

- 이름: `week01` ~ `week32` (두 자리 숫자)
- 주차 마감 시점의 커밋에 **annotated tag** (`git tag -a`)로 1개만.
- 게이트 주차(2·6·12·16·20·24·30·32주)는 가중치 2배이므로 특히 누락 금지.
- 태그를 잘못 찍었을 때:
  ```bash
  git tag -d week03 && git push origin :refs/tags/week03   # 삭제 후 다시 생성
  ```

## 5. 올리지 말 것 (`.gitignore`로 제외)

- Isaac Sim 설치 파일, conda 환경, 학습 체크포인트·로그(`*.pt`, `runs/`, `outputs/`, `logs/rsl_rl/` 등)
- 학습 결과는 파일 대신 **수치(보상, 성공률, 학습 시간)와 그래프 이미지**로 보고서에 기록
- 100MB 이상 파일 (GitHub 제한). 대용량 USD/메시가 필요하면 Git LFS 또는 링크로 대체.
