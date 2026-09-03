# claude-template — 새 프로젝트 적용 방법

QUVI에서 다듬은 Claude Code 운영 세트의 범용 버전. Opus/Sonnet 어느 모델이 메인이어도 동일하게 동작한다.

## 구성

```
CLAUDE.md                        # 프로젝트 지침 템플릿 (<...> placeholder 채워서 사용)
.claude/
  settings.json                  # PreToolUse 훅 연결 (경로가 $CLAUDE_PROJECT_DIR 기준 — 수정 불필요)
  hooks/orchestration-gate.sh    # 게이트 래퍼
  hooks/orchestration-gate.py    # 메인 에이전트 직접 코드 수정을 턴당 3개 파일로 제한
  rules/orchestration.md         # 게이트 동작 설명
  agents/default-worker.md       # 구현 담당 (Sonnet)
  agents/deep-reasoner.md        # 병렬 심층 분석·교차 검증 담당 (Opus)
  agents/task-worker.md          # 잡무 담당 (Haiku)
```

## 적용 절차 (새 프로젝트 루트에서)

```bash
cp -r /home/ksj/바탕화면/claude-template/CLAUDE.md /home/ksj/바탕화면/claude-template/.claude <새 프로젝트 루트>/
chmod +x <새 프로젝트 루트>/.claude/hooks/orchestration-gate.sh
```

1. `CLAUDE.md`의 `<...>` placeholder를 전부 채운다. 채울 수 없는 섹션(SSoT 등)은 빈 채로 두지 말고 삭제 — 빈 규칙은 노이즈다.
2. `.gitignore`에 추가:
   ```
   CLAUDE.md
   .claude/
   ```
3. 검증 명령 섹션은 실제로 돌아가는 명령으로 채운다 (Claude가 수정 후 스스로 검증하는 근거).

## 작성 원칙 (CLAUDE.md를 고칠 때)

- 각 줄마다 "이게 없으면 Claude가 실수하는가?" — 아니면 삭제.
- 코드를 읽어서 알 수 있는 구조 설명은 쓰지 않는다. 코드로 추론 불가능한 것만(파일 간 결합, 알려진 함정, 검증 명령).
- 길어질수록 규칙이 노이즈에 묻혀 오히려 무시된다.
