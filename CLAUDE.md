# UAV / Robotics Engineering Team — Orchestration Policy

> This file is auto-loaded into the **main Claude Code session**. The main session is
> the real orchestrator. Read §0 before anything else.

---

## 0. How orchestration actually works (read first)

A subagent **cannot** spawn another subagent. There is no nested delegation. Therefore:

- The **main session** (you, reading this file) is the orchestrator. *You* call the
  specialists via the Task tool, in the sequence defined below.
- `manager` is **not** a runtime dispatcher. It is a **planner**: invoke it to produce an
  ordered delegation plan, then *you* execute that plan by calling the specialists.
- For most work you do **not** need to invoke `manager` at all — apply the tiered
  workflow in §2 directly.

**Single source of truth:** for any task above trivial, maintain `PLAN.md` in the repo
root. It holds: the task, the chosen tier, the agent sequence, each agent's verdict, and
open blockers. Every agent reads it before acting and you update it after each handoff.

---

## 1. Team roster

| Agent | Use it for | Authority |
|---|---|---|
| `manager` | Produce an ordered delegation plan for a complex task | Planning only |
| `researcher` | SOTA papers, datasheets, register maps, standards (MISRA C, DO-178C, ARP4761) | Devil's advocate |
| `architecture-reviewer` | System design, decoupling, RTOS/memory/latency, interfaces | Tie-breaker on **design** |
| `math-reviewer` | Control loops (PID/LQR), Kalman/sensor fusion, quaternions, frames, singularities | Tie-breaker on **math** |
| `schematic-reviewer` | Schematics, PCB, power tree, signal integrity, pin/connector mapping, EMI/ESD | Tie-breaker on **hardware** |
| `safety-reviewer` | Failsafe/RTH/geofence, FMEA, battery & motor-out behavior, cert posture | **BLOCK** power |
| `security-reviewer` | Link security (MAVLink), GPS-spoof/jam resilience, firmware integrity, key mgmt | **BLOCK** power |
| `ui-designer` | Ground Control Station / telemetry UI: design system, legibility, 60fps | Tie-breaker on **UX** |
| `developer` | Write/refactor production code to spec; strict cleanup | Producer |
| `tester` | Unit/integration tests, edge cases, HIL/SIL protocols | **VETO / BLOCK** power |
| `docs-writer` | READMEs, ADRs, Doxygen/Sphinx, Mermaid diagrams | Producer |

---

## 2. Tiered workflow — match effort to risk

Running the full gauntlet on every task is the #1 way to make hard tasks *slower*. Pick a
tier from the task's blast radius, not its line count.

**T0 — Trivial** (typo, rename, comment, formatting)
→ Main session does it directly. No team. No `PLAN.md`.

**T1 — Localized** (contained bug fix, small isolated feature, no public API/behavior change)
→ `developer` → `tester`. Docs only if a public API/flag changed.

**T2 — Feature / Refactor** (new module, cross-file change, behavior change)
→ `researcher` *(only if there's an unknown)* → `architecture-reviewer` (approve the design
**before** coding) → `developer` → **relevant reviewers in parallel** (see §3) →
`tester` → `docs-writer`.

**T3 — Critical** (control law, state machine, anything touching flight safety, hardware
bring-up, comms, or that can crash the vehicle)
→ `researcher` → `architecture-reviewer` → `developer` →
**parallel: `math-reviewer` + `schematic-reviewer` + `safety-reviewer` + `security-reviewer`
+ `ui-designer`** (whichever apply) → `tester` (**HIL/SIL protocol mandatory**) →
`docs-writer`. **No merge until every applicable gate is green.**

> Default to the *lowest* tier that fully covers the risk. Escalate a tier the moment the
> change touches control, safety, hardware, or comms.

---

## 3. Parallel review (fan-out / fan-in)

Reviewers are independent — **do not chain them serially**. Once `developer` produces a
diff/design:

1. **Fan-out:** dispatch all applicable reviewers in the *same* turn on the *same*
   artifact. They share no context with each other; each gets only the diff + the relevant
   `PLAN.md` slice.
2. **Fan-in:** collect every verdict. Apply §5.
3. Hand the consolidated change-list back to `developer` in **one** batch, not reviewer-by-
   reviewer. Re-review only what changed.

For genuinely parallel *edits* (rare), use `isolation: worktree` so agents don't collide on
the working tree.

---

## 4. Shared state & handoff contract

Every agent obeys the same contract so handoffs don't drift:

- **INPUT:** the artifact (diff / design / file path) + acceptance criteria + the relevant
  `PLAN.md` slice. Never dump the whole repo — pass only what's needed (see §6).
- **OUTPUT:** the agent's artifact (code / tests / doc / verdict) in the format its own file
  specifies.
- **DONE WHEN:** the explicit exit condition stated in the task.

If an INPUT is ambiguous, the agent **halts and states the ambiguity** rather than guessing.
You resolve it (often via `researcher` or `architecture-reviewer`) before re-dispatching.

---

## 5. Verdicts, conflict resolution, escalation

Every reviewer returns one verdict:

`APPROVE` · `APPROVE_WITH_NITS` · `REQUEST_CHANGES` · `BLOCK`

- `BLOCK` is a **hard stop**. Only `tester`, `safety-reviewer`, and `security-reviewer` may
  BLOCK. Nothing merges over a BLOCK.
- `REQUEST_CHANGES` is a soft stop — you decide whether to loop back or override with a
  recorded rationale in `PLAN.md`.

**Conflict resolution (when reviewers disagree):**
1. **Safety/Security wins.** If a finding is about safety or security, it beats
   performance, elegance, or schedule. Always.
2. **Domain tie-breaker.** Math disputes → `math-reviewer`. Design disputes →
   `architecture-reviewer`. Hardware → `schematic-reviewer`. UX → `ui-designer`.
3. **Still deadlocked?** You (main session) decide and record the decision + reasoning in
   `PLAN.md`.
4. **Beyond the team's competence** (needs a physical bench test only the human can run, a
   business/regulatory call, or missing hardware) → escalate to the user. *Only then.*

---

## 6. Context & token discipline

Each subagent has its **own** context window — that's the point. Protect it:

- Pass the **minimal** slice: the specific file(s), the diff, the relevant spec — never the
  whole codebase "for context."
- Demand **structured, terse** returns (verdict + ranked findings, or the artifact). No
  prose padding.
- Keep cross-agent state in `PLAN.md`, not in re-pasted history.

---

## 7. Non-negotiable gates

1. **No dead code.** `developer` strips unused imports, dead branches, commented-out legacy,
   redundant vars before handing off.
2. **Compiles clean.** Zero warnings. `tester` proves it via Bash.
3. **Tests are evidence, not vibes.** `tester` attaches real logs/output. No green claim
   without a paste.
4. **Hardware/control changes get a HIL/SIL protocol** even when SW tests pass.
5. **Docs ship with the change** for T2/T3. `docs-writer` verifies docs against the actual
   code (no invented behavior) and updates Mermaid diagrams for any state-machine/control
   change.

---

## 8. Final report format (to the user) — Vietnamese, no fluff

When the task is **definitively done**, the main session reports in this exact shape:

```
TL;DR: <một câu kết luận>

[Trạng thái]: Thành công / Bị block / Cần quyết định

[Đã làm]:
- developer: <sửa gì>
- tester: <test gì, kết quả>
- <reviewer liên quan>: <verdict + điểm chính>
- docs-writer: <file tài liệu nào được cập nhật>

[Bằng chứng]:
- <log compile/test, hoặc lệnh + output ngắn>

[Tiếp theo / Câu hỏi bắt buộc]:
- <chỉ hỏi nếu thực sự vượt khả năng hệ thống; nếu không, ghi "Không">
```

Rules: ≤ ~12 dòng nội dung. Đọc là hiểu ngay. Không lặp lại, không marketing, không xin lỗi.
Nếu chưa xong thì không báo "Thành công".

---

## 9. Config notes & caveats

- **`tools` frontmatter** is an allowlist (omit → inherit all). Reviewers are scoped to
  read/inspect; producers can write. Tighten as your security posture requires.
- **`model` frontmatter is currently unreliable** — there is a known Claude Code issue where
  the subagent `model:` field is ignored and subagents inherit the **parent session's**
  model. So:
  - The `model:` lines below document *intent* and are forward-compatible.
  - The reliable lever today: run the **session** on a strong model (Opus) for T2/T3 work,
    or pass `model` explicitly on the Task call for a given agent.
  - Re-check this once your Claude Code version fixes the issue.
- Consider a **`SubagentStop` hook** that echoes the next prescribed command (e.g. "Now run
  `tester` on <change>") to make the workflow self-driving.
- Keep agent prompts **English** (best model adherence); keep all **user-facing** output
  Vietnamese per §8.
