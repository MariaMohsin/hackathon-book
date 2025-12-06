---
id: 20251204-1
title: Update Book Project Constitution
stage: constitution
date: 2025-12-04
surface: agent
model: claude-sonnet-4-5-20250929
feature: hac-book
branch: master
user: TODO(USER)
command: /sp.constitution
labels: ["constitution", "project-setup"]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
  - ./.specify/memory/constitution.md
tests:

---

## Prompt

```text
Project: Write a technical book using Docusaurus + Claude Code + Spec-Kit Plus, deployed on GitHub Pages.

Core Principles

Accuracy (no hallucinations; verify with official docs)

Simple, clear writing (Grade 8–10)

Practical-first: every chapter includes commands, code, or examples

AI-native: show how Claude Code + Spec-Kit help at each step

Standards

Code must run exactly as written

Folder structures must match a real Docusaurus project

Use Spec-Kit workflow: constitution → specs → drafts

Cite only official docs (no academic style required)

Consistent formatting, clean structure, short paragraphs

Constraints

10–15 chapters + Preface

800–1500 words per chapter

Total length: 12k–20k words

Built in Docusaurus and deployed on GitHub Pages

All code blocks must specify language (bash, ts etc.)

Workflow Rules

Validate commands using Claude Code (CCR)

Use Spec-Kit for chapters and tasks

Test build locally before writing instructions

No unverified claims or made-up tools

Chapters must start with learning objectives

Success Criteria

Book builds without errors

GitHub Pages deploys successfully

All examples run on a fresh machine

Consistent tone, clean formatting, reproducible steps

Entire book follows this constitution + chapter specs

Anti-Goals

No long theory without examples

No untested code

No inconsistent folder paths

No wordy or academic writing
```

## Response snapshot

Updated project constitution with core principles, standards, constraints, workflow rules, success criteria, and anti-goals. Version incremented to 1.1.0. Sync Impact Report included.

## Outcome

- ✅ Impact: Constitution updated successfully, project guidelines established.
- 🧪 Tests: N/A
- 📁 Files: 1 file modified: ./.specify/memory/constitution.md
- 🔁 Next prompts: User to review and proceed with next steps.
- 🧠 Reflection: Successfully updated constitution according to user input and template. Handled script failure by using agent-native PHR creation. Noted manual follow-up for README.md and docs/quickstart.md.

## Evaluation notes (flywheel)

- Failure modes observed: create-phr.sh script not found, fell back to agent-native PHR creation.
- Graders run and results (PASS/FAIL): N/A
- Prompt variant (if applicable): N/A
- Next experiment: Ensure create-phr.sh script is available or improve agent's ability to locate and execute it.
