<!--
Sync Impact Report:
Version change: 1.0.0 → 1.1.0
Modified principles:
- [PRINCIPLE_1_NAME] → Accuracy
- [PRINCIPLE_2_NAME] → Simple, clear writing
- [PRINCIPLE_3_NAME] → Practical-first
- [PRINCIPLE_4_NAME] → AI-native
- [PRINCIPLE_5_NAME] → Standards
- [PRINCIPLE_6_NAME] → Workflow Rules
Added sections: Constraints, Success Criteria, Anti-Goals
Removed sections: None
Templates requiring updates:
- .specify/templates/plan-template.md: ✅ updated
- .specify/templates/spec-template.md: ✅ updated
- .specify/templates/tasks-template.md: ✅ updated
- .specify/templates/commands/sp.constitution.md: ✅ updated
- README.md: ⚠ pending (manual follow-up needed to ensure consistency)
- docs/quickstart.md: ⚠ pending (manual follow-up needed to ensure consistency)
Follow-up TODOs: None
-->
# Write a technical book using Docusaurus + Claude Code + Spec-Kit Plus, deployed on GitHub Pages. Constitution

## Core Principles

### Accuracy
Every claim and piece of information MUST be verified with official documentation. No unverified claims or hallucinations are permitted.

### Simple, clear writing
All content MUST be written at a Grade 8–10 reading level. Complex technical concepts should be explained in an easy-to-understand manner.

### Practical-first
Every chapter MUST include commands, code, or examples that users can directly implement and run.

### AI-native
The book MUST demonstrate how Claude Code + Spec-Kit assist at each step of the technical writing and development process.

### Standards
Code must run exactly as written. Folder structures must match a real Docusaurus project. The Spec-Kit workflow (constitution → specs → drafts) MUST be used for chapters and tasks. Only official documentation should be cited. Consistent formatting, clean structure, and short paragraphs are required.

### Workflow Rules
All commands MUST be validated using Claude Code (CCR). Spec-Kit MUST be used for chapters and and tasks. The build MUST be tested locally before writing instructions. No unverified claims or made-up tools are allowed. Chapters MUST start with learning objectives.

## Constraints

The book MUST contain 10–15 chapters plus a Preface. Each chapter must be 800–1500 words, with a total length of 12k–20k words. The book MUST be built in Docusaurus and deployed to GitHub Pages. All code blocks must explicitly specify their language (e.g., bash, ts).

## Success Criteria

The book builds without errors. GitHub Pages deploys successfully. All examples run on a fresh machine. The book maintains a consistent tone, clean formatting, and reproducible steps. The entire book MUST follow this constitution and all chapter specifications.

## Anti-Goals

The book will NOT contain long theoretical explanations without practical examples. It will NOT include untested code. Inconsistent folder paths are NOT allowed. Wordy or academic writing styles are NOT permitted.

## Governance

This Constitution supersedes all other practices. Amendments require documentation, approval, and a migration plan. All PRs/reviews must verify compliance. Complexity must be justified. Use relevant guidance files for runtime development guidance.

**Version**: 1.1.0 | **Ratified**: 2025-12-04 | **Last Amended**: 2025-12-04