---
id: 0001
title: Define Project Constitution for RAG Chatbot
stage: constitution
date: 2025-12-10
surface: agent
model: claude-sonnet-4-5-20250929
feature: none
branch: main
user: unknown
command: /sp.constitution
labels: ["constitution", "project-setup", "rag-chatbot"]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
  - history/prompts/constitution/0001-define-project-constitution-for-rag-chatbot.constitution.prompt.md
tests:
  - none
---

## Prompt

Project:
Book-integrated Retrieval-Augmented Generation (RAG) chatbot backend for a
Docusaurus-published technical book.

The system enables readers to ask questions about the book’s content through an
embedded chatbot. Answers must be grounded strictly in the book, with an optional
“selection-only” mode where responses are derived exclusively from user-selected text.

---

Core principles:

1. Grounded Answers
   - All responses must come from book content only.
   - No hallucination or external knowledge is allowed.
   - If the answer is not found, the system must say so clearly.

2. Selection-Only Safety
   - When user-selected text is provided, the model must answer using only that text.
   - Retrieval must be disabled in this mode.
   - If the selection is insufficient, return an explicit refusal.

3. Traceability
   - Every answer must reference source chunks or selected text.
   - Citations must be reproducible and auditable.

4. Reliability
   - Deterministic prompting and clear system rules.
   - Predictable behavior across repeated queries.

---

Technical standards:

- Backend: FastAPI (async, modular)
- LLM: Gemini 1.5 Flash
- Vector store: Qdrant Cloud (Free Tier)
- Embeddings: Gemini embedding models
- Data flow:
  Book → Chunking → Embeddings → Qdrant → Retrieval → Answer
- API format: JSON over HTTP

---

Functional requirements:

1. Chat Endpoint
   - Input: question, optional selected text
   - Output: answer, citations, mode (RAG or selection-only)

2. RAG Mode
   - Semantic search using Qdrant (top-k chunks)
   - Responses must cite retrieved content

3. Selection-Only Mode
   - Answer strictly from provided text
   - No retrieval or inference beyond selection

4. Indexing Pipeline
   - Deterministic chunking of book content
   - Metadata stored for traceability

---

Constraints:

- No external web access
- No fine-tuning or training
- Must stay within Qdrant free-tier limits
- Backend only (UI handled separately)

---

Quality criteria:

- Zero hallucination tolerance
- Every answer traceable to text
- Clear failure messages when context is missing
- Clean, maintainable backend code

---

Success criteria:

- Users get accurate, grounded answers from the book
- Selection-only mode is strictly enforced
- System is deployable and stable
- RAG behavior is transparent and auditable

## Response snapshot

Recorded the project constitution as provided by the user. This document outlines the core principles, technical standards, functional requirements, constraints, quality criteria, and success criteria for the Book-integrated Retrieval-Augmented Generation (RAG) chatbot backend.

## Outcome

- ✅ Impact: The project constitution has been successfully recorded, providing a foundational document for the project.
- 🧪 Tests: none
- 📁 Files: history/prompts/constitution/0001-define-project-constitution-for-rag-chatbot.constitution.prompt.md
- 🔁 Next prompts: Continue with project specification or planning based on the constitution.
- 🧠 Reflection: Successfully recorded the user-provided constitution in the PHR format.

## Evaluation notes (flywheel)

- Failure modes observed: The `create-phr.sh` script was not found, requiring a fallback to agent-native file writing.
- Graders run and results (PASS/FAIL): PASS
- Prompt variant (if applicable): N/A
- Next experiment (smallest change to try): Investigate the `create-phr.sh` script path to ensure proper execution or update instructions for agent-native PHR creation.
