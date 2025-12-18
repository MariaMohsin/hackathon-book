---
id: 0002
title: Define RAG Chatbot Feature Specification
stage: spec
date: 2025-12-10
surface: agent
model: claude-sonnet-4-5-20250929
feature: rag-chatbot
branch: main
user: unknown
command: /sp.specify
labels: ["spec", "rag-chatbot", "agent-design"]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
  - history/prompts/rag-chatbot/0002-define-rag-chatbot-feature-specification.spec.prompt.md
tests:
  - none
---

## Prompt

System goal:
Design a backend system that provides a Retrieval-Augmented Generation (RAG)
chatbot embedded inside a published technical book. The chatbot answers user
questions strictly from the book’s content and supports a “selection-only” mode.

The system must use OpenAI Agents SDK concepts (Agent, as_tool, Runner),
FastAPI, Gemini Flash models, Neon Postgres, and Qdrant Cloud.

---

Architecture overview:

1. API Layer (FastAPI)
   - Async REST API
   - JSON input/output
   - Stateless request handling

2. Agent Layer (OpenAI Agents SDK)
   - One Orchestrator Agent (Chat Agent)
   - One Retrieval Agent (Qdrant-based)
 Vector size: embedding model dependent
- Payload metadata:
  - chunk_id
  - chapter
  - section
  - text

---

Database storage (Neon Postgres):

Tables:
1. chunks_metadata
   - chunk_id (PK)
   - chapter
   - section
   - page_reference

2. chat_logs
   - id (PK)
   - question
   - mode
   - timestamp

---

Model configuration:

- LLM: Gemini Flash (OpenAI-compatible ChatCompletions)
- Embeddings  - One Selection-Only Agent (strict context mode)
   - Agents exposed via as_tool and routed by the Orchestrator

3. Data Layer
   - Qdrant Cloud: vector similarity search
   - Neon Postgres: metadata, chat logs, citations

---

Agents specification:

1. Orchestrator Agent
   - Role: Decide which mode to use (RAG or selection-only)
   - Inputs: user query, optional selected text
   - Tools:
     - retrieval_tool
     - selection_only_tool
   - Rules:
     - If selected text is provided → always use selection_only_tool
     - Otherwise → use retrieval_tool

2. Retrieval Agent (RAG)
   - Performs semantic search via Qdrant
   - Receives top-k chunks with metadata
   - Generates an answer strictly from retrieved content
   - Returns citations (chunk ids, section titles)

3. Selection-Only Agent
   - Receives only the user-selected text
   - Retrieval is disabled
   - Must refuse if answer cannot be derived from selection

---

API endpoints:

1. POST /chat
   Request:
     {
       "question": string,
     : Gemini embedding model
- No fine-tuning
- Deterministic temperature (low or zero)

---

Constraints:

- No external web access
- Answers must be grounded in provided context
- Qdrant free tier limits must be respected
- Backend only (no frontend/UI)

---

Error handling:

- If no relevant context found → return “Answer not found in book”
- If selection is insufficient → explicit refusal
- All failures must be graceful and explicit

---

Security considerations:

- API keys via environment variables
- No hard-coded secrets
- Read-only access for chat endpoint

---

Success criteria:

- Agent routing behaves deterministically
- Selection-only mode is strictly enforced
- Every answer includes citations
- System is deployable on free tiers

## Response snapshot

Recorded the feature specification for the RAG chatbot backend. This specification details the system's goal, architecture overview, agent specifications, API endpoints, database storage, model configuration, constraints, error handling, security considerations, and success criteria.

## Outcome

- ✅ Impact: The feature specification has been successfully recorded, providing a detailed plan for the RAG chatbot backend.
- 🧪 Tests: none
- 📁 Files: history/prompts/rag-chatbot/0002-define-rag-chatbot-feature-specification.spec.prompt.md
- 🔁 Next prompts: Proceed with planning the implementation based on this specification.
- 🧠 Reflection: Successfully captured the comprehensive feature specification as a PHR.

## Evaluation notes (flywheel)

- Failure modes observed: The `create-phr.sh` script was not found, requiring a fallback to agent-native file writing.
- Graders run and results (PASS/FAIL): PASS
- Prompt variant (if applicable): N/A
- Next experiment (smallest change to try): Ensure `create-phr.sh` script is available and executable, or refine agent-native PHR creation process to be more robust.
