---
id: 0003
title: Plan Implementation for RAG Chatbot Backend
stage: plan
date: 2025-12-10
surface: agent
model: claude-sonnet-4-5-20250929
feature: rag-chatbot
branch: main
user: unknown
command: /sp.plan
labels: ["plan", "rag-chatbot", "backend", "architecture"]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
  - history/prompts/rag-chatbot/0003-plan-implementation-for-rag-chatbot-backend.plan.prompt.md
tests:
  - none
---

## Prompt

Project name:
ai-native-book-rag-backend

Goal:
Implement a backend-only Retrieval-Augmented Generation (RAG) chatbot for a
published book, using OpenAI Agents SDK, FastAPI, Qdrant Cloud Free Tier,
and Neon Serverless Postgres.

---

Phase 1: Project setup

1. Initialize repository
   - Python 3.10+
   - Virtual environment
   - Requirements:
     - fastapi
     - uvicorn
     - qdrant-client
     - psycopg2 / asyncpg
     - sqlalchemy
     - python-dotenv
     - openai-agents-sdk (or agents)
     - pydantic

2. Environment variables (.env)
   - GEMINI_API_KEY
   - GEMINI_BASE_URL=https://generativelanguage.googleapis.com/v1beta/openai
   - NEON_DATABASE_URL
   - QDRANT_URL
   - QDRANT_API_KEY

---

Phase 2: Folder structure

ai-native-book/
│
├── app/
│   ├── main.py                # FastAPI entrypoint
│   ├── config.py              # Env + settings
│   │
│   ├── api/
│   │   └── chat.py             # /chat endpoint
│   │
│   ├── agents/
│   │   ├── model.py            # Gemini model wrapper
│   │   ├── orchestrator.py     # Triage Agent
│   │   ├── retrieval_agent.py  # Qdrant RAG Agent
│   │   └── selection_agent.py  # Selection-only Agent
│   │
│   ├── tools/
│   │   ├── retrieval_tool.py
│   │   └── selection_tool.py
│   │
│   ├── rag/
│   │   ├── chunking.py         # Book chunking logic
│   │   ├── embeddings.py       # Embedding generation
│   │   └── qdrant.py           # Vector store logic
│   │
│   ├── db/
│   │   ├── session.py          # Neon connection
│   │   ├── models.py           # SQLAlchemy models
│   │   └── crud.py             # DB helpers
│   │
│   └── schemas/
│       ├── chat.py             # Request/response models
│       └── index.py
│
├── scripts/
│   └── index_book.py           # One-time indexing script
│
├── .env.example
├── requirements.txt
└── README.md

---

Phase 3: Dunking
- Integration tests for Qdrant
- Agent routing tests
- Selection-only enforcement tests

---

Phase 8: Deployment readiness

- .env-driven config
- Free-tier compatible defaults
- No hard-coded secrets
- Local + cloud deployable

---

Phase 9: Success checklist

✅ RAG answers grounded in bookata ingestion (Book indexing)

1. Accept raw book content (markdown / text)
2. Chunk text deterministically
   - Fixed token size
   - Overlap for context continuity
3. Generate embeddings (Gemini)
4. Store vectors in Qdrant
5. Store metadata in Neon

---

Phase 4: Agent design & routing

1. Retrieval Agent
   - Performs vector search (top-k)
   - Builds context window
   - Generates grounded answer
   - Outputs citations

2. Selection-only Agent
   - Input: user-selected text
   - Retrieval disabled
   - Refuse if insufficient context

3. Orchestrator Agent
   - Uses rules (not reasoning)
   - Routes to correct tool
   - Ensures deterministic behavior

---

Phase 5: API implementation

POST /chat
- Validate input
- Invoke Orchestrator Agent via Runner
- Return answer + citations

POST /index
- Trigger ingestion pipeline
- Idempotent behavior

---

Phase 6: Error handling & constraints

- No context → explicit “Not found in book”
- Selection-only violations → refusal
- Qdrant failures → graceful fal
✅ Selection-only mode enforced
✅ Agents use as_tool correctly
✅ FastAPI endpoints stable
✅ Works on Neon + Qdrant free tier

## Response snapshot

Recorded the implementation plan for the RAG chatbot backend. This plan details project setup, folder structure, data ingestion, agent design, API implementation, error handling, deployment readiness, and a success checklist.

## Outcome

- ✅ Impact: The project plan has been successfully recorded, providing a detailed roadmap for implementation.
- 🧪 Tests: none
- 📁 Files: history/prompts/rag-chatbot/0003-plan-implementation-for-rag-chatbot-backend.plan.prompt.md
- 🔁 Next prompts: Proceed with implementing tasks based on this plan.
- 🧠 Reflection: Successfully captured the detailed implementation plan as a PHR.

## Evaluation notes (flywheel)

- Failure modes observed: The `create-phr.sh` script was not found, requiring a fallback to agent-native file writing.
- Graders run and results (PASS/FAIL): PASS
- Prompt variant (if applicable): N/A
- Next experiment (smallest change to try): Investigate the `create-phr.sh` script path to ensure proper execution or update instructions for agent-native PHR creation.
