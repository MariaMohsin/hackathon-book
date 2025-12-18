# AI-Native Book RAG Backend

A backend-only Retrieval-Augmented Generation (RAG) chatbot for published books, built with:
- **LLM**: Gemini 2.0 Flash (via OpenAI SDK compatibility)
- **Vector Store**: Qdrant Cloud (Free Tier)
- **Database**: Neon Serverless Postgres
- **Framework**: FastAPI + Python

## Overview

This system allows you to:
1. **Index a book** into a vector database with embeddings
2. **Ask questions** about the book content
3. **Receive grounded answers** with citations from the book
4. **Process selected text** without performing retrieval

## Architecture

### Agents

1. **Retrieval Agent** - Full RAG with semantic search
   - Generates embedding for query
   - Searches Qdrant for top-k similar chunks
   - Builds context window from results
   - Generates grounded answer with citations

2. **Selection Agent** - Process user-provided text only
   - Analyzes only the text the user selects
   - No retrieval/searching
   - Refusal if query can't be answered from selection

3. **Orchestrator Agent** - Deterministic routing
   - Routes to Retrieval if no selected text
   - Routes to Selection if user provided text
   - Rule-based, not reasoning-based

### Data Flow

```
User Query
    ↓
Orchestrator (has selected_text?)
    ├─→ YES: Selection Agent → Answer (no retrieval)
    └─→ NO: Retrieval Agent
            ├→ Generate embedding
            ├→ Vector search (Qdrant)
            ├→ Build context
            └→ Generate answer with citations
    ↓
Response + Citations + Session Log
```

## Setup

### 1. Prerequisites

- Python 3.10+
- Virtual environment

### 2. Environment Variables

Copy `.env.example` to `.env` and fill in:

```bash
# Gemini API Configuration
GEMINI_API_KEY=your_gemini_api_key
GEMINI_BASE_URL=https://generativelanguage.googleapis.com/v1beta/openai

# Neon Serverless Postgres
NEON_DATABASE_URL=postgresql://user:password@host/dbname

# Qdrant Cloud Free Tier
QDRANT_URL=https://your-cluster.qdrant.io
QDRANT_API_KEY=your_qdrant_key

# Application
DEBUG=False
LOG_LEVEL=INFO
ENVIRONMENT=development
```

### 3. Install Dependencies

```bash
pip install -r requirements.txt
```

### 4. Create Database

```bash
python -c "from app.db.session import Base, engine; Base.metadata.create_all(bind=engine)"
```

## Usage

### Index a Book

```bash
python scripts/index_book.py path/to/book.md
```

Options:
- `--reset-qdrant`: Clear existing Qdrant collection
- `--reset-postgres`: Clear existing Postgres chunks

### Start Server

```bash
python -m uvicorn app.main:app --reload --host 0.0.0.0 --port 8000
```

Visit: http://localhost:8000/docs (Swagger UI)

### API Endpoints

#### 1. Chat Endpoint

**POST** `/api/chat`

Request:
```json
{
  "query": "What is ROS2?",
  "selected_text": null,
  "session_id": "session-123"
}
```

Response:
```json
{
  "answer": "ROS2 is the next generation of ROS...",
  "citations": [
    {
      "index": 1,
      "text": "ROS2 is...",
      "score": 0.92,
      "metadata": {"section": "Introduction"},
      "id": "chunk-123"
    }
  ],
  "agent_type": "retrieval",
  "routing_reason": "Query requires book knowledge",
  "query": "What is ROS2?",
  "session_id": "session-123",
  "timestamp": "2025-12-10T10:30:00"
}
```

#### 2. Index Endpoint

**POST** `/api/index`

Request:
```json
{
  "book_content": "# Chapter 1\n\nROS2 is...",
  "collection_reset": false
}
```

Response:
```json
{
  "success": true,
  "chunks_indexed": 42,
  "embeddings_created": 42,
  "collection_info": {
    "points_count": 42,
    "vectors_count": 42
  },
  "message": "Successfully indexed 42 chunks"
}
```

#### 3. Health Check

**GET** `/api/health`

Response:
```json
{
  "status": "healthy",
  "environment": "development"
}
```

## Project Structure

```
app/
├── main.py                 # FastAPI entry point
├── config.py               # Environment settings
│
├── api/
│   └── chat.py             # /chat and /index endpoints
│
├── agents/
│   ├── model.py            # Gemini wrapper
│   ├── orchestrator.py     # Routing logic
│   ├── retrieval_agent.py  # RAG agent
│   └── selection_agent.py  # Selection-only agent
│
├── tools/
│   ├── retrieval_tool.py   # Tool definitions
│   └── selection_tool.py   # Selection logic
│
├── rag/
│   ├── chunking.py         # Text chunking
│   ├── embeddings.py       # Gemini embeddings
│   └── qdrant.py           # Vector store client
│
├── db/
│   ├── session.py          # SQLAlchemy setup
│   ├── models.py           # ORM models
│   └── crud.py             # Database operations
│
└── schemas/
    ├── chat.py             # Request/response models
    └── index.py            # Index schemas

scripts/
└── index_book.py           # Book indexing script
```

## Features

✅ **Grounded Answers** - All responses cite sources from the book
✅ **Selection Mode** - Analyze user-provided text without retrieval
✅ **Free Tier Ready** - Uses Qdrant Cloud Free Tier + Neon Free Tier
✅ **Session Tracking** - Logs all conversations in Postgres
✅ **Deterministic Routing** - Rules-based agent selection
✅ **Token-Aware Chunking** - Fixed-size chunks with overlap
✅ **Citations** - Retrieval metadata and confidence scores
✅ **Error Handling** - Graceful fallbacks for missing context

## Configuration

### Chunking Settings

Edit `app/api/chat.py` `chunk_text()` call:
```python
chunks = chunk_text(
    text=request.book_content,
    chunk_size=1000,      # Tokens per chunk
    overlap=100,          # Token overlap
)
```

### Retrieval Settings

Edit `app/agents/retrieval_agent.py`:
```python
search_results = self.vector_store.search(
    query_vector=query_embedding,
    top_k=5,              # Number of results
    score_threshold=0.5,  # Minimum similarity
)
```

### Temperature Settings

- **Retrieval Agent**: `temperature=0.2` (deterministic, grounded)
- **Selection Agent**: `temperature=0.2` (deterministic, grounded)
- **Orchestrator**: `temperature=0.0` (rule-based routing)

## Testing

```bash
# Run tests (pytest)
pytest tests/

# With coverage
pytest --cov=app tests/
```

## Deployment

### Local Development
```bash
python -m uvicorn app.main:app --reload
```

### Production
```bash
gunicorn -w 4 -b 0.0.0.0:8000 app.main:app
```

### Docker
```dockerfile
FROM python:3.10
WORKDIR /app
COPY requirements.txt .
RUN pip install -r requirements.txt
COPY . .
CMD ["uvicorn", "app.main:app", "--host", "0.0.0.0"]
```

## Constraints & Design Decisions

1. **No Tool Calling for Routing** - Orchestrator uses deterministic rules, not LLM reasoning
2. **Low Temperature Responses** - All agents use `temperature=0.2` for consistency
3. **Explicit Refusals** - Selection agent refuses if query can't be answered from text
4. **No Context = No Hallucination** - Returns "Not found in book" instead of guessing
5. **Session Logging** - All conversations logged in Postgres for audit/analysis
6. **Token-Deterministic Chunking** - Reproducible chunks across runs

## Troubleshooting

### API Connection Issues
- Verify `GEMINI_API_KEY` is valid
- Check Qdrant URL and API key
- Ensure Neon database URL is correct

### Embedding Generation Fails
- Check Gemini API quota
- Verify network connectivity to Google AI Studio

### Qdrant Connection Fails
- Ensure Qdrant Cloud cluster is active
- Check API key permissions (read + write)

### No Results from Search
- Verify book has been indexed: `/api/index`
- Check Qdrant collection has points: check collection info
- Adjust `score_threshold` in retrieval agent

## Future Enhancements

- [ ] Multi-turn conversation context
- [ ] Streaming responses
- [ ] Fine-grained access control
- [ ] Analytics dashboard
- [ ] Alternative embedding models
- [ ] Prompt engineering templates
- [ ] A/B testing framework
- [ ] Advanced chunking strategies (semantic, graph-based)

## References

- [Gemini API Docs](https://ai.google.dev/)
- [Qdrant Docs](https://qdrant.tech/documentation/)
- [Neon Docs](https://neon.tech/docs/)
- [FastAPI](https://fastapi.tiangolo.com/)
- [SQLAlchemy](https://docs.sqlalchemy.org/)

---

Built with ❤️ for AI-native knowledge systems.
