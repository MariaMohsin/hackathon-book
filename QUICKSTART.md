# Quick Start Guide

## 5-Minute Setup

### 1. Clone and Enter Project
```bash
cd ai-native-book
```

### 2. Create Virtual Environment
```bash
python -m venv venv
source venv/bin/activate  # On Windows: venv\Scripts\activate
```

### 3. Set Up Environment
```bash
cp .env.example .env
# Edit .env with your API keys:
# - GEMINI_API_KEY
# - QDRANT_URL + QDRANT_API_KEY
# - NEON_DATABASE_URL
```

### 4. Install Dependencies
```bash
pip install -r requirements.txt
```

### 5. Initialize Database
```bash
python -c "from app.db.session import Base, engine; Base.metadata.create_all(bind=engine)"
```

### 6. Start Server
```bash
python -m uvicorn app.main:app --reload
```

Visit: http://localhost:8000/docs

## Example Workflow

### Step 1: Index Your Book

Create a file `sample_book.md`:
```markdown
# Introduction to ROS2

## What is ROS2?

ROS2 is the next generation of ROS (Robot Operating System). It provides a flexible framework for writing robotic software.

## Key Features

- Distributed architecture
- Real-time capabilities
- Security built-in
```

Index it:
```bash
python scripts/index_book.py sample_book.md --reset-qdrant
```

### Step 2: Query the Book

Using curl:
```bash
curl -X POST "http://localhost:8000/api/chat" \
  -H "Content-Type: application/json" \
  -d '{
    "query": "What is ROS2?",
    "session_id": "session-1"
  }'
```

### Step 3: Process Selected Text

```bash
curl -X POST "http://localhost:8000/api/chat" \
  -H "Content-Type: application/json" \
  -d '{
    "query": "Explain the key features",
    "selected_text": "ROS2 is the next generation of ROS...",
    "session_id": "session-1"
  }'
```

## API Testing with Python

```python
import httpx

client = httpx.Client(base_url="http://localhost:8000")

# Chat endpoint
response = client.post("/api/chat", json={
    "query": "What is ROS2?",
    "session_id": "session-1"
})

print(response.json())
```

## Troubleshooting

### Port Already in Use
```bash
# Change port
python -m uvicorn app.main:app --port 8001
```

### Import Errors
```bash
# Reinstall in development mode
pip install -e .
```

### Database Errors
```bash
# Reset database
python -c "from app.db.session import Base, engine; Base.metadata.drop_all(bind=engine); Base.metadata.create_all(bind=engine)"
```

## Next Steps

1. **Add Your Book**: Replace sample book with your actual book content
2. **Tune Chunking**: Adjust `chunk_size` and `overlap` in `app/api/chat.py`
3. **Monitor Metrics**: Check latency and citation scores
4. **Deploy**: Follow deployment guide in README.md
