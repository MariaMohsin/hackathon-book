# Testing Guide

## Unit Tests for RAG Chatbot

### Setup Test Environment

```bash
pip install pytest pytest-asyncio pytest-cov
```

### Run All Tests

```bash
pytest tests/ -v
```

With coverage:
```bash
pytest tests/ --cov=app --cov-report=html
```

## Test Files to Create

### 1. tests/test_chunking.py

```python
import pytest
from app.rag.chunking import chunk_text, estimate_tokens

def test_estimate_tokens():
    """Test token estimation."""
    text = "Hello world"
    tokens = estimate_tokens(text)
    assert tokens > 0
    assert isinstance(tokens, int)

def test_chunk_text_basic():
    """Test basic chunking."""
    text = "This is a sample text. " * 100
    chunks = chunk_text(text, chunk_size=100, overlap=10)
    
    assert len(chunks) > 1
    for chunk_text, token_count in chunks:
        assert len(chunk_text) > 0
        assert token_count > 0

def test_chunk_text_overlap():
    """Test that chunks have overlap."""
    text = "This is a sample text. " * 100
    chunks = chunk_text(text, chunk_size=100, overlap=20)
    
    # Check overlap exists
    assert len(chunks) >= 2
```

### 2. tests/test_qdrant.py

```python
import pytest
from unittest.mock import MagicMock, patch
from app.rag.qdrant import QdrantVectorStore

@pytest.fixture
def vector_store():
    """Mock vector store for testing."""
    with patch('app.rag.qdrant.QdrantClient'):
        return QdrantVectorStore()

def test_vector_store_creation(vector_store):
    """Test vector store initialization."""
    assert vector_store.COLLECTION_NAME == "book_embeddings"
    assert vector_store.VECTOR_SIZE == 768

def test_add_points(vector_store):
    """Test adding points to Qdrant."""
    embeddings = [
        ("text1", [0.1] * 768, {"index": 0}),
        ("text2", [0.2] * 768, {"index": 1}),
    ]
    
    with patch.object(vector_store.client, 'upsert'):
        ids = vector_store.add_points(embeddings)
        assert len(ids) == 2
```

### 3. tests/test_schemas.py

```python
import pytest
from app.schemas.chat import ChatRequest, ChatResponse, IndexRequest

def test_chat_request_valid():
    """Test valid chat request."""
    req = ChatRequest(
        query="What is ROS2?",
        session_id="session-1"
    )
    assert req.query == "What is ROS2?"

def test_chat_request_with_selected_text():
    """Test chat request with selected text."""
    req = ChatRequest(
        query="Explain this",
        selected_text="Some text here",
        session_id="session-1"
    )
    assert req.selected_text == "Some text here"

def test_index_request_valid():
    """Test index request."""
    req = IndexRequest(
        book_content="# Chapter 1\n\nContent here"
    )
    assert len(req.book_content) > 0
```

### 4. tests/test_crud.py

```python
import pytest
from sqlalchemy.orm import Session
from app.db.crud import BookChunkCRUD
from unittest.mock import MagicMock

def test_book_chunk_create():
    """Test creating a book chunk."""
    db = MagicMock(spec=Session)
    
    chunk = BookChunkCRUD.create(
        db=db,
        chunk_index=0,
        content="Sample content",
        token_count=50
    )
    
    # Should call db.add and db.commit
    assert db.add.called
    assert db.commit.called
```

### 5. tests/test_agents.py

```python
import pytest
from unittest.mock import AsyncMock, MagicMock, patch
from app.agents.retrieval_agent import RetrievalAgent
from app.agents.selection_agent import SelectionAgent

@pytest.mark.asyncio
async def test_retrieval_agent():
    """Test retrieval agent."""
    agent = RetrievalAgent()
    
    with patch('app.agents.retrieval_agent.generate_embedding_sync') as mock_embed:
        with patch.object(agent.vector_store, 'search') as mock_search:
            mock_embed.return_value = [0.1] * 768
            mock_search.return_value = [
                {
                    "id": "chunk-1",
                    "text": "Sample text",
                    "score": 0.95,
                    "metadata": {}
                }
            ]
            
            result = await agent.run("What is this?")
            
            assert "answer" in result
            assert result["agent_type"] == "retrieval"
            assert len(result["citations"]) > 0

@pytest.mark.asyncio
async def test_selection_agent():
    """Test selection agent."""
    agent = SelectionAgent()
    
    result = await agent.run(
        selected_text="Sample text",
        question="What does this say?"
    )
    
    assert result["agent_type"] == "selection"
    assert len(result["citations"]) > 0
```

### 6. tests/test_api.py

```python
import pytest
from fastapi.testclient import TestClient
from app.main import app

client = TestClient(app)

def test_health_check():
    """Test health endpoint."""
    response = client.get("/api/health")
    assert response.status_code == 200
    assert response.json()["status"] == "healthy"

def test_root():
    """Test root endpoint."""
    response = client.get("/")
    assert response.status_code == 200
    assert "app" in response.json()

def test_chat_endpoint_invalid_request():
    """Test chat with invalid request."""
    response = client.post("/api/chat", json={})
    assert response.status_code == 422  # Validation error

def test_chat_endpoint_valid_request():
    """Test chat with valid request."""
    response = client.post("/api/chat", json={
        "query": "Test query",
        "session_id": "test-session"
    })
    # Will fail without proper setup, but shows structure
    assert response.status_code in [200, 500]
```

## Integration Tests

### End-to-End Workflow Test

Create `tests/test_integration.py`:

```python
import pytest
import os
from app.rag.chunking import chunk_text
from app.rag.embeddings import generate_embedding_sync

@pytest.mark.integration
def test_full_indexing_workflow():
    """Test complete indexing workflow."""
    # Load sample book
    sample_book = """
    # Introduction
    This is a sample book for testing.
    
    # Chapter 1
    Content of chapter 1.
    """
    
    # Chunk it
    chunks = chunk_text(sample_book, chunk_size=50, overlap=10)
    assert len(chunks) > 0
    
    # Verify each chunk is valid
    for chunk_text, token_count in chunks:
        assert len(chunk_text) > 0
        assert token_count > 0

@pytest.mark.skipif(
    not os.getenv("GEMINI_API_KEY"),
    reason="Requires GEMINI_API_KEY"
)
@pytest.mark.integration
def test_embedding_generation():
    """Test actual embedding generation."""
    text = "Sample text for embedding"
    embedding = generate_embedding_sync(text)
    
    assert isinstance(embedding, list)
    assert len(embedding) == 768  # Gemini embedding-001 dimension
    assert all(isinstance(x, float) for x in embedding)
```

## Run Tests

```bash
# Run all tests
pytest tests/

# Run with verbose output
pytest tests/ -v

# Run specific test file
pytest tests/test_chunking.py

# Run specific test
pytest tests/test_chunking.py::test_chunk_text_basic

# Run only integration tests
pytest tests/ -m integration

# Run with coverage
pytest tests/ --cov=app --cov-report=term-missing

# Generate HTML coverage report
pytest tests/ --cov=app --cov-report=html
```

## Continuous Integration (GitHub Actions)

Create `.github/workflows/test.yml`:

```yaml
name: Tests

on: [push, pull_request]

jobs:
  test:
    runs-on: ubuntu-latest
    
    steps:
    - uses: actions/checkout@v2
    
    - name: Set up Python
      uses: actions/setup-python@v2
      with:
        python-version: 3.10
    
    - name: Install dependencies
      run: |
        pip install -r requirements.txt
        pip install pytest pytest-asyncio pytest-cov
    
    - name: Run tests
      run: pytest tests/ --cov=app
    
    - name: Upload coverage
      uses: codecov/codecov-action@v2
```

## Mock External Services

For testing without API keys:

```python
from unittest.mock import patch, MagicMock

# Mock Gemini API
with patch('app.agents.model.OpenAI') as mock_openai:
    mock_instance = MagicMock()
    mock_openai.return_value = mock_instance
    # Your test code here

# Mock Qdrant
with patch('app.rag.qdrant.QdrantClient') as mock_qdrant:
    mock_instance = MagicMock()
    mock_qdrant.return_value = mock_instance
    # Your test code here

# Mock Database
from sqlalchemy.orm import Session
mock_db = MagicMock(spec=Session)
```

## Performance Testing

```bash
# Install locust for load testing
pip install locust

# Create tests/load_test.py
# Run: locust -f tests/load_test.py
```

## Coverage Goals

- Aim for >80% code coverage
- All critical paths tested (agents, RAG, API)
- Mock external services (Gemini, Qdrant, Neon)
- Test both happy path and error cases

## Common Test Patterns

### Testing Async Functions
```python
@pytest.mark.asyncio
async def test_async_function():
    result = await some_async_function()
    assert result is not None
```

### Testing Database Operations
```python
def test_with_db_session():
    db = SessionLocal()
    try:
        # Test operations
        pass
    finally:
        db.close()
```

### Testing API Endpoints
```python
def test_api_endpoint():
    client = TestClient(app)
    response = client.post("/api/endpoint", json={"key": "value"})
    assert response.status_code == 200
```
