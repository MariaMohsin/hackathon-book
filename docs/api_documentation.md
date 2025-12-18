# API Documentation

## Base URL
`http://localhost:8000/api` (or your deployed URL)

## Endpoints

### Chat Endpoint
`POST /api/chat`

#### Request
```json
{
  "question": "string",
  "selected_text": "string or null",
  "session_id": "string or null"
}
```

#### Response
```json
{
  "answer": "string",
  "mode": "rag | selection-only",
  "citations": [
    {
      "source_id": "string",
      "excerpt": "string",
      "page_reference": "number or null",
      "section": "string or null",
      "score": "number or null"
    }
  ],
  "session_id": "string",
  "timestamp": "ISO date string"
}
```

#### Example
```bash
curl -X POST "http://localhost:8000/api/chat" \
  -H "Content-Type: application/json" \
  -d '{
    "question": "What is the main concept discussed in chapter 3?",
    "selected_text": null,
    "session_id": "session-123"
  }'
```

### Index Endpoint
`POST /api/index`

#### Request
```json
{
  "book_content": "string",
  "book_metadata": {
    "title": "string",
    "author": "string",
    "section": "string",
    "page_reference": "number",
    "chapter": "string"
  }
}
```

#### Response
```json
{
  "success": "boolean",
  "chunks_indexed": "number",
  "message": "string"
}
```

#### Example
```bash
curl -X POST "http://localhost:8000/api/index" \
  -H "Content-Type: application/json" \
  -d '{
    "book_content": "# Chapter 1\nThe main topic is... # Chapter 2\n...",
    "book_metadata": {
      "title": "My Technical Book",
      "author": "Author Name"
    }
  }'
```

### Health Check Endpoints

#### Basic Health
`GET /api/health`

#### Extended Health
`GET /api/health/extended`

#### Readiness
`GET /api/health/readiness`

## Authentication
The API currently does not require authentication but implements rate limiting to prevent abuse.

## Rate Limits
- 50 requests per hour per IP address
- Requests exceeding the limit will receive a 429 status code