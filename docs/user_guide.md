# RAG Chatbot for Technical Book - User Guide

## Overview
This RAG (Retrieval-Augmented Generation) chatbot allows you to interact with technical book content using natural language queries. The system retrieves relevant information from the book and generates grounded responses with citations.

## Getting Started

### Prerequisites
- Python 3.8+
- API keys for:
  - Cohere (for embeddings and text generation)
  - Qdrant Cloud (for vector storage)
  - Neon Postgres (for metadata storage)

### Setup
1. Clone the repository
2. Create a virtual environment: `python -m venv venv`
3. Activate it: `source venv/bin/activate` (Linux/Mac) or `venv\Scripts\activate` (Windows)
4. Install dependencies: `pip install -r requirements.txt`
5. Create a `.env` file with your API keys based on `.env.example`
6. Start the server: `uvicorn app.main:app --reload --port 8000`

## Usage

### Chat with the RAG Model
Send a `POST` request to `/api/chat` with your question:

```json
{
  "question": "What is the main concept in chapter 3?",
  "selected_text": null,
  "session_id": "session-123"
}
```

### Use Selection-Only Mode
If you want to ask about specific text, provide it in the `selected_text` field:

```json
{
  "question": "Explain this concept",
  "selected_text": "The concept of retrieval-augmented generation is...",
  "session_id": "session-123"
}
```

### Index Your Book Content
To add your book content to the RAG system, send a `POST` request to `/api/index`:

```json
{
  "book_content": "# Chapter 1\nThe main topic is...",
  "book_metadata": {
    "title": "My Technical Book",
    "author": "Author Name"
  }
}
```

## Features

### RAG Mode
- When `selected_text` is `null`, the system searches the indexed book content
- Returns answers grounded in the book with citations
- Shows source information for fact-checking

### Selection-Only Mode
- When `selected_text` contains text, the system only uses that text
- Provides focused answers to questions about specific passages
- Does not perform retrieval from the larger book content

### Citations
- All answers include source citations
- Citations contain excerpts, page references, and section information
- Sources are traceable and auditable

## Best Practices

### For Better Results
- Ask specific, clear questions
- Use the selection-only mode for detailed analysis of specific passages
- Provide context in your questions (e.g., "In chapter 3, what does the author mean by...")

### Performance Tips
- The system processes books in chunks to maintain context
- Very long questions might be trimmed for optimal performance
- Consider breaking complex questions into simpler components

## Troubleshooting

### Common Issues
- **"Answer not found in book"**: The requested information might not be in the indexed content
- **Rate limit exceeded**: You've exceeded the request limit per hour
- **API connection errors**: Check your API keys and network connection

### Error Messages
- The system provides clear error messages when something goes wrong
- Check the logs for more detailed information about errors

## API Limits
- Rate limiting: 50 requests per hour per IP
- If you need higher limits, contact your system administrator

## Security
- All API keys should be stored in environment variables
- The system does not store your questions permanently unless configured to do so
- All external API calls are secured with HTTPS