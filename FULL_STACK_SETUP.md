# Full Stack RAG Chatbot - Setup & Deployment Guide

Complete guide for running the AI-native book RAG chatbot with frontend and backend.

## 📋 Architecture

```
┌─────────────────────────────────────────────────────────┐
│                   Frontend (Next.js)                    │
│                   :3000                                 │
│              (React + TypeScript)                       │
└────────────────────┬────────────────────────────────────┘
                     │
                     │ HTTP (Axios)
                     ↓
┌─────────────────────────────────────────────────────────┐
│                   Backend (FastAPI)                     │
│                   :8000                                 │
│          (Python + Agent-based RAG)                     │
├─────────────────────────────────────────────────────────┤
│                                                         │
│  ┌─────────────┐  ┌──────────────┐  ┌────────────────┐ │
│  │  Gemini LLM │  │ Qdrant       │  │ Neon Postgres  │ │
│  │ (Cloud API) │  │ (Free Tier)  │  │ (Free Tier)    │ │
│  └─────────────┘  └──────────────┘  └────────────────┘ │
│                                                         │
└─────────────────────────────────────────────────────────┘
```

## 🚀 Quick Start (5 Minutes)

### Step 1: Start Backend (Terminal 1)

```bash
cd ai-native-book
source venv/bin/activate  # On Windows: venv\Scripts\activate

python -m uvicorn app.main:app --reload --host 127.0.0.1 --port 8000
```

Expected output:
```
INFO:     Uvicorn running on http://127.0.0.1:8000
```

### Step 2: Start Frontend (Terminal 2)

```bash
cd ai-native-book/frontend
npm install
npm run dev
```

Expected output:
```
▲ Next.js 14.0.0
- Local:        http://localhost:3000
```

### Step 3: Open Browser

Visit **http://localhost:3000** 🎉

## 📦 Full Setup Instructions

### Prerequisites

- **Python**: 3.10+
- **Node.js**: 18+
- **API Keys**: 
  - Gemini API key (Google AI Studio)
  - Qdrant API key (qdrant.io cloud)
  - Neon database URL (neon.tech)

### Backend Setup

#### 1. Create Virtual Environment

```bash
cd ai-native-book
python -m venv venv
source venv/bin/activate  # Windows: venv\Scripts\activate
```

#### 2. Install Python Dependencies

```bash
pip install -r requirements.txt
```

#### 3. Configure Environment

```bash
cp .env.example .env
# Edit .env with your API keys
```

Required variables:
```env
GEMINI_API_KEY=your_key_here
NEON_DATABASE_URL=postgresql://...
QDRANT_URL=https://...
QDRANT_API_KEY=your_key_here
DEBUG=False
ENVIRONMENT=production
```

#### 4. Initialize Database

```bash
python -c "from app.db.session import Base, engine; Base.metadata.create_all(bind=engine)"
```

#### 5. Start Backend Server

```bash
python -m uvicorn app.main:app --host 127.0.0.1 --port 8000
```

Test health: `curl http://localhost:8000/api/health`

### Frontend Setup

#### 1. Install Dependencies

```bash
cd frontend
npm install
```

#### 2. Configure Environment

```bash
# .env.local
NEXT_PUBLIC_API_URL=http://localhost:8000
```

#### 3. Start Development Server

```bash
npm run dev
```

#### 4. Open Application

Visit **http://localhost:3000** in your browser

## 🎯 Usage Guide

### Scenario 1: Index a Book

1. Click **"Index Book"** button (top-right)
2. Paste book content (markdown or plain text)
3. Click **"Index Book"**
4. Wait for confirmation

### Scenario 2: Ask Questions

1. Type: "What is ROS2?"
2. Click **"Send"**
3. View answer with citations

### Scenario 3: Analyze Selected Text

1. Click **"Index Book"** > paste a specific paragraph
2. In chat, type a question
3. System analyzes ONLY that text (selection mode)

## 🏗️ Project Structure

```
ai-native-book/
├── app/                          [Backend - Python/FastAPI]
│   ├── main.py
│   ├── config.py
│   ├── agents/
│   ├── api/
│   ├── rag/
│   ├── db/
│   └── schemas/
│
├── frontend/                     [Frontend - Next.js/React]
│   ├── src/
│   │   ├── app/
│   │   ├── components/
│   │   ├── lib/
│   │   └── store/
│   ├── package.json
│   └── README.md
│
├── scripts/
│   └── index_book.py
│
├── requirements.txt
├── README.md
└── .env.example
```

## 🔧 Configuration

### Backend Configuration (app/config.py)

```python
# Temperature for responses (lower = deterministic)
temperature = 0.2

# Vector search settings
top_k = 5
score_threshold = 0.5

# Chunking settings
chunk_size = 1000
overlap = 100
```

### Frontend Configuration (.env.local)

```
# API endpoint (production: use public domain)
NEXT_PUBLIC_API_URL=http://localhost:8000
```

## 🧪 Testing

### Test Backend API

```bash
# Health check
curl http://localhost:8000/api/health

# Chat endpoint
curl -X POST http://localhost:8000/api/chat \
  -H "Content-Type: application/json" \
  -d '{"query": "What is this book about?", "session_id": "test-1"}'

# Index endpoint
curl -X POST http://localhost:8000/api/index \
  -H "Content-Type: application/json" \
  -d '{"book_content": "# Chapter 1\nContent here..."}'
```

### Test Frontend

1. Open **http://localhost:3000**
2. Try sending a message
3. Check browser console for errors

## 🚢 Production Deployment

### Backend Deployment (Heroku Example)

```bash
# 1. Create Procfile
echo "web: gunicorn -w 4 app.main:app" > Procfile

# 2. Deploy to Heroku
heroku login
heroku create my-rag-backend
git push heroku main

# 3. Set environment variables
heroku config:set GEMINI_API_KEY=xxx
heroku config:set NEON_DATABASE_URL=xxx
heroku config:set QDRANT_URL=xxx
heroku config:set QDRANT_API_KEY=xxx
```

### Frontend Deployment (Vercel)

```bash
# 1. Connect GitHub repo to Vercel
# 2. Set environment variables in Vercel dashboard
NEXT_PUBLIC_API_URL=https://my-rag-backend.herokuapp.com

# 3. Deploy
git push origin main  # Auto-deploys on Vercel
```

### Using Docker

Backend Dockerfile:
```dockerfile
FROM python:3.10-slim
WORKDIR /app
COPY requirements.txt .
RUN pip install -r requirements.txt
COPY app/ app/
CMD ["uvicorn", "app.main:app", "--host", "0.0.0.0", "--port", "8000"]
```

Frontend Dockerfile:
```dockerfile
FROM node:18-alpine
WORKDIR /app
COPY package*.json .
RUN npm install
COPY . .
RUN npm run build
CMD ["npm", "start"]
```

## 📊 Monitoring

### Backend Logs

```bash
# View live logs
tail -f /var/log/app.log

# Check API health
curl http://localhost:8000/api/health
```

### Frontend Logs

- **Browser Console**: `F12` or `Cmd+Option+I`
- **Network Tab**: Check API requests
- **Application Tab**: Check localStorage/state

## 🐛 Troubleshooting

### Backend Issues

| Issue | Solution |
|-------|----------|
| `ModuleNotFoundError` | Run `pip install -r requirements.txt` |
| `Connection refused` | Ensure backend is running on port 8000 |
| `API key invalid` | Check `.env` file - keys must be valid |
| `Database connection error` | Verify Neon database URL in `.env` |
| `Qdrant connection error` | Check Qdrant Cloud cluster is active |

### Frontend Issues

| Issue | Solution |
|-------|----------|
| `Module not found` | Run `npm install` in frontend directory |
| `API calls failing` | Check backend is running at correct URL |
| `CORS errors` | Backend CORS is enabled for all origins |
| `Port 3000 in use` | Run on different port: `npm run dev -- -p 3001` |

### Common Errors

**"Could not connect to backend"**
```bash
# Check backend is running
curl http://localhost:8000/api/health

# Check frontend can reach it
# Edit .env.local if using different URL
```

**"Embeddings not working"**
- Verify Gemini API key is correct
- Check API quota isn't exceeded
- Ensure book content is being uploaded

## 📚 API Documentation

### Backend Endpoints

#### GET /api/health
Health check endpoint
```bash
curl http://localhost:8000/api/health
```

#### POST /api/chat
Main chat interface
```bash
curl -X POST http://localhost:8000/api/chat \
  -d '{"query": "Hello", "session_id": "123"}'
```

#### POST /api/index
Index a book
```bash
curl -X POST http://localhost:8000/api/index \
  -d '{"book_content": "# Book..."}'
```

### Frontend Components

- `ChatBox.tsx` - Main interface
- `MessageList.tsx` - Display messages
- `IndexPanel.tsx` - Upload/index books
- `SelectionPanel.tsx` - Select text to analyze

## 📖 Learning Resources

- **FastAPI**: https://fastapi.tiangolo.com/
- **Next.js**: https://nextjs.org/
- **Tailwind CSS**: https://tailwindcss.com/
- **React**: https://react.dev/
- **Zustand**: https://github.com/pmndrs/zustand

## 🤝 Contributing

Feel free to submit issues and enhancement requests!

## 📄 License

MIT

---

**Happy Building! 🚀**

For questions or issues, refer to the individual README files:
- Backend: `/README.md`
- Frontend: `/frontend/README.md`
