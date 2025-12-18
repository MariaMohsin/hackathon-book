# RAG Chatbot Frontend

Modern Next.js + React + TypeScript frontend for the AI-native book RAG chatbot.

## Features

✅ **Modern Chat Interface** - Real-time messaging with markdown support
✅ **Book Indexing** - Upload and index books directly from UI
✅ **Selected Text Analysis** - Analyze user-selected text without retrieval
✅ **Citations** - View source citations with confidence scores
✅ **Dark Theme** - Beautiful dark gradient UI with Tailwind CSS
✅ **Type-Safe** - Full TypeScript support
✅ **State Management** - Zustand for global state
✅ **Responsive** - Works on desktop and mobile

## Tech Stack

- **Framework**: Next.js 14
- **UI**: React 18 + TypeScript
- **Styling**: Tailwind CSS
- **State**: Zustand
- **HTTP**: Axios
- **Icons**: Lucide React
- **Markdown**: React Markdown

## Setup

### 1. Install Dependencies

```bash
cd frontend
npm install
# or
yarn install
```

### 2. Environment Variables

```bash
# .env.local
NEXT_PUBLIC_API_URL=http://localhost:8000
```

### 3. Start Development Server

```bash
npm run dev
# or
yarn dev
```

Visit **http://localhost:3000**

## Project Structure

```
frontend/
├── src/
│   ├── app/
│   │   ├── layout.tsx          [Root layout]
│   │   ├── page.tsx            [Home page]
│   │   └── globals.css         [Global styles]
│   ├── components/
│   │   ├── ChatBox.tsx         [Main chat interface]
│   │   ├── MessageList.tsx     [Message display]
│   │   ├── SelectionPanel.tsx  [Text selection]
│   │   └── IndexPanel.tsx      [Book indexing]
│   ├── lib/
│   │   └── api.ts              [API client]
│   └── store/
│       └── chatStore.ts        [Zustand store]
├── public/                     [Static assets]
├── package.json
├── next.config.js
├── tsconfig.json
└── tailwind.config.ts
```

## How to Use

### 1. Index a Book

- Click **"Index Book"** button
- Upload a markdown/text file or paste content
- Click **"Index Book"** to process
- Wait for confirmation message

### 2. Ask Questions

- Type your question in the input box
- Click **"Send"** or press Enter
- The RAG agent will search the book and respond with citations

### 3. Analyze Selected Text

- Copy text from the book
- Click the text selection panel
- Paste the text
- Ask a question - it will analyze ONLY that text

## API Integration

The frontend connects to the backend API at `http://localhost:8000`:

### POST /api/chat
```typescript
{
  query: "Your question",
  selected_text?: "Optional selected text",
  session_id?: "Optional session ID"
}
```

### POST /api/index
```typescript
{
  book_content: "Book markdown/text",
  collection_reset?: false
}
```

## Building for Production

```bash
npm run build
npm start
```

## Environment Variables

| Variable | Description | Default |
|----------|-------------|---------|
| `NEXT_PUBLIC_API_URL` | Backend API base URL | `http://localhost:8000` |

## Troubleshooting

### Connection Refused
- Ensure backend is running on `http://localhost:8000`
- Check `.env.local` API_URL

### CORS Errors
- Backend has CORS enabled for all origins
- Check browser console for details

### Styling Issues
- Run `npm install` to ensure Tailwind is properly installed
- Restart dev server

## Development

### Run Tests
```bash
npm test
```

### Build for Production
```bash
npm run build
npm start
```

### Format Code
```bash
npx prettier --write .
```

## Features Explained

### Chat Box
- Main interface for sending queries
- Real-time message display
- Loading indicators
- Error handling

### Message List
- User messages (blue, right-aligned)
- Assistant messages (gray, left-aligned)
- Markdown rendering
- Citation display with confidence scores

### Selection Panel
- Paste or type text to select
- Shows selected text in messages
- Clear selection button
- Tooltip explaining usage

### Index Panel
- File upload with drag-drop
- Paste option for content
- Real-time indexing feedback
- Success confirmation

## Architecture

```
Frontend (Next.js)
    ↓
API Client (Axios)
    ↓
Backend API (FastAPI)
    ↓
RAG Pipeline
├─ Orchestrator Agent
├─ Retrieval Agent
└─ Selection Agent
```

## Performance

- **Lazy Loading**: Components load on demand
- **Image Optimization**: Next.js optimized images
- **Code Splitting**: Automatic route code splitting
- **State Management**: Efficient Zustand store

## Browser Support

- Chrome/Edge (latest)
- Firefox (latest)
- Safari (latest)

## Future Enhancements

- [ ] Multi-file support
- [ ] Conversation history export
- [ ] Dark/Light theme toggle
- [ ] Voice input/output
- [ ] Streaming responses
- [ ] Advanced analytics
- [ ] User authentication

## License

MIT

---

Built with ❤️ for AI-native knowledge systems.
