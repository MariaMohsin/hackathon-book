# Scripts for running the RAG Chatbot

## Running the Backend Server

To start the backend API server:

```bash
cd C:\Users\HP\Desktop\ai-native-book
python -m uvicorn app.main:app --host 0.0.0.0 --port 8000
```

## Running the Frontend Server

To start the frontend (in a separate terminal):

```bash
cd C:\Users\HP\Desktop\ai-native-book\frontend
npm install
npm run dev
```

Then visit http://localhost:3000 to access the chatbot interface.

## Using the RAG Chatbot

1. **Index a book**: Click the "Index Book" button and paste your book content or upload a text file.
2. **Ask questions**: Type questions about the book in the input field and press Enter/Send.
3. **Selection mode**: Select text and click the selection button to ask questions about specific text only.
4. **View citations**: Responses will include citations from the source book.

## Environment Configuration

The frontend automatically connects to your backend at http://localhost:8000 as specified in the NEXT_PUBLIC_API_URL environment variable.