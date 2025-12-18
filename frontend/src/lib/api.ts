
import axios from 'axios'

const rawApiUrl = process.env.NEXT_PUBLIC_API_URL
let API_BASE_URL: string

if (!rawApiUrl) {
  API_BASE_URL = 'http://localhost:8000'
} else {
  try {
    const url = new URL(rawApiUrl)
    API_BASE_URL = url.href
  } catch {
    API_BASE_URL = 'http://localhost:8000'
  }
}

const apiClient = axios.create({
  baseURL: API_BASE_URL,
  headers: {
    'Content-Type': 'application/json',
  },
})

export interface ChatRequest {
  query: string
  selected_text?: string
  session_id?: string
}

export interface Citation {
  index: number
  text: string
  score?: number
  metadata?: Record<string, any>
  id?: string
}

export interface ChatResponse {
  answer: string
  citations: Citation[]
  agent_type: string
  routing_reason?: string
  query: string
  session_id?: string
  timestamp: string
}

export interface IndexRequest {
  book_content: string
  collection_reset?: boolean
}

export interface IndexResponse {
  success: boolean
  chunks_indexed: number
  embeddings_created: number
  collection_info?: Record<string, any>
  message: string
}

export const chatAPI = {
  async chat(request: ChatRequest): Promise<ChatResponse> {
    try {
      const response = await apiClient.post<ChatResponse>('/chat', request)
      const data = response.data

      // SAFELY normalize answer
      let normalizedAnswer = data.answer

      if (typeof data.answer !== 'string') {
        const maybeObj = data.answer as any
        normalizedAnswer =
          maybeObj?.msg ??
          maybeObj?.detail ??
          'Error: Invalid response from server'
      }

      return {
        ...data,
        answer: normalizedAnswer,
        citations: Array.isArray(data.citations) ? data.citations : [],
      }
    } catch (error) {
      if (axios.isAxiosError(error)) {
        const detail = (error.response?.data as any)?.detail

        return {
          answer: typeof detail === 'string'
            ? `Error: ${detail}`
            : 'Error: Request failed',
          citations: [],
          agent_type: 'error',
          routing_reason: 'error',
          query: request.query,
          session_id: request.session_id,
          timestamp: new Date().toISOString(),
        }
      }

      return {
        answer: 'Error: Failed to connect to server',
        citations: [],
        agent_type: 'error',
        routing_reason: 'error',
        query: request.query,
        session_id: request.session_id,
        timestamp: new Date().toISOString(),
      }
    }
  },

  async indexBook(request: IndexRequest): Promise<IndexResponse> {
    const response = await apiClient.post('/api/index', request)
    return response.data
  },

  async health() {
    const response = await apiClient.get('/api/health')
    return response.data
  },
}

export default apiClient
