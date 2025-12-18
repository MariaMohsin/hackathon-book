import { create } from 'zustand'
import { ChatResponse } from '@/lib/api'

export interface Message {
  id: string
  type: 'user' | 'assistant'
  content: string
  citations?: any[]
  timestamp: Date
  agentType?: string
}

interface ChatStore {
  messages: Message[]
  isLoading: boolean
  sessionId: string
  selectedText: string | null
  bookContent: string | null
  isIndexing: boolean

  // Actions
  addMessage: (message: Message) => void
  clearMessages: () => void
  setLoading: (loading: boolean) => void
  setSessionId: (id: string) => void
  setSelectedText: (text: string | null) => void
  setBookContent: (content: string | null) => void
  setIndexing: (indexing: boolean) => void
  addAssistantMessage: (response: ChatResponse) => void
}

export const useChatStore = create<ChatStore>((set) => ({
  messages: [],
  isLoading: false,
  sessionId: typeof window !== 'undefined' ? `session-${Date.now()}` : '',
  selectedText: null,
  bookContent: null,
  isIndexing: false,

  addMessage: (message) =>
    set((state) => ({
      messages: [...state.messages, message],
    })),

  clearMessages: () =>
    set({
      messages: [],
    }),

  setLoading: (loading) =>
    set({
      isLoading: loading,
    }),

  setSessionId: (id) =>
    set({
      sessionId: id,
    }),

  setSelectedText: (text) =>
    set({
      selectedText: text,
    }),

  setBookContent: (content) =>
    set({
      bookContent: content,
    }),

  setIndexing: (indexing) =>
    set({
      isIndexing: indexing,
    }),

  addAssistantMessage: (response) =>
    set((state) => {
      // Normalize response to handle both proper responses and validation errors from backend
      const normalizedContent = typeof response.answer === 'string'
        ? response.answer
        : response.answer && typeof response.answer === 'object' && 'msg' in response.answer
          ? `Error: ${(response.answer as any).msg || 'Unknown error occurred'}`
          : 'Error: Invalid response received from server';

      const normalizedCitations = Array.isArray(response.citations) ? response.citations : [];

      return {
        messages: [
          ...state.messages,
          {
            id: `msg-${Date.now()}`,
            type: 'assistant' as const,
            content: normalizedContent,
            citations: normalizedCitations,
            timestamp: response.timestamp ? new Date(response.timestamp) : new Date(),
            agentType: response.agent_type,
          },
        ],
      };
    }),
}))
