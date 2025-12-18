'use client'

import React, { useState, useRef, useEffect } from 'react'
import { Send, Plus, Trash2, Loader } from 'lucide-react'
import { useChatStore } from '@/store/chatStore'
import { chatAPI } from '@/lib/api'
import MessageList from './MessageList'
import SelectionPanel from './SelectionPanel'
import IndexPanel from './IndexPanel'

export default function ChatBox() {
  const [input, setInput] = useState('')
  const [showSelection, setShowSelection] = useState(false)
  const [showIndex, setShowIndex] = useState(false)
  const [apiError, setApiError] = useState<string | null>(null)
  const messagesEndRef = useRef<HTMLDivElement>(null)

  const {
    messages,
    isLoading,
    sessionId,
    selectedText,
    addMessage,
    clearMessages,
    setLoading,
    addAssistantMessage,
  } = useChatStore()

  const scrollToBottom = () => {
    messagesEndRef.current?.scrollIntoView({ behavior: 'smooth' })
  }

  useEffect(() => {
    scrollToBottom()
  }, [messages])

  const handleSendMessage = async (e: React.FormEvent) => {
    e.preventDefault()
    if (!input.trim() || isLoading) return

    setApiError(null)
    const userMessage = input.trim()
    setInput('')

    // Add user message
    addMessage({
      id: `msg-${Date.now()}`,
      type: 'user',
      content: userMessage,
      timestamp: new Date(),
    })

    setLoading(true)
    try {
      const response = await chatAPI.chat({
        query: userMessage,
        selected_text: selectedText || undefined,
        session_id: sessionId,
      })

      addAssistantMessage(response)
    } catch (error: any) {
      const errorMsg = error.response?.data?.detail || error.message || 'Failed to get response'
      setApiError(errorMsg)
      addMessage({
        id: `msg-${Date.now()}`,
        type: 'assistant',
        content: `Error: ${errorMsg}`,
        timestamp: new Date(),
      })
    } finally {
      setLoading(false)
    }
  }

  return (
    <div className="flex flex-col h-screen bg-gradient-to-br from-slate-900 via-slate-800 to-slate-900">
      {/* Header */}
      <div className="bg-gradient-to-r from-blue-600 to-purple-600 text-white p-4 shadow-lg">
        <div className="max-w-6xl mx-auto flex justify-between items-center">
          <h1 className="text-2xl font-bold">📚 AI Book RAG Chatbot</h1>
          <div className="flex gap-2">
            <button
              onClick={() => setShowIndex(!showIndex)}
              className="px-4 py-2 bg-white/20 hover:bg-white/30 rounded-lg transition-colors flex items-center gap-2"
            >
              <Plus size={18} /> Index Book
            </button>
            <button
              onClick={() => clearMessages()}
              className="px-4 py-2 bg-red-500/20 hover:bg-red-500/30 rounded-lg transition-colors flex items-center gap-2"
            >
              <Trash2 size={18} /> Clear
            </button>
          </div>
        </div>
      </div>

      {/* Index Panel */}
      {showIndex && <IndexPanel onClose={() => setShowIndex(false)} />}

      {/* Main Content */}
      <div className="flex-1 overflow-hidden flex">
        {/* Messages Area */}
        <div className="flex-1 flex flex-col">
          {/* Messages */}
          <div className="flex-1 overflow-y-auto p-4 space-y-4">
            {messages.length === 0 ? (
              <div className="h-full flex items-center justify-center text-center">
                <div className="text-slate-400">
                  <div className="text-4xl mb-4">💬</div>
                  <p className="text-lg font-semibold">No messages yet</p>
                  <p className="text-sm mt-2">Start by asking a question about the book!</p>
                </div>
              </div>
            ) : (
              <>
                <MessageList messages={messages} />
                <div ref={messagesEndRef} />
              </>
            )}
          </div>

          {/* Error Message */}
          {apiError && (
            <div className="bg-red-500/20 border border-red-500 text-red-100 p-3 m-4 rounded-lg">
              {apiError}
            </div>
          )}

          {/* Input Area */}
          <div className="border-t border-slate-700 bg-slate-800 p-4">
            <form onSubmit={handleSendMessage} className="max-w-4xl mx-auto">
              <div className="flex gap-2">
                <div className="flex-1 relative">
                  <input
                    type="text"
                    value={input}
                    onChange={(e) => setInput(e.target.value)}
                    placeholder={
                      selectedText
                        ? `Analyzing selected text... (${selectedText.substring(0, 30)}...)`
                        : 'Ask a question about the book...'
                    }
                    disabled={isLoading}
                    className="w-full px-4 py-3 rounded-lg bg-slate-700 text-white placeholder-slate-400 border border-slate-600 focus:border-blue-500 focus:outline-none disabled:opacity-50"
                  />
                </div>
                <button
                  type="submit"
                  disabled={isLoading || !input.trim()}
                  className="px-6 py-3 bg-gradient-to-r from-blue-500 to-purple-500 text-white rounded-lg hover:from-blue-600 hover:to-purple-600 disabled:opacity-50 transition-all flex items-center gap-2 font-semibold"
                >
                  {isLoading ? <Loader size={20} className="animate-spin" /> : <Send size={20} />}
                  {isLoading ? 'Loading...' : 'Send'}
                </button>
              </div>
            </form>
          </div>
        </div>

        {/* Selection Panel */}
        {showSelection && <SelectionPanel onClose={() => setShowSelection(false)} />}
      </div>
    </div>
  )
}
