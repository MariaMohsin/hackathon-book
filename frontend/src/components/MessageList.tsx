'use client'

import React from 'react'
import { Message } from '@/store/chatStore'
import Markdown from 'react-markdown'
import { ExternalLink, Quote } from 'lucide-react'

interface MessageListProps {
  messages: Message[]
}

export default function MessageList({ messages }: MessageListProps) {
  return (
    <div className="space-y-4">
      {messages.map((message) => (
        <div key={message.id} className={`flex ${message.type === 'user' ? 'justify-end' : 'justify-start'}`}>
          <div
            className={`max-w-2xl rounded-lg p-4 ${
              message.type === 'user'
                ? 'bg-gradient-to-r from-blue-500 to-blue-600 text-white rounded-br-none'
                : 'bg-slate-700 text-slate-100 rounded-bl-none border border-slate-600'
            }`}
          >
            {/* Message Content */}
            <div className="prose prose-invert max-w-none">
              <div className="text-sm leading-relaxed">
                {message.type === 'assistant' ? (
                  <Markdown className="prose-sm">
                    {typeof message.content === 'string'
                      ? message.content
                      : typeof message.content === 'object' && message.content !== null && 'msg' in message.content
                        ? `Error: ${(message.content as any).msg}`
                        : 'Error: Invalid response format'}
                  </Markdown>
                ) : (
                  <p>
                    {typeof message.content === 'string'
                      ? message.content
                      : 'Invalid message content'}
                  </p>
                )}
              </div>
            </div>

            {/* Agent Type Badge */}
            {message.agentType && (
              <div className="mt-2 inline-block">
                <span className="text-xs px-2 py-1 bg-white/20 rounded-full">
                  🤖 {message.agentType === 'retrieval' ? 'RAG Search' : 'Text Analysis'}
                </span>
              </div>
            )}

            {/* Citations */}
            {message.citations && message.citations.length > 0 && (
              <div className="mt-3 pt-3 border-t border-white/20 space-y-2">
                <p className="text-xs font-semibold opacity-75 flex items-center gap-1">
                  <Quote size={14} /> Citations
                </p>
                <div className="space-y-1">
                  {message.citations.slice(0, 3).map((citation, idx) => (
                    <div key={idx} className="text-xs opacity-75 p-2 bg-white/10 rounded">
                      <p className="truncate">{citation.text}</p>
                      {citation.score && (
                        <p className="text-xs opacity-60">Match: {(citation.score * 100).toFixed(0)}%</p>
                      )}
                    </div>
                  ))}
                </div>
                {message.citations.length > 3 && (
                  <p className="text-xs opacity-50">+{message.citations.length - 3} more citations</p>
                )}
              </div>
            )}

            {/* Timestamp */}
            <div className="text-xs opacity-60 mt-2">{message.timestamp.toLocaleTimeString()}</div>
          </div>
        </div>
      ))}
    </div>
  )
}
