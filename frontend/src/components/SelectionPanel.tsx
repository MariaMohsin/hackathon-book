'use client'

import React, { useState } from 'react'
import { useChatStore } from '@/store/chatStore'
import { X } from 'lucide-react'

interface SelectionPanelProps {
  onClose: () => void
}

export default function SelectionPanel({ onClose }: SelectionPanelProps) {
  const { selectedText, setSelectedText } = useChatStore()

  return (
    <div className="w-80 bg-slate-700 border-l border-slate-600 p-4 flex flex-col max-h-screen overflow-y-auto">
      <div className="flex justify-between items-center mb-4">
        <h2 className="text-lg font-bold text-white">Selected Text</h2>
        <button
          onClick={onClose}
          className="p-1 hover:bg-slate-600 rounded-lg transition-colors"
        >
          <X size={20} className="text-slate-400" />
        </button>
      </div>

      {selectedText ? (
        <div className="space-y-4">
          <div className="bg-slate-600 rounded-lg p-3">
            <p className="text-sm text-slate-300 leading-relaxed">{selectedText}</p>
          </div>
          <button
            onClick={() => setSelectedText(null)}
            className="w-full px-4 py-2 bg-red-500/20 hover:bg-red-500/30 text-red-200 rounded-lg transition-colors text-sm font-medium"
          >
            Clear Selection
          </button>
          <p className="text-xs text-slate-400">
            💡 Your next query will analyze only this selected text without performing a book search.
          </p>
        </div>
      ) : (
        <div className="text-center py-8">
          <p className="text-slate-400 text-sm">
            💬 Select text from the book or paste it here to analyze it specifically.
          </p>
          <textarea
            placeholder="Paste text here..."
            onChange={(e) => setSelectedText(e.target.value || null)}
            className="w-full mt-4 p-3 bg-slate-600 text-white rounded-lg border border-slate-500 focus:border-blue-500 outline-none text-sm resize-none h-32"
          />
        </div>
      )}
    </div>
  )
}
