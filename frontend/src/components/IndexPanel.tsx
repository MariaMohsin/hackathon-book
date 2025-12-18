'use client'

import React, { useState } from 'react'
import { useChatStore } from '@/store/chatStore'
import { chatAPI } from '@/lib/api'
import { Upload, X, Loader } from 'lucide-react'

interface IndexPanelProps {
  onClose: () => void
}

export default function IndexPanel({ onClose }: IndexPanelProps) {
  const [bookContent, setBookContent] = useState('')
  const [error, setError] = useState<string | null>(null)
  const { setIndexing, isIndexing } = useChatStore()

  const handleFileUpload = async (e: React.ChangeEvent<HTMLInputElement>) => {
    const file = e.target.files?.[0]
    if (!file) return

    const reader = new FileReader()
    reader.onload = (event) => {
      setBookContent(event.target?.result as string)
    }
    reader.readAsText(file)
  }

  const handleIndexBook = async () => {
    if (!bookContent.trim()) {
      setError('Please paste or upload book content')
      return
    }

    setError(null)
    setIndexing(true)

    try {
      const response = await chatAPI.indexBook({
        book_content: bookContent,
        collection_reset: false,
      })

      if (response.success) {
        alert(`✅ Indexed ${response.chunks_indexed} chunks successfully!`)
        setBookContent('')
        onClose()
      } else {
        setError('Indexing failed: ' + response.message)
      }
    } catch (err: any) {
      setError(err.response?.data?.detail || 'Failed to index book')
    } finally {
      setIndexing(false)
    }
  }

  return (
    <div className="fixed inset-0 bg-black/50 flex items-center justify-center z-50">
      <div className="bg-slate-800 rounded-lg shadow-xl w-full max-w-2xl mx-4 max-h-96 overflow-y-auto border border-slate-700">
        <div className="sticky top-0 flex justify-between items-center p-4 border-b border-slate-700 bg-slate-800">
          <h2 className="text-xl font-bold text-white">📚 Index New Book</h2>
          <button
            onClick={onClose}
            className="p-1 hover:bg-slate-700 rounded-lg transition-colors"
          >
            <X size={20} className="text-slate-400" />
          </button>
        </div>

        <div className="p-6 space-y-4">
          {/* Upload File */}
          <div>
            <label className="block text-sm font-semibold text-slate-200 mb-2">Upload Book File</label>
            <label className="flex items-center justify-center w-full p-4 border-2 border-dashed border-slate-600 rounded-lg cursor-pointer hover:border-blue-500 transition-colors">
              <div className="flex items-center gap-2 text-slate-300">
                <Upload size={20} />
                <span>Choose file or drag here</span>
              </div>
              <input type="file" accept=".txt,.md" onChange={handleFileUpload} className="hidden" />
            </label>
          </div>

          {/* Or Paste Content */}
          <div>
            <label className="block text-sm font-semibold text-slate-200 mb-2">Or Paste Book Content</label>
            <textarea
              value={bookContent}
              onChange={(e) => setBookContent(e.target.value)}
              placeholder="Paste your book content here (markdown or plain text)..."
              className="w-full p-3 bg-slate-700 text-white rounded-lg border border-slate-600 focus:border-blue-500 outline-none resize-none h-32"
            />
          </div>

          {/* Error */}
          {error && <div className="p-3 bg-red-500/20 border border-red-500 text-red-100 rounded-lg text-sm">{error}</div>}

          {/* Info */}
          <div className="p-3 bg-blue-500/10 border border-blue-500/30 text-blue-100 rounded-lg text-sm">
            💡 The book will be split into chunks and stored in Qdrant for semantic search.
          </div>

          {/* Action Buttons */}
          <div className="flex gap-2 justify-end pt-4">
            <button
              onClick={onClose}
              className="px-4 py-2 bg-slate-700 hover:bg-slate-600 text-white rounded-lg transition-colors"
            >
              Cancel
            </button>
            <button
              onClick={handleIndexBook}
              disabled={isIndexing || !bookContent.trim()}
              className="px-6 py-2 bg-gradient-to-r from-blue-500 to-purple-500 hover:from-blue-600 hover:to-purple-600 text-white rounded-lg transition-all disabled:opacity-50 font-semibold flex items-center gap-2"
            >
              {isIndexing ? <Loader size={18} className="animate-spin" /> : <Upload size={18} />}
              {isIndexing ? 'Indexing...' : 'Index Book'}
            </button>
          </div>
        </div>
      </div>
    </div>
  )
}
