import type { Metadata } from 'next'
import './globals.css'

export const metadata: Metadata = {
  title: 'AI Book RAG Chatbot',
  description: 'Retrieval-Augmented Generation Chatbot for Books',
}

export default function RootLayout({
  children,
}: {
  children: React.ReactNode
}) {
  return (
    <html lang="en">
      <body>{children}</body>
    </html>
  )
}
