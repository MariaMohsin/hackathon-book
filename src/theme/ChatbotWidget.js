import React, { useState } from 'react';

export default function ChatbotWidget() {
  const [isMinimized, setIsMinimized] = useState(true);
  const [messages, setMessages] = useState([
    { id: 1, text: "Hello! I'm your book assistant. Ask me anything about the book content.", sender: 'bot' }
  ]);
  const [inputValue, setInputValue] = useState('');
  const [isLoading, setIsLoading] = useState(false);
  const [isIndexing, setIsIndexing] = useState(false);

  // Use a constant backend URL - you can change this to your deployed backend URL
  const BACKEND_URL = 'https://mariamohsin-rag-baknd.hf.space'; // Change this to your actual backend URL

  const toggleMinimize = () => setIsMinimized(!isMinimized);

  const handleSendMessage = async () => {
    if (!inputValue.trim()) return;

    const userMessage = { id: Date.now(), text: inputValue, sender: 'user' };
    setMessages(prev => [...prev, userMessage]);
    setInputValue('');
    setIsLoading(true);

    try {
      const response = await fetch(`${BACKEND_URL}/chat`, {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({
          query: inputValue,
          temperature: 0.3,
          max_tokens: 500
        })
      });

      if (response.ok) {
        const data = await response.json();
        const botMessage = { id: Date.now() + 1, text: data.answer, sender: 'bot' };
        setMessages(prev => [...prev, botMessage]);
      } else {
        const errorData = await response.json();
        setMessages(prev => [...prev, { id: Date.now() + 1, text: `Sorry, I couldn't process your request: ${errorData.detail || 'Unknown error'}. Please try again.`, sender: 'bot' }]);
      }
    } catch (error) {
      console.error(error);
      setMessages(prev => [...prev, { id: Date.now() + 1, text: "Sorry, there was an error connecting to the server. Please try again later.", sender: 'bot' }]);
    } finally {
      setIsLoading(false);
    }
  };

  const handleIndexContent = async () => {
    const hasIndexedKey = localStorage.getItem('bookContentIndexed');
    if (hasIndexedKey) {
      setMessages(prev => [...prev, { id: Date.now(), text: "Book content is already indexed. To re-index, please clear the previous index first.", sender: 'bot' }]);
      return;
    }

    setMessages(prev => [...prev, { id: Date.now(), text: "Indexing book content... This may take a few moments.", sender: 'bot' }]);
    setIsIndexing(true);

    try {
      const bookContent = `# Physical AI & Humanoid Robotics Book

## Introduction
This book covers various aspects of physical AI and humanoid robotics.

## Chapter 1: Introduction to Robotics
Robotics is an interdisciplinary branch of engineering and science that includes mechanical engineering, electrical engineering, computer science, and others.

## Chapter 2: Simulation Environments
Modern robotics development relies heavily on simulation environments to test and validate robot behaviors.

### Gazebo and Unity
Gazebo and Unity are popular simulation platforms that provide physics simulation capabilities.

### NVIDIA Isaac
NVIDIA Isaac provides tools for developing robotics applications with AI.

## Chapter 3: Robot Operating System (ROS)
The Robot Operating System (ROS) is flexible framework for writing robot software.

## Chapter 4: Vision Language Action (VLA) Models
VLA models combine computer vision, natural language processing, and action execution.

## Chapter 5: Sensor Systems
Robots rely on various sensors to perceive their environment.

### Motion Sensors
Sensors that detect movement and orientation.

### Vision Sensors
Cameras and computer vision systems for visual perception.
`;

      const response = await fetch(`${BACKEND_URL}/index`, {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({
          documents: [bookContent]  // Send as documents array for the /index endpoint
        })
      });

      if (response.ok) {
        const data = await response.json();
        setMessages(prev => [...prev, { id: Date.now() + 1, text: `Successfully indexed ${data.chunks_processed} book chunks! You can now ask questions about the book content.`, sender: 'bot' }]);
        localStorage.setItem('bookContentIndexed', 'true');
      } else {
        const errorData = await response.json();
        setMessages(prev => [...prev, { id: Date.now() + 1, text: `Failed to index content. Error: ${errorData.detail || 'Unknown error'}`, sender: 'bot' }]);
      }
    } catch (error) {
      console.error(error);
      setMessages(prev => [...prev, { id: Date.now() + 1, text: "Error indexing content. Please check your backend server.", sender: 'bot' }]);
    } finally {
      setIsIndexing(false);
    }
  };

  const handleKeyPress = e => {
    if (e.key === 'Enter' && !e.shiftKey) {
      e.preventDefault();
      handleSendMessage();
    }
  };

  return (
    <div style={{ position: 'fixed', bottom: 20, right: 20, zIndex: 10000, fontFamily: 'inherit' }}>
      {!isMinimized && (
        <div style={{ position: 'absolute', bottom: 70, right: 0, width: 400, height: 500, boxShadow: '0 8px 32px rgba(0,0,0,0.2)', borderRadius: 16, overflow: 'hidden', backgroundColor: '#fff', display: 'flex', flexDirection: 'column' }}>
          <div style={{ background: '#2563eb', color: 'white', padding: 16, display: 'flex', justifyContent: 'space-between', alignItems: 'center' }}>
            <h3 style={{ margin: 0, fontSize: 16 }}>Book Assistant</h3>
            <button onClick={toggleMinimize} style={{ background: 'rgba(255,255,255,0.2)', color: 'white', border: 'none', borderRadius: '50%', width: 30, height: 30, cursor: 'pointer' }}>×</button>
          </div>

          <div style={{ flex: 1, padding: 16, overflowY: 'auto', display: 'flex', flexDirection: 'column' }}>
            {messages.map(m => (
              <div key={m.id} style={{ alignSelf: m.sender === 'user' ? 'flex-end' : 'flex-start', marginBottom: 10, maxWidth: '80%' }}>
                <div style={{ padding: '10px 14px', borderRadius: 18, backgroundColor: m.sender === 'user' ? '#dbeafe' : '#f3f4f6', color: '#374151', wordWrap: 'break-word' }}>{m.text}</div>
              </div>
            ))}
            {isLoading && <div style={{ alignSelf: 'flex-start', marginBottom: 10, maxWidth: '80%' }}><div style={{ padding: '10px 14px', borderRadius: 18, backgroundColor: '#f3f4f6', color: '#374151' }}>Thinking...</div></div>}
          </div>

          <div style={{ padding: 12, borderTop: '1px solid #e5e7eb', display: 'flex', flexDirection: 'column' }}>
            <div style={{ marginBottom: 8 }}>
              <button onClick={handleIndexContent} disabled={isIndexing} style={{ width: '100%', padding: 8, backgroundColor: isIndexing ? '#9ca3af' : '#10b981', color: 'white', border: 'none', borderRadius: 18, cursor: isIndexing ? 'not-allowed' : 'pointer', fontSize: 14 }}>
                {isIndexing ? 'Indexing...' : 'Index New Content'}
              </button>
            </div>
            <div style={{ display: 'flex' }}>
              <textarea value={inputValue} onChange={e => setInputValue(e.target.value)} onKeyPress={handleKeyPress} placeholder="Ask about the book..." style={{ flex: 1, padding: 10, border: '1px solid #d1d5db', borderRadius: 18, resize: 'none', maxHeight: 100, fontSize: 14 }} rows={1} />
              <button onClick={handleSendMessage} disabled={isLoading || !inputValue.trim()} style={{ marginLeft: 8, padding: '10px 16px', backgroundColor: isLoading || !inputValue.trim() ? '#9ca3af' : '#2563eb', color: 'white', border: 'none', borderRadius: 18, cursor: isLoading || !inputValue.trim() ? 'not-allowed' : 'pointer' }}>Send</button>
            </div>
          </div>
        </div>
      )}

      <button onClick={toggleMinimize} style={{ background: '#2563eb', color: 'white', border: 'none', borderRadius: '50%', width: 60, height: 60, display: 'flex', alignItems: 'center', justifyContent: 'center', cursor: 'pointer', boxShadow: '0 4px 12px rgba(37,99,235,0.4)', fontSize: 24, zIndex: 10001 }}>
        {isMinimized ? '🤖' : '×'}
      </button>
    </div>
  );
}