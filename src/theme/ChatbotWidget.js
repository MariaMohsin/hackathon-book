import React, { useState } from 'react';

const API_BASE_URL = "https://<RENDER_BACKEND_URL>.onrender.com";

export default function ChatbotWidget() {
  const [isMinimized, setIsMinimized] = useState(true);
  const [messages, setMessages] = useState([
    { id: 1, text: "Hello! I'm your book assistant. Ask me anything about the book content.", sender: 'bot' }
  ]);
  const [inputValue, setInputValue] = useState('');
  const [isLoading, setIsLoading] = useState(false);
  const [isIndexing, setIsIndexing] = useState(false);

  const toggleMinimize = () => {
    setIsMinimized(!isMinimized);
  };

  const handleSendMessage = async () => {
    if (!inputValue.trim()) return;

    // Add user message
    const userMessage = {
      id: Date.now(),
      text: inputValue,
      sender: 'user'
    };

    setMessages(prev => [...prev, userMessage]);
    setInputValue('');
    setIsLoading(true);

    try {
                         
      const response = await fetch(`https://mariamohsin-rag-baknd.hf.space/api/chat`, {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify({
          query: inputValue,  // Using 'query' to match backend schema that accepts both 'query' and 'question'
          selected_text: ''   // Include selected_text field as expected by backend
        })
      });

      if (response.ok) {
        const data = await response.json();
        const botMessage = {
          id: Date.now() + 1,
          text: data.answer,
          sender: 'bot'
        };
        setMessages(prev => [...prev, botMessage]);
      } else {
        const errorMessage = {
          id: Date.now() + 1,
          text: "Sorry, I couldn't process your request. Please try again.",
          sender: 'bot'
        };
        setMessages(prev => [...prev, errorMessage]);
      }
    } catch (error) {
      console.error('Error:', error);
      const errorMessage = {
        id: Date.now() + 1,
        text: "Sorry, there was an error connecting to the server. Please try again later.",
        sender: 'bot'
      };
      setMessages(prev => [...prev, errorMessage]);
    } finally {
      setIsLoading(false);
    }
  };

  const handleIndexContent = async () => {
    // Check if content is already indexed
    const hasIndexedKey = localStorage.getItem('bookContentIndexed');
    if (hasIndexedKey) {
      const existingMessage = {
        id: Date.now(),
        text: "Book content is already indexed. To re-index, please clear the previous index first.",
        sender: 'bot'
      };
      setMessages(prev => [...prev, existingMessage]);
      return;
    }

    // Show a message indicating indexing has started
    const indexingMessage = {
      id: Date.now(),
      text: "Indexing book content... This may take a few moments.",
      sender: 'bot'
    };
    setMessages(prev => [...prev, indexingMessage]);
    setIsIndexing(true);

    try {
      // Load the combined book content from the static directory
      let bookContent = "# Physical AI & Humanoid Robotics Book\n\n";

      try {
        const response = await fetch('/static/book-content.md');
        if (response.ok) {
          bookContent = await response.text();
          console.log("Loaded content from static assets");
        } else {
          throw new Error(`Failed to fetch book content: ${response.status} ${response.statusText}`);
        }
      } catch (error) {
        console.error("Could not load content from static assets:", error);
        // Fallback: Use minimal content if static file is not available
        bookContent = `# Physical AI & Humanoid Robotics Book

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
      }

      const response = await fetch(`${API_BASE_URL}/api/index`, {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify({
          book_content: bookContent,
          book_metadata: {
            title: "Physical AI & Humanoid Robotics Book",
            author: "AI Native Book Project",
            section: "Complete Book",
            language: "en"
          }
        })
      });

      if (response.ok) {
        const data = await response.json();
        // Add success message
        const successMessage = {
          id: Date.now() + 1,
          text: `Successfully indexed ${data.chunks_indexed} book chunks! You can now ask questions about the book content.`,
          sender: 'bot'
        };
        setMessages(prev => [...prev, successMessage]);
        // Set flag to indicate content has been indexed
        localStorage.setItem('bookContentIndexed', 'true');
      } else {
        const errorData = await response.json();
        const errorMessage = {
          id: Date.now() + 1,
          text: `Failed to index content. Error: ${errorData.detail || 'Unknown error'}`,
          sender: 'bot'
        };
        setMessages(prev => [...prev, errorMessage]);
      }
    } catch (error) {
      console.error('Indexing Error:', error);
      const errorMessage = {
        id: Date.now() + 1,
        text: "Error indexing content. Please check your backend server.",
        sender: 'bot'
      };
      setMessages(prev => [...prev, errorMessage]);
    } finally {
      setIsIndexing(false);
    }
  };

  const handleKeyPress = (e) => {
    if (e.key === 'Enter' && !e.shiftKey) {
      e.preventDefault();
      handleSendMessage();
    }
  };

  return (
    <div
      style={{
        position: 'fixed',
        bottom: '20px',
        right: '20px',
        zIndex: 10000,
        fontFamily: 'inherit',
      }}
    >
      {!isMinimized && (
        <div
          style={{
            position: 'absolute',
            bottom: '70px',
            right: '0',
            width: '400px',
            height: '500px',
            boxShadow: '0 8px 32px rgba(0, 0, 0, 0.2)',
            borderRadius: '16px',
            overflow: 'hidden',
            backgroundColor: '#fff',
            display: 'flex',
            flexDirection: 'column',
          }}
        >
          {/* Chat Header */}
          <div
            style={{
              background: '#2563eb',
              color: 'white',
              padding: '16px',
              display: 'flex',
              justifyContent: 'space-between',
              alignItems: 'center',
            }}
          >
            <h3 style={{ margin: 0, fontSize: '16px' }}>Book Assistant</h3>
            <button
              onClick={toggleMinimize}
              aria-label="Close chatbot"
              style={{
                background: 'rgba(255, 255, 255, 0.2)',
                color: 'white',
                border: 'none',
                borderRadius: '50%',
                width: '30px',
                height: '30px',
                display: 'flex',
                alignItems: 'center',
                justifyContent: 'center',
                cursor: 'pointer',
                fontSize: '18px',
              }}
            >
              ×
            </button>
          </div>

          {/* Chat Messages */}
          <div
            style={{
              flex: 1,
              padding: '16px',
              overflowY: 'auto',
              display: 'flex',
              flexDirection: 'column',
            }}
          >
            {messages.map((message) => (
              <div
                key={message.id}
                style={{
                  alignSelf: message.sender === 'user' ? 'flex-end' : 'flex-start',
                  marginBottom: '10px',
                  maxWidth: '80%',
                }}
              >
                <div
                  style={{
                    padding: '10px 14px',
                    borderRadius: '18px',
                    backgroundColor: message.sender === 'user' ? '#dbeafe' : '#f3f4f6',
                    color: '#374151',
                    wordWrap: 'break-word',
                  }}
                >
                  {message.text}
                </div>
              </div>
            ))}
            {isLoading && (
              <div
                style={{
                  alignSelf: 'flex-start',
                  marginBottom: '10px',
                  maxWidth: '80%',
                }}
              >
                <div
                  style={{
                    padding: '10px 14px',
                    borderRadius: '18px',
                    backgroundColor: '#f3f4f6',
                    color: '#374151',
                  }}
                >
                  Thinking...
                </div>
              </div>
            )}
          </div>

          {/* Input Area */}
          <div
            style={{
              padding: '12px',
              borderTop: '1px solid #e5e7eb',
              display: 'flex',
              flexDirection: 'column',
            }}
          >
            {/* Indexing Button */}
            <div style={{ marginBottom: '8px' }}>
              <button
                onClick={handleIndexContent}
                disabled={isIndexing}
                style={{
                  width: '100%',
                  padding: '8px',
                  backgroundColor: isIndexing ? '#9ca3af' : '#10b981',
                  color: 'white',
                  border: 'none',
                  borderRadius: '18px',
                  cursor: isIndexing ? 'not-allowed' : 'pointer',
                  fontSize: '14px',
                }}
              >
                {isIndexing ? 'Indexing...' : 'Index New Content'}
              </button>
            </div>

            {/* Chat Input */}
            <div style={{ display: 'flex' }}>
              <textarea
                value={inputValue}
                onChange={(e) => setInputValue(e.target.value)}
                onKeyPress={handleKeyPress}
                placeholder="Ask about the book..."
                style={{
                  flex: 1,
                  padding: '10px',
                  border: '1px solid #d1d5db',
                  borderRadius: '18px',
                  resize: 'none',
                  maxHeight: '100px',
                  fontSize: '14px',
                }}
                rows={1}
              />
              <button
                onClick={handleSendMessage}
                disabled={isLoading || !inputValue.trim()}
                style={{
                  marginLeft: '8px',
                  padding: '10px 16px',
                  backgroundColor: isLoading || !inputValue.trim() ? '#9ca3af' : '#2563eb',
                  color: 'white',
                  border: 'none',
                  borderRadius: '18px',
                  cursor: isLoading || !inputValue.trim() ? 'not-allowed' : 'pointer',
                }}
              >
                Send
              </button>
            </div>
          </div>
        </div>
      )}

      <button
        onClick={toggleMinimize}
        aria-label={isMinimized ? 'Open chatbot' : 'Close chatbot'}
        style={{
          background: '#2563eb',
          color: 'white',
          border: 'none',
          borderRadius: '50%',
          width: '60px',
          height: '60px',
          display: 'flex',
          alignItems: 'center',
          justifyContent: 'center',
          cursor: 'pointer',
          boxShadow: '0 4px 12px rgba(37, 99, 235, 0.4)',
          fontSize: '24px',
          zIndex: 10001,
          transition: 'transform 0.2s ease, box-shadow 0.2s ease',
        }}
      >
        {isMinimized ? '🤖' : '×'}
      </button>
    </div>
  );
}
