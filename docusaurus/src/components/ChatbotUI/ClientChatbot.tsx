import React, { useState, useEffect, useRef } from 'react';

interface Message {
  id: string;
  content: string;
  role: 'user' | 'assistant';
  timestamp: Date;
}

// Configuration for the chatbot API
const CHATBOT_CONFIG = {
  API_BASE_URL: process.env.REACT_APP_CHATBOT_API_URL ||
               (typeof window !== 'undefined' && window.location.hostname === 'localhost'
                 ? 'http://localhost:8000/api/v1'
                 : 'https://irazaahmed-rag-chatbot.hf.space/api/v1'), // Use Hugging Face Space URL for production
  CHAT_ENDPOINT: '/chat',
  TIMEOUT_MS: 30000, // 30 seconds
  SESSION_PREFIX: 'web-session-'
};

const ClientChatbot: React.FC = () => {
  const [isOpen, setIsOpen] = useState(false);
  const [messages, setMessages] = useState<Message[]>([
    {
      id: 'welcome',
      content: 'Hello! I\'m your AI assistant for Physical AI & Humanoid Robotics. How can I help you today?',
      role: 'assistant',
      timestamp: new Date()
    }
  ]);
  const [inputValue, setInputValue] = useState('');
  const [isLoading, setIsLoading] = useState(false);
  const messagesEndRef = useRef<HTMLDivElement>(null);

  // Auto-scroll to bottom when messages change
  useEffect(() => {
    scrollToBottom();
  }, [messages]);

  const scrollToBottom = () => {
    messagesEndRef.current?.scrollIntoView({ behavior: 'smooth' });
  };

  const toggleChat = () => {
    setIsOpen(!isOpen);
  };

  const closeChat = () => {
    setIsOpen(false);
  };

  const handleSendMessage = async () => {
    if (!inputValue.trim() || isLoading) return;

    // Add user message to the conversation
    const userMessage: Message = {
      id: Date.now().toString(),
      content: inputValue,
      role: 'user',
      timestamp: new Date(),
    };

    // Add loading indicator
    const loadingMessage: Message = {
      id: 'loading',
      content: 'Thinking...',
      role: 'assistant',
      timestamp: new Date(),
    };

    setMessages(prev => [...prev, userMessage, loadingMessage]);
    setInputValue('');
    setIsLoading(true);

    try {
      // Call the backend API to get the response
      const response = await fetch(`${CHATBOT_CONFIG.API_BASE_URL}${CHATBOT_CONFIG.CHAT_ENDPOINT}`, {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify({
          query: inputValue,
          session_id: CHATBOT_CONFIG.SESSION_PREFIX + Date.now().toString(),
          parameters: {
            top_k: 5,
            threshold: 0.6,
            filters: null
          }
        })
      });

      // Remove loading indicator by filtering it out
      setMessages(prev => prev.filter(msg => msg.id !== 'loading'));

      if (!response.ok) {
        throw new Error(`HTTP error! status: ${response.status}`);
      }

      const data = await response.json();

      // Add assistant response to the conversation
      const assistantMessage: Message = {
        id: Date.now().toString(),
        content: data.answer || data.response || data.content || 'No response received',
        role: 'assistant',
        timestamp: new Date(),
      };

      setMessages(prev => [...prev, assistantMessage]);
    } catch (error) {
      // Remove loading indicator
      setMessages(prev => prev.filter(msg => msg.id !== 'loading'));

      // Add error message to the conversation
      const errorMessage: Message = {
        id: Date.now().toString(),
        content: 'Sorry, I encountered an issue processing your request. Please make sure the backend service is running.',
        role: 'assistant',
        timestamp: new Date(),
      };

      setMessages(prev => [...prev, errorMessage]);
      console.error('Error sending message:', error);
    } finally {
      setIsLoading(false);
    }
  };

  const handleKeyPress = (e: React.KeyboardEvent<HTMLTextAreaElement>) => {
    if (e.key === 'Enter' && !e.shiftKey) {
      e.preventDefault();
      handleSendMessage();
    }
  };

  if (typeof window === 'undefined') {
    // Don't render anything on the server
    return null;
  }

  return (
    <>
      {/* Floating button */}
      {!isOpen && (
        <div
          id="chatbot-fab"
          onClick={toggleChat}
          aria-label="Open AI Assistant"
          style={{
            position: 'fixed',
            bottom: '20px',
            right: '20px',
            width: '60px',
            height: '60px',
            borderRadius: '50%',
            backgroundColor: '#1a73e8',
            color: 'white',
            display: 'flex',
            alignItems: 'center',
            justifyContent: 'center',
            cursor: 'pointer',
            boxShadow: '0 4px 12px rgba(0, 0, 0, 0.15)',
            zIndex: '1000',
            fontSize: '24px',
            fontWeight: 'bold',
            border: 'none',
            userSelect: 'none',
            transition: 'all 0.3s ease',
            fontFamily: 'inherit',
            textAlign: 'center',
            lineHeight: '1'
          }}
          onMouseEnter={(e) => {
            e.currentTarget.style.transform = 'scale(1.05)';
            e.currentTarget.style.backgroundColor = '#1863cc';
          }}
          onMouseLeave={(e) => {
            e.currentTarget.style.transform = 'scale(1)';
            e.currentTarget.style.backgroundColor = '#1a73e8';
          }}
        >
          💬
        </div>
      )}

      {/* Chat window */}
      {isOpen && (
        <div
          id="chatbot-window"
          style={{
            position: 'fixed',
            bottom: '90px',
            right: '20px',
            width: '380px',
            height: '500px',
            maxWidth: 'calc(100vw - 40px)',
            maxHeight: 'calc(100vh - 120px)',
            backgroundColor: 'white',
            borderRadius: '12px',
            boxShadow: '0 8px 30px rgba(0, 0, 0, 0.2)',
            display: 'flex',
            flexDirection: 'column',
            overflow: 'hidden',
            zIndex: '1001',
            fontFamily: '-apple-system, BlinkMacSystemFont, \'Segoe UI\', Roboto, Oxygen, Ubuntu, Cantarell, \'Open Sans\', \'Helvetica Neue\', sans-serif',
            border: '1px solid #dee2e6'
          }}
        >
          {/* Header */}
          <div
            className="chatbot-header"
            style={{
              padding: '16px',
              backgroundColor: '#f8f9fa',
              display: 'flex',
              justifyContent: 'space-between',
              alignItems: 'center',
              borderBottom: '1px solid #dee2e6'
            }}
          >
            <h3 style={{
              margin: 0,
              fontSize: '1.1rem',
              color: '#212529',
              fontWeight: 600
            }}>
              AI Assistant
            </h3>
            <button
              onClick={closeChat}
              style={{
                background: 'none',
                border: 'none',
                fontSize: '1.5rem',
                cursor: 'pointer',
                color: '#6c757d',
                width: '32px',
                height: '32px',
                display: 'flex',
                alignItems: 'center',
                justifyContent: 'center',
                borderRadius: '50%',
                transition: 'background-color 0.2s',
                fontFamily: 'inherit',
                fontWeight: 'normal',
                lineHeight: 1,
                padding: 0
              }}
              onMouseEnter={(e) => {
                e.currentTarget.style.backgroundColor = '#e9ecef';
              }}
              onMouseLeave={(e) => {
                e.currentTarget.style.backgroundColor = 'transparent';
              }}
            >
              ×
            </button>
          </div>

          {/* Messages container */}
          <div
            className="chat-messages"
            style={{
              flex: 1,
              padding: '16px',
              overflowY: 'auto',
              display: 'flex',
              flexDirection: 'column',
              gap: '10px',
              backgroundColor: '#ffffff'
            }}
          >
            {messages.map((message) => (
              <div
                key={message.id}
                className={`message ${message.role}`}
                style={{
                  display: 'flex',
                  justifyContent: message.role === 'user' ? 'flex-end' : 'flex-start',
                  marginBottom: '10px'
                }}
              >
                <div
                  className={`message-bubble ${message.role}`}
                  style={{
                    padding: '8px 12px',
                    borderRadius: '18px',
                    maxWidth: '80%',
                    wordWrap: 'break-word',
                    backgroundColor: message.role === 'user' ? '#e3f2fd' : '#f5f5f5',
                    color: message.role === 'user' ? '#1a73e8' : '#212529'
                  }}
                >
                  {message.content}
                </div>
              </div>
            ))}
            <div ref={messagesEndRef} />
          </div>

          {/* Input area */}
          <div
            className="chat-input-area"
            style={{
              padding: '16px',
              backgroundColor: '#f8f9fa',
              borderTop: '1px solid #dee2e6'
            }}
          >
            <div
              className="input-container"
              style={{
                display: 'flex',
                gap: '10px'
              }}
            >
              <textarea
                value={inputValue}
                onChange={(e) => setInputValue(e.target.value)}
                onKeyPress={handleKeyPress}
                placeholder="Ask a question..."
                disabled={isLoading}
                rows={1}
                style={{
                  flex: 1,
                  padding: '10px',
                  border: '1px solid #ced4da',
                  borderRadius: '18px',
                  resize: 'none',
                  fontSize: '14px',
                  fontFamily: 'inherit',
                  minHeight: '40px',
                  maxHeight: '100px',
                  outline: 'none'
                }}
                onInput={(e) => {
                  const target = e.target as HTMLTextAreaElement;
                  target.style.height = 'auto';
                  target.style.height = Math.min(target.scrollHeight, 100) + 'px';
                }}
              />
              <button
                onClick={handleSendMessage}
                disabled={isLoading || !inputValue.trim()}
                style={{
                  width: '50px',
                  height: '40px',
                  padding: '8px',
                  border: 'none',
                  borderRadius: '4px',
                  backgroundColor: isLoading || !inputValue.trim() ? '#ccc' : '#1a73e8',
                  color: 'white',
                  cursor: isLoading || !inputValue.trim() ? 'not-allowed' : 'pointer',
                  fontFamily: 'inherit'
                }}
              >
                {isLoading ? '...' : 'Send'}
              </button>
            </div>
          </div>
        </div>
      )}
    </>
  );
};

export default ClientChatbot;