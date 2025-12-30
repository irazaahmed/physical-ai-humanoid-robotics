import React, { useState, useEffect, useRef } from 'react';

interface Message {
  id: string;
  content: string;
  role: 'user' | 'assistant';
  timestamp: Date;
}

// Configuration for the chatbot API - using browser-safe approach
const CHATBOT_CONFIG = {
  API_BASE_URL: 'http://localhost:8000/api/v1', // Default API URL
  CHAT_ENDPOINT: '/chat',
  TIMEOUT_MS: 30000, // 30 seconds
  SESSION_PREFIX: 'web-session-'
};

const ClientOnlyChatbot: React.FC = () => {
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
        content: 'Sorry, I\'m having trouble connecting right now. Please check that the backend service is running.',
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

  return (
    <>
      {/* Floating button */}
      {!isOpen && (
        <button
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
            fontFamily: '-apple-system, BlinkMacSystemFont, \'Segoe UI\', Roboto, Oxygen, Ubuntu, Cantarell, \'Open Sans\', \'Helvetica Neue\', sans-serif',
            textAlign: 'center',
            lineHeight: '1',
            padding: 0
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
        </button>
      )}

      {/* Chat window */}
      {isOpen && (
        <div
          id="chatbot-window"
          style={{
            position: 'fixed',
            bottom: '90px',
            right: '20px',
            width: '400px',
            height: '550px',
            maxWidth: 'calc(100vw - 20px)',
            maxHeight: 'calc(100vh - 40px)',
            backgroundColor: '#ffffff',
            borderRadius: '16px',
            boxShadow: '0 10px 40px rgba(0, 0, 0, 0.15)',
            display: 'flex',
            flexDirection: 'column',
            overflow: 'hidden',
            zIndex: '1001',
            fontFamily: '-apple-system, BlinkMacSystemFont, \'Segoe UI\', Roboto, Oxygen, Ubuntu, Cantarell, \'Open Sans\', \'Helvetica Neue\', sans-serif',
            border: '1px solid #e9ecef'
          }}
        >
          {/* Header */}
          <div
            className="chatbot-header"
            style={{
              padding: '16px 20px',
              backgroundColor: '#f8f9fa',
              display: 'flex',
              justifyContent: 'space-between',
              alignItems: 'center',
              borderBottom: '1px solid #e9ecef'
            }}
          >
            <div style={{
              display: 'flex',
              alignItems: 'center',
              gap: '10px'
            }}>
              <div style={{
                width: '12px',
                height: '12px',
                borderRadius: '50%',
                backgroundColor: '#28a745',
                boxShadow: '0 0 6px rgba(40, 167, 69, 0.5)'
              }}></div>
              <h3 style={{
                margin: 0,
                fontSize: '1.1rem',
                color: '#212529',
                fontWeight: 600
              }}>
                AI Assistant
              </h3>
            </div>
            <button
              onClick={closeChat}
              aria-label="Close chat"
              style={{
                background: 'none',
                border: 'none',
                fontSize: '1.25rem',
                cursor: 'pointer',
                color: '#6c757d',
                width: '36px',
                height: '36px',
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
              padding: '20px',
              overflowY: 'auto',
              display: 'flex',
              flexDirection: 'column',
              gap: '15px',
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
                  marginBottom: '0'
                }}
              >
                <div
                  className={`message-bubble ${message.role}`}
                  style={{
                    padding: '12px 16px',
                    borderRadius: '20px',
                    maxWidth: '85%',
                    wordWrap: 'break-word',
                    backgroundColor: message.role === 'user' ? '#007bff' : '#f8f9fa',
                    color: message.role === 'user' ? '#ffffff' : '#212529',
                    boxShadow: message.role === 'user'
                      ? '0 2px 8px rgba(0, 123, 255, 0.2)'
                      : '0 2px 4px rgba(0, 0, 0, 0.05)',
                    border: message.role === 'assistant' ? '1px solid #e9ecef' : 'none',
                    fontSize: '0.95rem',
                    lineHeight: '1.4'
                  }}
                >
                  {message.content === 'Thinking...' && message.role === 'assistant' ? (
                    <div style={{ display: 'flex', alignItems: 'center', gap: '6px' }}>
                      <div style={{
                        width: '8px',
                        height: '8px',
                        borderRadius: '50%',
                        backgroundColor: '#6c757d',
                        animation: 'pulse 1.5s infinite'
                      }}></div>
                      <div style={{
                        width: '8px',
                        height: '8px',
                        borderRadius: '50%',
                        backgroundColor: '#6c757d',
                        animation: 'pulse 1.5s infinite 0.3s'
                      }}></div>
                      <div style={{
                        width: '8px',
                        height: '8px',
                        borderRadius: '50%',
                        backgroundColor: '#6c757d',
                        animation: 'pulse 1.5s infinite 0.6s'
                      }}></div>
                    </div>
                  ) : (
                    message.content
                  )}
                </div>
              </div>
            ))}
            <div ref={messagesEndRef} />
          </div>

          {/* Input area */}
          <div
            className="chat-input-area"
            style={{
              padding: '16px 20px',
              backgroundColor: '#ffffff',
              borderTop: '1px solid #e9ecef'
            }}
          >
            <form
              onSubmit={(e) => {
                e.preventDefault();
                handleSendMessage();
              }}
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
                  padding: '12px 16px',
                  border: '1px solid #ced4da',
                  borderRadius: '20px',
                  resize: 'none',
                  fontSize: '0.95rem',
                  fontFamily: 'inherit',
                  minHeight: '46px',
                  maxHeight: '120px',
                  outline: 'none',
                  backgroundColor: '#ffffff',
                  color: '#212529',
                  transition: 'border-color 0.2s'
                }}
                onFocus={(e) => {
                  e.target.style.borderColor = '#007bff';
                }}
                onBlur={(e) => {
                  e.target.style.borderColor = '#ced4da';
                }}
                onInput={(e) => {
                  const target = e.target as HTMLTextAreaElement;
                  target.style.height = 'auto';
                  target.style.height = Math.min(target.scrollHeight, 120) + 'px';
                }}
              />
              <button
                type="submit"
                disabled={isLoading || !inputValue.trim()}
                aria-label="Send message"
                style={{
                  width: '46px',
                  height: '46px',
                  padding: '0',
                  border: 'none',
                  borderRadius: '50%',
                  backgroundColor: isLoading || !inputValue.trim() ? '#adb5bd' : '#007bff',
                  color: 'white',
                  cursor: isLoading || !inputValue.trim() ? 'not-allowed' : 'pointer',
                  fontFamily: 'inherit',
                  fontSize: '1.1rem',
                  display: 'flex',
                  alignItems: 'center',
                  justifyContent: 'center',
                  transition: 'background-color 0.2s'
                }}
                onMouseEnter={(e) => {
                  if (!isLoading && inputValue.trim()) {
                    e.currentTarget.style.backgroundColor = '#0069d9';
                  }
                }}
                onMouseLeave={(e) => {
                  if (!isLoading && inputValue.trim()) {
                    e.currentTarget.style.backgroundColor = '#007bff';
                  }
                }}
              >
                {isLoading ? (
                  <div style={{
                    width: '12px',
                    height: '12px',
                    borderTop: '2px solid white',
                    borderRight: '2px solid white',
                    borderRadius: '50%',
                    animation: 'spin 1s linear infinite'
                  }}></div>
                ) : '➤'}
              </button>
            </form>
          </div>

          <style jsx>{`
            @keyframes pulse {
              0%, 100% { opacity: 1; }
              50% { opacity: 0.4; }
            }

            @keyframes spin {
              0% { transform: rotate(0deg); }
              100% { transform: rotate(360deg); }
            }
          `}</style>
        </div>
      )}
    </>
  );
};

export default ClientOnlyChatbot;