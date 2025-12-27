import React, { useState } from 'react';
import './ChatContainer.css';

interface Message {
  id: string;
  content: string;
  role: 'user' | 'assistant';
  timestamp: Date;
}

// Configuration for the chatbot API
const CHATBOT_CONFIG = {
  // API endpoint configuration - defaults to localhost but can be overridden
  API_BASE_URL: process.env.REACT_APP_CHATBOT_API_URL || 'http://localhost:8001/api/v1',
  CHAT_ENDPOINT: '/chat',
  TIMEOUT_MS: 30000, // 30 seconds
  MAX_RETRIES: 3,
  SESSION_PREFIX: 'web-session-'
};

const ChatContainer: React.FC = () => {
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

  const handleSendMessage = async () => {
    if (!inputValue.trim() || isLoading) return;

    // Add user message to the conversation
    const userMessage: Message = {
      id: Date.now().toString(),
      content: inputValue,
      role: 'user',
      timestamp: new Date(),
    };

    setMessages(prev => [...prev, userMessage]);
    setInputValue('');
    setIsLoading(true);

    try {
      // Create an AbortController for timeout handling
      const controller = new AbortController();
      const timeoutId = setTimeout(() => controller.abort(), CHATBOT_CONFIG.TIMEOUT_MS);

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
        }),
        signal: controller.signal, // Use the abort signal for timeout
      });

      clearTimeout(timeoutId); // Clear timeout if request completes

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
      console.error('Error sending message:', error);

      // Add error message to the conversation
      const errorMessage: Message = {
        id: Date.now().toString(),
        content: 'Sorry, I encountered an issue processing your request. Please make sure the backend service is running.',
        role: 'assistant',
        timestamp: new Date(),
      };

      setMessages(prev => [...prev, errorMessage]);
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
    <div className="chat-container">
      <div className="chat-header">
        <h3>AI Assistant</h3>
      </div>

      <div className="chat-messages">
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
              }}
            >
              {message.content}
            </div>
          </div>
        ))}
        {isLoading && (
          <div
            style={{
              display: 'flex',
              justifyContent: 'flex-start',
              marginBottom: '10px'
            }}
          >
            <div
              style={{
                padding: '8px 12px',
                borderRadius: '18px',
                backgroundColor: '#f5f5f5',
              }}
            >
              Thinking...
            </div>
          </div>
        )}
      </div>

      <div className="chat-input-area">
        <div className="input-container">
          <textarea
            value={inputValue}
            onChange={(e) => setInputValue(e.target.value)}
            onKeyPress={handleKeyPress}
            placeholder="Ask a question..."
            disabled={isLoading}
            rows={1}
            className="chat-input"
            style={{
              width: 'calc(100% - 60px)',
              padding: '10px',
              border: '1px solid #ddd',
              borderRadius: '4px',
              resize: 'none',
              fontSize: '14px',
              minHeight: '40px',
              maxHeight: '100px'
            }}
          />
          <button
            onClick={handleSendMessage}
            disabled={isLoading || !inputValue.trim()}
            className="send-button"
            style={{
              width: '50px',
              height: '40px',
              marginLeft: '10px',
              padding: '8px',
              border: 'none',
              borderRadius: '4px',
              backgroundColor: isLoading || !inputValue.trim() ? '#ccc' : '#1a73e8',
              color: 'white',
              cursor: isLoading || !inputValue.trim() ? 'not-allowed' : 'pointer'
            }}
          >
            {isLoading ? '...' : 'Send'}
          </button>
        </div>
        <div className="input-hint" style={{ fontSize: '12px', color: '#999', marginTop: '5px' }}>
          Press Enter to send, Shift+Enter for new line
        </div>
      </div>
    </div>
  );
};

export default ChatContainer;