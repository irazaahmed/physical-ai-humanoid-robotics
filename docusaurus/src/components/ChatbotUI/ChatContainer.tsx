import React, { useState, useRef, useEffect } from 'react';
import MessageDisplay from './MessageDisplay';
import './ChatContainer.css';

interface Message {
  id: string;
  content: string;
  role: 'user' | 'assistant';
  timestamp: Date;
}

const ChatContainer: React.FC = () => {
  const [messages, setMessages] = useState<Message[]>([]);
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

    // Retry mechanism for API calls
    const maxRetries = 3;
    let retryCount = 0;
    let lastError: Error | null = null;

    while (retryCount <= maxRetries) {
      try {
        // Create an AbortController for timeout handling
        const controller = new AbortController();
        const timeoutId = setTimeout(() => controller.abort(), 30000); // 30 second timeout

        // Call the backend API to get the response
        const response = await fetch('http://localhost:8001/api/v1/chat', {
          method: 'POST',
          headers: {
            'Content-Type': 'application/json',
          },
          body: JSON.stringify({
            query: inputValue,
            session_id: 'web-session-' + Date.now().toString()
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
        lastError = null; // Clear any previous error on success
        break; // Exit retry loop on success
      } catch (error) {
        // Clear timeout if request fails
        if (typeof clearTimeout !== 'undefined') {
          // We can't clear the timeout here since it's already been triggered if this is a timeout error
        }

        // Check if the error is due to timeout
        if (error instanceof Error && error.name === 'AbortError') {
          lastError = new Error('Request timed out. Please try again.');
        } else {
          lastError = error as Error;
        }

        retryCount++;

        // If we've exhausted retries, break out of the loop
        if (retryCount > maxRetries) {
          break;
        }

        // Wait before retrying (exponential backoff: 1s, 2s, 4s)
        await new Promise(resolve => setTimeout(resolve, Math.pow(2, retryCount) * 1000));
      }
    }

    if (lastError) {
      console.error('Error sending message after retries:', lastError);

      // Add error message to the conversation
      const errorMessage: Message = {
        id: Date.now().toString(),
        content: 'Sorry, I encountered an issue processing your request. Please try again.',
        role: 'assistant',
        timestamp: new Date(),
      };

      setMessages(prev => [...prev, errorMessage]);
    }

    setIsLoading(false);
  };

  const handleKeyPress = (e: React.KeyboardEvent) => {
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

      <MessageDisplay messages={messages} isLoading={isLoading} />

      <div className="chat-input-area">
        <div className="input-container">
          <textarea
            value={inputValue}
            onChange={(e) => setInputValue(e.target.value)}
            onKeyPress={handleKeyPress}
            placeholder="Ask a question about Physical AI & Humanoid Robotics..."
            disabled={isLoading}
            rows={1}
            className="chat-input"
          />
          <button
            onClick={handleSendMessage}
            disabled={isLoading || !inputValue.trim()}
            className="send-button"
          >
            {isLoading ? 'Sending...' : 'Send'}
          </button>
        </div>
        <div className="input-hint">
          Press Enter to send, Shift+Enter for new line
        </div>
      </div>
    </div>
  );
};

export default ChatContainer;