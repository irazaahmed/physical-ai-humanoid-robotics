import React, { useState } from 'react';
import ChatContainer from '../ChatbotUI/ChatContainer';
import './FloatingChatbot.css';

const FloatingChatbot: React.FC = () => {
  const [isOpen, setIsOpen] = useState(false);

  const toggleChat = () => {
    setIsOpen(!isOpen);
  };

  return (
    <div className="floating-chatbot">
      {isOpen ? (
        <div className="chatbot-window">
          <div className="chatbot-header">
            <h3>AI Assistant</h3>
            <button className="close-button" onClick={toggleChat} aria-label="Close chat">
              ×
            </button>
          </div>
          <div className="chatbot-content">
            <ChatContainer />
          </div>
        </div>
      ) : (
        <button
          className="chatbot-fab"
          onClick={toggleChat}
          aria-label="Open chatbot"
        >
          <svg
            xmlns="http://www.w3.org/2000/svg"
            width="24"
            height="24"
            viewBox="0 0 24 24"
            fill="none"
            stroke="currentColor"
            strokeWidth="2"
            strokeLinecap="round"
            strokeLinejoin="round"
            className="chat-icon"
          >
            <path d="M21 15a2 2 0 0 1-2 2H7l-4 4V5a2 2 0 0 1 2-2h14a2 2 0 0 1 2 2z"></path>
          </svg>
        </button>
      )}
    </div>
  );
};

export default FloatingChatbot;