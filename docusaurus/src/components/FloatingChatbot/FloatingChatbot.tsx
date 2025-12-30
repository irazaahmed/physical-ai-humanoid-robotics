import React, { useEffect } from 'react';

// Configuration for the chatbot API
const CHATBOT_CONFIG = {
  API_BASE_URL: process.env.REACT_APP_CHATBOT_API_URL || 'http://localhost:8000/api/v1',
  CHAT_ENDPOINT: '/chat',
  TIMEOUT_MS: 30000, // 30 seconds
  SESSION_PREFIX: 'web-session-'
};

// A safe floating chatbot with full functionality
const FloatingChatbot: React.FC = () => {
  useEffect(() => {
    // Only run in browser environment
    if (typeof window !== 'undefined') {
      let isOpen = false;
      let chatWindow: HTMLElement | null = null;

      // Create the floating button element
      const button = document.createElement('div');
      button.id = 'chatbot-fab';
      button.innerHTML = '💬';

      // Apply styles directly to the button
      Object.assign(button.style, {
        position: 'fixed',
        bottom: '20px',
        right: '20px',
        width: '55px',
        height: '55px',
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
        fontFamily: 'inherit'
      });

      // Add hover effect
      button.onmouseenter = () => {
        button.style.transform = 'scale(1.05)';
        button.style.backgroundColor = '#1863cc';
      };
      button.onmouseleave = () => {
        button.style.transform = 'scale(1)';
        button.style.backgroundColor = '#1a73e8';
      };

      // Function to create and show the chat window
      const showChatWindow = () => {
        if (chatWindow) {
          // If already open, just bring it to front
          chatWindow.style.display = 'block';
          chatWindow.style.zIndex = '1001';
          return;
        }

        isOpen = true;

        // Create chat window container
        chatWindow = document.createElement('div');
        chatWindow.id = 'chatbot-window';
        chatWindow.style.cssText = `
          position: fixed;
          bottom: 90px;
          right: 20px;
          width: 340px;
          height: 480px;
          max-width: calc(100vw - 40px);
          max-height: calc(100vh - 120px);
          background-color: white;
          border-radius: 12px;
          box-shadow: 0 8px 30px rgba(0, 0, 0, 0.2);
          display: flex;
          flex-direction: column;
          overflow: hidden;
          z-index: 1001;
          font-family: inherit;
          border: 1px solid #dee2e6;
        `;

        // Create header
        const header = document.createElement('div');
        header.className = 'chatbot-header';
        header.style.cssText = `
          padding: 16px;
          background-color: #f8f9fa;
          display: flex;
          justify-content: space-between;
          align-items: center;
          border-bottom: 1px solid #dee2e6;
        `;

        const title = document.createElement('h3');
        title.textContent = 'AI Assistant';
        title.style.cssText = `
          margin: 0;
          font-size: 1.1rem;
          color: #212529;
        `;

        const closeButton = document.createElement('button');
        closeButton.innerHTML = '×';
        closeButton.style.cssText = `
          background: none;
          border: none;
          font-size: 1.5rem;
          cursor: pointer;
          color: #6c757d;
          width: 32px;
          height: 32px;
          display: flex;
          align-items: center;
          justify-content: center;
          border-radius: 50%;
          transition: background-color 0.2s;
          font-family: inherit;
        `;

        closeButton.onmouseenter = () => {
          closeButton.style.backgroundColor = '#e9ecef';
        };
        closeButton.onmouseleave = () => {
          closeButton.style.backgroundColor = 'transparent';
        };

        closeButton.onclick = () => {
          chatWindow!.style.display = 'none';
          isOpen = false;
        };

        header.appendChild(title);
        header.appendChild(closeButton);

        // Create messages container
        const messagesContainer = document.createElement('div');
        messagesContainer.className = 'chat-messages';
        messagesContainer.style.cssText = `
          flex: 1;
          padding: 16px;
          overflow-y: auto;
          display: flex;
          flex-direction: column;
          gap: 10px;
          background-color: #ffffff;
        `;

        // Add welcome message
        const welcomeMessage = document.createElement('div');
        welcomeMessage.className = 'message assistant';
        welcomeMessage.style.cssText = `
          display: flex;
          justify-content: flex-start;
          margin-bottom: 10px;
        `;

        const welcomeBubble = document.createElement('div');
        welcomeBubble.className = 'message-bubble assistant';
        welcomeBubble.textContent = 'Hello! I\'m your AI assistant for Physical AI & Humanoid Robotics. How can I help you today?';
        welcomeBubble.style.cssText = `
          padding: 8px 12px;
          border-radius: 18px;
          max-width: 80%;
          word-wrap: break-word;
          background-color: #f5f5f5;
          color: #212529;
        `;

        welcomeMessage.appendChild(welcomeBubble);
        messagesContainer.appendChild(welcomeMessage);

        // Create input area
        const inputArea = document.createElement('div');
        inputArea.className = 'chat-input-area';
        inputArea.style.cssText = `
          padding: 16px;
          background-color: #f8f9fa;
          border-top: 1px solid #dee2e6;
        `;

        const inputContainer = document.createElement('div');
        inputContainer.className = 'input-container';
        inputContainer.style.cssText = `
          display: flex;
          gap: 10px;
        `;

        // Create textarea (single-line input)
        const textarea = document.createElement('input');
        textarea.type = 'text';
        textarea.placeholder = 'Ask a question...';
        textarea.style.cssText = `
          flex: 1;
          padding: 10px 15px;
          border: 1px solid #ced4da;
          border-radius: 18px;
          resize: none;
          font-size: 14px;
          font-family: inherit;
          min-height: 40px;
          height: 40px;
          max-height: 40px;
          outline: none;
          overflow: hidden;
          white-space: nowrap;
          text-overflow: ellipsis;
        `;

        // Prevent textarea from resizing
        textarea.oninput = () => {
          // No auto-resize functionality - single line input only
        };

        // Create send button
        const sendButton = document.createElement('button');
        sendButton.textContent = 'Send';
        sendButton.style.cssText = `
          width: 50px;
          height: 40px;
          padding: 8px;
          border: none;
          border-radius: 4px;
          background-color: #1a73e8;
          color: white;
          cursor: pointer;
          font-family: inherit;
        `;

        // Function to add a message to the chat
        const addMessage = (content: string, role: 'user' | 'assistant') => {
          const messageDiv = document.createElement('div');
          messageDiv.className = `message ${role}`;
          messageDiv.style.cssText = `
            display: flex;
            justify-content: ${role === 'user' ? 'flex-end' : 'flex-start'};
            margin-bottom: 10px;
          `;

          const messageBubble = document.createElement('div');
          messageBubble.className = `message-bubble ${role}`;
          messageBubble.textContent = content;
          messageBubble.style.cssText = `
            padding: 8px 12px;
            border-radius: 18px;
            max-width: 80%;
            word-wrap: break-word;
            background-color: ${role === 'user' ? '#e3f2fd' : '#f5f5f5'};
            color: ${role === 'user' ? '#1a73e8' : '#212529'};
          `;

          messageDiv.appendChild(messageBubble);
          messagesContainer.appendChild(messageDiv);

          // Scroll to bottom
          messagesContainer.scrollTop = messagesContainer.scrollHeight;
        };

        // Function to handle sending a message
        const sendMessage = async () => {
          const message = textarea.value.trim();
          if (!message) return;

          // Add user message
          addMessage(message, 'user');
          textarea.value = '';

          // Show loading indicator
          const loadingDiv = document.createElement('div');
          loadingDiv.className = 'message assistant';
          loadingDiv.style.cssText = `
            display: flex;
            justify-content: flex-start;
            margin-bottom: 10px;
          `;

          const loadingBubble = document.createElement('div');
          loadingBubble.className = 'message-bubble assistant';
          loadingBubble.textContent = 'Thinking...';
          loadingBubble.style.cssText = `
            padding: 8px 12px;
            border-radius: 18px;
            max-width: 80%;
            word-wrap: break-word;
            background-color: #f5f5f5;
            color: #6c757d;
            font-style: italic;
          `;

          loadingDiv.appendChild(loadingBubble);
          messagesContainer.appendChild(loadingDiv);
          messagesContainer.scrollTop = messagesContainer.scrollHeight;

          try {
            // Call the backend API to get the response
            const response = await fetch(`${CHATBOT_CONFIG.API_BASE_URL}${CHATBOT_CONFIG.CHAT_ENDPOINT}`, {
              method: 'POST',
              headers: {
                'Content-Type': 'application/json',
              },
              body: JSON.stringify({
                query: message,
                session_id: CHATBOT_CONFIG.SESSION_PREFIX + Date.now().toString()
              })
            });

            // Remove loading indicator
            messagesContainer.removeChild(loadingDiv);

            if (!response.ok) {
              throw new Error(`HTTP error! status: ${response.status}`);
            }

            const data = await response.json();

            // Add assistant response
            const responseText = data.answer || data.response || data.content || 'No response received';
            addMessage(responseText, 'assistant');
          } catch (error) {
            // Remove loading indicator
            messagesContainer.removeChild(loadingDiv);

            // Add error message
            addMessage('Sorry, I encountered an issue processing your request. Please make sure the backend service is running.', 'assistant');
            console.error('Error sending message:', error);
          }
        };

        // Event listeners
        sendButton.onclick = sendMessage;

        textarea.onkeypress = (e) => {
          if (e.key === 'Enter') {
            e.preventDefault();
            sendMessage();
          }
        };

        inputContainer.appendChild(textarea);
        inputContainer.appendChild(sendButton);
        inputArea.appendChild(inputContainer);

        // Assemble the chat window
        chatWindow.appendChild(header);
        chatWindow.appendChild(messagesContainer);
        chatWindow.appendChild(inputArea);

        // Add to document body
        document.body.appendChild(chatWindow);

        // Focus the textarea when the chat opens
        setTimeout(() => {
          textarea.focus();
        }, 100);
      };

      // Add click handler to the button
      button.onclick = showChatWindow;

      // Add to document body
      document.body.appendChild(button);

      // Cleanup function
      return () => {
        if (button.parentNode) {
          button.parentNode.removeChild(button);
        }
        if (chatWindow && chatWindow.parentNode) {
          chatWindow.parentNode.removeChild(chatWindow);
        }
      };
    }
  }, []);

  // This component renders nothing since it adds the chat interface directly to the DOM
  return null;
};

export default FloatingChatbot;