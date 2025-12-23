import React, { useEffect, useState } from 'react';

const DynamicChatbot = () => {
  const [isClient, setIsClient] = useState(false);
  const [ChatbotComponent, setChatbotComponent] = useState<React.ComponentType | null>(null);

  useEffect(() => {
    // Mark as client-side only
    setIsClient(true);

    // Only import and render the chatbot on the client side
    if (typeof window !== 'undefined') {
      import('./ClientChatbot').then((module) => {
        setChatbotComponent(() => module.default);
      }).catch(error => {
        console.error('Failed to load chatbot component:', error);
      });
    }
  }, []);

  // Only render the chatbot component on the client side after it's loaded
  if (isClient && ChatbotComponent) {
    return <ChatbotComponent />;
  }

  // Render nothing on the server or if the component hasn't loaded yet
  return null;
};

export default DynamicChatbot;