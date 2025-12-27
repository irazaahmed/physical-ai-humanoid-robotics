import React from 'react';
import ClientOnlyChatbot from '../components/ChatbotUI/ClientOnlyChatbot';

// Root component with properly isolated client-side chatbot using standard browser check
const Root = ({ children }) => {
  const isBrowser = typeof window !== 'undefined';

  return (
    <>
      {children}
      {isBrowser && <ClientOnlyChatbot />}
    </>
  );
};

export default Root;