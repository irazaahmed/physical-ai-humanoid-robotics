import React from 'react';
import { useIsBrowser } from '@docusaurus/theme-common';
import ClientOnlyChatbot from '../components/ChatbotUI/ClientOnlyChatbot';

// Root component with properly isolated client-side chatbot using Docusaurus useIsBrowser hook
const Root = ({ children }) => {
  const isBrowser = useIsBrowser();

  return (
    <>
      {children}
      {isBrowser && <ClientOnlyChatbot />}
    </>
  );
};

export default Root;