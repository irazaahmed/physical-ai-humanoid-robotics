// Client module to load the chatbot after the page is ready
import ExecutionEnvironment from '@docusaurus/ExecutionEnvironment';

export default function ChatbotLoader() {
  if (ExecutionEnvironment.canUseDOM) {
    // Create a script element to load the chatbot
    const script = document.createElement('script');
    script.type = 'module';
    script.innerHTML = `
      // Wait for the DOM to be fully loaded
      document.addEventListener('DOMContentLoaded', () => {
        // Create the chatbot container
        const chatbotContainer = document.createElement('div');
        chatbotContainer.id = 'floating-chatbot-container';
        document.body.appendChild(chatbotContainer);

        // Load the chatbot asynchronously
        try {
          // This would be replaced with actual React rendering
          // For now, we'll add a simple floating button that works independently
          const chatButton = document.createElement('div');
          chatButton.style.position = 'fixed';
          chatButton.style.bottom = '20px';
          chatButton.style.right = '20px';
          chatButton.style.width = '60px';
          chatButton.style.height = '60px';
          chatButton.style.borderRadius = '50%';
          chatButton.style.backgroundColor = '#1a73e8';
          chatButton.style.color = 'white';
          chatButton.style.display = 'flex';
          chatButton.style.alignItems = 'center';
          chatButton.style.justifyContent = 'center';
          chatButton.style.cursor = 'pointer';
          chatButton.style.boxShadow = '0 4px 12px rgba(0, 0, 0, 0.15)';
          chatButton.style.zIndex = '1000';
          chatButton.style.fontSize = '24px';
          chatButton.innerHTML = '💬';
          chatButton.title = 'AI Assistant';

          chatButton.onclick = () => {
            alert('Chatbot would open here. Make sure the backend service is running on port 8001.');
          };

          document.body.appendChild(chatButton);
        } catch (error) {
          console.error('Error loading chatbot:', error);
        }
      });
    `;

    document.head.appendChild(script);
  }

  return null;
}