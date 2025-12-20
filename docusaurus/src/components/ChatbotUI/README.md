# Chatbot UI User Guide

## Overview
The AI Assistant chatbot provides a conversational interface for asking questions about Physical AI & Humanoid Robotics. The interface allows you to interact naturally with the textbook content through a simple chat interface.

## How to Use

### Asking Questions
1. Type your question in the text input area at the bottom of the chat
2. Press Enter or click the Send button to submit your question
3. Use Shift+Enter if you need to add a line break without sending

### Viewing Responses
- Your questions will appear on the right side with a distinct background
- AI responses will appear on the left side
- Responses contain information sourced from the textbook content
- Responses are displayed without technical metadata for clarity

### Conversation Flow
- The chat maintains your conversation history within the current session
- You can ask follow-up questions based on previous responses
- All interactions are processed by the backend RAG system

## Features

### Visual Indicators
- Typing indicators show when the AI is processing your request
- Loading states provide feedback during API communication
- Clear visual distinction between user and AI messages

### Error Handling
- If there are network issues, user-friendly error messages will appear
- The system will attempt to retry failed requests automatically
- No technical details are exposed to maintain security

### Responsive Design
- The chat interface works on both desktop and mobile devices
- Layout adjusts to different screen sizes for optimal experience

## Technical Details (For Reference)
- All processing happens on the backend system
- The UI is a thin presentation layer
- No personal data is stored or transmitted
- Questions and responses remain in your current session only