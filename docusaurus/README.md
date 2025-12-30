# Physical AI & Humanoid Robotics Textbook

This is the official textbook website for the Physical AI & Humanoid Robotics course, built using [Docusaurus](https://docusaurus.io/), a modern static website generator.

## Project 1 — Module 1: Robotic Nervous System (ROS 2)

This project focuses on generating Module 1 textbook content using Spec-Kit Plus and Claude Code, integrating content into Docusaurus, and producing deployable documentation for the ROS 2 fundamentals module.

## Opening the Book

To open the book locally:

```bash
npm start
```

Then visit [http://localhost:3000](http://localhost:3000) to access the book landing page and navigate to the content.

The book content can be accessed directly at `/docs/module1/intro`.

## Installation

```bash
npm install
```

## Local Development

```bash
npm start
```

This command starts a local development server and opens up a browser window. Most changes are reflected live without having to restart the server.

## Build

```bash
npm run build
```

This command generates static content into the `build` directory and can be served using any static contents hosting service.

## AI Chatbot Integration

This textbook includes an AI chatbot powered by OpenRouter that appears on every page as a floating icon at the bottom-right. The chatbot can answer questions about the textbook content using RAG (Retrieval-Augmented Generation) to provide accurate responses based on the textbook materials.

### Backend Setup

Before using the chatbot, you need to start the backend service:

1. Navigate to the backend directory:
   ```bash
   cd ../backend
   ```

2. Install the required dependencies:
   ```bash
   pip install -r requirements.txt
   ```

3. Start the agent service on port 8001:
   ```bash
   START_AGENT_SERVICE=true python -m src.main
   ```

### Frontend Setup

1. Ensure the backend service is running before starting the frontend
2. Start the Docusaurus development server:
   ```bash
   npm start
   ```

The floating chatbot icon should appear at the bottom-right of every page once both services are running.

### Troubleshooting

If you experience a black screen issue:
1. Make sure you've cleared the cache: `npx docusaurus clear`
2. Ensure the backend service is running before starting the frontend
3. Check that you're using Node.js version 20 or higher
4. The chatbot is now implemented safely to prevent breaking the main site

## Deployment

Using SSH:

```bash
USE_SSH=true npm run deploy
```

Not using SSH:

```bash
GIT_USER=<Your GitHub username> npm run deploy
```

If you are using GitHub pages for hosting, this command is a convenient way to build the website and push to the `gh-pages` branch.
