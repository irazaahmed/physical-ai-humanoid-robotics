# Chatbot Setup Instructions

## Backend Setup

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

   Or using environment variables:
   ```bash
   export START_AGENT_SERVICE=true
   python -m src.main
   ```

The backend service will start on port 8001 with the `/api/v1/chat` endpoint available.

## Frontend Setup

1. Navigate to the docusaurus directory:
   ```bash
   cd docusaurus
   ```

2. Install dependencies:
   ```bash
   npm install
   ```

3. Set the API URL for the chatbot (optional - defaults to localhost:8001):
   ```bash
   # Create a .env file in the docusaurus directory
   echo "REACT_APP_CHATBOT_API_URL=http://localhost:8001/api/v1" > .env
   ```

4. Start the Docusaurus development server:
   ```bash
   npm start
   ```

## Production Build

For production deployment, ensure the backend API is accessible from your frontend domain and update the `REACT_APP_CHATBOT_API_URL` accordingly.

## Troubleshooting

- If the chatbot doesn't appear, ensure the backend service is running on port 8001
- Check browser console for CORS errors
- Verify that the GEMINI_API_KEY is properly configured in the backend .env file
- Make sure the agent service starts with `START_AGENT_SERVICE=true`