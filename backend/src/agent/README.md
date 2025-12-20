# Agent Service Usage Guide

The Agent Service is part of the RAG Chatbot system and provides intelligent query processing capabilities using OpenAI's Agents SDK.

## Overview

The agent service processes user queries by:
1. Understanding the query using OpenAI's language models
2. Determining if information retrieval is needed
3. Calling the retrieval tool to fetch relevant textbook content
4. Synthesizing a response based on the retrieved information
5. Returning the response with source citations

## API Endpoints

### Chat Endpoint
- **Method**: `POST`
- **Path**: `/api/v1/chat`
- **Description**: Process conversational queries and return synthesized responses

#### Request Body
```json
{
  "query": "Your question here",
  "session_id": "optional-session-id",
  "parameters": {
    "top_k": 5,
    "threshold": 0.6
  }
}
```

#### Example Request
```bash
curl -X POST http://localhost:8001/api/v1/chat \
  -H "Content-Type: application/json" \
  -d '{
    "query": "Explain how PID controllers work",
    "parameters": {
      "top_k": 5,
      "threshold": 0.6
    }
  }'
```

### Query Endpoint
- **Method**: `POST`
- **Path**: `/api/v1/query`
- **Description**: Process direct information queries and return structured results

#### Request Body
```json
{
  "query": "Your question here",
  "filters": {
    "module": "optional module filter"
  },
  "parameters": {
    "top_k": 5,
    "threshold": 0.6
  }
}
```

### Health Check Endpoint
- **Method**: `GET`
- **Path**: `/api/v1/health`
- **Description**: Check the health status of the agent service

## Environment Variables

The agent service requires the following environment variables:

```bash
OPENAI_API_KEY=your_openai_api_key
RATE_LIMIT=10/minute  # Default: 10 requests per minute per IP
MAX_TOOL_CALLS_PER_QUERY=5  # Maximum tool calls per query
RESPONSE_TIMEOUT_SECONDS=30  # Query response timeout
DEFAULT_TOP_K=5  # Default number of results to retrieve
DEFAULT_THRESHOLD=0.6  # Default similarity threshold
```

## Rate Limiting

The service enforces rate limiting at 10 requests per minute per IP address to prevent abuse and manage API costs.

## Error Handling

The service returns appropriate HTTP status codes:
- `200`: Successful request
- `400`: Invalid request format or parameters
- `429`: Rate limit exceeded
- `500`: Internal server error

## Running the Service

To start the agent service:

```bash
START_AGENT_SERVICE=true python -m backend.src.main
```

The service will start on port 8001 by default.

## Configuration

The agent behavior can be configured via the environment variables listed above. Most settings have sensible defaults, but you can adjust them based on your needs:

- `MAX_TOOL_CALLS_PER_QUERY`: Limits the number of retrieval calls per query to prevent infinite loops
- `RESPONSE_TIMEOUT_SECONDS`: Sets the maximum time allowed for processing a query
- `DEFAULT_TOP_K`: Number of results to retrieve by default
- `DEFAULT_THRESHOLD`: Minimum similarity score for retrieved results