# Quickstart Guide: Project 2 – RAG Chatbot Agent Layer

## Overview
This guide provides a quick setup and usage introduction for the RAG Chatbot Agent Layer. The agent layer serves as an intelligent interface that uses OpenAI Agents to enhance the retrieval pipeline with reasoning and synthesis capabilities.

## Prerequisites

- Python 3.11+
- OpenAI API key with access to Agents API
- Access to the retrieval pipeline from Spec-2
- Qdrant Cloud access details
- Valid Cohere API key for retrieval pipeline

## Environment Setup

1. **Environment Variables**
   Create a `.env` file with the following variables:
   ```
   OPENAI_API_KEY=your_openai_api_key
   COHERE_API_KEY=your_cohere_api_key
   QDRANT_API_KEY=your_qdrant_api_key
   QDRANT_URL=your_qdrant_cloud_url
   TEXTBOOK_BASE_URL=https://your-textbook-site.com
   ```

2. **Install Dependencies**
   The agent layer uses the same requirements as the retrieval pipeline:
   ```
   pip install -r backend/requirements.txt
   ```

## Project Structure
```
backend/src/agent-layer/
├── __init__.py
├── main.py                 # Application entry point
├── agent/
│   ├── __init__.py
│   ├── core.py             # Main agent implementation
│   ├── tools.py            # Tool definitions and integrations
│   └── orchestrator.py     # Agent execution orchestrator
├── api/
│   ├── __init__.py
│   └── routes.py           # API endpoint definitions
├── models/
│   ├── __init__.py
│   └── schemas.py          # Data models and validation schemas
├── utils/
│   ├── __init__.py
│   ├── config.py           # Configuration management
│   ├── logging.py          # Logging utilities
│   └── validators.py       # Validation functions
└── services/
    ├── __init__.py
    └── retrieval_client.py # Client for retrieval pipeline integration
```

## Getting Started

1. **Initialize the Agent System**
   ```python
   from agent_layer.agent.orchestrator import AgentOrchestrator
   
   # Initialize the orchestrator
   orchestrator = AgentOrchestrator()
   
   # Validate the setup
   if orchestrator.validate_setup():
       print("Agent layer initialized successfully")
   else:
       print("Setup validation failed - check configuration")
   ```

2. **Making API Requests**
   Once the service is running, send requests to the endpoints:
   
   ### Chat Endpoint
   ```bash
   curl -X POST http://localhost:8000/api/v1/chat \
     -H "Content-Type: application/json" \
     -d '{
       "query": "Explain how PID controllers work in robotics",
       "session_id": "unique-session-id"
     }'
   ```

   ### Query Endpoint
   ```bash
   curl -X POST http://localhost:8000/api/v1/query \
     -H "Content-Type: application/json" \
     -d '{
       "query": "What are the key components of a robotic control system?",
       "parameters": {
         "top_k": 5,
         "threshold": 0.6
       }
     }'
   ```

## Configuration Options

- `OPENAI_MODEL`: The model to use for the agent (default: gpt-4-turbo)
- `DEFAULT_TOP_K`: Number of results to retrieve (default: 5)
- `DEFAULT_THRESHOLD`: Minimum similarity threshold (default: 0.6)
- `MAX_TOOL_CALLS_PER_QUERY`: Maximum number of tool calls (default: 5)
- `RATE_LIMIT_REQUESTS`: Number of requests allowed per minute per IP (default: 10)

## Basic Usage Example

```python
from agent_layer.agent.orchestrator import AgentOrchestrator

# Initialize the agent orchestrator
agent = AgentOrchestrator()

# Process a query
response = agent.process_query(
    query="How does inverse kinematics work in robotic arms?",
    top_k=3,
    threshold=0.7
)

print(f"Answer: {response.answer}")
print(f"Sources: {response.sources}")
print(f"Confidence: {response.confidence_score}")
```

## Testing the Implementation

Run the agent service with:
```
cd backend
uvicorn src.agent_layer.main:app --reload
```

The service will be available at `http://localhost:8000`.

## Integration with Retrieval Pipeline

The agent layer integrates with the existing retrieval pipeline:
1. Processes user queries using OpenAI Agent reasoning capabilities
2. Calls the retrieval pipeline as a tool to fetch relevant content
3. Synthesizes responses based on retrieved information
4. Returns contextual answers with source citations

The retrieval pipeline from Spec-2 is accessed via the retrieval client which adheres to the tool contract defined in the API specifications.