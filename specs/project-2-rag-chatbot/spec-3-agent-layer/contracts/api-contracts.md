# API Contracts: Project 2 – RAG Chatbot Agent Layer

## Agent Service API Endpoints

### POST /api/v1/chat
**Operation**: Process a conversational query and return a synthesized response
**Description**: Accepts a user query and returns an agent-generated response based on retrieved textbook content

#### Request
```json
{
  "query": "string",
  "session_id": "string (optional)",
  "parameters": {
    "top_k": "integer (optional, default: 5)",
    "threshold": "float (optional, default: 0.6)"
  }
}
```

#### Response (200 OK)
```json
{
  "response_id": "uuid",
  "query": "How does a PID controller work?",
  "answer": "A PID controller is a control loop feedback mechanism that calculates an error value as the difference between a desired setpoint and a measured process variable...",
  "sources": [
    {
      "url": "https://textbook.example.com/module3/chapter2",
      "module": "Module 3: Control Systems",
      "chapter": "PID Controllers",
      "section": "Basic Principles",
      "similarity_score": 0.87,
      "content_preview": "A PID controller has three terms: Proportional, Integral, Derivative..."
    }
  ],
  "confidence_score": 0.92,
  "processing_time_ms": 1250,
  "timestamp": "2025-12-15T10:30:00Z"
}
```

#### Error Response (400 Bad Request)
```json
{
  "error": "Invalid query format",
  "message": "Query must be between 3 and 10000 characters",
  "timestamp": "2025-12-15T10:30:00Z"
}
```

#### Error Response (429 Rate Limited)
```json
{
  "error": "Rate limit exceeded",
  "message": "Too many requests from your IP address. Please try again later.",
  "retry_after": 60,
  "timestamp": "2025-12-15T10:30:00Z"
}
```

### POST /api/v1/query
**Operation**: Process a direct information query and return structured results
**Description**: Accepts a query and returns the relevant content chunks with metadata

#### Request
```json
{
  "query": "string",
  "filters": {
    "module": "string (optional)",
    "chapter": "string (optional)"
  },
  "parameters": {
    "top_k": "integer (optional, default: 5)",
    "threshold": "float (optional, default: 0.6)"
  }
}
```

#### Response (200 OK)
```json
{
  "result_id": "uuid",
  "query": "Explain ROS 2 communication patterns",
  "results": [
    {
      "id": "chunk_abc123",
      "content": "ROS 2 uses a publisher-subscriber pattern for asynchronous message passing between nodes...",
      "similarity_score": 0.91,
      "rank": 0,
      "metadata": {
        "url": "https://textbook.example.com/ros2/communication",
        "module": "Module 2: ROS 2 Framework",
        "chapter": "Communication Patterns",
        "section": "Publisher-Subscriber Model",
        "chunk_index": 3,
        "hash": "sha256_hash_value"
      }
    }
  ],
  "search_parameters": {
    "k": 5,
    "threshold": 0.6,
    "distance_metric": "cosine"
  },
  "execution_time_ms": 845,
  "timestamp": "2025-12-15T10:30:00Z"
}
```

### GET /api/v1/health
**Operation**: Check the health status of the agent service
**Description**: Returns the operational status of the agent service and its dependencies

#### Response (200 OK)
```json
{
  "status": "healthy",
  "timestamp": "2025-12-15T10:30:00Z",
  "dependencies": {
    "openai_api": "connected",
    "qdrant_connection": "connected",
    "retrieval_pipeline": "available"
  },
  "metrics": {
    "active_sessions": 5,
    "queries_processed": 1250,
    "avg_response_time_ms": 1120
  }
}
```

### Tool Interface Contract (Internal)

#### Retrieval Tool Call
**Called by**: Agent when it needs to retrieve information
**Description**: Wrapper for the retrieval pipeline from Spec-2

##### Input
```json
{
  "query": "string",
  "parameters": {
    "top_k": "integer (default: 5)",
    "threshold": "float (default: 0.6)",
    "filters": "object (optional, e.g., {'module': 'Module 1'})"
  }
}
```

##### Output
```json
{
  "retrieval_id": "uuid",
  "chunks": [
    {
      "id": "string",
      "content": "string",
      "similarity_score": "float (0.0-1.0)",
      "rank": "integer",
      "metadata": {
        "url": "string",
        "module": "string",
        "chapter": "string",
        "section": "string",
        "chunk_index": "integer",
        "hash": "string"
      }
    }
  ],
  "execution_time_ms": "float",
  "retrieval_timestamp": "datetime"
}
```

## Validation Requirements

### Request Validation
- Query text must be 3-10000 characters
- Top-k parameter must be 1-50
- Threshold parameter must be 0.0-1.0
- Session ID must be valid UUID format if provided

### Response Validation
- Answer content must be provided for successful queries
- Sources array must contain all required metadata fields
- Confidence score must be between 0.0 and 1.0
- Processing times must be positive numbers

## Error Handling Contracts

### Client Errors (4xx)
- 400: Invalid request format or parameters
- 422: Unprocessable request due to content issues
- 429: Rate limit exceeded

### Server Errors (5xx)
- 500: Internal server error
- 502: Gateway/proxy error from external services (OpenAI, Qdrant)
- 503: Service temporarily unavailable (dependency failure)

## Rate Limiting Contract
- Limit: 10 requests per minute per IP address
- Exempted: Internal service-to-service calls
- Enforcement: Applied at the API gateway level before processing