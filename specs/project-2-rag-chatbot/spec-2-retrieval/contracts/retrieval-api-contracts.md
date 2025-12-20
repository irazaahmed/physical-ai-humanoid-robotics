# Retrieval Pipeline API Contracts

## Search API

### POST /api/v1/retrieve
**Description**: Perform semantic search and retrieve relevant content based on a query
**Request Body**:
```json
{
  "query": "string",
  "k": 5,
  "threshold": 0.6
}
```
**Response**:
```json
{
  "id": "uuid",
  "query": "How does a PID controller work?",
  "results": [
    {
      "id": "unique_chunk_id",
      "content": "A PID controller is a control loop feedback mechanism...",
      "similarity_score": 0.85,
      "rank": 0,
      "metadata": {
        "url": "https://textbook.example.com/module3/chapter2",
        "module": "Module 3: AI-Robot Brain",
        "chapter": "PID Controllers",
        "section": "Basic Principles",
        "chunk_index": 12,
        "hash": "sha256_hash_value"
      }
    }
  ],
  "search_parameters": {
    "k": 5,
    "threshold": 0.6,
    "distance_metric": "cosine"
  },
  "execution_time_ms": 845
}
```

### POST /api/v1/query-embedding
**Description**: Generate embedding vector for a query text
**Request Body**:
```json
{
  "query": "string"
}
```
**Response**:
```json
{
  "query_id": "uuid",
  "query": "How does a PID controller work?",
  "embedding": [0.1, 0.3, ..., 0.8],
  "vector_size": 1024,
  "model_used": "embed-multilingual-v3.0"
}
```

### POST /api/v1/validate-results
**Description**: Validate search results based on predefined criteria
**Request Body**:
```json
{
  "search_result_id": "string",
  "results": [
    {
      "id": "string",
      "content": "string",
      "similarity_score": 0.85,
      "metadata": {}
    }
  ]
}
```
**Response**:
```json
{
  "validation_id": "uuid",
  "search_result_id": "string",
  "is_valid": true,
  "confidence_score": 0.92,
  "issues_found": [],
  "validation_rules_applied": [
    "relevance_threshold_check",
    "metadata_completeness",
    "result_diversity"
  ]
}
```