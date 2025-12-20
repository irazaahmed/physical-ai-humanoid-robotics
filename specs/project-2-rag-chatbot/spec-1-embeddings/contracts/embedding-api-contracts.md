# Embedding Pipeline API Contracts

## Ingestion API

### POST /api/v1/ingest
**Description**: Initiate the ingestion process for a set of URLs
**Request Body**:
```json
{
  "urls": ["https://textbook.example.com/module1/chapter1", ...],
  "options": {
    "normalize_content": true,
    "chunk_size": 512,
    "overlap_size": 64,
    "resume_from_failure": true
  }
}
```
**Response**:
```json
{
  "job_id": "uuid",
  "status": "initiated",
  "total_pages": 120,
  "estimated_duration_minutes": 60
}
```

### GET /api/v1/ingest/{job_id}
**Description**: Get the status of an ingestion job
**Response**:
```json
{
  "job_id": "uuid",
  "status": "IN_PROGRESS|COMPLETED|FAILED",
  "progress": {
    "total_pages": 120,
    "processed_pages": 45,
    "failed_pages": 0
  },
  "start_time": "2025-12-15T10:00:00Z",
  "end_time": "2025-12-15T11:30:00Z",
  "resume_point": "https://textbook.example.com/module1/chapter45"
}
```

## Embedding API

### POST /api/v1/embed
**Description**: Generate embeddings for a text chunk
**Request Body**:
```json
{
  "text": "Text content to generate embeddings for...",
  "metadata": {
    "url": "https://textbook.example.com/module1/chapter1",
    "module": "Module 1: Robotic Nervous System",
    "chapter": "Introduction to ROS 2",
    "section": "Basic Concepts",
    "chunk_index": 0
  }
}
```
**Response**:
```json
{
  "embedding_id": "uuid",
  "vector_size": 1024,
  "vector": [0.1, 0.3, ..., 0.8], // Actual vector values
  "metadata": {
    "url": "https://textbook.example.com/module1/chapter1",
    "module": "Module 1: Robotic Nervous System",
    "chapter": "Introduction to ROS 2",
    "section": "Basic Concepts",
    "chunk_index": 0
  }
}
```

## Storage API

### POST /api/v1/storage/vector
**Description**: Store an embedding vector in Qdrant with metadata
**Request Body**:
```json
{
  "id": "unique_chunk_identifier",
  "vector": [0.1, 0.3, ..., 0.8],
  "payload": {
    "url": "https://textbook.example.com/module1/chapter1",
    "module": "Module 1: Robotic Nervous System",
    "chapter": "Introduction to ROS 2",
    "section": "Basic Concepts",
    "chunk_index": 0,
    "hash": "sha256_hash_of_content"
  }
}
```
**Response**:
```json
{
  "status": "stored",
  "vector_id": "unique_chunk_identifier",
  "collection_name": "physical_ai_textbook_embeddings"
}
```