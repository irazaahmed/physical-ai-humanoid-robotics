# Data Model: Project 2 – RAG Chatbot Embedding Pipeline

## Entity: TextbookContent
- **Fields**:
  - `id` (string): Unique identifier for the content
  - `url` (string): The source URL of the textbook page
  - `module` (string): The module identifier (e.g., "Module 1: Robotic Nervous System")
  - `chapter` (string): The chapter identifier (e.g., "Introduction to ROS 2")
  - `section` (string): The section within the chapter
  - `raw_content` (string): The extracted raw text content
  - `processed_content` (string): The normalized and cleaned content
  - `created_at` (datetime): Timestamp of when the content was first processed
  - `updated_at` (datetime): Timestamp of when the content was last updated

## Entity: ContentChunk
- **Fields**:
  - `id` (string): Unique identifier for the chunk
  - `content_id` (string): Reference to the parent TextbookContent
  - `chunk_index` (integer): The sequential index of this chunk within the content
  - `text` (string): The actual text content of this chunk (max 512 tokens)
  - `token_count` (integer): Number of tokens in the chunk
  - `overlap_text` (string): Overlapping text from previous chunk (max 64 tokens)
  - `hash` (string): Hash of the chunk content for idempotency checks

## Entity: EmbeddingVector
- **Fields**:
  - `id` (string): Unique identifier for the embedding (often same as chunk ID)
  - `chunk_id` (string): Reference to the parent ContentChunk
  - `vector` (array[float]): The high-dimensional vector representation from Cohere
  - `vector_size` (integer): Dimension of the vector (e.g., 1024 for Cohere models)
  - `created_at` (datetime): Timestamp of when the embedding was generated

## Entity: ProcessingJob
- **Fields**:
  - `id` (string): Unique identifier for the processing job
  - `status` (enum): Current status of the job (PENDING, IN_PROGRESS, COMPLETED, FAILED)
  - `start_time` (datetime): When the job started
  - `end_time` (datetime): When the job ended
  - `total_pages` (integer): Total number of pages to process
  - `processed_pages` (integer): Number of pages processed so far
  - `failed_pages` (integer): Number of pages that failed to process
  - `error_log` (string): Any error messages from the job
  - `resume_point` (string): URL or identifier to resume from if job fails

## Entity: QdrantEmbeddingRecord
- **Fields** (This represents the structure stored in Qdrant):
  - `id` (string): The record ID (same as chunk ID for consistency)
  - `vector` (array[float]): The embedding vector
  - `payload` (object): Metadata object containing:
    - `url` (string): Source URL of the content
    - `module` (string): Module identifier
    - `chapter` (string): Chapter identifier
    - `section` (string): Section identifier
    - `chunk_index` (integer): Index of the chunk
    - `hash` (string): Content hash for idempotency
    - `created_at` (datetime): When the record was created

## Relationships
- `TextbookContent` (1) to `ContentChunk` (many): One textbook content can be split into many chunks
- `ContentChunk` (1) to `EmbeddingVector` (1): Each chunk has one corresponding embedding vector
- `ProcessingJob` (1) to `ContentChunk` (many): One job processes many chunks

## Validation Rules
- TextbookContent.url must be a valid URL format
- ContentChunk.token_count must be <= 512 tokens
- ContentChunk.overlap_text must be <= 64 tokens
- ContentChunk.hash must be unique to ensure idempotency
- QdrantEmbeddingRecord.payload must contain all required metadata fields
- ProcessingJob.status must be one of the defined enum values

## State Transitions
- ProcessingJob: PENDING → IN_PROGRESS → COMPLETED | FAILED