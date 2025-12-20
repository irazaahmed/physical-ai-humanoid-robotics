# Data Model: Project 2 – RAG Chatbot Retrieval Pipeline

## Entity: QueryText
- **Fields**:
  - `id` (string): Unique identifier for the query
  - `text` (string): The original user query text
  - `timestamp` (datetime): When the query was received
  - `session_id` (string): Identifier for the user session (optional)
  - `query_metadata` (object): Additional metadata about the query

## Entity: QueryEmbedding
- **Fields**:
  - `id` (string): Unique identifier for this embedding (matches QueryText.id)
  - `query_id` (string): Reference to the original query
  - `vector` (array[float]): The embedding vector representation of the query
  - `vector_size` (integer): Dimension of the embedding vector (typically 1024 for Cohere)
  - `model_used` (string): The model that generated this embedding
  - `created_at` (datetime): Timestamp of embedding generation

## Entity: RetrievedChunk
- **Fields**:
  - `id` (string): Unique identifier for this retrieved chunk
  - `query_embedding_id` (string): Reference to the query embedding that retrieved this chunk
  - `content` (string): The actual content text of the chunk
  - `similarity_score` (float): Semantic similarity score between query and chunk
  - `rank` (integer): Position in the ranked results (0-indexed)
  - `metadata` (object): Associated metadata including:
    - `url` (string): Source URL of the content
    - `module` (string): Module identifier containing the chunk
    - `chapter` (string): Chapter identifier containing the chunk
    - `section` (string): Section title of the content
    - `chunk_index` (integer): The index of this chunk within the original content
    - `hash` (string): Content hash for duplication detection

## Entity: SearchResult
- **Fields**:
  - `id` (string): Unique identifier for this search result
  - `query_id` (string): Reference to the original query
  - `query_embedding_id` (string): Reference to the generated query embedding
  - `chunks` (list): Ordered list of RetrievedChunk entities
  - `search_parameters` (object): Search parameters used:
    - `k` (integer): Number of requested top results
    - `relevance_threshold` (float): Minimum similarity score to include
    - `distance_metric` (string): Distance metric used (e.g. cosine, euclidean)
  - `execution_time_ms` (float): Time taken to perform the search
  - `retrieval_timestamp` (datetime): When the search was performed

## Entity: ValidationResult
- **Fields**:
  - `id` (string): Unique identifier for this validation
  - `search_result_id` (string): Reference to the search result being validated
  - `is_valid` (boolean): Whether the results pass validation criteria
  - `validation_issues` (list[string]): List of validation issues found
  - `confidence_score` (float): Overall confidence in the retrieved results
  - `validation_rules_applied` (list[string]): Rules applied during validation
  - `created_at` (datetime): Timestamp of validation

## Relationships
- `QueryText` (1) to `QueryEmbedding` (1): One-to-one mapping between query and its embedding
- `QueryEmbedding` (1) to `RetrievedChunk` (many): One query embedding retrieves multiple content chunks
- `QueryText` (1) to `SearchResult` (1): One query produces one search result
- `SearchResult` (1) to `ValidationResult` (1): One search result has one validation outcome

## Validation Rules
- QueryText.text must be non-empty and contain at least 3 characters
- QueryEmbedding.vector must have a consistent size matching Cohere model output
- RetrievedChunk.similarity_score must be between 0 and 1
- RetrievedChunk.metadata must contain all required fields (url, module, chapter)
- SearchResult.chunks length must be <= k parameter specified in search_parameters
- ValidationResult.confidence_score must be between 0 and 1

## State Transitions
- QueryText: PENDING → PROCESSING → COMPLETE
- QueryEmbedding: CREATED → SUCCESS / FAILED
- SearchResult: INITIALIZED → RETRIEVED → VALIDATED