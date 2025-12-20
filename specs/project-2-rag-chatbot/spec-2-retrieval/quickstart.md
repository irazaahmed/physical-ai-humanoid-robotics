# Quickstart Guide: Project 2 – RAG Chatbot Retrieval Pipeline

## Prerequisites

- Python 3.11 or higher
- Cohere API key
- Qdrant Cloud account and API key
- Existing vector database with embedded textbook content

## Environment Setup

1. **Create virtual environment and install dependencies**:
   ```bash
   python -m venv venv
   source venv/bin/activate  # On Linux/MacOS
   # venv\Scripts\activate  # On Windows
   pip install cohere qdrant-client python-dotenv pydantic
   ```

2. **Set up environment variables**:
   Create a `.env` file with:
   ```
   COHERE_API_KEY=your_cohere_api_key
   QDRANT_URL=your_qdrant_cloud_url
   QDRANT_API_KEY=your_qdrant_api_key
   ```

## Project Structure

```
backend/
├── src/
│   ├── retrieval/
│   │   ├── __init__.py
│   │   ├── main.py              # Entry point for retrieval pipeline
│   │   ├── query_processor.py   # Query embedding generation
│   │   ├── search_engine.py     # Qdrant similarity search
│   │   ├── result_ranker.py     # Result ranking and filtering
│   │   ├── validator.py         # Validation logic for retrieval
│   │   └── models.py            # Data models for the pipeline
│   └── utils/
│       ├── config.py            # Configuration management
│       └── logging.py           # Logging utilities
```

## Getting Started

1. **Initialize the retrieval pipeline**:
   ```python
   from src.retrieval.main import RetrievalPipeline
   
   # Initialize the pipeline with configuration
   pipeline = RetrievalPipeline()
   ```

2. **Perform a retrieval for a query**:
   ```python
   query = "How does a robot's PID controller work?"
   results = pipeline.retrieve(query, k=5, threshold=0.6)
   
   # Process the results
   for chunk in results.chunks:
       print(f"Score: {chunk.similarity_score}")
       print(f"Content: {chunk.content}")
       print(f"Source: {chunk.metadata.url}")
   ```

3. **Access retrieval statistics**:
   ```python
   # Get performance metrics
   stats = pipeline.get_performance_stats()
   print(f"Average response time: {stats.avg_response_time} ms")
   print(f"Success rate: {stats.success_rate}%")
   ```

## Basic Usage Example

```python
from src.retrieval.main import RetrievalPipeline

# Initialize the pipeline
pipeline = RetrievalPipeline()

# Example query
query = "Explain ROS 2 service calls"

# Retrieve results with configurable parameters
search_result = pipeline.retrieve(
    query=query,
    k=5,              # Return top 5 results
    threshold=0.6     # Minimum relevance threshold
)

# Process and display results
for i, chunk in enumerate(search_result.chunks):
    print(f"Result {i+1} (Score: {chunk.similarity_score:.3f}):")
    print(f"  Source: {chunk.metadata.module} - {chunk.metadata.chapter}")
    print(f"  URL: {chunk.metadata.url}")
    print(f"  Excerpt: {chunk.content[:200]}...")
    print()
```

## Configuration

The retrieval pipeline can be configured with these parameters:

- `k`: Number of top results to return (default: 5)
- `threshold`: Minimum similarity score for relevance (default: 0.6)
- `distance_metric`: Distance metric for similarity search (default: cosine)
- `model`: Cohere model to use for embeddings (default: embed-multilingual-v3.0)

These can be adjusted via environment variables or code configuration.