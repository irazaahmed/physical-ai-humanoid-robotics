# Quickstart Guide: Project 2 – RAG Chatbot Embedding Pipeline

## Prerequisites

- Python 3.11 or higher
- UV package manager
- Access to Cohere API (API key)
- Access to Qdrant Cloud (API key and URL)

## Environment Setup

1. **Create the backend directory and navigate to it**:
   ```bash
   mkdir backend
   cd backend
   ```

2. **Initialize a new Python project with UV**:
   ```bash
   uv init
   ```

3. **Install required dependencies**:
   ```bash
   uv pip install fastapi uvicorn cohere python-dotenv qdrant-client beautifulsoup4 aiohttp lxml
   ```

4. **Set up environment variables**:
   Create a `.env` file in the backend directory with:
   ```
   COHERE_API_KEY=your_cohere_api_key
   QDRANT_URL=your_qdrant_cloud_url
   QDRANT_API_KEY=your_qdrant_api_key
   TEXTBOOK_BASE_URL=https://your-deployed-textbook-url.com
   ```

## Project Structure

```
backend/
├── main.py                 # FastAPI application entry point
├── .env                    # Environment variables
├── .gitignore              # Git ignore file
├── requirements.txt        # Project dependencies
├── src/
│   ├── __init__.py
│   ├── ingestion/
│   │   ├── __init__.py
│   │   ├── crawler.py      # Web crawling functionality
│   │   ├── processor.py    # Content normalization and chunking
│   │   └── orchestrator.py # Pipeline orchestration
│   ├── embedding/
│   │   ├── __init__.py
│   │   ├── generator.py    # Cohere embedding generation
│   │   └── storage.py      # Qdrant storage operations
│   └── models/
│       ├── __init__.py
│       └── data_models.py  # Pydantic models for data validation
└── tests/
    ├── __init__.py
    └── test_ingestion.py   # Unit tests
```

## Getting Started

1. **Run the application locally**:
   ```bash
   cd backend
   uvicorn main:app --reload
   ```

2. **Start the embedding pipeline**:
   ```bash
   python -m src.ingestion.orchestrator
   ```

3. **Access the API documentation**:
   - Open your browser to `http://localhost:8000/docs` to view the interactive API documentation
   - Use the `/ingest` endpoint to start the ingestion process

## Basic Usage Example

```python
from src.ingestion.orchestrator import IngestionOrchestrator
from src.embedding.generator import CohereEmbedder
from src.embedding.storage import QdrantStorage

# Initialize components
embedder = CohereEmbedder()
storage = QdrantStorage()
orchestrator = IngestionOrchestrator(embedder, storage)

# Start ingestion
urls = [
    "https://your-textbook-url.com/module1/chapter1",
    "https://your-textbook-url.com/module1/chapter2"
]

# Process the URLs
result = orchestrator.ingest_urls(urls)
print(f"Ingestion completed: {result}")
```

## Configuration

The pipeline can be configured using environment variables in the `.env` file:

- `CHUNK_SIZE`: Size of text chunks (default: 512 tokens)
- `OVERLAP_SIZE`: Overlap between chunks (default: 64 tokens)
- `BATCH_SIZE`: Number of chunks to process in each batch (default: 10)
- `RESUME_ON_FAILURE`: Whether to resume from failure point (default: true)