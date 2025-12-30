# Research Summary: Project 2 – RAG Chatbot Embedding Pipeline

## Decision: Backend Technology Stack
**Rationale**: Selected Python with FastAPI based on the project constitution which specifies FastAPI for RAG services. FastAPI provides excellent async processing capabilities needed for the embedding pipeline and has built-in support for creating APIs for the RAG system.

**Alternatives considered**: 
- Flask: More established but less async-focused
- Node.js: Good for I/O operations but Python is better for ML/DS workflows
- Go: Good performance but less ML ecosystem support

## Decision: Package Management
**Rationale**: Selected UV package manager for its speed and efficiency in Python dependency management. UV is a fast alternative to pip that can speed up the development process.

**Alternatives considered**:
- pip + venv: Standard but slower
- Poetry: Feature-rich but potentially overkill for this project
- Conda: Good for data science but may be heavier than needed

## Decision: Embedding Model Provider
**Rationale**: Cohere embeddings were specified in the feature requirements. Cohere provides high-quality embeddings suitable for RAG applications with good API reliability.

**Alternatives considered**:
- OpenAI embeddings: Available and high quality but not specified in requirements
- Hugging Face models: Self-hostable but would add complexity
- OpenRouter: Another viable option but Cohere was specified

## Decision: Vector Database
**Rationale**: Qdrant Cloud was specified in the requirements and constitution. It provides managed vector storage with good Python client support and scalability.

**Alternatives considered**:
- Pinecone: Popular but different API approach
- Weaviate: Alternative vector DB with GraphQL interface
- Chroma: Open-source but less suitable for cloud deployment

## Decision: Web Scraping Approach
**Rationale**: Using BeautifulSoup with requests or aiohttp for sync/async crawling. This provides reliable HTML parsing capabilities to extract clean text content from the textbook website.

**Alternatives considered**:
- Selenium: More robust for JS-heavy sites but overkill for a Docusaurus site
- Scrapy: Powerful for large-scale scraping but might be overkill
- Playwright: Good for modern web apps but not needed here

## Decision: Content Chunking Strategy
**Rationale**: Following constitution specification of 512 tokens with 64-token overlap for optimal retrieval accuracy. This balances context preservation with retrieval precision.

**Alternatives considered**:
- Different token sizes: 256, 1024 tokens - but constitution specifies 512
- Different overlap: No overlap or larger overlap - but 64-token overlap is specified

## Decision: Idempotency Implementation
**Rationale**: Using content hash or URL + chunk index as unique identifier to prevent duplicates. This allows safe re-runs of the pipeline without creating duplicate vectors in Qdrant.

**Alternatives considered**:
- Database tracking: More complex but more feature-rich
- File-based tracking: Simpler but less robust
- Qdrant-based tracking: Using Qdrant's capabilities to prevent duplicates