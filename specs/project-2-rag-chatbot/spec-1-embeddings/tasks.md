---
tasks_id: project-2-tasks
project_ref: project-2-rag-chatbot
version: 1.0.0
author: Qwen
date: 2025-12-15
based_on_plan: plan.md
---

# Tasks: Project 2 – RAG Chatbot Spec 1 – Website ingestion, embeddings, and vector storage

## Project Summary

This plan outlines the implementation of the embedding pipeline for the Project 2 RAG Chatbot system. The focus is on creating a backend system that crawls the deployed textbook website, extracts content, normalizes and chunks it according to constitution rules, generates embeddings using Cohere models, and stores them in Qdrant Cloud with appropriate metadata. The plan specifically excludes retrieval logic, querying, and UI components.

## Phase 1: Setup Tasks

### Initialize Project Structure
- [X] T001 Create root-level backend folder per implementation plan
- [X] T002 Initialize UV project inside backend folder per implementation plan
- [X] T003 Add dependencies (Cohere SDK, Qdrant client, crawler utilities) to project
- [X] T004 Create project directory structure: backend/src/, backend/src/ingestion/, backend/src/embedding/, backend/src/storage/, backend/src/models/, backend/src/utils/
- [X] T005 Set up environment variables configuration for Cohere, Qdrant, and textbook URLs

## Phase 2: Foundational Tasks

### Setup Foundational Components
- [X] T006 Create configuration module to handle environment variables and settings per implementation plan
- [X] T007 Implement logging and error handling utilities for the pipeline
- [X] T008 Set up Cohere client with proper error handling and API key validation
- [X] T009 Set up Qdrant client with proper error handling and cloud connection validation
- [X] T010 Create data models for TextbookContent, ContentChunk, and EmbeddingVector based on data-model.md

## Phase 3: [US1] Content Crawling Implementation

### [US1] Web Crawling and Content Extraction
**Story Goal**: System successfully accesses all textbook website URLs and extracts clean textual content without errors.
**Independent Test**: System successfully accesses all textbook website URLs and extracts clean textual content without errors.

**Acceptance Scenarios**:
1. **Given** the system has access to the textbook website URLs, **When** the crawling process is initiated, **Then** all pages are accessed and textual content is extracted without errors
2. **Given** the system is crawling textbook pages, **When** it encounters various content types (text, code blocks, lists), **Then** it properly extracts only the textual content while preserving structure

- [X] T011 [US1] Implement website URL crawler with BeautifulSoup and requests/aiohttp
- [X] T012 [US1] Create function to extract clean text content from HTML, preserving structure
- [X] T013 [US1] Implement crawler with proper handling of different content types (text, code blocks, lists)
- [X] T014 [US1] Add retry logic for handling network errors during crawling (FR-009)
- [X] T015 [US1] Test crawler with sample textbook URLs to ensure clean content extraction

## Phase 4: [US2] Content Normalization and Chunking Implementation

### [US2] Content Processing Pipeline
**Story Goal**: System normalizes content and creates appropriately-sized chunks that match constitution requirements.
**Independent Test**: System normalizes content and creates appropriately-sized chunks that match constitution requirements.

**Acceptance Scenarios**:
1. **Given** raw content from textbook pages, **When** normalization process runs, **Then** content is cleaned and formatted consistently
2. **Given** normalized content, **When** chunking process runs, **Then** content is split into chunks of 512 tokens with 64-token overlap per constitution rules

- [X] T016 [US2] Implement content normalization function to clean and format text according to constitution
- [X] T017 [US2] Create text chunking algorithm to split content into 512 tokens with 64-token overlap (FR-004)
- [X] T018 [US2] Implement validation to ensure chunks comply with constitution specifications
- [X] T019 [US2] Add text cleaning utilities to remove unwanted elements while preserving meaning
- [X] T020 [US2] Test normalization and chunking with sample textbook content

## Phase 5: [US3] Embedding Generation Implementation

### [US3] Embedding Pipeline
**Story Goal**: System successfully generates embeddings for all content chunks with appropriate vector representations.
**Independent Test**: System successfully generates embeddings for all content chunks with appropriate vector representations.

**Acceptance Scenarios**:
1. **Given** properly chunked content, **When** embedding generation process runs, **Then** valid vector representations are created using Cohere models
2. **Given** generated embeddings, **When** quality validation occurs, **Then** embeddings accurately represent the semantic meaning of input text

- [X] T021 [US3] Implement function to generate embeddings using Cohere embedding models (FR-005)
- [X] T022 [US3] Add error handling for Cohere API errors with appropriate fallback mechanisms (FR-010)
- [X] T023 [US3] Create embedding validation to ensure vector representations accurately represent content
- [X] T024 [US3] Implement rate limiting handling for Cohere API to manage quota constraints
- [X] T025 [US3] Test embedding generation with sample content chunks

## Phase 6: [US4] Vector Storage Implementation

### [US4] Vector Storage and Metadata Management
**Story Goal**: System stores vectors in Qdrant Cloud with correct metadata that enables later retrieval.
**Independent Test**: System stores vectors in Qdrant Cloud with correct metadata that enables later retrieval.

**Acceptance Scenarios**:
1. **Given** generated embeddings and metadata, **When** storage process runs, **Then** vectors are successfully stored in Qdrant Cloud
2. **Given** stored vectors in Qdrant, **When** retrieval is requested by another system, **Then** vectors can be found and accessed with their associated metadata

- [X] T026 [US4] Create Qdrant collection with proper schema for embeddings and metadata (FR-006)
- [X] T027 [US4] Implement function to upsert vectors with metadata (module, chapter, section, page URL, chunk index) (FR-007)
- [X] T028 [US4] Add error handling for Qdrant Cloud errors with appropriate retry mechanisms (FR-011)
- [X] T029 [US4] Create utility to validate successful storage in Qdrant with correct metadata
- [X] T030 [US4] Test vector storage with sample embeddings and metadata

## Phase 7: [US5] Idempotent Ingestion Implementation

### [US5] Pipeline Orchestration and Idempotency
**Story Goal**: System can run the ingestion pipeline multiple times without creating duplicate entries.
**Independent Test**: System can run the ingestion pipeline multiple times without creating duplicate entries.

**Acceptance Scenarios**:
1. **Given** a complete ingestion run, **When** the pipeline is run again with the same content, **Then** no duplicate vectors are created
2. **Given** partial failure during ingestion, **When** the pipeline is re-run, **Then** only missing content is processed without duplicating existing entries

- [X] T031 [US5] Implement idempotent ingestion mechanism to prevent duplicate vectors (FR-008)
- [X] T032 [US5] Create checkpointing system to track ingestion progress for resuming from failure (FR-015)
- [X] T033 [US5] Implement duplicate detection using content hash or URL + chunk index
- [X] T034 [US5] Add functionality to resume ingestion from the point of failure
- [X] T035 [US5] Test idempotent behavior by re-running ingestion on partially completed content

## Phase 8: Validation and Testing Tasks

### Final Validation and Integration
- [X] T036 Validate that all book URLs are embedded successfully
- [X] T037 Verify that vectors exist in Qdrant with correct metadata
- [X] T038 Test that pipeline can be re-run safely without creating duplicates
- [X] T039 Run integration test of full pipeline: crawl → normalize → chunk → embed → store
- [X] T040 Perform stress test with full textbook content to validate performance within 4-hour goal

## Dependencies

### User Story Completion Order
1. US1 (Content Crawling) → US2, US3, US4, US5
2. US2 (Normalization/Chunking) → US3, US4, US5
3. US3 (Embedding Generation) → US4, US5
4. US4 (Vector Storage) → US5
5. US5 (Idempotent Ingestion) - Final completion

### Parallel Execution Opportunities
- T016, T017, T018, T019 [P] can be executed in parallel after US1 completion [P]
- T021, T022, T023, T024 [P] can be executed in parallel after T016-T020 completion [P]
- T026, T027, T028, T029 [P] can be executed in parallel after T021-T025 completion [P]

## Implementation Strategy

### MVP Scope
- Focus on US1 (crawling) and US2 (normalization/chunking) for initial working version
- Ensure basic web crawling and content extraction works with sample textbook URLs
- Add simple embedding and storage in later iterations

### Incremental Delivery
1. Complete Phase 1 & 2: Project setup and foundational components
2. Complete US1: Basic web crawling with clean content extraction
3. Complete US2: Content normalization and proper chunking per constitution
4. Complete US3: Embedding generation with Cohere
5. Complete US4: Vector storage in Qdrant with metadata
6. Complete US5: Full pipeline with idempotency and resumability
7. Complete Phase 8: Full validation and integration testing

## Success Criteria Validation

- [ ] SC-001: 100% of textbook website URLs are successfully crawled and content extracted with 95% success rate
- [ ] SC-002: All extracted content is normalized and chunked according to constitution specifications with 99% compliance
- [ ] SC-003: Embeddings are generated for 100% of content chunks with 99% success rate from Cohere API
- [ ] SC-004: All embeddings are successfully stored in Qdrant Cloud with correct metadata for 100% of vectors
- [ ] SC-005: Ingestion pipeline can be re-run without creating any duplicate vectors in the store (0% duplication rate)
- [ ] SC-006: System completes full ingestion of textbook content within 4 hours for typical content volume
- [ ] SC-007: Metadata is stored with 99.9% completeness for all required fields per constitution
- [ ] SC-008: System achieves 99% uptime during ingestion process with appropriate error handling
- [ ] SC-010: System can handle temporary service failures (Cohere, Qdrant, website) with automatic recovery