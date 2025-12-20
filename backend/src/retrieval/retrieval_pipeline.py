from typing import List, Optional
from .config import Config
from .models import QueryText, QueryEmbedding, RetrievedChunk, SearchResult, ValidationResult
from .cohere_client import CohereClient
from .qdrant_searcher import QdrantSearcher
from .validation_utils import validate_retrieval_results, validate_query_text
from datetime import datetime
import uuid
import time
from .logging import setup_logging

logger = setup_logging()

class RetrievalPipeline:
    """Main pipeline class to coordinate components of the retrieval system"""
    
    def __init__(self, k: int = None, threshold: float = None):
        """
        Initialize the retrieval pipeline with Cohere and Qdrant clients
        
        Args:
            k: Number of top results to return (default from config)
            threshold: Minimum similarity score threshold (default from config)
        """
        # Validate configuration
        config_errors = Config.validate()
        if config_errors:
            raise ValueError(f"Configuration validation failed: {', '.join(config_errors)}")
        
        # Initialize components
        self.cohere_client = CohereClient()
        self.qdrant_searcher = QdrantSearcher()
        
        # Set default values from config or provided values
        self.k = k if k is not None else Config.DEFAULT_TOP_K
        self.threshold = threshold if threshold is not None else Config.DEFAULT_THRESHOLD
    
    def setup(self) -> bool:
        """
        Set up the retrieval pipeline by validating connections and configurations
        
        Returns:
            True if setup was successful, False otherwise
        """
        try:
            # Validate Cohere API key
            if not self.cohere_client.validate_api_key():
                logger.error("Cohere API key validation failed")
                return False
            
            # Validate Qdrant Cloud connection
            if not self.qdrant_searcher.validate_connection():
                logger.error("Qdrant Cloud connection validation failed")
                return False
            
            logger.info("Retrieval pipeline setup completed successfully")
            return True
            
        except Exception as e:
            logger.error(f"Error during pipeline setup: {str(e)}")
            return False
    
    def preprocess_query(self, query_text: str) -> str:
        """
        Preprocess the query text by cleaning and validating

        Args:
            query_text: Raw query text from user

        Returns:
            Clean, validated query text
        """
        # Validate query first
        validation_issues = validate_query_text(query_text)
        if validation_issues:
            raise ValueError(f"Query validation failed: {', '.join(validation_issues)}")

        # Basic text cleaning
        cleaned_query = query_text.strip()

        # Additional preprocessing steps
        # Remove extra whitespace while preserving meaning
        import re
        cleaned_query = re.sub(r'\s+', ' ', cleaned_query)

        # Log the preprocessing
        logger.info(f"Preprocessed query: '{cleaned_query[:50]}{'...' if len(cleaned_query) > 50 else ''}' "
                   f"(original length: {len(query_text)}, processed length: {len(cleaned_query)})")
        return cleaned_query
    
    def generate_query_embedding(self, query: str) -> QueryEmbedding:
        """
        Generate embedding vector for the query using Cohere
        
        Args:
            query: The preprocessed query text
            
        Returns:
            QueryEmbedding object containing the vector representation
        """
        try:
            # Generate embedding using Cohere
            embeddings = self.cohere_client.generate_embeddings([query])
            
            if not embeddings or len(embeddings) == 0:
                raise Exception("No embeddings generated for the query")
            
            query_embedding = embeddings[0]  # We only embedded one query
            
            # Create QueryEmbedding object
            query_embedding_obj = QueryEmbedding(
                id=str(uuid.uuid4()),
                query_id=str(uuid.uuid4()),  # This would normally be tied to a query object
                vector=query_embedding,
                vector_size=len(query_embedding),
                model_used=Config.COHERE_MODEL
            )
            
            logger.info(f"Successfully generated embedding for query (size: {len(query_embedding)})")
            return query_embedding_obj
            
        except Exception as e:
            logger.error(f"Failed to generate query embedding: {str(e)}")
            raise
    
    def retrieve(self, 
                query: str, 
                k: Optional[int] = None, 
                threshold: Optional[float] = None,
                filter_conditions: Optional[dict] = None) -> SearchResult:
        """
        Main retrieval function that takes a query and returns relevant content
        
        Args:
            query: The user query text
            k: Number of top results to return (uses default if None)
            threshold: Minimum similarity score threshold (uses default if None)
            filter_conditions: Optional filters for metadata fields (e.g., {'module': 'Module 1'})
            
        Returns:
            SearchResult object containing the retrieved content chunks
        """
        start_time = time.time()
        
        # Use provided values or defaults
        top_k = k if k is not None else self.k
        sim_threshold = threshold if threshold is not None else self.threshold
        
        try:
            logger.info(f"Starting retrieval for query: '{query[:50]}{'...' if len(query) > 50 else ''}' "
                       f"with k={top_k}, threshold={sim_threshold}")
            
            # 1. Preprocess query
            processed_query = self.preprocess_query(query)
            
            # 2. Generate query embedding
            query_embedding_obj = self.generate_query_embedding(processed_query)
            
            # 3. Perform similarity search in Qdrant
            retrieved_chunks = self.qdrant_searcher.search_similar(
                query_embedding=query_embedding_obj.vector,
                top_k=top_k,
                threshold=sim_threshold,
                filter_conditions=filter_conditions
            )

            # 3.1 Rank results by similarity score (highest first)
            ranked_chunks = self.rank_results(retrieved_chunks)

            # 4. Validate results
            validation_issues = validate_retrieval_results(ranked_chunks, sim_threshold)
            if validation_issues:
                logger.warning(f"Retrieved results had {len(validation_issues)} validation issues")

            # Check for empty results and handle gracefully
            if not ranked_chunks:
                logger.warning(f"No relevant results found for query with threshold {sim_threshold}")

            # 5. Create search result object
            search_result = SearchResult(
                id=str(uuid.uuid4()),
                query_id=str(uuid.uuid4()),  # Would tie to actual query in full implementation
                query_embedding_id=query_embedding_obj.id,
                chunks=ranked_chunks,
                search_parameters={
                    "k": str(top_k),
                    "threshold": str(sim_threshold),
                    "distance_metric": "cosine"  # Assuming cosine similarity
                },
                execution_time_ms=time.time() - start_time
            )

            logger.info(f"Retrieval completed in {search_result.execution_time_ms:.2f}ms, "
                       f"found {len(ranked_chunks)} results")
            
            return search_result
            
        except Exception as e:
            logger.error(f"Retrieval process failed: {str(e)}")
            # Even if retrieval fails, we should return an appropriate result
            error_result = SearchResult(
                id=str(uuid.uuid4()),
                query_id=str(uuid.uuid4()),
                query_embedding_id="",  # No embedding was generated on failure
                chunks=[],  # No results retrieved
                search_parameters={
                    "k": str(top_k),
                    "threshold": str(sim_threshold),
                    "distance_metric": "cosine"
                },
                execution_time_ms=time.time() - start_time
            )
            
            raise e
    
    def validate_results(self, search_result: SearchResult) -> ValidationResult:
        """
        Validate the search results to confirm correctness
        
        Args:
            search_result: The SearchResult object to validate
            
        Returns:
            ValidationResult object with validation status and issues
        """
        result_id = search_result.id
        issues = []
        
        # Check if results exist
        if len(search_result.chunks) == 0:
            issues.append("No retrieved results found")
        
        # Validate similarity scores and metadata completeness
        chunk_issues = validate_retrieval_results(search_result.chunks, self.threshold)
        issues.extend(chunk_issues)
        
        # Create validation result
        validation_result = ValidationResult(
            id=str(uuid.uuid4()),
            search_result_id=result_id,
            is_valid=(len(issues) == 0),
            validation_issues=issues,
            confidence_score=0.0 if issues else min(1.0, len(search_result.chunks) / self.k),  # Basic confidence score
            validation_rules_applied=[
                "relevance_threshold_check",
                "metadata_completeness",
                "similarity_score_validation"
            ]
        )
        
        logger.info(f"Validation completed for {result_id}, is_valid: {validation_result.is_valid}")
        return validation_result

    def rank_results(self, chunks: List[RetrievedChunk]) -> List[RetrievedChunk]:
        """
        Rank retrieved results by similarity score (highest to lowest) and remove duplicates and diversify.

        Args:
            chunks: List of retrieved chunks to rank

        Returns:
            List of chunks ordered by similarity score (highest first) without duplicates and diversified
        """
        # Remove duplicates based on content (keeping the one with highest similarity score)
        seen_content = set()
        unique_chunks = []

        for chunk in chunks:
            content_key = chunk.content.strip().lower()  # Normalize content for comparison
            if content_key not in seen_content:
                seen_content.add(content_key)
                unique_chunks.append(chunk)

        # Sort unique chunks by similarity score in descending order (highest scores first)
        ranked_chunks = sorted(unique_chunks, key=lambda chunk: chunk.similarity_score, reverse=True)

        # Apply diversity by ensuring different modules/chapters are represented when possible
        diversified_chunks = self._apply_diversity(ranked_chunks)

        # Update the rank field in each chunk to reflect the new order
        for i, chunk in enumerate(diversified_chunks):
            chunk.rank = i

        original_count = len(chunks)
        deduplicated_count = len(diversified_chunks)
        if original_count != deduplicated_count:
            logger.info(f"Deduplicated from {original_count} to {deduplicated_count} chunks")

        logger.info(f"Ranked {len(diversified_chunks)} chunks by similarity score and diversity")
        return diversified_chunks

    def _apply_diversity(self, chunks: List[RetrievedChunk]) -> List[RetrievedChunk]:
        """
        Apply diversity algorithm to ensure varied content in results.

        Args:
            chunks: List of ranked chunks

        Returns:
            List of chunks with diversity applied
        """
        # If we have too few chunks, diversity is not relevant
        if len(chunks) <= 1:
            return chunks

        # For now, we implement a simple diversity by ensuring different URLs/modules appear
        # This can be enhanced with more sophisticated diversity algorithms
        seen_modules = set()
        diversified = []

        # First, try to get one chunk per module if possible
        for chunk in chunks:
            module = chunk.metadata.module or "unknown"
            if module not in seen_modules:
                seen_modules.add(module)
                diversified.append(chunk)

        # If we don't have enough chunks yet, add more with different chapters
        if len(diversified) < len(chunks):
            seen_chapters = {f"{chunk.metadata.module}-{chunk.metadata.chapter}" for chunk in diversified}
            for chunk in chunks:
                if len(diversified) >= len(chunks):
                    break
                chapter_key = f"{chunk.metadata.module}-{chunk.metadata.chapter}"
                if chapter_key not in seen_chapters:
                    seen_chapters.add(chapter_key)
                    diversified.append(chunk)

        # If we still need more chunks to fill out the list, add remaining ones
        for chunk in chunks:
            if len(diversified) >= len(chunks):
                break
            if chunk not in diversified:
                diversified.append(chunk)

        return diversified[:len(chunks)]  # Ensure we don't exceed original count