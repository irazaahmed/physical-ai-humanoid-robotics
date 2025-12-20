from qdrant_client import QdrantClient
from qdrant_client.http.models import (
    Distance,
    VectorParams,
    PointStruct,
    QueryResponse,
    SearchRequest,
    Filter,
    FieldCondition,
    MatchValue
)
from typing import List, Dict, Any, Optional, Tuple
from .config import Config
from .logging import SearchError, handle_error, setup_logging
from .models import RetrievedChunk, Metadata

logger = setup_logging()

class QdrantSearcher:
    """Qdrant client wrapper for similarity search operations"""
    
    def __init__(self):
        """Initialize the Qdrant client with cloud connection details"""
        errors = Config.validate()
        if errors:
            raise SearchError(f"Configuration validation failed: {', '.join(errors)}")
        
        # Initialize Qdrant client for cloud connection
        self.client = QdrantClient(
            url=Config.QDRANT_URL,
            api_key=Config.QDRANT_API_KEY,
            timeout=30,  # 30 seconds
            https=True  # Explicitly specify HTTPS for cloud connection
        )
        self.collection_name = Config.COLLECTION_NAME
    
    def validate_connection(self) -> bool:
        """
        Validate the connection to Qdrant Cloud
        
        Returns:
            True if connection is valid, False otherwise
        """
        try:
            # Try to get collection info to validate connection
            _ = self.client.get_collection(self.collection_name)
            logger.info("Successfully validated Qdrant Cloud connection")
            return True
        except Exception as e:
            logger.error(f"Failed to validate Qdrant Cloud connection: {str(e)}")
            return False
    
    def search_similar(
        self,
        query_embedding: List[float],
        top_k: int = 5,
        threshold: float = 0.6,
        filter_conditions: Optional[Dict[str, Any]] = None
    ) -> List[RetrievedChunk]:
        """
        Perform similarity search in Qdrant to find relevant content

        Args:
            query_embedding: The embedding vector to search for similar vectors
            top_k: Number of top results to return (configurable k parameter)
            threshold: Minimum similarity score threshold for results
            filter_conditions: Optional filters for metadata fields (e.g., {'module': 'Module 1'})

        Returns:
            List of RetrievedChunk objects with similarity scores and metadata
        """
        try:
            logger.info(f"Starting similarity search with k={top_k}, threshold={threshold}")

            # Create filters if needed
            qdrant_filters = None
            if filter_conditions:
                conditions = []
                for field, value in filter_conditions.items():
                    conditions.append(FieldCondition(key=field, match=MatchValue(value=value)))

                if conditions:
                    from qdrant_client.http.models import Filter
                    qdrant_filters = Filter(must=conditions)

            # Perform search in Qdrant with configurable parameters using query_points
            response = self.client.query_points(
                collection_name=self.collection_name,
                query=query_embedding,
                query_filter=qdrant_filters,
                limit=top_k,  # Top-k retrieval with configurable k
                score_threshold=threshold,  # Relevance threshold filtering
                with_payload=True,  # Ensure we get payload data
                with_vectors=False  # Don't need the vectors themselves
            )

            # Extract results from the response object - query_points returns a different structure
            results = response.points if hasattr(response, 'points') else response

            # Filter results based on threshold and create RetrievedChunk objects
            retrieved_chunks = []
            for i, result in enumerate(results):
                try:
                    # Double-check that result meets our threshold, just in case
                    if result.score < threshold:
                        continue  # Skip if below our threshold

                    # Extract metadata with fallback values
                    payload = result.payload or {}  # Ensure not None

                    # Create Metadata object with all required fields, with empty strings as fallbacks
                    metadata = Metadata(
                        url=payload.get('url', ''),
                        module=payload.get('module', ''),
                        chapter=payload.get('chapter', ''),
                        section=payload.get('section', ''),
                        chunk_index=payload.get('chunk_index', 0),
                        hash=payload.get('hash', '')
                    )

                    # Create RetrievedChunk object
                    chunk = RetrievedChunk(
                        id=result.id,
                        query_embedding_id="",  # Will be set by calling function
                        content=payload.get('content', '') or payload.get('text', '') or "",  # Check multiple field names
                        similarity_score=result.score,
                        rank=i,  # Sequential ranking
                        metadata=metadata
                    )
                    retrieved_chunks.append(chunk)
                except AttributeError as e:
                    logger.warning(f"Skipping result due to missing attributes: {str(e)}")
                    continue  # Skip this result and continue with the next
                except Exception as e:
                    logger.warning(f"Error processing result {i}: {str(e)}")
                    continue  # Skip this result and continue with the next

            logger.info(f"Search completed successfully, returned {len(retrieved_chunks)} chunks "
                       f"from collection: {self.collection_name}")
            return retrieved_chunks

        except Exception as e:
            logger.error(f"Similarity search failed: {str(e)}")
            handle_error(e, logger, f"Performing similarity search in collection: {self.collection_name}")
            return []
    
    def get_vector_dimension(self) -> Optional[int]:
        """
        Get the vector dimension of the collection.
        
        Returns:
            Dimension of the vectors in the collection, or None if error
        """
        try:
            collection_info = self.client.get_collection(self.collection_name)
            # Assuming the first vector configuration to get the size
            if hasattr(collection_info.config.params, 'vectors'):
                vector_params = collection_info.config.params.vectors
                if isinstance(vector_params, dict):
                    # Multiple vector configurations possible, get first one
                    vector_name = next(iter(vector_params))
                    return vector_params[vector_name].size if vector_params[vector_name] else None
                else:
                    # Single vector configuration
                    return vector_params.size if vector_params else None
            else:
                logger.error(f"Could not determine vector dimension for collection: {self.collection_name}")
                return None
        except Exception as e:
            logger.error(f"Error getting vector dimension: {str(e)}")
            return None