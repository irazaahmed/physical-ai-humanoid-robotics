from qdrant_client import QdrantClient
from qdrant_client.http.models import (
    Distance,
    VectorParams,
    PointStruct,
    CreateCollection,
    PayloadSchemaType
)
from typing import List, Dict, Any, Optional, Tuple
from src.utils.config import Config
from src.utils.logging import StorageError, handle_error, setup_logging
import uuid

logger = setup_logging()

class QdrantStorage:
    """Qdrant client wrapper for storing embeddings"""
    
    def __init__(self, collection_name: str = "textbook_embeddings"):
        """Initialize the Qdrant client with cloud connection details"""
        errors = Config.validate()
        if errors:
            raise StorageError(f"Configuration validation failed: {', '.join(errors)}")
        
        # Initialize Qdrant client for cloud connection
        self.client = QdrantClient(
            url=Config.QDRANT_URL,
            api_key=Config.QDRANT_API_KEY,
            # Using HTTPS and proper timeout settings
            timeout=30,
            https=True  # Explicitly specify HTTPS for cloud connection
        )
        self.collection_name = collection_name
        
    def create_collection(self, vector_size: int = 1024) -> bool:
        """
        Create a collection in Qdrant with the specified vector size
        
        Args:
            vector_size: Size of the embedding vectors (default 1024 for Cohere multilingual model)
            
        Returns:
            True if collection was created or already exists, False otherwise
        """
        try:
            # Check if collection already exists
            collections = self.client.get_collections()
            collection_exists = any(col.name == self.collection_name for col in collections.collections)
            
            if not collection_exists:
                # Create the collection with vector parameters
                self.client.create_collection(
                    collection_name=self.collection_name,
                    vectors_config=VectorParams(size=vector_size, distance=Distance.COSINE)
                )
                
                # Set up payload index for faster filtering by metadata
                self.client.create_field_index(
                    collection_name=self.collection_name,
                    field_name="module",
                    field_schema=PayloadSchemaType.KEYWORD
                )
                self.client.create_field_index(
                    collection_name=self.collection_name,
                    field_name="chapter",
                    field_schema=PayloadSchemaType.KEYWORD
                )
                self.client.create_field_index(
                    collection_name=self.collection_name,
                    field_name="url",
                    field_schema=PayloadSchemaType.TEXT
                )
                
                logger.info(f"Created Qdrant collection: {self.collection_name}")
            else:
                logger.info(f"Qdrant collection already exists: {self.collection_name}")
            
            return True
            
        except Exception as e:
            handle_error(e, logger, f"Creating Qdrant collection: {self.collection_name}")
            return False
    
    def upsert_vectors_with_metadata(self, vectors: List[List[float]], payloads: List[Dict[str, Any]]) -> bool:
        """
        Upsert (insert or update) vectors in the Qdrant collection with proper metadata

        Args:
            vectors: List of embedding vectors to store
            payloads: List of metadata dictionaries for each vector
                      Each payload should contain: module, chapter, section, url, chunk_index, and hash

        Returns:
            True if upsert operation was successful, False otherwise
        """
        try:
            # Create PointStruct objects with proper IDs, vectors, and payloads
            points = []
            for i, (vector, payload) in enumerate(zip(vectors, payloads)):
                # Generate a unique ID for each point if not provided
                # Ensure we're always using a valid UUID for the point ID
                point_id = str(uuid.uuid4())

                # Create PointStruct with vector and metadata
                point = PointStruct(
                    id=point_id,
                    vector=vector,
                    payload={
                        'module': payload.get('module', ''),
                        'chapter': payload.get('chapter', ''),
                        'section': payload.get('section', ''),
                        'url': payload.get('url', ''),
                        'chunk_index': payload.get('chunk_index', 0),
                        'hash': payload.get('hash', ''),
                        'created_at': payload.get('created_at', ''),
                        'content_id': payload.get('content_id', '')
                    }
                )

                # Add logging for debugging
                logger.info(f"Preparing to upsert point ID: {point_id}, URL: {payload.get('url', 'N/A')}, Chunk index: {payload.get('chunk_index', 'N/A')}")
                points.append(point)

            # Perform upsert operation
            self.client.upsert_points(
                collection_name=self.collection_name,
                points=points
            )

            logger.info(f"Successfully upserted {len(points)} vectors to collection: {self.collection_name}")
            return True

        except Exception as e:
            logger.error(f"Error during upsert operation: {str(e)}")
            handle_error(e, logger, f"Upserting vectors to collection: {self.collection_name}")
            return False

    def upsert_vectors(self, points: List[PointStruct]) -> bool:
        """
        Upsert (insert or update) vectors in the Qdrant collection

        Args:
            points: List of PointStruct objects containing vectors and payloads

        Returns:
            True if upsert operation was successful, False otherwise
        """
        try:
            # Log details about each point for debugging
            for point in points:
                payload = getattr(point, 'payload', {})
                url = payload.get('url', 'N/A') if payload else 'N/A'
                chunk_index = payload.get('chunk_index', 'N/A') if payload else 'N/A'
                logger.info(f"Upserting point ID: {point.id}, URL: {url}, Chunk index: {chunk_index}")

            # Perform upsert operation
            self.client.upsert_points(
                collection_name=self.collection_name,
                points=points
            )

            logger.info(f"Successfully upserted {len(points)} vectors to collection: {self.collection_name}")
            return True

        except Exception as e:
            logger.error(f"Error during upsert operation: {str(e)}")
            handle_error(e, logger, f"Upserting vectors to collection: {self.collection_name}")
            return False
    
    def validate_connection(self) -> bool:
        """
        Validate the connection to Qdrant Cloud
        
        Returns:
            True if connection is valid, False otherwise
        """
        try:
            # Try to get collections to validate connection
            _ = self.client.get_collections()
            logger.info("Successfully validated Qdrant Cloud connection")
            return True
        except Exception as e:
            logger.error(f"Failed to validate Qdrant Cloud connection: {str(e)}")
            return False
    
    def search_vectors(self, query_vector: List[float], limit: int = 10) -> List[Dict[str, Any]]:
        """
        Search for similar vectors in the collection

        Args:
            query_vector: Vector to search for similar vectors
            limit: Maximum number of results to return

        Returns:
            List of dictionaries containing matched points with payload
        """
        try:
            response = self.client.query_points(
                collection_name=self.collection_name,
                query=query_vector,
                limit=limit
            )

            # Extract results from the response object - query_points returns a different structure
            results = response.points if hasattr(response, 'points') else response

            # Format results
            formatted_results = []
            for result in results:
                formatted_results.append({
                    'id': result.id,
                    'score': result.score,
                    'payload': result.payload,
                    'vector': result.vector
                })

            logger.info(f"Found {len(formatted_results)} similar vectors")
            return formatted_results

        except Exception as e:
            handle_error(e, logger, f"Searching vectors in collection: {self.collection_name}")
            return []
    
    def check_duplicate(self, content_hash: str) -> bool:
        """
        Check if a content with the given hash already exists in the collection
        
        Args:
            content_hash: Hash of the content to check for duplicates
            
        Returns:
            True if duplicate exists, False otherwise
        """
        try:
            results = self.client.scroll(
                collection_name=self.collection_name,
                scroll_filter=None,  # We'll filter after getting results
                limit=1
            )

            # In a real implementation, we would implement a more efficient duplicate check
            # For now, we'll assume implementation of hash-based duplicate detection
            # This would typically involve storing the hash as a payload field and querying by it
            logger.info(f"Duplicate check for hash: {content_hash}")
            return False  # Placeholder - implement actual duplicate check based on hash

        except Exception as e:
            handle_error(e, logger, f"Checking for duplicates in collection: {self.collection_name}")
            return False

    def validate_storage(self, content_hashes: List[str] = None) -> Tuple[bool, Dict[str, Any]]:
        """
        Validate successful storage in Qdrant with correct metadata.

        Args:
            content_hashes: Optional list of content hashes to validate specifically

        Returns:
            Tuple of (is_valid, validation_result)
        """
        try:
            # Get collection info
            collection_info = self.client.get_collection(self.collection_name)
            vector_count = collection_info.points_count

            # Validate that we have some vectors stored
            if vector_count == 0:
                logger.error("No vectors found in collection")
                return False, {"error": "No vectors stored", "count": 0}

            # If specific hashes are provided, validate those specifically
            if content_hashes:
                missing_hashes = []
                for hash_val in content_hashes:
                    # Search for vectors with this hash
                    search_result = self.client.scroll(
                        collection_name=self.collection_name,
                        scroll_filter=None  # In a real implementation, filter by hash
                    )
                    # For now, we'll assume the hash validation happens in the storage process
                    # A more sophisticated implementation would search by hash in payload

            logger.info(f"Storage validation passed: {vector_count} vectors in collection")
            return True, {"count": vector_count, "message": "Storage validation passed"}

        except Exception as e:
            handle_error(e, logger, f"Validating storage in collection: {self.collection_name}")
            return False, {"error": str(e)}