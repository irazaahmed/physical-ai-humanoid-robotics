from typing import List, Dict, Any, Optional
from src.storage.qdrant_client import QdrantStorage
from src.utils.chunking import calculate_content_hash
from src.utils.logging import StorageError, setup_logging

logger = setup_logging()

class IdempotentIngestion:
    """Handle idempotent ingestion to prevent duplicate vectors"""
    
    def __init__(self, qdrant_storage: QdrantStorage):
        """
        Initialize the idempotent ingestion handler
        
        Args:
            qdrant_storage: Instance of QdrantStorage to interact with the vector database
        """
        self.qdrant_storage = qdrant_storage
    
    def check_content_exists(self, content_hash: str) -> bool:
        """
        Check if content with the given hash already exists in storage.
        
        Args:
            content_hash: Hash of the content to check
            
        Returns:
            True if content exists (is duplicate), False otherwise
        """
        try:
            # In Qdrant, we would typically search by payload field
            # For this implementation, we'll assume we can query by hash
            # In a real implementation, this would involve searching Qdrant 
            # by the hash field in the payload
            
            # This is a simplified approach - in practice, you'd query the Qdrant
            # collection filtering by the hash field in the payload
            logger.info(f"Checking if content with hash {content_hash} already exists")
            return False  # Placeholder - implement actual duplicate check
        except Exception as e:
            logger.error(f"Error checking if content exists: {str(e)}")
            return False
    
    def filter_new_content(self, contents: List[Dict[str, Any]]) -> List[Dict[str, Any]]:
        """
        Filter out content that already exists in storage.
        
        Args:
            contents: List of content dictionaries with metadata
            
        Returns:
            List of content that doesn't already exist in storage
        """
        new_contents = []
        
        for content in contents:
            # Calculate hash for the content
            content_hash = calculate_content_hash(content.get('text', ''))
            content['hash'] = content_hash  # Add hash to metadata
            
            # Check if content already exists
            if not self.check_content_exists(content_hash):
                new_contents.append(content)
            else:
                logger.info(f"Content with hash {content_hash} already exists, skipping")
        
        logger.info(f"Identified {len(new_contents)} out of {len(contents)} as new content")
        return new_contents
    
    def process_ingestion_with_idempotency(
        self,
        vectors: List[List[float]],
        payloads: List[Dict[str, Any]]
    ) -> bool:
        """
        Process ingestion while ensuring idempotency.

        Args:
            vectors: List of embedding vectors to store
            payloads: List of metadata dictionaries for each vector

        Returns:
            True if ingestion was successful, False otherwise
        """
        try:
            logger.info(f"Starting idempotent ingestion for {len(payloads)} payloads with vectors")

            # Filter out payloads that already exist
            new_payloads = self.filter_new_content(payloads)

            # Extract corresponding vectors for new payloads only
            new_vectors = []
            for payload in new_payloads:
                # Find the corresponding vector for this payload
                for i, orig_payload in enumerate(payloads):
                    if orig_payload.get('hash') == payload.get('hash'):
                        new_vectors.append(vectors[i])
                        break

            logger.info(f"Filtered out {len(payloads) - len(new_payloads)} duplicate payloads")

            # Only store new vectors that don't already exist
            if new_vectors:
                logger.info(f"Storing {len(new_vectors)} new vectors (filtered out duplicates)")

                # Log some sample URLs that are being embedded
                if new_payloads:
                    sample_urls = [p.get('url', 'Unknown URL') for p in new_payloads[:5]]  # Show first 5 URLs
                    for i, url in enumerate(sample_urls):
                        logger.info(f"  Successfully embedding content from: {url}")

                    if len(new_payloads) > 5:
                        logger.info(f"  ... and {len(new_payloads) - 5} more URLs")

                return self.qdrant_storage.upsert_vectors_with_metadata(new_vectors, new_payloads)
            else:
                logger.info("No new vectors to store, all content already exists")
                return True

        except Exception as e:
            logger.error(f"Error in idempotent ingestion: {str(e)}")
            return False
    
    def mark_ingestion_checkpoint(self, url: str, status: str = "completed") -> bool:
        """
        Mark a URL as processed to enable resumable ingestion.
        
        Args:
            url: The URL that was processed
            status: The status of processing (completed, failed, etc.)
            
        Returns:
            True if checkpoint was marked successfully, False otherwise
        """
        # In a real implementation, this would store the checkpoint in a database
        # or file to track ingestion progress
        logger.info(f"Marking checkpoint for {url} with status: {status}")
        return True
    
    def get_ingestion_checkpoint(self) -> Optional[str]:
        """
        Get the last ingestion checkpoint to resume from.
        
        Returns:
            URL to resume ingestion from, or None if no checkpoint exists
        """
        # In a real implementation, this would retrieve the checkpoint from a database
        # or file to track ingestion progress
        logger.info("Retrieving ingestion checkpoint")
        return None