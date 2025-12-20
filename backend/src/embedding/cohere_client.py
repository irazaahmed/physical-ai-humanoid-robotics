import cohere
from typing import List, Optional
from src.utils.config import Config
from src.utils.logging import EmbeddingError, handle_error, setup_logging

logger = setup_logging()

class CohereClient:
    """Cohere client wrapper for generating embeddings"""
    
    def __init__(self):
        """Initialize the Cohere client with API key from config"""
        errors = Config.validate()
        if errors:
            raise EmbeddingError(f"Configuration validation failed: {', '.join(errors)}")
        
        self.client = cohere.Client(Config.COHERE_API_KEY)
        self.model = "embed-multilingual-v3.0"  # Using multilingual model for broader text support
        
    def generate_embeddings(self, texts: List[str], max_retries: int = 3) -> List[List[float]]:
        """
        Generate embeddings for a list of texts using Cohere
        
        Args:
            texts: List of text strings to embed
            max_retries: Maximum number of retry attempts for API failures
            
        Returns:
            List of embedding vectors (each vector is a list of floats)
            
        Raises:
            EmbeddingError: If the embedding generation fails
        """
        for attempt in range(max_retries):
            try:
                response = self.client.embed(
                    texts=texts,
                    model=self.model,
                    input_type="search_document"  # Optimize for search/retrieval use case
                )
                
                # Validate that we got embeddings back
                if not response or not response.embeddings:
                    raise EmbeddingError("No embeddings returned from Cohere API")
                
                embeddings = response.embeddings
                
                # Ensure the number of embeddings matches the number of input texts
                if len(embeddings) != len(texts):
                    raise EmbeddingError(
                        f"Mismatch: Expected {len(texts)} embeddings, got {len(embeddings)}"
                    )
                
                logger.info(f"Successfully generated embeddings for {len(texts)} text chunks")
                return embeddings
                
            except Exception as e:
                logger.warning(f"Attempt {attempt + 1} failed to generate embeddings: {str(e)}")
                if attempt == max_retries - 1:  # Last attempt
                    handle_error(e, logger, "Cohere embedding generation")
        
        # This should never be reached due to the raise in the loop, but added for type safety
        raise EmbeddingError("Failed to generate embeddings after all retry attempts")
    
    def validate_api_key(self) -> bool:
        """
        Validate the Cohere API key by making a simple request
        
        Returns:
            True if API key is valid, False otherwise
        """
        try:
            # Try to embed a simple test text
            test_embedding = self.generate_embeddings(["test"])
            return len(test_embedding) > 0
        except Exception as e:
            logger.error(f"Failed to validate Cohere API key: {str(e)}")
            return False