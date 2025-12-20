import cohere
from typing import List, Optional
from .config import Config
from .logging import EmbeddingGenerationError, handle_error, setup_logging

logger = setup_logging()

class CohereClient:
    """Cohere client wrapper for generating query embeddings"""
    
    def __init__(self):
        """Initialize the Cohere client with API key from config"""
        errors = Config.validate()
        if errors:
            raise EmbeddingGenerationError(f"Configuration validation failed: {', '.join(errors)}")
        
        self.client = cohere.Client(Config.COHERE_API_KEY)
        self.model = Config.COHERE_MODEL
    
    def generate_embeddings(self, texts: List[str], max_retries: int = 3) -> List[List[float]]:
        """
        Generate embeddings for a list of query texts using Cohere

        Args:
            texts: List of query texts to embed
            max_retries: Maximum number of retry attempts for API failures

        Returns:
            List of embedding vectors (each vector is a list of floats)

        Raises:
            EmbeddingGenerationError: If the embedding generation fails
        """
        import time
        import random

        last_exception = None

        for attempt in range(max_retries):
            try:
                response = self.client.embed(
                    texts=texts,
                    model=self.model,
                    input_type="search_query"  # Optimize for search queries
                )

                # Validate that we got embeddings back
                if not response or not response.embeddings:
                    raise EmbeddingGenerationError("No embeddings returned from Cohere API")

                embeddings = response.embeddings

                # Ensure the number of embeddings matches the number of input texts
                if len(embeddings) != len(texts):
                    raise EmbeddingGenerationError(
                        f"Mismatch: Expected {len(texts)} embeddings, got {len(embeddings)}"
                    )

                # Validate that all embeddings have the correct dimensionality
                expected_dim = len(embeddings[0]) if embeddings else 0
                for i, embedding in enumerate(embeddings):
                    if len(embedding) != expected_dim:
                        raise EmbeddingGenerationError(
                            f"Embedding {i} has different dimensionality: {len(embedding)} vs {expected_dim}"
                        )

                logger.info(f"Successfully generated embeddings for {len(texts)} query texts (dimension: {expected_dim})")
                return embeddings

            except Exception as e:
                last_exception = e
                logger.warning(f"Attempt {attempt + 1} failed to generate embeddings: {str(e)}")

                # Perform exponential backoff with jitter for the next attempt
                if attempt < max_retries - 1:
                    sleep_time = (2 ** attempt) + random.uniform(0, 1)  # Exponential backoff with jitter
                    logger.info(f"Waiting {sleep_time:.2f}s before retry...")
                    time.sleep(sleep_time)

        # If all retries exhausted, log the final error
        if last_exception:
            handle_error(last_exception, logger, "Cohere query embedding generation")

        raise EmbeddingGenerationError("Failed to generate embeddings after all retry attempts")

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