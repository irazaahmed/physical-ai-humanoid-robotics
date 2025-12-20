import os
from dotenv import load_dotenv
from typing import Optional

# Load environment variables from .env file
load_dotenv()

class Config:
    """Configuration class to manage environment variables and settings for retrieval pipeline"""
    
    # Cohere configuration
    COHERE_API_KEY: str = os.getenv("COHERE_API_KEY", "")
    
    # Qdrant configuration
    QDRANT_API_KEY: str = os.getenv("QDRANT_API_KEY", "")
    QDRANT_URL: str = os.getenv("QDRANT_URL", "")
    
    # Cohere model settings
    COHERE_MODEL: str = os.getenv("COHERE_MODEL", "embed-multilingual-v3.0")
    
    # Qdrant collection settings
    COLLECTION_NAME: str = os.getenv("QDRANT_COLLECTION_NAME", "textbook_embeddings")
    
    # Retrieval settings
    DEFAULT_TOP_K: int = int(os.getenv("DEFAULT_TOP_K", "5"))
    DEFAULT_THRESHOLD: float = float(os.getenv("DEFAULT_THRESHOLD", "0.6"))
    
    # Validation
    @classmethod
    def validate(cls) -> list[str]:
        """Validate that all required environment variables are set"""
        errors = []
        if not cls.COHERE_API_KEY:
            errors.append("COHERE_API_KEY is not set")
        if not cls.QDRANT_API_KEY:
            errors.append("QDRANT_API_KEY is not set")
        if not cls.QDRANT_URL:
            errors.append("QDRANT_URL is not set")
        return errors