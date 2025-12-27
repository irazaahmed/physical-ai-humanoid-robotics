import os
from dotenv import load_dotenv
from typing import Optional

# Load environment variables from .env file
load_dotenv()

class Config:
    """Configuration class to manage environment variables and settings"""

    # Google Gemini configuration
    GEMINI_API_KEY: str = os.getenv("GEMINI_API_KEY", "")
    GEMINI_MODEL: str = os.getenv("GEMINI_MODEL", "gemini-2.5-flash")

    # Cohere configuration
    COHERE_API_KEY: str = os.getenv("COHERE_API_KEY", "")

    # Qdrant configuration
    QDRANT_API_KEY: str = os.getenv("QDRANT_API_KEY", "")
    QDRANT_URL: str = os.getenv("QDRANT_URL", "")

    # Textbook configuration
    TEXTBOOK_BASE_URL: str = os.getenv("TEXTBOOK_BASE_URL", "")

    # Application configuration
    PORT: int = int(os.getenv("PORT", 8001))
    START_AGENT_SERVICE: bool = os.getenv("START_AGENT_SERVICE", "false").lower() == "true"
    DEFAULT_TOP_K: int = int(os.getenv("DEFAULT_TOP_K", "5"))
    DEFAULT_THRESHOLD: float = float(os.getenv("DEFAULT_THRESHOLD", "0.5"))

    # Embedding configuration (from constitution: 512 tokens with 64-token overlap)
    CHUNK_SIZE: int = 512  # tokens
    OVERLAP_SIZE: int = 64  # tokens
    
    # Validation
    @classmethod
    def validate(cls) -> list[str]:
        """Validate that all required environment variables are set"""
        errors = []
        if not cls.GEMINI_API_KEY:
            errors.append("GEMINI_API_KEY is not set")
        if not cls.COHERE_API_KEY:
            errors.append("COHERE_API_KEY is not set")
        if not cls.QDRANT_API_KEY:
            errors.append("QDRANT_API_KEY is not set")
        if not cls.QDRANT_URL:
            errors.append("QDRANT_URL is not set")
        if not cls.TEXTBOOK_BASE_URL:
            errors.append("TEXTBOOK_BASE_URL is not set")
        return errors