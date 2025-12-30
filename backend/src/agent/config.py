"""
Configuration module for the agent service.

This module handles loading and validating configuration settings for the agent,
including API keys and service parameters.
"""
import os
from typing import Optional
from dotenv import load_dotenv

# Load environment variables from .env file
load_dotenv()


class Config:
    """
    Configuration class for the agent service.
    """

    # OpenRouter Configuration
    OPENROUTER_API_KEY: Optional[str] = os.getenv("OPENROUTER_API_KEY")
    OPENROUTER_MODEL: str = os.getenv("OPENROUTER_MODEL", "openai/gpt-3.5-turbo")  # Using a default model

    # Rate Limiting Configuration
    RATE_LIMIT: str = os.getenv("RATE_LIMIT", "10/minute")  # 10 requests per minute per IP

    # Agent Configuration
    MAX_TOOL_CALLS_PER_QUERY: int = int(os.getenv("MAX_TOOL_CALLS_PER_QUERY", "5"))
    RESPONSE_TIMEOUT_SECONDS: int = int(os.getenv("RESPONSE_TIMEOUT_SECONDS", "30"))

    # Retrieval Configuration
    DEFAULT_TOP_K: int = int(os.getenv("DEFAULT_TOP_K", "5"))
    DEFAULT_THRESHOLD: float = float(os.getenv("DEFAULT_THRESHOLD", "0.6"))

    # Validation
    @classmethod
    def validate(cls) -> None:
        """
        Validate that all required configuration values are present.

        Raises:
            ValueError: If any required configuration is missing
        """
        if not cls.OPENROUTER_API_KEY:
            raise ValueError("OPENROUTER_API_KEY environment variable is required")