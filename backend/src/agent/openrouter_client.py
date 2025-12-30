"""
OpenRouter Client for the agent service.

This module provides a client for interacting with OpenRouter's API,
providing LLM capabilities for the agent service.
"""
import os
import requests
import json
from typing import Dict, Any, List, Optional
from .config import Config
from .logging import logger


class OpenRouterClient:
    """
    Client for interacting with OpenRouter API.
    """

    def __init__(self):
        """
        Initialize the OpenRouter client with API key and model configuration.
        """
        # Validate configuration
        if not Config.OPENROUTER_API_KEY:
            raise ValueError("OPENROUTER_API_KEY environment variable is required")
        if not Config.OPENROUTER_MODEL:
            raise ValueError("OPENROUTER_MODEL environment variable is required")

        self.api_key = Config.OPENROUTER_API_KEY
        self.model = Config.OPENROUTER_MODEL
        self.base_url = "https://openrouter.ai/api/v1"

        logger.info(f"OpenRouter client initialized with model: {self.model}")

    def generate_content(
        self,
        prompt: str,
        max_tokens: int = 1000,
        temperature: float = 0.3
    ) -> str:
        """
        Generate content using OpenRouter API.

        Args:
            prompt: The input prompt for content generation
            max_tokens: Maximum number of tokens to generate
            temperature: Sampling temperature for generation

        Returns:
            Generated content string
        """
        headers = {
            "Authorization": f"Bearer {self.api_key}",
            "Content-Type": "application/json"
        }

        data = {
            "model": self.model,
            "messages": [
                {"role": "user", "content": prompt}
            ],
            "max_tokens": max_tokens,
            "temperature": temperature
        }

        try:
            response = requests.post(
                f"{self.base_url}/chat/completions",
                headers=headers,
                json=data,
                timeout=30  # 30 second timeout
            )

            # Check for specific HTTP error codes
            if response.status_code == 400:
                error_details = response.text
                logger.error(f"OpenRouter API returned 400 Bad Request: {error_details}")
                raise Exception(f"OpenRouter API request failed with 400: {error_details}")
            elif response.status_code == 401:
                logger.error("OpenRouter API returned 401 Unauthorized - check your API key")
                raise Exception("OpenRouter API authentication failed - check your API key")
            elif response.status_code == 429:
                logger.error("OpenRouter API returned 429 Rate Limited")
                raise Exception("OpenRouter API rate limit exceeded")
            elif response.status_code >= 400:
                error_details = response.text
                logger.error(f"OpenRouter API returned error {response.status_code}: {error_details}")
                raise Exception(f"OpenRouter API request failed with status {response.status_code}: {error_details}")

            response.raise_for_status()  # This will raise an exception for other HTTP errors

            result = response.json()

            # Extract the content from the response
            if 'choices' in result and len(result['choices']) > 0:
                content = result['choices'][0]['message']['content']
                if content is None or content.strip() == "":
                    raise Exception("OpenRouter returned empty response content")
                return content
            else:
                raise Exception(f"Unexpected response format from OpenRouter: {result}")

        except requests.exceptions.RequestException as e:
            logger.error(f"Error calling OpenRouter API: {str(e)}")
            if hasattr(e, 'response') and e.response is not None:
                error_details = e.response.text
                logger.error(f"OpenRouter API error details: {error_details}")
                status_code = e.response.status_code
                raise Exception(f"OpenRouter API request failed with status {status_code}: {error_details}")
            else:
                raise Exception(f"OpenRouter API request failed: {str(e)}")
        except Exception as e:
            logger.error(f"Error processing OpenRouter response: {str(e)}")
            raise