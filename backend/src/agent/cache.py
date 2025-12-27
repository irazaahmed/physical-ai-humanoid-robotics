"""Query caching for improved performance and reduced API costs."""
import hashlib
from typing import Optional
from datetime import datetime, timedelta
from .models import AgentResponse


class QueryCache:
    """Simple in-memory cache for query results."""

    def __init__(self, ttl_minutes: int = 60):
        """
        Initialize cache with TTL.

        Args:
            ttl_minutes: Time-to-live in minutes for cached items
        """
        self.cache = {}
        self.ttl = timedelta(minutes=ttl_minutes)

    def _get_key(self, query: str, session_id: str) -> str:
        """Create cache key based on query and session."""
        content = f"{query}:{session_id}".encode()
        return hashlib.md5(content).hexdigest()

    def get(self, query: str, session_id: str) -> Optional[AgentResponse]:
        """
        Get cached response if available and not expired.

        Args:
            query: The query string
            session_id: Session identifier

        Returns:
            Cached AgentResponse or None if not found/expired
        """
        key = self._get_key(query, session_id)
        if key in self.cache:
            cached_item = self.cache[key]
            if datetime.now() < cached_item['expires_at']:
                return cached_item['response']
            else:
                # Remove expired item
                del self.cache[key]
        return None

    def set(self, query: str, session_id: str, response: AgentResponse):
        """
        Cache a response.

        Args:
            query: The query string
            session_id: Session identifier
            response: The AgentResponse to cache
        """
        key = self._get_key(query, session_id)
        expires_at = datetime.now() + self.ttl
        self.cache[key] = {
            'response': response,
            'expires_at': expires_at
        }


# Global cache instance
query_cache = QueryCache()