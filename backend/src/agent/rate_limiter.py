"""
Rate limiting configuration for the agent service.

This module implements rate limiting using the slowapi library
to enforce the requirement of 10 requests per minute per IP (FR-016).
"""
from slowapi import Limiter, _rate_limit_exceeded_handler
from slowapi.util import get_remote_address
from slowapi.errors import RateLimitExceeded
from fastapi import FastAPI, Request
from .config import Config


# Initialize the limiter with the default rate from config
limiter = Limiter(
    key_func=get_remote_address,  # Rate limit by IP address
    default_limits=[Config.RATE_LIMIT]  # Use the rate from config (e.g., "10/minute")
)


def setup_rate_limiter(app: FastAPI) -> None:
    """
    Set up rate limiting for the FastAPI application.
    
    Args:
        app: The FastAPI application instance to add rate limiting to
    """
    # Attach the limiter to the app
    app.state.limiter = limiter
    app.add_exception_handler(RateLimitExceeded, _rate_limit_exceeded_handler)


def get_rate_limit() -> str:
    """
    Get the current rate limit value.
    
    Returns:
        The rate limit string (e.g., "10/minute")
    """
    return Config.RATE_LIMIT


def apply_rate_limit():
    """
    Decorator to apply rate limiting to a route.
    
    Usage:
        @app.post("/api/v1/query")
        @apply_rate_limit()
        async def query_endpoint(request: Request):
            # Your endpoint logic here
            pass
    """
    # Use the default rate limit from config
    return limiter.limit(Config.RATE_LIMIT)