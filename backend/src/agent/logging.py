"""
Logging module for the agent service.

This module provides consistent logging functionality for the agent,
including structured logging for requests, responses, and errors.
It also includes basic metrics collection and monitoring capabilities.
"""
import logging
import sys
from typing import Any
from .config import Config
from datetime import datetime
import time
import threading


# Global metrics storage
_metrics_lock = threading.Lock()
_metrics = {
    "requests_processed": 0,
    "requests_failed": 0,
    "total_processing_time": 0.0,  # in milliseconds
    "avg_processing_time": 0.0,   # in milliseconds
    "active_sessions": set(),
}


def setup_logging() -> logging.Logger:
    """
    Set up logging configuration for the agent service.

    Returns:
        logging.Logger: Configured logger instance
    """
    # Create logger
    logger = logging.getLogger("agent")
    logger.setLevel(logging.INFO)

    # Prevent adding multiple handlers if logger already exists
    if logger.handlers:
        return logger

    # Create console handler
    console_handler = logging.StreamHandler(sys.stdout)
    console_handler.setLevel(logging.INFO)

    # Create formatter
    formatter = logging.Formatter(
        '%(asctime)s - %(name)s - %(levelname)s - %(message)s'
    )
    console_handler.setFormatter(formatter)

    # Add handler to logger
    logger.addHandler(console_handler)

    return logger


# Global logger instance
logger = setup_logging()


class AgentError(Exception):
    """Base exception class for agent-related errors."""
    pass


class ToolExecutionError(AgentError):
    """Exception raised when a tool execution fails."""
    pass


class ConfigurationError(AgentError):
    """Exception raised when there's a configuration issue."""
    pass


class QueryProcessingError(AgentError):
    """Exception raised when query processing fails."""
    pass


def get_metrics() -> dict:
    """
    Get current metrics for monitoring.

    Returns:
        Dictionary containing current metrics
    """
    with _metrics_lock:
        # Create a copy to avoid race conditions
        metrics_copy = _metrics.copy()
        # Calculate additional derived metrics
        if metrics_copy["requests_processed"] > 0:
            metrics_copy["avg_processing_time"] = (
                metrics_copy["total_processing_time"] / metrics_copy["requests_processed"]
            )
        else:
            metrics_copy["avg_processing_time"] = 0.0

        # Add success rate
        total_requests = metrics_copy["requests_processed"] + metrics_copy["requests_failed"]
        if total_requests > 0:
            success_rate = (metrics_copy["requests_processed"] / total_requests) * 100
            metrics_copy["success_rate"] = f"{success_rate:.2f}%"
        else:
            metrics_copy["success_rate"] = "0.00%"

        return metrics_copy


def reset_metrics() -> None:
    """Reset all metrics to initial values."""
    with _metrics_lock:
        _metrics["requests_processed"] = 0
        _metrics["requests_failed"] = 0
        _metrics["total_processing_time"] = 0.0
        _metrics["active_sessions"] = set()


def log_request(query: str, session_id: str = None) -> None:
    """
    Log an incoming request and update metrics.

    Args:
        query: The user query string
        session_id: Optional session identifier
    """
    logger.info(f"Processing request - Query: {query[:100]}{'...' if len(query) > 100 else ''}, Session: {session_id}")

    # Update metrics
    with _metrics_lock:
        _metrics["requests_processed"] += 1
        if session_id:
            _metrics["active_sessions"].add(session_id)


def log_response(response: str, processing_time: float, sources: list = None) -> None:
    """
    Log a response from the agent and update metrics.

    Args:
        response: The response string
        processing_time: Time taken to process the request in milliseconds
        sources: List of sources used in the response
    """
    source_count = len(sources) if sources else 0
    logger.info(f"Generated response - Time: {processing_time}ms, Sources: {source_count}, Response length: {len(response)}")

    # Update metrics
    with _metrics_lock:
        _metrics["total_processing_time"] += processing_time
        _metrics["avg_processing_time"] = (
            _metrics["total_processing_time"] / _metrics["requests_processed"]
        )


def log_error(error: Exception, context: str = "") -> None:
    """
    Log an error with context and update failure metrics.

    Args:
        error: The exception that occurred
        context: Additional context about where the error occurred
    """
    logger.error(f"Error in {context}: {str(error)}", exc_info=True)

    # Update metrics
    with _metrics_lock:
        _metrics["requests_failed"] += 1


def handle_error(error: Exception, context: str = "") -> Any:
    """
    Handle an error by logging it and returning an appropriate response.

    Args:
        error: The exception that occurred
        context: Additional context about where the error occurred

    Returns:
        Any: An appropriate error response
    """
    log_error(error, context)

    # Return an error response appropriate to the context
    return {
        "error": "An error occurred while processing your request",
        "message": str(error) if Config.GEMINI_API_KEY else "Configuration error",
        "timestamp": datetime.utcnow().isoformat()
    }