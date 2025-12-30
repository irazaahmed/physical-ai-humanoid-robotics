"""
Validation utilities for the agent service.

This module provides validation functions for query and response data.
"""
from typing import Any, Dict, List
from .models import UserQuery, AgentResponse, RetrievedChunk, QueryRequest, ChatRequest
from .logging import logger, QueryProcessingError
from .config import Config


def validate_query_text(query: str) -> bool:
    """
    Validate the query text based on length requirements.
    
    Args:
        query: The query text to validate
        
    Returns:
        bool: True if valid, False otherwise
    """
    if not query:
        return False
    
    # Query text must be between 3 and 10000 characters
    if len(query) < 3 or len(query) > 10000:
        return False
    
    return True


def validate_top_k(top_k: int) -> bool:
    """
    Validate the top_k parameter.
    
    Args:
        top_k: The number of results to retrieve
        
    Returns:
        bool: True if valid, False otherwise
    """
    if top_k < 1 or top_k > 50:
        return False
    
    return True


def validate_threshold(threshold: float) -> bool:
    """
    Validate the threshold parameter.
    
    Args:
        threshold: The similarity threshold
        
    Returns:
        bool: True if valid, False otherwise
    """
    if threshold < 0.0 or threshold > 1.0:
        return False
    
    return True


def validate_session_id(session_id: str) -> bool:
    """
    Validate the session ID format.

    Args:
        session_id: The session identifier to validate

    Returns:
        bool: True if valid, False otherwise
    """
    if not session_id:
        return True  # Session ID is optional

    import re

    # Check for UUID format (for manually provided session IDs)
    uuid_pattern = re.compile(r'^[a-f0-9]{8}-[a-f0-9]{4}-[a-f0-9]{4}-[a-f0-9]{4}-[a-f0-9]{12}$', re.I)
    if uuid_pattern.match(session_id):
        return True

    # Check for "session_" prefix format (for auto-generated session IDs)
    session_pattern = re.compile(r'^session_[a-f0-9]{8}-[a-f0-9]{4}-[a-f0-9]{4}-[a-f0-9]{4}-[a-f0-9]{12}$', re.I)
    if session_pattern.match(session_id):
        return True

    # Check for reasonable alphanumeric session IDs (like "browser-test", "user123", etc.)
    # Allow letters, numbers, hyphens, and underscores, with reasonable length
    alphanumeric_pattern = re.compile(r'^[a-zA-Z0-9_-]{1,64}$')
    return bool(alphanumeric_pattern.match(session_id))


def validate_retrieved_chunks(chunks: List[RetrievedChunk]) -> bool:
    """
    Validate retrieved chunks based on the data model requirements.
    
    Args:
        chunks: List of retrieved chunks to validate
        
    Returns:
        bool: True if valid, False otherwise
    """
    for chunk in chunks:
        # Validate similarity score
        if chunk.similarity_score < 0.0 or chunk.similarity_score > 1.0:
            return False
        
        # Validate rank
        if chunk.rank < 0:
            return False
        
        # Validate metadata requirements
        required_metadata = ['url', 'module', 'chapter', 'section', 'chunk_index', 'hash']
        for key in required_metadata:
            if key not in chunk.metadata:
                return False
    
    return True


def validate_source_content(response: str, sources: List[Dict[str, Any]]) -> bool:
    """
    Validate that the response content is properly attributed to sources.
    
    Args:
        response: The response content
        sources: List of sources used in the response
        
    Returns:
        bool: True if valid, False otherwise
    """
    # This is a basic check - for a real implementation, we'd want more sophisticated validation
    # For now, we'll just ensure sources have required fields
    for source in sources:
        required_fields = ['url', 'module', 'chapter', 'section']
        for field in required_fields:
            if field not in source:
                return False
    
    return True


def validate_query_request(request: QueryRequest) -> Dict[str, Any]:
    """
    Validate a query request and return any errors.
    
    Args:
        request: The query request to validate
        
    Returns:
        Dict with validation results - empty if valid
    """
    errors = {}
    
    if not validate_query_text(request.query):
        errors['query'] = "Query must be between 3 and 10000 characters"
    
    if not validate_top_k(request.parameters.top_k):
        errors['top_k'] = "Top-k parameter must be between 1 and 50"
    
    if not validate_threshold(request.parameters.threshold):
        errors['threshold'] = "Threshold parameter must be between 0.0 and 1.0"
    
    return errors


def validate_chat_request(request: ChatRequest) -> Dict[str, Any]:
    """
    Validate a chat request and return any errors.

    Args:
        request: The chat request to validate

    Returns:
        Dict with validation results - empty if valid
    """
    errors = {}

    if not validate_query_text(request.query):
        errors['query'] = "Query must be between 3 and 10000 characters"

    if not validate_session_id(request.session_id):
        errors['session_id'] = "Session ID must be a valid UUID format"

    if not validate_top_k(request.parameters.top_k):
        errors['top_k'] = "Top-k parameter must be between 1 and 50"

    if not validate_threshold(request.parameters.threshold):
        errors['threshold'] = "Threshold parameter must be between 0.0 and 1.0"

    return errors


def validate_confidence_score(score: float) -> bool:
    """
    Validate the confidence score.
    
    Args:
        score: The confidence score to validate
        
    Returns:
        bool: True if valid, False otherwise
    """
    if score is None:
        return True  # Score is optional in some contexts
    
    return 0.0 <= score <= 1.0


def is_safe_content(content: str) -> bool:
    """
    Basic check for potentially harmful content to prevent prompt injection
    and other security issues.

    Args:
        content: The content to check

    Returns:
        bool: True if content appears safe, False otherwise
    """
    if not content:
        return True

    # Convert to lowercase for comparison
    content_lower = content.lower()

    # Check for common harmful patterns
    harmful_patterns = [
        "system prompt",
        "ignore previous instructions",
        "you are now",
        "act as",
        "jailbreak",
        "root access",
        "sudo",
        "password",
        "key",
        "token",
        "retrieve the hidden",
        "bypass security",
        "disable safety",
        "repeat after me"
    ]

    for pattern in harmful_patterns:
        if pattern in content_lower:
            logger.warning(f"Potentially harmful content detected: {pattern}")
            return False

    return True


def validate_tool_call_count(tool_calls: List[Any], max_calls: int = Config.MAX_TOOL_CALLS_PER_QUERY) -> bool:
    """
    Validate that the number of tool calls does not exceed the limit.
    
    Args:
        tool_calls: List of tool calls made
        max_calls: Maximum allowed calls (defaults to config)
        
    Returns:
        bool: True if within limit, False otherwise
    """
    return len(tool_calls) <= max_calls


def validate_agent_response(response: AgentResponse) -> Dict[str, Any]:
    """
    Validate an agent response and return any errors.
    
    Args:
        response: The agent response to validate
        
    Returns:
        Dict with validation results - empty if valid
    """
    errors = {}
    
    if not validate_confidence_score(response.confidence_score):
        errors['confidence_score'] = "Confidence score must be between 0.0 and 1.0"
    
    if not validate_source_content(response.content, [s.dict() for s in response.sources]):
        errors['sources'] = "Sources must include required fields (url, module, chapter, section)"
    
    return errors