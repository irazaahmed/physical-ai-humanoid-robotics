"""
API routes for the agent service.

This module defines the FastAPI routes for the agent service,
following the contracts specified in api-contracts.md.

The agent service provides intelligent query processing capabilities using Google Gemini
to perform reasoning and information retrieval from the textbook content.

API Endpoints:
- POST /api/v1/chat: Process conversational queries and return synthesized responses
- POST /api/v1/query: Process direct information queries and return structured results
- GET /api/v1/health: Check the health status of the agent service and dependencies

Rate Limiting:
All public endpoints are limited to 10 requests per minute per IP address to prevent
abuse of the agent service and associated API costs.

Error Handling:
The service returns appropriate HTTP status codes and structured error responses
for various error conditions including validation errors, rate limiting, and
service unavailability.
"""
from fastapi import APIRouter, Depends, Request
from fastapi.responses import JSONResponse
from typing import Dict, Any
from .models import QueryRequest, QueryResponse, ChatRequest, ChatResponse, HealthResponse, HealthStatus
from .core import get_agent
from .validation_utils import validate_query_request, validate_chat_request
from .rate_limiter import apply_rate_limit
from .config import Config
from .logging import logger, log_request, log_response, handle_error
from .cache import query_cache
from datetime import datetime
import time
import uuid


# Create the API router
router = APIRouter()


@router.post("/query", response_model=QueryResponse)
@apply_rate_limit()
async def query_endpoint(request: Request, query_request: QueryRequest = Depends()):
    """
    Process a direct information query and return structured results.
    POST /api/v1/query
    """
    start_time = time.time()

    try:
        # Log the incoming request
        log_request(query_request.query, "api_query")

        # Validate the request
        validation_errors = validate_query_request(query_request)
        if validation_errors:
            return JSONResponse(
                status_code=400,
                content={
                    "error": "Invalid request format",
                    "message": "Your request contains invalid parameters. Please check the query length and parameters.",
                    "validation_errors": validation_errors,
                    "timestamp": datetime.utcnow().isoformat()
                }
            )

        # Check cache first (using a default session for non-conversational queries)
        session_id = f"query_session_{str(uuid.uuid4())}"
        cached_response = query_cache.get(query_request.query, session_id)
        if cached_response:
            logger.info(f"Cache hit for query: '{query_request.query[:50]}...'")

            # Prepare cached response
            result = QueryResponse(
                result_id=cached_response.query_id,
                query=query_request.query,
                results=[],  # We'll populate this based on the agent's internal retrieval
                search_parameters={
                    "k": str(query_request.parameters.top_k),
                    "threshold": str(query_request.parameters.threshold),
                    "distance_metric": "cosine"
                },
                execution_time_ms=0,  # Cached response
                timestamp=datetime.utcnow()
            )

            # Since the agent handles retrieval internally, we'll return the information differently
            # For the query endpoint, we'll return the sources found as results
            for source in cached_response.sources:
                # Note: In a full implementation, we would map the agent's sources to RetrievedChunk format
                # For now, we'll create a basic chunk representation
                from .models import RetrievedChunk
                chunk = RetrievedChunk(
                    id=str(uuid.uuid4()),
                    content=source.content_preview,
                    similarity_score=source.similarity_score,
                    rank=0,  # Would be determined by the retrieval ranking
                    metadata={
                        "url": source.url,
                        "module": source.module,
                        "chapter": source.chapter,
                        "section": source.section,
                        "chunk_index": 0,  # Placeholder
                        "hash": "",  # Placeholder
                        "url_fragment": getattr(source, 'url_fragment', ''),
                        "page_reference": getattr(source, 'page_reference', '')
                    }
                )
                result.results.append(chunk)

            # Log the response
            log_response(cached_response.content, 0, cached_response.sources)
            return result

        # Get the agent instance
        agent = get_agent()

        # Process the query using the agent's simple processing function
        response = agent.process_simple_query(query_request.query)

        # Cache the response
        query_cache.set(query_request.query, session_id, response)

        # Calculate execution time
        execution_time = (time.time() - start_time) * 1000  # Convert to milliseconds

        # Prepare the response following the contract
        result = QueryResponse(
            result_id=str(uuid.uuid4()),
            query=query_request.query,
            results=[],  # We'll populate this based on the agent's internal retrieval
            search_parameters={
                "k": str(query_request.parameters.top_k),
                "threshold": str(query_request.parameters.threshold),
                "distance_metric": "cosine"
            },
            execution_time_ms=execution_time,
            timestamp=datetime.utcnow()
        )

        # Since the agent handles retrieval internally, we'll return the information differently
        # For the query endpoint, we'll return the sources found as results
        for source in response.sources:
            # Note: In a full implementation, we would map the agent's sources to RetrievedChunk format
            # For now, we'll create a basic chunk representation
            from .models import RetrievedChunk
            chunk = RetrievedChunk(
                id=str(uuid.uuid4()),
                content=source.content_preview,
                similarity_score=source.similarity_score,
                rank=0,  # Would be determined by the retrieval ranking
                metadata={
                    "url": source.url,
                    "module": source.module,
                    "chapter": source.chapter,
                    "section": source.section,
                    "chunk_index": 0,  # Placeholder
                    "hash": "",  # Placeholder
                    "url_fragment": getattr(source, 'url_fragment', ''),
                    "page_reference": getattr(source, 'page_reference', '')
                }
            )
            result.results.append(chunk)

        # Log the response
        log_response(response.content, execution_time, response.sources)

        return result

    except Exception as e:
        error_response = handle_error(e, "query_endpoint")
        # Provide more user-friendly error message
        if "Configuration error" in error_response.get("message", ""):
            error_response["message"] = "Service configuration error. Please contact the administrator."
        else:
            error_response["message"] = f"We encountered an issue processing your query: {str(e)}"
        return JSONResponse(
            status_code=500,
            content=error_response
        )


@router.post("/chat", response_model=ChatResponse)
@apply_rate_limit()
async def chat_endpoint(request: Request, chat_request: ChatRequest = Depends()):
    """
    Process a conversational query and return a synthesized response.
    POST /api/v1/chat
    """
    start_time = time.time()

    try:
        # Generate session ID if not provided
        session_id = chat_request.session_id or f"session_{str(uuid.uuid4())}"

        # Log the incoming request
        log_request(chat_request.query, session_id)

        # Validate the request
        validation_errors = validate_chat_request(chat_request)
        if validation_errors:
            return JSONResponse(
                status_code=400,
                content={
                    "error": "Invalid request format",
                    "message": "Your request contains invalid parameters. Please check the query length and parameters.",
                    "validation_errors": validation_errors,
                    "timestamp": datetime.utcnow().isoformat()
                }
            )

        # Check cache first
        cached_response = query_cache.get(chat_request.query, session_id)
        if cached_response:
            logger.info(f"Cache hit for query: '{chat_request.query[:50]}...'")

            # Prepare cached response
            chat_response = ChatResponse(
                response_id=cached_response.id,
                query=chat_request.query,
                answer=cached_response.content,
                sources=cached_response.sources,
                confidence_score=cached_response.confidence_score,
                processing_time_ms=0,  # Cached response
                timestamp=datetime.utcnow()
            )

            # Log the response
            log_response(cached_response.content, 0, cached_response.sources)
            return chat_response

        # Get the agent instance
        agent = get_agent()

        # Process the query using the agent
        response = agent.process_query(chat_request.query, session_id)

        # Cache the response
        query_cache.set(chat_request.query, session_id, response)

        # Calculate execution time
        execution_time = (time.time() - start_time) * 1000  # Convert to milliseconds

        # Prepare the response following the contract
        chat_response = ChatResponse(
            response_id=response.id,
            query=chat_request.query,
            answer=response.content,
            sources=response.sources,
            confidence_score=response.confidence_score,
            processing_time_ms=execution_time,
            timestamp=datetime.utcnow()
        )

        # Log the response
        log_response(response.content, execution_time, response.sources)

        return chat_response

    except Exception as e:
        error_response = handle_error(e, "chat_endpoint")
        # Provide more user-friendly error message
        if "Configuration error" in error_response.get("message", ""):
            error_response["message"] = "Service configuration error. Please contact the administrator."
        else:
            error_response["message"] = f"We encountered an issue processing your query: {str(e)}"
        return JSONResponse(
            status_code=500,
            content=error_response
        )


@router.get("/health", response_model=HealthResponse)
async def health_endpoint():
    """
    Check the health status of the agent service.
    GET /api/v1/health
    """
    try:
        # Check dependencies
        dependencies_status = {
            "gemini_api": "checking",
            "qdrant_connection": "checking",
            "retrieval_pipeline": "checking"
        }

        # Check Google Gemini API
        try:
            agent = get_agent()
            dependencies_status["gemini_api"] = "connected"
        except Exception:
            dependencies_status["gemini_api"] = "disconnected"
        
        # Check retrieval pipeline
        try:
            from .retrieval_tool import RetrievalTool
            tool = RetrievalTool()  # This will validate the pipeline during initialization
            dependencies_status["retrieval_pipeline"] = "available"
        except Exception:
            dependencies_status["retrieval_pipeline"] = "unavailable"
        
        # Note: Qdrant connection is checked during retrieval tool initialization
        
        # Determine overall health status
        is_healthy = all(status == "connected" or status == "available" 
                        for status in dependencies_status.values())
        
        health_status = HealthStatus.HEALTHY if is_healthy else HealthStatus.UNHEALTHY
        
        # Prepare metrics
        metrics = {
            "active_sessions": 0,  # Placeholder - would track actual sessions in a real implementation
            "queries_processed": 0,  # Placeholder - would track actual metrics
            "avg_response_time_ms": 0  # Placeholder - would track actual metrics
        }
        
        health_response = HealthResponse(
            status=health_status,
            timestamp=datetime.utcnow(),
            dependencies=dependencies_status,
            metrics=metrics
        )
        
        return health_response
        
    except Exception as e:
        # On any error, return a health status that indicates the service is unhealthy
        error_response = HealthResponse(
            status=HealthStatus.UNHEALTHY,
            timestamp=datetime.utcnow(),
            dependencies={
                "gemini_api": "error",
                "qdrant_connection": "error",
                "retrieval_pipeline": "error"
            },
            metrics=None
        )
        return error_response