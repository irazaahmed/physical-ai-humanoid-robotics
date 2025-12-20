"""
Tool interface wrapper for the retrieval pipeline.

This module provides a clean interface for the agent to call the retrieval pipeline
as a tool, following the contract defined in api-contracts.md.
"""
from typing import Dict, Any, List, Optional
from .models import RetrievedChunk
from ..retrieval.retrieval_pipeline import RetrievalPipeline
from ..retrieval.models import Metadata
from .logging import logger, ToolExecutionError
from .config import Config
import time
import uuid


class RetrievalTool:
    """
    Wrapper class to expose the retrieval pipeline as an agent tool.
    """
    
    def __init__(self):
        """
        Initialize the retrieval tool wrapper.
        """
        # Initialize the retrieval pipeline with default values
        self.pipeline = RetrievalPipeline(
            k=Config.DEFAULT_TOP_K,
            threshold=Config.DEFAULT_THRESHOLD
        )
        
        # Setup the pipeline (validate connections)
        if not self.pipeline.setup():
            raise ToolExecutionError("Failed to initialize retrieval pipeline")
    
    def call_retrieval(self, query: str, parameters: Optional[Dict[str, Any]] = None) -> Dict[str, Any]:
        """
        Call the retrieval pipeline with the provided query and parameters.

        Args:
            query: The search query text
            parameters: Optional parameters for the retrieval (top_k, threshold, filters)

        Returns:
            Dictionary containing retrieval results following the contract from api-contracts.md
        """
        start_time = time.time()
        
        try:
            # Use default parameters if not provided
            if parameters is None:
                parameters = {}
            
            # Extract parameters with defaults
            top_k = parameters.get('top_k', Config.DEFAULT_TOP_K)
            threshold = parameters.get('threshold', Config.DEFAULT_THRESHOLD)
            filters = parameters.get('filters', None)
            
            # Perform retrieval using the existing pipeline
            search_result = self.pipeline.retrieve(
                query=query,
                k=top_k,
                threshold=threshold,
                filter_conditions=filters
            )
            
            # Convert the search result to the format expected by the agent
            result_chunks = []
            for chunk in search_result.chunks:
                # Create a RetrievedChunk in the format expected by our agent models
                agent_chunk = RetrievedChunk(
                    id=chunk.id,
                    content=chunk.content,
                    similarity_score=chunk.similarity_score,
                    rank=chunk.rank,
                    metadata=chunk.metadata.dict()
                )
                result_chunks.append(agent_chunk)
            
            # Calculate execution time
            execution_time = time.time() - start_time
            
            # Create the result following the contract from api-contracts.md
            result = {
                "retrieval_id": str(uuid.uuid4()),
                "chunks": [chunk.dict() for chunk in result_chunks],
                "execution_time_ms": execution_time * 1000,  # Convert to milliseconds
                "retrieval_timestamp": search_result.retrieval_timestamp.isoformat(),
                "total_chunks_found": len(result_chunks),
                "search_parameters": search_result.search_parameters
            }
            
            logger.info(f"Retrieval tool executed successfully in {execution_time * 1000:.2f}ms for query: '{query[:50]}{'...' if len(query) > 50 else ''}'")
            
            return result
            
        except Exception as e:
            execution_time = time.time() - start_time
            logger.error(f"Retrieval tool execution failed after {execution_time * 1000:.2f}ms: {str(e)}")
            
            # Return an error result in the expected format
            error_result = {
                "retrieval_id": str(uuid.uuid4()),
                "chunks": [],
                "execution_time_ms": execution_time * 1000,
                "retrieval_timestamp": time.time(),
                "total_chunks_found": 0,
                "search_parameters": {"k": str(parameters.get('top_k', Config.DEFAULT_TOP_K)),
                                    "threshold": str(parameters.get('threshold', Config.DEFAULT_THRESHOLD))},
                "error": str(e)
            }
            
            raise ToolExecutionError(f"Retrieval tool execution failed: {str(e)}") from e