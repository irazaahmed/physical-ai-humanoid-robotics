"""
Core agent implementation using Google Gemini.

This module provides the reasoning-capable agent that uses tools to process queries
and generate contextual responses based on textbook content.
"""
import google.generativeai as genai
from typing import Dict, Any, List, Optional, Tuple
from .models import AgentResponse, Source, UserQuery
from .retrieval_tool import RetrievalTool
from .logging import logger, QueryProcessingError, setup_logging
from .config import Config
from .validation_utils import validate_tool_call_count, is_safe_content
import time
import uuid
from datetime import datetime


class Agent:
    """
    Core agent class that uses Google Gemini for reasoning and content generation.
    """

    def __init__(self):
        """
        Initialize the agent with Google Gemini client and tools.
        """
        # Validate configuration
        try:
            Config.validate()
        except ValueError as e:
            logger.error(f"Configuration validation failed: {e}")
            raise

        # Initialize Google Gemini client
        genai.configure(api_key=Config.GEMINI_API_KEY)
        self.client = genai.GenerativeModel(Config.GEMINI_MODEL)

        # Initialize tools
        self.retrieval_tool = RetrievalTool()

        logger.info(f"Agent initialized with Google Gemini model: {Config.GEMINI_MODEL}")
    
    def process_query(self, user_query: str, session_id: Optional[str] = None) -> AgentResponse:
        """
        Process a user query and generate a response.

        Args:
            user_query: The user's query string
            session_id: Optional session identifier for conversation context

        Returns:
            AgentResponse with the generated answer and metadata
        """
        start_time = time.time()

        # Validate query safety first
        if not is_safe_content(user_query):
            raise QueryProcessingError("Potentially harmful content detected in query")

        try:
            logger.info(f"Processing query: '{user_query[:100]}{'...' if len(user_query) > 100 else ''}'")

            # Step 1: Retrieve relevant content
            retrieval_result = self.retrieval_tool.call_retrieval(
                query=user_query,
                parameters={
                    "top_k": Config.DEFAULT_TOP_K,
                    "threshold": Config.DEFAULT_THRESHOLD
                }
            )

            # Extract sources from retrieval results
            sources = []
            retrieved_content = []
            for chunk in retrieval_result.get("chunks", []):
                metadata = chunk.get("metadata", {})
                source = Source(
                    url=metadata.get("url", ""),
                    module=metadata.get("module", ""),
                    chapter=metadata.get("chapter", ""),
                    section=metadata.get("section", ""),
                    similarity_score=chunk.get("similarity_score", 0.0),
                    content_preview=chunk["content"][:100] + "..."
                )
                sources.append(source)
                retrieved_content.append(chunk["content"])

            # Step 2: Formulate prompt with retrieved content and user query
            if retrieved_content:
                context = "\n\n".join(retrieved_content)
                prompt = f"""
                You are an expert agent specializing in Physical AI & Humanoid Robotics.
                Answer the user's query based on the provided context from the textbook content.

                Context:
                {context}

                User Query: {user_query}

                Provide a comprehensive and accurate answer based on the context.
                If the context doesn't contain enough information to answer the query,
                clearly state that the information is not available in the provided context.
                """
            else:
                # If no relevant content found, generate a response indicating this
                prompt = f"""
                The user asked: {user_query}

                The system could not find any relevant content in the textbook to answer this query.
                Please acknowledge this limitation and provide a helpful response to the user.
                """

            # Step 3: Generate response using Google Gemini
            response = self.client.generate_content(
                prompt,
                generation_config={
                    "max_output_tokens": 1000,  # Limit response length
                    "temperature": 0.3,  # Lower temperature for more consistent answers
                }
            )

            # Check if the response was blocked
            if not response.candidates or response.candidates[0].finish_reason < 1:
                raise QueryProcessingError("Response generation was blocked or failed")

            # Extract the generated content
            agent_response = response.text

            # Calculate processing time
            processing_time = (time.time() - start_time) * 1000  # Convert to milliseconds

            # Estimate confidence based on number of sources and retrieval quality
            confidence_score = self._calculate_confidence_with_sources(sources)

            # Create the response object
            response_obj = AgentResponse(
                id=str(uuid.uuid4()),
                query_id=str(uuid.uuid4()),
                content=agent_response,
                confidence_score=confidence_score,
                sources=sources,
                timestamp=datetime.utcnow(),
                processing_time_ms=processing_time
            )

            logger.info(f"Query processed successfully in {processing_time:.2f}ms")
            return response_obj

        except Exception as e:
            processing_time = (time.time() - start_time) * 1000
            logger.error(f"Error processing query after {processing_time:.2f}ms: {str(e)}")
            raise QueryProcessingError(f"Failed to process query: {str(e)}") from e

    def process_complex_query(self, user_query: str, max_steps: int = 3) -> AgentResponse:
        """
        Process a complex query that may require multiple retrieval steps or decomposition.

        Args:
            user_query: The user's complex query string
            max_steps: Maximum number of processing steps allowed

        Returns:
            AgentResponse with the generated answer and metadata
        """
        start_time = time.time()

        # Validate query safety first
        if not is_safe_content(user_query):
            raise QueryProcessingError("Potentially harmful content detected in query")

        try:
            logger.info(f"Processing complex query: '{user_query[:100]}{'...' if len(user_query) > 100 else ''}'")

            # For complex queries, we may want to do query decomposition
            # First, let's try to identify if this is a comparative or multi-part query
            decomposed_queries = self._decompose_query(user_query)

            # Process each part of the decomposed query
            all_chunks = []
            all_sources = []

            for i, sub_query in enumerate(decomposed_queries[:max_steps]):
                # Call the retrieval tool for each sub-query
                retrieval_result = self.retrieval_tool.call_retrieval(
                    query=sub_query,
                    parameters={
                        "top_k": Config.DEFAULT_TOP_K,
                        "threshold": Config.DEFAULT_THRESHOLD
                    }
                )

                # Collect all retrieved chunks
                for chunk in retrieval_result.get("chunks", []):
                    # Extract source information
                    metadata = chunk.get("metadata", {})
                    source = Source(
                        url=metadata.get("url", ""),
                        module=metadata.get("module", ""),
                        chapter=metadata.get("chapter", ""),
                        section=metadata.get("section", ""),
                        similarity_score=chunk.get("similarity_score", 0.0),
                        content_preview=chunk["content"][:100] + "..."
                    )
                    all_sources.append(source)
                    all_chunks.append(chunk)

            # Synthesize a response from all collected information
            if all_chunks:
                # Formulate prompt with all retrieved content and original query
                context = "\n\n".join([chunk["content"] for chunk in all_chunks])
                prompt = f"""
                You are an expert agent specializing in Physical AI & Humanoid Robotics.
                Answer the user's complex query based on the provided context from the textbook content.

                The query was decomposed into multiple parts, and relevant content was retrieved for each part.

                Context:
                {context}

                Original Complex Query: {user_query}

                Please provide a comprehensive and coherent answer that addresses all aspects of the complex query.
                If certain parts of the query cannot be answered based on the context, clearly state this.
                """

                # Generate response using Google Gemini
                response = self.client.generate_content(
                    prompt,
                    generation_config={
                        "max_output_tokens": 1500,  # Allow more tokens for complex queries
                        "temperature": 0.4,  # Slightly higher temperature for creative synthesis
                    }
                )

                # Check if the response was blocked
                if not response.candidates or response.candidates[0].finish_reason < 1:
                    raise QueryProcessingError("Response generation was blocked or failed")

                response_content = response.text

                # Calculate confidence based on quality and quantity of sources
                avg_similarity = sum(s.similarity_score for s in all_sources) / len(all_sources) if all_sources else 0.0
                confidence_score = min(avg_similarity * 1.2, 1.0)  # Boost for complex processing
            else:
                response_content = "I couldn't find relevant information to answer your complex query."
                confidence_score = 0.2
                all_sources = []

            # Calculate processing time
            processing_time = (time.time() - start_time) * 1000  # Convert to milliseconds

            # Create the response object
            response_obj = AgentResponse(
                id=str(uuid.uuid4()),
                query_id=str(uuid.uuid4()),
                content=response_content,
                confidence_score=confidence_score,
                sources=all_sources,
                timestamp=datetime.utcnow(),
                processing_time_ms=processing_time
            )

            logger.info(f"Complex query processed successfully in {processing_time:.2f}ms using {len(decomposed_queries)} sub-queries")
            return response_obj

        except Exception as e:
            processing_time = (time.time() - start_time) * 1000
            logger.error(f"Error processing complex query after {processing_time:.2f}ms: {str(e)}")
            raise QueryProcessingError(f"Failed to process complex query: {str(e)}") from e

    def _decompose_query(self, query: str) -> List[str]:
        """
        Decompose a complex query into simpler sub-queries.

        Args:
            query: The original complex query

        Returns:
            List of simpler sub-queries
        """
        # Simple decomposition strategy - identify keywords suggesting multiple parts
        query_lower = query.lower()

        # Check for comparison words (e.g., "compare X and Y", "difference between X and Y")
        if "compare" in query_lower or "difference between" in query_lower or "versus" in query_lower:
            # For comparison queries, extract the two concepts to compare
            import re
            # Pattern to match "compare X and Y" or "difference between X and Y"
            patterns = [
                r'compare\s+(.*?)\s+and\s+(.*?)(?:\.|$)',
                r'difference\s+between\s+(.*?)\s+and\s+(.*?)(?:\.|$)',
                r'(.*)\s+versus\s+(.*?)(?:\.|$)'
            ]

            for pattern in patterns:
                match = re.search(pattern, query, re.IGNORECASE)
                if match:
                    part1 = match.group(1).strip()
                    part2 = match.group(2).strip()
                    return [part1, part2]

        # Check for multiple questions in one query
        question_parts = re.split(r'[;,.]\s*and\s+|[,]\s*and\s+', query)
        if len(question_parts) > 1:
            return [part.strip() for part in question_parts if part.strip()]

        # If no obvious decomposition, return the original query
        return [query]

    def _synthesize_response_from_multiple_sources(self, original_query: str, chunks: List, max_steps: int) -> str:
        """
        Synthesize a coherent response from multiple retrieved chunks.

        Args:
            original_query: The original user query
            chunks: List of retrieved content chunks
            max_steps: Maximum number of steps considered

        Returns:
            Synthesized response string
        """
        if not chunks:
            return "I couldn't find any relevant information to answer your query."

        # Group chunks by module/chapter to organize the response
        module_groups = {}
        for chunk in chunks:
            module = chunk.metadata.get('module', 'Unknown Module')
            if module not in module_groups:
                module_groups[module] = []
            module_groups[module].append(chunk)

        # Build the response
        response_parts = [f"Based on the textbook content, here's information about: '{original_query}'"]

        for module, module_chunks in module_groups.items():
            response_parts.append(f"\n{module}:")

            # Sort chunks by similarity score (highest first) and limit to top 3 per module
            sorted_chunks = sorted(module_chunks, key=lambda c: c.similarity_score, reverse=True)[:3]

            for chunk in sorted_chunks:
                response_parts.append(f"  - {chunk.content}")

        return "\n".join(response_parts)
    
    def _calculate_confidence_with_sources(self, sources: List[Source]) -> float:
        """
        Calculate a confidence score based on the quality of retrieved information.

        Args:
            sources: List of sources used in the response

        Returns:
            Confidence score between 0.0 and 1.0
        """
        if not sources:
            return 0.1  # Low confidence if no sources

        # For a simple confidence calculation, we can average the similarity scores
        # from the sources if they have scores, or base it on other factors
        if sources:
            avg_similarity = sum(s.similarity_score for s in sources) / len(sources)
            # Adjust based on number of sources (more sources might indicate better confidence)
            source_factor = min(len(sources) / 5.0, 1.0)  # Max boost for 5+ sources
            confidence = (avg_similarity * 0.7) + (source_factor * 0.3)
            return min(confidence, 1.0)  # Cap at 1.0

        return 0.5  # Default medium confidence
    
    def process_simple_query(self, user_query: str) -> AgentResponse:
        """
        Process a simple query without complex reasoning or multiple steps.

        Args:
            user_query: The user's query string

        Returns:
            AgentResponse with the generated answer
        """
        start_time = time.time()

        # Validate query safety first
        if not is_safe_content(user_query):
            raise QueryProcessingError("Potentially harmful content detected in query")

        try:
            logger.info(f"Processing simple query: '{user_query[:100]}{'...' if len(user_query) > 100 else ''}'")

            # Directly call the retrieval tool for simple queries
            retrieval_result = self.retrieval_tool.call_retrieval(
                query=user_query,
                parameters={
                    "top_k": Config.DEFAULT_TOP_K,
                    "threshold": Config.DEFAULT_THRESHOLD
                }
            )

            if not retrieval_result.get("chunks") or len(retrieval_result["chunks"]) == 0:
                # No relevant content found
                response_content = "I couldn't find any relevant information in the textbook about your query."
                sources = []
                confidence_score = 0.2
            else:
                # Extract sources from the retrieval result
                sources = []
                retrieved_content = []
                for chunk in retrieval_result["chunks"]:
                    metadata = chunk.get("metadata", {})
                    source = Source(
                        url=metadata.get("url", ""),
                        module=metadata.get("module", ""),
                        chapter=metadata.get("chapter", ""),
                        section=metadata.get("section", ""),
                        similarity_score=chunk.get("similarity_score", 0.0),
                        content_preview=chunk["content"][:100] + "..."
                    )
                    sources.append(source)
                    retrieved_content.append(chunk["content"])

                # Formulate prompt with retrieved content and user query
                context = "\n\n".join(retrieved_content)
                prompt = f"""
                You are an expert agent specializing in Physical AI & Humanoid Robotics.
                Answer the user's query based on the provided context from the textbook content.

                Context:
                {context}

                User Query: {user_query}

                Provide a concise and accurate answer based on the context.
                """

                # Generate response using Google Gemini
                response = self.client.generate_content(
                    prompt,
                    generation_config={
                        "max_output_tokens": 800,  # Limit response length for simple queries
                        "temperature": 0.3,  # Lower temperature for consistent answers
                    }
                )

                # Check if the response was blocked
                if not response.candidates or response.candidates[0].finish_reason < 1:
                    raise QueryProcessingError("Response generation was blocked or failed")

                response_content = response.text

                # Calculate confidence based on the highest similarity score
                similarity_scores = [chunk.get("similarity_score", 0.0) for chunk in retrieval_result["chunks"]]
                confidence_score = max(similarity_scores) if similarity_scores else 0.0

            # Calculate processing time
            processing_time = (time.time() - start_time) * 1000  # Convert to milliseconds

            # Create the response object
            response_obj = AgentResponse(
                id=str(uuid.uuid4()),
                query_id=str(uuid.uuid4()),
                content=response_content,
                confidence_score=confidence_score,
                sources=sources,
                timestamp=datetime.utcnow(),
                processing_time_ms=processing_time
            )

            logger.info(f"Simple query processed successfully in {processing_time:.2f}ms")
            return response_obj

        except Exception as e:
            processing_time = (time.time() - start_time) * 1000
            logger.error(f"Error processing simple query after {processing_time:.2f}ms: {str(e)}")
            raise QueryProcessingError(f"Failed to process simple query: {str(e)}") from e


# Global agent instance
# In a production system, this would be managed differently to avoid global state
agent_instance = None


def get_agent() -> Agent:
    """
    Get the global agent instance, creating it if it doesn't exist.
    
    Returns:
        Agent instance
    """
    global agent_instance
    if agent_instance is None:
        agent_instance = Agent()
    return agent_instance