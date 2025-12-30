"""
Core agent implementation using OpenRouter.

This module provides the reasoning-capable agent that uses tools to process queries
and generate contextual responses based on textbook content.
"""
from .openrouter_client import OpenRouterClient
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
    Core agent class that uses OpenRouter for reasoning and content generation.
    """

    def __init__(self):
        """
        Initialize the agent with OpenRouter client and tools.
        """
        # Validate configuration
        try:
            Config.validate()
        except ValueError as e:
            logger.error(f"Configuration validation failed: {e}")
            raise

        # Initialize OpenRouter client
        self.client = OpenRouterClient()

        # Initialize tools
        self.retrieval_tool = RetrievalTool()

        # Initialize session storage
        self.sessions = {}

        logger.info(f"Agent initialized with OpenRouter model: {Config.OPENROUTER_MODEL}")
    
    def _calculate_dynamic_retrieval_params(self, query: str, base_params: Dict[str, Any]) -> Dict[str, Any]:
        """
        Calculate dynamic retrieval parameters based on query characteristics.

        Args:
            query: The user query string
            base_params: Base parameters with potential overrides

        Returns:
            Dictionary with optimized retrieval parameters
        """
        # Get base values
        top_k = base_params.get('top_k', Config.DEFAULT_TOP_K)
        threshold = base_params.get('threshold', Config.DEFAULT_THRESHOLD)

        # Analyze query length and complexity
        query_length = len(query.strip())
        word_count = len(query.split())

        # Adjust top_k based on query length and complexity
        if word_count <= 3:
            # Short queries: be more focused to reduce noise
            top_k = max(2, min(top_k, 3))
        elif word_count <= 8:
            # Medium queries: use default
            top_k = top_k
        else:
            # Longer/conceptual queries: allow more context
            top_k = min(top_k * 2, 10)  # Cap at 10 for very long queries

        # Adjust threshold based on query type
        # For shorter queries that might be more ambiguous, be slightly more permissive
        if word_count <= 4:
            # Lower threshold for short queries to catch more relevant results
            threshold = max(0.4, threshold - 0.1)
        elif word_count >= 15:
            # Higher threshold for longer queries that are more specific
            threshold = min(0.75, threshold + 0.05)

        return {
            'top_k': top_k,
            'threshold': threshold,
            'filters': base_params.get('filters')
        }

    def _detect_query_type(self, query: str) -> str:
        """
        Detect the type of query to guide answer style.

        Args:
            query: The user query string

        Returns:
            Query type: 'explanatory', 'procedural', 'conceptual', or 'general'
        """
        query_lower = query.lower().strip()
        words = query_lower.split()

        # Keywords for different query types
        explanatory_keywords = ['what is', 'what are', 'define', 'explain', 'describe', 'meaning of', 'tell me about']
        procedural_keywords = ['how to', 'how do', 'steps to', 'process', 'procedure', 'instructions', 'way to']
        conceptual_keywords = ['why', 'principle', 'theory', 'concept', 'relationship', 'difference between', 'compare']

        # Check for explanatory queries
        for keyword in explanatory_keywords:
            if keyword in query_lower:
                return 'explanatory'

        # Check for procedural queries
        for keyword in procedural_keywords:
            if keyword in query_lower:
                return 'procedural'

        # Check for conceptual queries
        for keyword in conceptual_keywords:
            if keyword in query_lower:
                return 'conceptual'

        # Default to general
        return 'general'

    def _evaluate_retrieval_quality(self, chunks: List[Dict], min_threshold: float = 0.5) -> tuple:
        """
        Evaluate the quality of retrieved content to determine if it's sufficient for answering.

        Args:
            chunks: List of retrieved chunks with similarity scores
            min_threshold: Minimum similarity score considered useful

        Returns:
            Tuple of (is_high_quality, avg_score, usable_chunks)
        """
        if not chunks:
            return False, 0.0, []

        similarity_scores = [chunk.get("similarity_score", 0.0) for chunk in chunks]
        avg_score = sum(similarity_scores) / len(similarity_scores)
        usable_chunks = [chunk for chunk in chunks if chunk.get("similarity_score", 0.0) >= min_threshold]

        # Consider high quality if: average score is above threshold OR at least one chunk is above 0.7
        is_high_quality = avg_score >= min_threshold or max(similarity_scores) >= 0.7

        return is_high_quality, avg_score, usable_chunks

    def _normalize_response(self, response_text: str) -> str:
        """
        Normalize the final response to improve quality and consistency.

        Args:
            response_text: The raw response from the LLM

        Returns:
            Cleaned and normalized response text
        """
        # Strip leading/trailing whitespace
        normalized = response_text.strip()

        # Remove common filler phrases that should not appear in responses
        filler_phrases = [
            "Sure, ",
            "Certainly, ",
            "Here is ",
            "Here's ",
            "Okay, ",
            "Well, ",
            "Actually, ",
            "Basically, ",
            "In fact, ",
            "Just ",
            "So, ",
            "Now, ",
            "First, ",
            "Firstly, ",
            "To begin with, "
        ]

        for phrase in filler_phrases:
            if normalized.startswith(phrase):
                normalized = normalized[len(phrase):]
                break  # Only remove one leading phrase

        # Remove leading newlines and extra whitespace
        normalized = normalized.lstrip('\n\r ')

        # Remove trailing punctuation if it's excessive
        while normalized and normalized[-1] in ['.', '!', '?', ',', ';']:
            normalized = normalized[:-1].rstrip()

        # Ensure proper capitalization and punctuation
        if normalized:
            normalized = normalized[0].upper() + normalized[1:]  # Capitalize first letter
            # Add period if no ending punctuation
            if normalized and normalized[-1] not in ['.', '!', '?']:
                normalized += '.'

        return normalized

    def process_query_with_parameters(self, user_query: str, session_id: Optional[str] = None, parameters: Optional[QueryParameters] = None) -> AgentResponse:
        """
        Process a user query with specific parameters and generate a response.

        Args:
            user_query: The user's query string
            session_id: Optional session identifier for conversation context
            parameters: Optional query parameters (top_k, threshold, filters)

        Returns:
            AgentResponse with the generated answer and metadata
        """
        start_time = time.time()

        # Validate query safety first
        if not is_safe_content(user_query):
            raise QueryProcessingError("Potentially harmful content detected in query")

        try:
            logger.info(f"Processing query: '{user_query[:100]}{'...' if len(user_query) > 100 else ''}'")

            # Get conversation history if session is provided
            conversation_context = ""
            if session_id:
                if session_id not in self.sessions:
                    self.sessions[session_id] = []

                # Add current query to session context
                session_context = self.sessions[session_id]
                # Limit context window to prevent memory bloat
                recent_context = session_context[-3:]  # Last 3 exchanges

                if recent_context:
                    conversation_context = "\n\nPrevious conversation:\n"
                    for exchange in recent_context:
                        conversation_context += f"Q: {exchange['query']}\nA: {exchange['response'][:200]}...\n\n"

            # Use provided parameters or defaults
            if parameters is None:
                base_params = {
                    "top_k": Config.DEFAULT_TOP_K,
                    "threshold": Config.DEFAULT_THRESHOLD
                }
            else:
                base_params = {
                    "top_k": parameters.top_k,
                    "threshold": parameters.threshold,
                    "filters": parameters.filters
                }

            # Calculate dynamic retrieval parameters based on query characteristics
            retrieval_params = self._calculate_dynamic_retrieval_params(user_query, base_params)

            # Step 1: Retrieve relevant content
            retrieval_result = self.retrieval_tool.call_retrieval(
                query=user_query,
                parameters=retrieval_params
            )

            # Extract sources from retrieval results
            sources = []
            retrieved_content = []
            retrieved_chunks = retrieval_result.get("chunks", [])
            for chunk in retrieved_chunks:
                metadata = chunk.get("metadata", {})
                source = Source(
                    url=metadata.get("url", ""),
                    module=metadata.get("module", ""),
                    chapter=metadata.get("chapter", ""),
                    section=metadata.get("section", ""),
                    similarity_score=chunk.get("similarity_score", 0.0),
                    content_preview=chunk["content"][:100] + "...",
                    url_fragment=metadata.get("url_fragment", ""),
                    page_reference=metadata.get("page_reference", "")
                )
                sources.append(source)
                retrieved_content.append(chunk["content"])

            # Step 2: Formulate prompt with retrieved content using strict hybrid policy
            query_type = self._detect_query_type(user_query)

            # Evaluate retrieval quality to handle partial matches
            is_high_quality, avg_score, usable_chunks = self._evaluate_retrieval_quality(retrieved_chunks)

            if retrieved_content and is_high_quality:
                # Use textbook content to answer the query
                context = "\n\n".join([chunk["content"] for chunk in usable_chunks])
                prompt = f"""
                You are an expert agent specializing in Physical AI & Humanoid Robotics.
                Answer the user's query based on the provided context from the textbook content.
                Consider the previous conversation context if provided.

                SYSTEM INSTRUCTION:
                You are an expert agent that answers user questions with strict adherence to the following policy:

                RULES:
                1. If textbook context is provided, answer ONLY using the textbook content - no exceptions.
                2. If textbook context is provided, NEVER mention the textbook, sources, retrieval, chunks, embeddings, RAG, confidence scores, or backend logic.
                3. If no textbook context is provided, use your general knowledge to answer the question naturally.
                4. If no textbook context is provided, NEVER say phrases like "the textbook does not contain", "based on my knowledge", "I could not find", "RAG", "context", etc.
                5. In ALL cases, provide ONLY the final answer in natural language - nothing else.
                6. Do NOT expose any backend logic, retrieval results, similarity scores, thresholds, sources, or system behavior.
                7. Do NOT show fallback messages or internal reasoning.
                8. Do NOT apologize for missing content.
                9. Do NOT mention OpenRouter, models, or APIs.
                10. Answer naturally as a normal AI assistant would, regardless of whether you're using textbook content or general knowledge.
                11. Provide concise but complete answers without unnecessary verbosity or filler phrases.
                12. For {query_type} queries, tailor your response appropriately:
                    - Explanatory: Focus on clear explanations and definitions
                    - Procedural: Provide step-by-step instructions or processes
                    - Conceptual: Explain principles, theories, and relationships
                    - General: Provide balanced and informative responses
                13. Use simple language unless the question is clearly advanced.
                14. Avoid restating the question; answer directly.

                Context:
                {context}

                {conversation_context}
                User Query: {user_query}

                Answer the user's query based ONLY on the provided textbook context, following the system instruction above.
                Provide ONLY your final answer in natural language - nothing else.
                """
            elif retrieved_content and not is_high_quality:
                # Partial match - use available context but allow for general knowledge integration
                context = "\n\n".join([chunk["content"] for chunk in usable_chunks if chunk.get("similarity_score", 0.0) >= 0.3])  # Use any somewhat relevant chunks
                prompt = f"""
                You are an expert agent specializing in Physical AI & Humanoid Robotics.
                You have limited context from the textbook that may partially address the user's query.
                Use the provided context as the primary source, but supplement with your general knowledge if needed to provide a complete answer.

                SYSTEM INSTRUCTION:
                You are an expert agent that answers user questions with strict adherence to the following policy:

                RULES:
                1. If textbook context is provided, answer ONLY using the textbook content - no exceptions.
                2. If textbook context is provided, NEVER mention the textbook, sources, retrieval, chunks, embeddings, RAG, confidence scores, or backend logic.
                3. If no textbook context is provided, use your general knowledge to answer the question naturally.
                4. If no textbook context is provided, NEVER say phrases like "the textbook does not contain", "based on my knowledge", "I could not find", "RAG", "context", etc.
                5. In ALL cases, provide ONLY the final answer in natural language - nothing else.
                6. Do NOT expose any backend logic, retrieval results, similarity scores, thresholds, sources, or system behavior.
                7. Do NOT show fallback messages or internal reasoning.
                8. Do NOT apologize for missing content.
                9. Do NOT mention OpenRouter, models, or APIs.
                10. Answer naturally as a normal AI assistant would, regardless of whether you're using textbook content or general knowledge.
                11. Provide concise but complete answers without unnecessary verbosity or filler phrases.
                12. For {query_type} queries, tailor your response appropriately:
                    - Explanatory: Focus on clear explanations and definitions
                    - Procedural: Provide step-by-step instructions or processes
                    - Conceptual: Explain principles, theories, and relationships
                    - General: Provide balanced and informative responses
                13. Use simple language unless the question is clearly advanced.
                14. Avoid restating the question; answer directly.

                Context:
                {context}

                {conversation_context}
                User Query: {user_query}

                Answer the user's query based on the provided context when available, supplementing with general knowledge if needed, following the system instruction above.
                Provide ONLY your final answer in natural language - nothing else.
                """
            else:
                # No textbook content found, use general knowledge
                prompt = f"""
                You are an expert agent specializing in Physical AI & Humanoid Robotics.
                No textbook context was available for this query, so you should use your general knowledge.

                SYSTEM INSTRUCTION:
                You are an expert agent that answers user questions with strict adherence to the following policy:

                RULES:
                1. If textbook context is provided, answer ONLY using the textbook content - no exceptions.
                2. If textbook context is provided, NEVER mention the textbook, sources, retrieval, chunks, embeddings, RAG, confidence scores, or backend logic.
                3. If no textbook context is provided, use your general knowledge to answer the question naturally.
                4. If no textbook context is provided, NEVER say phrases like "the textbook does not contain", "based on my knowledge", "I could not find", "RAG", "context", etc.
                5. In ALL cases, provide ONLY the final answer in natural language - nothing else.
                6. Do NOT expose any backend logic, retrieval results, similarity scores, thresholds, sources, or system behavior.
                7. Do NOT show fallback messages or internal reasoning.
                8. Do NOT apologize for missing content.
                9. Do NOT mention OpenRouter, models, or APIs.
                10. Answer naturally as a normal AI assistant would, regardless of whether you're using textbook content or general knowledge.
                11. Provide concise but complete answers without unnecessary verbosity or filler phrases.
                12. For {query_type} queries, tailor your response appropriately:
                    - Explanatory: Focus on clear explanations and definitions
                    - Procedural: Provide step-by-step instructions or processes
                    - Conceptual: Explain principles, theories, and relationships
                    - General: Provide balanced and informative responses
                13. Use simple language unless the question is clearly advanced.
                14. Avoid restating the question; answer directly.

                {conversation_context}
                User Query: {user_query}

                Answer the user's query using your general knowledge, following the system instruction above.
                Provide ONLY your final answer in natural language - nothing else.
                """

            # Step 3: Generate response using OpenRouter
            raw_response = self.client.generate_content(
                prompt,
                max_tokens=1000,  # Limit response length
                temperature=0.3   # Lower temperature for more consistent answers
            )

            # Normalize the response to improve quality and consistency
            agent_response = self._normalize_response(raw_response)

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

            # Store the exchange in session history if session_id provided
            if session_id:
                self.sessions[session_id].append({
                    'query': user_query,
                    'response': agent_response,
                    'timestamp': datetime.utcnow()
                })
                # Limit session history to prevent memory bloat
                if len(self.sessions[session_id]) > 10:
                    self.sessions[session_id] = self.sessions[session_id][-10:]

            logger.info(f"Query processed successfully in {processing_time:.2f}ms")
            return response_obj

        except Exception as e:
            processing_time = (time.time() - start_time) * 1000
            logger.error(f"Error processing query after {processing_time:.2f}ms: {str(e)}")
            raise QueryProcessingError(f"Failed to process query: {str(e)}") from e

    def process_query(self, user_query: str, session_id: Optional[str] = None) -> AgentResponse:
        """
        Process a user query and generate a response (using default parameters).

        Args:
            user_query: The user's query string
            session_id: Optional session identifier for conversation context

        Returns:
            AgentResponse with the generated answer and metadata
        """
        # Call the parameterized version with default parameters
        from .models import QueryParameters
        default_params = QueryParameters()
        return self.process_query_with_parameters(user_query, session_id, default_params)

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
                # Calculate dynamic retrieval parameters for each sub-query
                base_params = {
                    "top_k": Config.DEFAULT_TOP_K,
                    "threshold": Config.DEFAULT_THRESHOLD
                }
                retrieval_params = self._calculate_dynamic_retrieval_params(sub_query, base_params)

                # Call the retrieval tool for each sub-query
                retrieval_result = self.retrieval_tool.call_retrieval(
                    query=sub_query,
                    parameters=retrieval_params
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

            # Detect query type for improved response
            query_type = self._detect_query_type(user_query)

            # Evaluate retrieval quality to handle partial matches in complex queries
            is_high_quality, avg_score, usable_chunks = self._evaluate_retrieval_quality(all_chunks)

            # Synthesize a response from all collected information
            if all_chunks and is_high_quality:
                # Formulate prompt with all retrieved content using strict hybrid policy
                context = "\n\n".join([chunk["content"] for chunk in usable_chunks])
                prompt = f"""
                You are an expert agent specializing in Physical AI & Humanoid Robotics.
                Answer the user's complex query based on the provided context from the textbook content.

                SYSTEM INSTRUCTION:
                You are an expert agent that answers user questions with strict adherence to the following policy:

                RULES:
                1. If textbook context is provided, answer ONLY using the textbook content - no exceptions.
                2. If textbook context is provided, NEVER mention the textbook, sources, retrieval, chunks, embeddings, RAG, confidence scores, or backend logic.
                3. If no textbook context is provided, use your general knowledge to answer the question naturally.
                4. If no textbook context is provided, NEVER say phrases like "the textbook does not contain", "based on my knowledge", "I could not find", "RAG", "context", etc.
                5. In ALL cases, provide ONLY the final answer in natural language - nothing else.
                6. Do NOT expose any backend logic, retrieval results, similarity scores, thresholds, sources, or system behavior.
                7. Do NOT show fallback messages or internal reasoning.
                8. Do NOT apologize for missing content.
                9. Do NOT mention OpenRouter, models, or APIs.
                10. Answer naturally as a normal AI assistant would, regardless of whether you're using textbook content or general knowledge.
                11. Provide concise but complete answers without unnecessary verbosity or filler phrases.
                12. For {query_type} queries, tailor your response appropriately:
                    - Explanatory: Focus on clear explanations and definitions
                    - Procedural: Provide step-by-step instructions or processes
                    - Conceptual: Explain principles, theories, and relationships
                    - General: Provide balanced and informative responses
                13. Use simple language unless the question is clearly advanced.
                14. Avoid restating the question; answer directly.

                Context:
                {context}

                Original Complex Query: {user_query}

                Answer the user's query based ONLY on the provided textbook context, following the system instruction above.
                Provide ONLY your final answer in natural language - nothing else.
                """

                # Generate response using OpenRouter
                response_content = self.client.generate_content(
                    prompt,
                    max_tokens=1500,  # Allow more tokens for complex queries
                    temperature=0.4   # Slightly higher temperature for creative synthesis
                )

                # Calculate confidence based on quality and quantity of sources
                avg_similarity = sum(s.similarity_score for s in all_sources) / len(all_sources) if all_sources else 0.0
                confidence_score = min(avg_similarity * 1.2, 1.0)  # Boost for complex processing
            elif all_chunks and not is_high_quality:
                # Partial match - use available context but allow for general knowledge integration
                usable_content = [chunk["content"] for chunk in usable_chunks if chunk.get("similarity_score", 0.0) >= 0.3]  # Use any somewhat relevant chunks
                context = "\n\n".join(usable_content)
                prompt = f"""
                You are an expert agent specializing in Physical AI & Humanoid Robotics.
                You have limited context from the textbook that may partially address the user's complex query.
                Use the provided context as the primary source, but supplement with your general knowledge if needed to provide a complete answer.

                SYSTEM INSTRUCTION:
                You are an expert agent that answers user questions with strict adherence to the following policy:

                RULES:
                1. If textbook context is provided, answer ONLY using the textbook content - no exceptions.
                2. If textbook context is provided, NEVER mention the textbook, sources, retrieval, chunks, embeddings, RAG, confidence scores, or backend logic.
                3. If no textbook context is provided, use your general knowledge to answer the question naturally.
                4. If no textbook context is provided, NEVER say phrases like "the textbook does not contain", "based on my knowledge", "I could not find", "RAG", "context", etc.
                5. In ALL cases, provide ONLY the final answer in natural language - nothing else.
                6. Do NOT expose any backend logic, retrieval results, similarity scores, thresholds, sources, or system behavior.
                7. Do NOT show fallback messages or internal reasoning.
                8. Do NOT apologize for missing content.
                9. Do NOT mention OpenRouter, models, or APIs.
                10. Answer naturally as a normal AI assistant would, regardless of whether you're using textbook content or general knowledge.
                11. Provide concise but complete answers without unnecessary verbosity or filler phrases.
                12. For {query_type} queries, tailor your response appropriately:
                    - Explanatory: Focus on clear explanations and definitions
                    - Procedural: Provide step-by-step instructions or processes
                    - Conceptual: Explain principles, theories, and relationships
                    - General: Provide balanced and informative responses
                13. Use simple language unless the question is clearly advanced.
                14. Avoid restating the question; answer directly.

                Context:
                {context}

                Original Complex Query: {user_query}

                Answer the user's query based on the provided context when available, supplementing with general knowledge if needed, following the system instruction above.
                Provide ONLY your final answer in natural language - nothing else.
                """

                # Generate response using OpenRouter
                raw_response = self.client.generate_content(
                    prompt,
                    max_tokens=1500,  # Allow more tokens for complex queries
                    temperature=0.4   # Slightly higher temperature for creative synthesis
                )
                # Normalize the response to improve quality and consistency
                response_content = self._normalize_response(raw_response)

                # Calculate confidence based on quality and quantity of usable sources
                usable_sources = [s for s in all_sources if s.similarity_score >= 0.3]
                if usable_sources:
                    avg_similarity = sum(s.similarity_score for s in usable_sources) / len(usable_sources) if usable_sources else 0.0
                    confidence_score = min(avg_similarity * 1.2, 0.8)  # Lower max for partial matches
                else:
                    confidence_score = 0.3  # Lower confidence for partial matches
            else:
                # No relevant content found - use OpenRouter to generate response with general knowledge
                prompt = f"""
                You are an expert agent specializing in Physical AI & Humanoid Robotics.
                No textbook context was available for this query, so you should use your general knowledge.

                SYSTEM INSTRUCTION:
                You are an expert agent that answers user questions with strict adherence to the following policy:

                RULES:
                1. If textbook context is provided, answer ONLY using the textbook content - no exceptions.
                2. If textbook context is provided, NEVER mention the textbook, sources, retrieval, chunks, embeddings, RAG, confidence scores, or backend logic.
                3. If no textbook context is provided, use your general knowledge to answer the question naturally.
                4. If no textbook context is provided, NEVER say phrases like "the textbook does not contain", "based on my knowledge", "I could not find", "RAG", "context", etc.
                5. In ALL cases, provide ONLY the final answer in natural language - nothing else.
                6. Do NOT expose any backend logic, retrieval results, similarity scores, thresholds, sources, or system behavior.
                7. Do NOT show fallback messages or internal reasoning.
                8. Do NOT apologize for missing content.
                9. Do NOT mention OpenRouter, models, or APIs.
                10. Answer naturally as a normal AI assistant would, regardless of whether you're using textbook content or general knowledge.
                11. Provide concise but complete answers without unnecessary verbosity or filler phrases.
                12. For {query_type} queries, tailor your response appropriately:
                    - Explanatory: Focus on clear explanations and definitions
                    - Procedural: Provide step-by-step instructions or processes
                    - Conceptual: Explain principles, theories, and relationships
                    - General: Provide balanced and informative responses
                13. Use simple language unless the question is clearly advanced.
                14. Avoid restating the question; answer directly.

                Original Complex Query: {user_query}

                Answer the user's query using your general knowledge, following the system instruction above.
                Provide ONLY your final answer in natural language - nothing else.
                """

                # Generate response using OpenRouter with general knowledge
                raw_response = self.client.generate_content(
                    prompt,
                    max_tokens=1500,  # Allow more tokens for complex queries
                    temperature=0.4   # Slightly higher for general knowledge responses
                )
                # Normalize the response to improve quality and consistency
                response_content = self._normalize_response(raw_response)
                confidence_score = 0.2  # Lower confidence for general knowledge responses
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
    
    def process_simple_query_with_parameters(self, user_query: str, parameters: Optional[QueryParameters] = None) -> AgentResponse:
        """
        Process a simple query with specific parameters without complex reasoning or multiple steps.

        Args:
            user_query: The user's query string
            parameters: Optional query parameters (top_k, threshold, filters)

        Returns:
            AgentResponse with the generated answer
        """
        start_time = time.time()

        # Validate query safety first
        if not is_safe_content(user_query):
            raise QueryProcessingError("Potentially harmful content detected in query")

        try:
            logger.info(f"Processing simple query: '{user_query[:100]}{'...' if len(user_query) > 100 else ''}'")

            # Use provided parameters or defaults
            if parameters is None:
                base_params = {
                    "top_k": Config.DEFAULT_TOP_K,
                    "threshold": Config.DEFAULT_THRESHOLD
                }
            else:
                base_params = {
                    "top_k": parameters.top_k,
                    "threshold": parameters.threshold,
                    "filters": parameters.filters
                }

            # Calculate dynamic retrieval parameters based on query characteristics
            retrieval_params = self._calculate_dynamic_retrieval_params(user_query, base_params)

            # Directly call the retrieval tool for simple queries
            retrieval_result = self.retrieval_tool.call_retrieval(
                query=user_query,
                parameters=retrieval_params
            )

            # Detect query type for improved response
            query_type = self._detect_query_type(user_query)

            if not retrieval_result.get("chunks") or len(retrieval_result["chunks"]) == 0:
                # No relevant content found - use OpenRouter to generate response with general knowledge
                prompt = f"""
                You are an expert agent specializing in Physical AI & Humanoid Robotics.
                No textbook context was available for this query, so you should use your general knowledge.

                SYSTEM INSTRUCTION:
                You are an expert agent that answers user questions with strict adherence to the following policy:

                RULES:
                1. If textbook context is provided, answer ONLY using the textbook content - no exceptions.
                2. If textbook context is provided, NEVER mention the textbook, sources, retrieval, chunks, embeddings, RAG, confidence scores, or backend logic.
                3. If no textbook context is provided, use your general knowledge to answer the question naturally.
                4. If no textbook context is provided, NEVER say phrases like "the textbook does not contain", "based on my knowledge", "I could not find", "RAG", "context", etc.
                5. In ALL cases, provide ONLY the final answer in natural language - nothing else.
                6. Do NOT expose any backend logic, retrieval results, similarity scores, thresholds, sources, or system behavior.
                7. Do NOT show fallback messages or internal reasoning.
                8. Do NOT apologize for missing content.
                9. Do NOT mention OpenRouter, models, or APIs.
                10. Answer naturally as a normal AI assistant would, regardless of whether you're using textbook content or general knowledge.
                11. Provide concise but complete answers without unnecessary verbosity or filler phrases.
                12. For {query_type} queries, tailor your response appropriately:
                    - Explanatory: Focus on clear explanations and definitions
                    - Procedural: Provide step-by-step instructions or processes
                    - Conceptual: Explain principles, theories, and relationships
                    - General: Provide balanced and informative responses
                13. Use simple language unless the question is clearly advanced.
                14. Avoid restating the question; answer directly.

                User Query: {user_query}

                Answer the user's query using your general knowledge, following the system instruction above.
                Provide ONLY your final answer in natural language - nothing else.
                """

                # Generate response using OpenRouter with general knowledge
                raw_response = self.client.generate_content(
                    prompt,
                    max_tokens=800,   # Limit response length for simple queries
                    temperature=0.4   # Slightly higher for general knowledge responses
                )
                # Normalize the response to improve quality and consistency
                response_content = self._normalize_response(raw_response)
                sources = []
                confidence_score = 0.2  # Lower confidence for general knowledge responses
            else:
                # Extract sources from the retrieval result
                sources = []
                retrieved_chunks = retrieval_result["chunks"]
                for chunk in retrieved_chunks:
                    metadata = chunk.get("metadata", {})
                    source = Source(
                        url=metadata.get("url", ""),
                        module=metadata.get("module", ""),
                        chapter=metadata.get("chapter", ""),
                        section=metadata.get("section", ""),
                        similarity_score=chunk.get("similarity_score", 0.0),
                        content_preview=chunk["content"][:100] + "...",
                        url_fragment=metadata.get("url_fragment", ""),
                        page_reference=metadata.get("page_reference", "")
                    )
                    sources.append(source)

                # Evaluate retrieval quality to handle partial matches
                is_high_quality, avg_score, usable_chunks = self._evaluate_retrieval_quality(retrieved_chunks)

                if is_high_quality:
                    # Use textbook content to answer the query
                    retrieved_content = [chunk["content"] for chunk in usable_chunks]
                    context = "\n\n".join(retrieved_content)
                    prompt = f"""
                    You are an expert agent specializing in Physical AI & Humanoid Robotics.
                    Answer the user's query based on the provided context from the textbook content.

                    SYSTEM INSTRUCTION:
                    You are an expert agent that answers user questions with strict adherence to the following policy:

                    RULES:
                    1. If textbook context is provided, answer ONLY using the textbook content - no exceptions.
                    2. If textbook context is provided, NEVER mention the textbook, sources, retrieval, chunks, embeddings, RAG, confidence scores, or backend logic.
                    3. If no textbook context is provided, use your general knowledge to answer the question naturally.
                    4. If no textbook context is provided, NEVER say phrases like "the textbook does not contain", "based on my knowledge", "I could not find", "RAG", "context", etc.
                    5. In ALL cases, provide ONLY the final answer in natural language - nothing else.
                    6. Do NOT expose any backend logic, retrieval results, similarity scores, thresholds, sources, or system behavior.
                    7. Do NOT show fallback messages or internal reasoning.
                    8. Do NOT apologize for missing content.
                    9. Do NOT mention OpenRouter, models, or APIs.
                    10. Answer naturally as a normal AI assistant would, regardless of whether you're using textbook content or general knowledge.
                    11. Provide concise but complete answers without unnecessary verbosity or filler phrases.
                    12. For {query_type} queries, tailor your response appropriately:
                        - Explanatory: Focus on clear explanations and definitions
                        - Procedural: Provide step-by-step instructions or processes
                        - Conceptual: Explain principles, theories, and relationships
                        - General: Provide balanced and informative responses
                    13. Use simple language unless the question is clearly advanced.
                    14. Avoid restating the question; answer directly.

                    Context:
                    {context}

                    User Query: {user_query}

                    Answer the user's query based ONLY on the provided textbook context, following the system instruction above.
                    Provide ONLY your final answer in natural language - nothing else.
                    """
                else:
                    # Partial match - use available context but allow for general knowledge integration
                    retrieved_content = [chunk["content"] for chunk in usable_chunks if chunk.get("similarity_score", 0.0) >= 0.3]  # Use any somewhat relevant chunks
                    context = "\n\n".join(retrieved_content)
                    prompt = f"""
                    You are an expert agent specializing in Physical AI & Humanoid Robotics.
                    You have limited context from the textbook that may partially address the user's query.
                    Use the provided context as the primary source, but supplement with your general knowledge if needed to provide a complete answer.

                    SYSTEM INSTRUCTION:
                    You are an expert agent that answers user questions with strict adherence to the following policy:

                    RULES:
                    1. If textbook context is provided, answer ONLY using the textbook content - no exceptions.
                    2. If textbook context is provided, NEVER mention the textbook, sources, retrieval, chunks, embeddings, RAG, confidence scores, or backend logic.
                    3. If no textbook context is provided, use your general knowledge to answer the question naturally.
                    4. If no textbook context is provided, NEVER say phrases like "the textbook does not contain", "based on my knowledge", "I could not find", "RAG", "context", etc.
                    5. In ALL cases, provide ONLY the final answer in natural language - nothing else.
                    6. Do NOT expose any backend logic, retrieval results, similarity scores, thresholds, sources, or system behavior.
                    7. Do NOT show fallback messages or internal reasoning.
                    8. Do NOT apologize for missing content.
                    9. Do NOT mention OpenRouter, models, or APIs.
                    10. Answer naturally as a normal AI assistant would, regardless of whether you're using textbook content or general knowledge.
                    11. Provide concise but complete answers without unnecessary verbosity or filler phrases.
                    12. For {query_type} queries, tailor your response appropriately:
                        - Explanatory: Focus on clear explanations and definitions
                        - Procedural: Provide step-by-step instructions or processes
                        - Conceptual: Explain principles, theories, and relationships
                        - General: Provide balanced and informative responses
                    13. Use simple language unless the question is clearly advanced.
                    14. Avoid restating the question; answer directly.

                    Context:
                    {context}

                    User Query: {user_query}

                    Answer the user's query based on the provided context when available, supplementing with general knowledge if needed, following the system instruction above.
                    Provide ONLY your final answer in natural language - nothing else.
                    """

                # Generate response using OpenRouter
                raw_response = self.client.generate_content(
                    prompt,
                    max_tokens=800,   # Limit response length for simple queries
                    temperature=0.3   # Lower temperature for consistent answers
                )

                # Normalize the response to improve quality and consistency
                response_content = self._normalize_response(raw_response)

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

    def process_simple_query(self, user_query: str) -> AgentResponse:
        """
        Process a simple query without complex reasoning or multiple steps (using default parameters).

        Args:
            user_query: The user's query string

        Returns:
            AgentResponse with the generated answer
        """
        # Call the parameterized version with default parameters
        from .models import QueryParameters
        default_params = QueryParameters()
        return self.process_simple_query_with_parameters(user_query, default_params)


import threading

# Thread-local storage to avoid global state issues
_local = threading.local()


def get_agent() -> Agent:
    """
    Get the agent instance for the current thread, creating it if it doesn't exist.

    Returns:
        Agent instance
    """
    if not hasattr(_local, 'agent'):
        _local.agent = Agent()
    return _local.agent