"""
Context Reasoner Layer - Uses OpenRouter to reason over fetched context
"""
from .openrouter_client import OpenRouterClient
from typing import List, Optional, NamedTuple
from .context_fetcher import RetrievedChunk
from .config import Config
from .logging import logger
import time


class ReasoningResult(NamedTuple):
    """Result of the reasoning process"""
    answer: str
    is_from_textbook: bool
    confidence_score: float
    reasoning_notes: str


class ContextReasoner:
    """Layer responsible for reasoning over context using OpenRouter"""

    def __init__(self):
        """Initialize the context reasoner with OpenRouter client"""
        # Initialize OpenRouter client
        self.client = OpenRouterClient()

    def _get_system_instruction(self) -> str:
        """
        Get the system instruction to ensure proper reasoning

        Returns:
            System instruction string
        """
        return """
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
        """

    def reason_over_context(self, query: str, context_chunks: List[RetrievedChunk], verdict: str) -> ReasoningResult:
        """
        Reason over the provided context to generate an answer, with fallback to general knowledge if needed.

        Args:
            query: The user's query
            context_chunks: List of context chunks retrieved from Qdrant
            verdict: The context match verdict (YES/PARTIAL/NO)

        Returns:
            ReasoningResult with answer, confidence, and other metadata
        """
        start_time = time.time()

        # If context is available (YES or PARTIAL), try to answer using textbook content
        if verdict in ["YES", "PARTIAL"] and context_chunks:
            return self._reason_with_context(query, context_chunks, start_time)
        else:
            # If no context (NO verdict), fall back to general knowledge
            return self._reason_without_context(query, start_time)

    def _reason_with_context(self, query: str, context_chunks: List[RetrievedChunk], start_time: float) -> ReasoningResult:
        """
        Generate answer using textbook context.

        Args:
            query: The user's query
            context_chunks: List of context chunks from textbook
            start_time: Start time for calculating processing time

        Returns:
            ReasoningResult with answer from textbook content
        """
        # Format the context for the model
        formatted_context = self._format_context_for_model(context_chunks)

        # Create the prompt for the model using textbook content
        if formatted_context.strip():
            # If context is available, answer using it without mentioning sources
            prompt = f"""
            SYSTEM INSTRUCTION: {self._get_system_instruction()}

            USER QUERY: {query}

            TEXTBOOK CONTEXT:
            {formatted_context}

            Answer the user's query based ONLY on the provided textbook context, following the system instruction above.
            Provide ONLY your final answer in natural language - nothing else.
            """
        else:
            # If no context is available, we should not use this method, but if we do, fall back
            prompt = f"""
            SYSTEM INSTRUCTION: {self._get_system_instruction()}

            USER QUERY: {query}

            No textbook context was provided.

            Answer the user's query using your general knowledge, following the system instruction above.
            Provide ONLY your final answer in natural language - nothing else.
            """

        try:
            # Generate content using OpenRouter based on textbook context
            answer = self.client.generate_content(
                prompt,
                max_tokens=2000,
                temperature=0.3   # Lower temperature for more consistent, fact-based responses
            )

            # Calculate confidence based on similarity scores and context availability
            confidence_score = self._calculate_confidence(context_chunks, True)

            # Create reasoning notes
            reasoning_notes = f"Processed {len(context_chunks)} textbook context chunks in {time.time() - start_time:.2f}s"

            logger.info(f"Textbook-based reasoning completed in {time.time() - start_time:.2f}s")

            return ReasoningResult(
                answer=answer,
                is_from_textbook=True,
                confidence_score=confidence_score,
                reasoning_notes=reasoning_notes
            )

        except Exception as e:
            logger.error(f"Error during textbook reasoning: {str(e)}")
            # Check if this is a quota exceeded error
            error_str = str(e).lower()
            if "quota" in error_str or "exceeded" in error_str or "rate limit" in error_str or "rate" in error_str:
                # Return a clean fallback message for quota errors
                return ReasoningResult(
                    answer="I'm temporarily unable to answer right now. Please try again shortly.",
                    is_from_textbook=False,
                    confidence_score=0.0,
                    reasoning_notes=f"Quota exceeded error: {str(e)}"
                )
            else:
                # If textbook reasoning fails for other reasons, fall back to general knowledge
                return self._reason_without_context(query, start_time)

    def _reason_without_context(self, query: str, start_time: float) -> ReasoningResult:
        """
        Generate answer using general knowledge when no textbook context is available.

        Args:
            query: The user's query
            start_time: Start time for calculating processing time

        Returns:
            ReasoningResult with answer from general knowledge
        """
        # Create a prompt for general knowledge response using the strict system instruction
        prompt = f"""
        SYSTEM INSTRUCTION: {self._get_system_instruction()}

        USER QUERY: {query}

        No textbook context was provided.

        Answer the user's query using your general knowledge, following the system instruction above.
        Provide ONLY your final answer in natural language - nothing else.
        """

        try:
            # Generate content using OpenRouter based on general knowledge
            answer = self.client.generate_content(
                prompt,
                max_tokens=2000,
                temperature=0.4   # Slightly higher for more creative responses when needed
            )

            # Calculate low confidence since this is general knowledge
            confidence_score = 0.2  # Lower confidence for general knowledge responses

            # Create reasoning notes
            reasoning_notes = f"Fallback to general knowledge, processed in {time.time() - start_time:.2f}s"

            logger.info(f"General knowledge reasoning completed in {time.time() - start_time:.2f}s")

            return ReasoningResult(
                answer=answer,
                is_from_textbook=False,
                confidence_score=confidence_score,
                reasoning_notes=reasoning_notes
            )

        except Exception as e:
            logger.error(f"Error during general knowledge reasoning: {str(e)}")
            # Check if this is a quota exceeded error
            error_str = str(e).lower()
            if "quota" in error_str or "exceeded" in error_str or "rate limit" in error_str or "rate" in error_str:
                # Return a clean fallback message for quota errors
                return ReasoningResult(
                    answer="I'm temporarily unable to answer right now. Please try again shortly.",
                    is_from_textbook=False,
                    confidence_score=0.0,
                    reasoning_notes=f"Quota exceeded error: {str(e)}"
                )
            else:
                # Return a safe fallback response when all else fails
                return ReasoningResult(
                    answer="I'm temporarily unable to generate an answer right now. Please try again shortly.",
                    is_from_textbook=False,
                    confidence_score=0.0,
                    reasoning_notes=f"Error occurred during reasoning: {str(e)}"
                )

    def _extract_model_verdict(self, answer: str) -> str:
        """
        Extract the verdict from the model's response.

        Args:
            answer: The full answer from the model

        Returns:
            Verdict string: "YES", "PARTIAL", or "NO"
        """
        answer_lower = answer.lower()

        if "yes" in answer_lower and not ("partial" in answer_lower or "no" in answer_lower):
            return "YES"
        elif "partial" in answer_lower:
            return "PARTIAL"
        elif "no" in answer_lower and not ("partial" in answer_lower or "yes" in answer_lower):
            return "NO"
        else:
            # Default to NO if no clear verdict is found
            return "NO"

    def _format_context_for_model(self, context_chunks: List[RetrievedChunk]) -> str:
        """
        Format the context chunks for the model

        Args:
            context_chunks: List of retrieved context chunks

        Returns:
            Formatted string of context
        """
        if not context_chunks:
            return ""

        formatted_context = []
        for i, chunk in enumerate(context_chunks, 1):
            chunk_text = f"""
Chunk {i}:
Module: {chunk.module}
Chapter: {chunk.chapter}
Section: {chunk.section}
URL: {chunk.url}
Similarity Score: {chunk.similarity_score:.4f}

Content:
{chunk.text}

---
"""
            formatted_context.append(chunk_text)

        return "\n".join(formatted_context)

    def _calculate_confidence(self, context_chunks: List[RetrievedChunk], is_from_textbook: bool) -> float:
        """
        Calculate confidence score based on context quality

        Args:
            context_chunks: List of context chunks
            is_from_textbook: Whether the answer is from textbook

        Returns:
            Confidence score between 0.0 and 1.0
        """
        if not is_from_textbook or not context_chunks:
            return 0.1  # Very low confidence if no textbook content

        # Calculate average similarity score
        avg_similarity = sum(chunk.similarity_score for chunk in context_chunks) / len(context_chunks) if context_chunks else 0.0

        # Adjust based on number of chunks (more relevant chunks = higher confidence)
        num_chunks = len(context_chunks)
        chunk_factor = min(num_chunks / 5.0, 0.5)  # Up to 0.5 for multiple chunks

        # Combine factors
        confidence = (avg_similarity * 0.7) + (chunk_factor * 0.3)

        return min(confidence, 1.0)  # Cap at 1.0