"""
Context Fetcher Layer - Fetches relevant textbook content from Qdrant
"""
from typing import List, Dict, Any, NamedTuple
from .retrieval_tool import RetrievalTool


class RetrievedChunk(NamedTuple):
    """Represents a retrieved chunk with all necessary metadata"""
    text: str
    module: str
    chapter: str
    section: str
    url: str
    similarity_score: float


class ContextFetcher:
    """Layer responsible for fetching relevant context from Qdrant"""

    def __init__(self):
        """Initialize the context fetcher with retrieval tool"""
        self.retrieval_tool = RetrievalTool()

    def fetch_context(self, query: str, top_k: int = 5, threshold: float = 0.6) -> List[RetrievedChunk]:
        """
        Fetch relevant textbook chunks from Qdrant

        Args:
            query: The user's query
            top_k: Number of top results to retrieve
            threshold: Minimum similarity threshold (raised to 0.6 for strict grounding)

        Returns:
            List of RetrievedChunk objects containing the context
        """
        # Call the retrieval tool with parameters
        retrieval_result = self.retrieval_tool.call_retrieval(
            query=query,
            parameters={
                "top_k": top_k,
                "threshold": threshold
            }
        )

        # Extract chunks from the result
        chunks = retrieval_result.get("chunks", [])

        # Convert to RetrievedChunk objects
        retrieved_chunks = []
        for chunk in chunks:
            metadata = chunk.get('metadata', {})

            retrieved_chunk = RetrievedChunk(
                text=chunk.get('content', ''),
                module=metadata.get('module', 'N/A'),
                chapter=metadata.get('chapter', 'N/A'),
                section=metadata.get('section', 'N/A'),
                url=metadata.get('url', 'N/A'),
                similarity_score=chunk.get('similarity_score', 0.0)
            )

            retrieved_chunks.append(retrieved_chunk)

        return retrieved_chunks

    def evaluate_context_match(self, chunks: List[RetrievedChunk], query: str) -> tuple[str, str]:
        """
        Evaluate if the retrieved context sufficiently answers the query.

        Args:
            chunks: List of retrieved chunks
            query: The original query

        Returns:
            Tuple of (verdict, explanation)
        """
        if not chunks:
            return "NO", "No context retrieved for the query."

        # Check for high similarity chunks (>= 0.6)
        high_similarity_chunks = [chunk for chunk in chunks if chunk.similarity_score >= 0.6]

        if not high_similarity_chunks:
            return "NO", f"Context insufficient or weakly related. Highest similarity score: {max(c.similarity_score for c in chunks):.3f}, required: 0.6"

        # Check for module relevance based on query domain
        module_relevance_warning = self._check_module_relevance(high_similarity_chunks, query)

        # Determine verdict based on number of high-relevance chunks
        if len(high_similarity_chunks) >= 2:
            verdict = "YES"
            explanation = f"Directly answered by textbook. Found {len(high_similarity_chunks)} high-similarity chunks (≥0.6). {module_relevance_warning}"
        elif len(high_similarity_chunks) == 1:
            verdict = "PARTIAL"
            explanation = f"Partially answered by textbook. Found 1 high-similarity chunk ({high_similarity_chunks[0].similarity_score:.3f}). {module_relevance_warning}"
        else:
            verdict = "NO"
            explanation = f"Context insufficient. No chunks meet required similarity threshold of 0.6. {module_relevance_warning}"

        return verdict, explanation

    def _check_module_relevance(self, chunks: List[RetrievedChunk], query: str) -> str:
        """
        Check if retrieved chunks come from relevant modules for the query domain.

        Args:
            chunks: List of high-similarity chunks
            query: The original query

        Returns:
            Warning message if modules are not relevant, empty string otherwise
        """
        query_lower = query.lower()

        # Define expected module mappings
        expected_modules = []
        if any(keyword in query_lower for keyword in ['ros', 'publisher', 'subscriber', 'node', 'topic', 'service']):
            expected_modules.append('Module1')
        if any(keyword in query_lower for keyword in ['vslam', 'perception', 'vision', 'slam']):
            expected_modules.append('Module3')
        if any(keyword in query_lower for keyword in ['vlm', 'vision-language', 'language-action', 'gpt']):
            expected_modules.append('Module4')

        if not expected_modules:
            return ""

        # Count chunks from expected modules
        relevant_chunks = [chunk for chunk in chunks if any(expected in chunk.module.lower() for expected in expected_modules)]

        if not relevant_chunks:
            return f"WARNING: Retrieved chunks may not be from expected module(s): {expected_modules}."

        return ""