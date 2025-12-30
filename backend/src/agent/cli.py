"""
Terminal-based CLI interface for the RAG agent.

This module provides a command-line interface that implements the three-layer architecture:
1) FETCH LAYER - Retrieves context from Qdrant
2) REASONING LAYER - Uses OpenRouter to reason over context
3) TERMINAL OUTPUT LAYER - Formats and displays results
"""
import sys
import os
import argparse
from typing import Optional
from dotenv import load_dotenv

# Load environment variables
load_dotenv()

from .context_fetcher import ContextFetcher, RetrievedChunk
from .context_reasoner import ContextReasoner, ReasoningResult
from .logging import logger, setup_logging
import time


def format_fetched_context(chunks: list[RetrievedChunk]):
    """
    Format and print the fetched textbook context in human-readable format.

    Args:
        chunks: List of retrieved chunks with metadata
    """
    if not chunks:
        print("=" * 80)
        print("NO TEXTBOOK CONTEXT FOUND – ANSWER MAY BE GENERAL KNOWLEDGE")
        print("=" * 80)
        print()
        return

    print("=" * 80)
    print("FETCHED TEXTBOOK CONTEXT:")
    print("=" * 80)

    for i, chunk in enumerate(chunks, 1):
        print(f"\nChunk {i}:")
        print(f"  Module: {chunk.module}")
        print(f"  Chapter: {chunk.chapter}")
        print(f"  Section: {chunk.section}")
        print(f"  Similarity Score: {chunk.similarity_score:.4f}")
        print(f"  Source URL: {chunk.url}")
        print(f"  Content Preview:")
        print(f"    {chunk.text[:500]}{'...' if len(chunk.text) > 500 else ''}")
        print("-" * 80)

    print()


def analyze_chunk_match(query: str, chunk: RetrievedChunk) -> str:
    """
    Analyze how well a chunk matches the query.

    Args:
        query: The original query
        chunk: The retrieved chunk

    Returns:
        Analysis string explaining the match
    """
    query_keywords = set(query.lower().split())
    chunk_text_lower = chunk.text.lower()

    # Find overlapping keywords
    chunk_words = set(chunk_text_lower.split())
    common_keywords = query_keywords.intersection(chunk_words)

    if len(common_keywords) > 0:
        match_status = "HIGH" if chunk.similarity_score >= 0.6 else "LOW"
        overlap_info = f"Concept overlap: {match_status} - Found keywords: {list(common_keywords)[:5]}"  # Limit to first 5
    else:
        overlap_info = "Concept overlap: NONE - No matching keywords found"

    return f"- Query keywords: {len(query_keywords)}\n- Found in chunk text: {len(common_keywords)}\n- {overlap_info}\n- Score justification: Similarity score {chunk.similarity_score:.3f}"


def print_context_analysis(query: str, chunks: list[RetrievedChunk]):
    """
    Print match analysis for each retrieved chunk.

    Args:
        query: The original query
        chunks: List of retrieved chunks
    """
    if not chunks:
        return

    print("MATCH ANALYSIS:")
    for i, chunk in enumerate(chunks, 1):
        print(f"  Chunk {i}:")
        analysis = analyze_chunk_match(query, chunk)
        for line in analysis.split('\n'):
            print(f"    {line}")
        print()
    print()


def print_context_verdict(chunks: list[RetrievedChunk], query: str) -> str:
    """
    Print the context match verdict based on strict evaluation and return the status.

    Args:
        chunks: List of retrieved chunks
        query: The original query

    Returns:
        Verdict status ("YES", "PARTIAL", or "NO")
    """
    # Use the context fetcher to evaluate the match
    fetcher = ContextFetcher()
    verdict, explanation = fetcher.evaluate_context_match(chunks, query)

    print("CONTEXT VERDICT:")
    if verdict == "YES":
        print(f"YES – {explanation}")
    elif verdict == "PARTIAL":
        print(f"PARTIAL – {explanation}")
    else:
        print(f"NO – {explanation}")
    print()
    return verdict


def main():
    """Main CLI entry point implementing the three-layer architecture."""
    parser = argparse.ArgumentParser(
        description="Terminal-based CLI for RAG agent with three-layer architecture",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Example usage:
  python -m src.agent.cli "Explain publisher subscriber pattern in ROS 2"          # Chatbot mode (default)
  python -m src.agent.cli "What are ROS nodes and topics?" --debug               # Debug mode
        """
    )
    parser.add_argument(
        "query",
        nargs="?",
        help="The query to process. If not provided, will prompt for input."
    )
    parser.add_argument(
        "--debug",
        action="store_true",
        help="Enable debug output mode showing all intermediate steps (default: chatbot mode)"
    )

    args = parser.parse_args()

    # Get query from command line or prompt user
    if args.query:
        query = args.query
    else:
        query = input("Enter your query: ")

    if not query.strip():
        print("Error: Query cannot be empty", file=sys.stderr)
        sys.exit(1)

    # Determine output mode
    debug_mode = args.debug

    if debug_mode:
        print(f"USER QUERY: {query}")
        print()

    try:
        # LAYER 1: FETCH LAYER - Fetch context from Qdrant
        if debug_mode:
            print("1) FETCH LAYER: Retrieving context from Qdrant...")
        fetch_start_time = time.time()
        context_fetcher = ContextFetcher()
        retrieved_chunks = context_fetcher.fetch_context(query=query, top_k=5, threshold=0.6)  # Increased threshold to 0.6 for stricter grounding
        fetch_time = time.time() - fetch_start_time
        if debug_mode:
            print(f"   [SUCCESS] Retrieved {len(retrieved_chunks)} chunks in {fetch_time:.2f}s")
            print()

            # Display the fetched context
            format_fetched_context(retrieved_chunks)

            # Print match analysis
            print_context_analysis(query, retrieved_chunks)

            # Print context verdict
            verdict = print_context_verdict(retrieved_chunks, query)
        else:
            # In chatbot mode, we still need the verdict for internal logic
            verdict, _ = context_fetcher.evaluate_context_match(retrieved_chunks, query)

        # Store fetch_time in a variable accessible in both modes
        total_fetch_time = fetch_time

        # LAYER 2: REASONING LAYER - Use OpenRouter to reason over context
        if debug_mode:
            print("2) REASONING LAYER: Processing with OpenRouter model...")
        reasoning_start_time = time.time()
        context_reasoner = ContextReasoner()

        # Get the verdict for internal logic
        if debug_mode:
            verdict, _ = context_fetcher.evaluate_context_match(retrieved_chunks, query)
            reasoning_result = context_reasoner.reason_over_context(query, retrieved_chunks, verdict)
        else:
            verdict, _ = context_fetcher.evaluate_context_match(retrieved_chunks, query)
            reasoning_result = context_reasoner.reason_over_context(query, retrieved_chunks, verdict)

        reasoning_time = time.time() - reasoning_start_time
        if debug_mode:
            print(f"   [SUCCESS] Reasoning completed in {reasoning_time:.2f}s")
            print()

        # LAYER 3: TERMINAL OUTPUT LAYER - Display final results
        if debug_mode:
            print("3) TERMINAL OUTPUT LAYER:")
            print("=" * 80)
            print("FINAL ANSWER:")
            print("=" * 80)

        # Output the answer based on mode
        # Check if there was an API error (like quota exceeded)
        if reasoning_result and ("exceeded your current quota" in reasoning_result.answer.lower() or
                                 "quota exceeded" in reasoning_result.answer.lower() or
                                 "rate limit" in reasoning_result.answer.lower()):
            if debug_mode:
                print(reasoning_result.answer)
            else:
                # In chatbot mode, show a clean error message
                print("I'm temporarily unable to generate an answer right now. Please try again.")
        else:
            if debug_mode:
                print(reasoning_result.answer)
                print()

                # Determine answer source based on verdict and reasoning
                if reasoning_result.is_from_textbook and verdict == "YES":
                    answer_source = "TEXTBOOK"
                elif reasoning_result.is_from_textbook and verdict == "PARTIAL":
                    answer_source = "PARTIAL"
                else:
                    answer_source = "GENERAL"

                print(f"Answer Source: {answer_source}")
                print(f"Confidence Score: {reasoning_result.confidence_score:.2f}")
                print(f"Processing Notes: {reasoning_result.reasoning_notes}")

                if answer_source == "GENERAL":
                    print("\nDISCLAIMER: Answer may contain general knowledge not present in textbook.")

                print("=" * 80)
                print(f"\nTotal execution time: {total_fetch_time + reasoning_time:.2f}s")

                # UI-ready structured output (for future Spec-4 implementation)
                print("\nUI-READY STRUCTURE:")
                print(f"- Query: {query}")
                print(f"- Verdict: {verdict}")
                print(f"- Confidence: {reasoning_result.confidence_score:.2f}")
                print(f"- Matched Chunks: {len(retrieved_chunks)}")
                print(f"- Answer Source: {answer_source}")
            else:
                # CHATBOT MODE: Only print the final answer
                print(reasoning_result.answer)

    except Exception as e:
        if args.debug:
            print(f"Error processing query: {str(e)}", file=sys.stderr)
            import traceback
            traceback.print_exc()
        else:
            # In chatbot mode, don't expose errors to user
            print("I'm temporarily unable to generate an answer right now. Please try again.")
        sys.exit(1)


if __name__ == "__main__":
    main()