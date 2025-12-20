"""
Terminal-based CLI interface for the RAG agent.

This module provides a command-line interface that reuses the existing
agent and retrieval logic without starting the FastAPI server.
"""
import sys
import os
import argparse
from typing import Optional, List
from dotenv import load_dotenv

# Load environment variables
load_dotenv()

from src.agent.core import get_agent
from src.agent.logging import logger, setup_logging
from src.agent.models import AgentResponse
from src.agent.retrieval_tool import RetrievalTool
import time


def print_retrieved_chunks(chunks: List[dict]):
    """
    Print retrieved textbook chunks in a human-readable format.

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
    print("RETRIEVED TEXTBOOK EVIDENCE:")
    print("=" * 80)

    for i, chunk in enumerate(chunks, 1):
        metadata = chunk.get('metadata', {})

        print(f"\nChunk {i}:")
        print(f"  Similarity Score: {chunk.get('similarity_score', 0.0):.4f}")
        print(f"  Module: {metadata.get('module', 'N/A')}")
        print(f"  Chapter: {metadata.get('chapter', 'N/A')}")
        print(f"  Section: {metadata.get('section', 'N/A')}")
        print(f"  URL: {metadata.get('url', 'N/A')}")
        print(f"  Content Preview:")
        print(f"    {chunk.get('content', '')[:500]}{'...' if len(chunk.get('content', '')) > 500 else ''}")
        print("-" * 80)

    print()


def query_agent_with_verification(query: str, debug: bool = False) -> Optional[AgentResponse]:
    """
    Query the agent with verification showing retrieved chunks first.

    Args:
        query: The query text to process
        debug: Whether to print debug information

    Returns:
        AgentResponse containing the answer and sources, or None if error
    """
    try:
        if debug:
            print(f"Debug: Processing query: {query}")

        # Initialize the retrieval tool
        retrieval_tool = RetrievalTool()

        # Run the retrieval pipeline to get textbook chunks
        print("Running retrieval pipeline...")
        start_time = time.time()

        retrieval_result = retrieval_tool.call_retrieval(
            query=query,
            parameters={
                "top_k": 5,  # Get top 5 chunks
                "threshold": 0.3  # Lower threshold to see more results
            }
        )

        retrieval_time = time.time() - start_time
        print(f"Retrieval completed in {retrieval_time:.2f}s")

        # Extract chunks from the result
        chunks = retrieval_result.get("chunks", [])

        # Print the retrieved chunks
        print_retrieved_chunks(chunks)

        # Now get the agent response using the same pipeline
        print("Generating final answer...")
        agent = get_agent()

        start_time = time.time()
        result = agent.process_query(query)  # Use process_query instead of process_simple_query for full functionality
        agent_time = time.time() - start_time

        print("=" * 80)
        print("FINAL ANSWER:")
        print("=" * 80)
        print(result.content)
        print()
        print(f"Processing time: {result.processing_time_ms:.2f}ms")
        print(f"Confidence score: {result.confidence_score:.2f}")

        if result.sources:
            print(f"Sources used: {len(result.sources)}")

        print()
        print(f"Total execution time: {agent_time:.2f}s")

        return result

    except Exception as e:
        print(f"Error processing query: {str(e)}", file=sys.stderr)
        if debug:
            import traceback
            traceback.print_exc()
        return None


def main():
    """Main CLI entry point."""
    parser = argparse.ArgumentParser(
        description="Terminal-based CLI for the RAG agent with verification",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Example usage:
  python -m src.agent.cli "Explain publisher subscriber pattern in ROS 2"
  python -m src.agent.cli --debug "What is a robot operating system?"
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
        help="Enable debug output"
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

    if args.debug:
        print(f"Query: {query}")
        print("-" * 50)

    # Process the query using the new verification function
    result = query_agent_with_verification(query, debug=args.debug)

    if result is None:
        print("Failed to process query. Check your configuration and API keys.", file=sys.stderr)
        sys.exit(1)


if __name__ == "__main__":
    main()