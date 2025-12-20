"""
Terminal-based CLI interface for the RAG agent.

This module provides a command-line interface that reuses the existing
agent and retrieval logic without starting the FastAPI server.
"""
import sys
import os
import argparse
from typing import Optional
from dotenv import load_dotenv

# Load environment variables
load_dotenv()

from src.agent.core import get_agent
from src.agent.logging import logger, setup_logging
from src.agent.models import AgentResponse


def query_agent(query: str, debug: bool = False) -> Optional[AgentResponse]:
    """
    Query the agent using the existing agent logic.

    Args:
        query: The query text to process
        debug: Whether to print debug information

    Returns:
        QueryProcessingResult containing the answer and sources, or None if error
    """
    try:
        if debug:
            print(f"Debug: Processing query: {query}")

        # Get the agent instance using the existing factory function
        agent = get_agent()

        if debug:
            print("Debug: Agent initialized successfully")

        # Process the query using the agent's existing logic
        result = agent.process_simple_query(query)

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
        description="Terminal-based CLI for the RAG agent",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Example usage:
  python -m src.agent.cli "Explain ROS 2 and its role in humanoid robotics"
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

    # Process the query using the existing agent logic
    result = query_agent(query, debug=args.debug)

    if result is None:
        print("Failed to process query. Check your configuration and API keys.", file=sys.stderr)
        sys.exit(1)

    # Print the results
    print(query)
    print("-" * 50)
    print(result.content)

    if args.debug and result.sources:
        print(f"\nDebug: Found {len(result.sources)} sources:")
        for i, source in enumerate(result.sources, 1):
            print(f"  Source {i}: {source.url} (similarity: {source.similarity_score:.3f})")


if __name__ == "__main__":
    main()