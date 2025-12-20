#!/usr/bin/env python3
"""
Main entrypoint for the retrieval pipeline of the RAG Chatbot system.
This module orchestrates the complete process of query ingestion, embedding generation,
similarity search, and result retrieval.
"""

import sys
import os
from pathlib import Path

# Add the backend/src to the path so we can import modules
sys.path.insert(0, str(Path(__file__).parent / "backend" / "src"))

def main():
    """Main function to run the retrieval pipeline"""
    try:
        from src.retrieval.config import Config
        from src.retrieval.retrieval_pipeline import RetrievalPipeline
        from src.retrieval.logging import setup_logging
        
        logger = setup_logging()
        
        # Validate configuration
        config_errors = Config.validate()
        if config_errors:
            logger.error(f"Configuration validation errors: {', '.join(config_errors)}")
            print(f"ERROR: Configuration validation failed: {', '.join(config_errors)}")
            print("Please set the required environment variables in your .env file.")
            return False
        
        # Initialize the pipeline
        print("Initializing retrieval pipeline...")
        pipeline = RetrievalPipeline()
        
        # Set up the pipeline
        if not pipeline.setup():
            logger.error("Failed to set up the retrieval pipeline")
            print("ERROR: Failed to set up the retrieval pipeline")
            return False
        
        print("Retrieval pipeline initialized successfully!")
        
        # Example query - in a real implementation this would come from user input
        sample_queries = [
            "What is a PID controller?",
            "Explain ROS 2 architecture", 
            "How does robot localization work?",
            "Describe the components of a robotic nervous system"
        ]
        
        print(f"\nRunning sample queries: {sample_queries}")
        
        for i, query in enumerate(sample_queries):
            print(f"\n{i+1}. Processing query: '{query}'")
            
            try:
                # Perform retrieval
                result = pipeline.retrieve(
                    query=query,
                    k=3,  # Return top 3 results
                    threshold=0.5  # Minimum similarity threshold
                )
                
                print(f"   Found {len(result.chunks)} results:")
                for j, chunk in enumerate(result.chunks):
                    print(f"     {j+1}. Score: {chunk.similarity_score:.3f}")
                    print(f"        Content: {chunk.content[:100]}...")
                    print(f"        Source: {chunk.metadata.url}")
                
            except Exception as e:
                logger.error(f"Error processing query '{query}': {str(e)}")
                print(f"   Error processing query: {str(e)}")
        
        print(f"\nRetrieval pipeline execution completed!")
        return True
        
    except ImportError as e:
        print(f"ERROR: Failed to import required modules: {e}")
        print("Make sure all dependencies are installed and paths are set correctly.")
        return False
        
    except Exception as e:
        print(f"ERROR: Unexpected error in main: {str(e)}")
        return False


if __name__ == "__main__":
    success = main()
    if not success:
        print("\nPipeline execution failed.")
        sys.exit(1)
    else:
        print("\nPipeline execution completed successfully.")
        sys.exit(0)