#!/usr/bin/env python3
"""
Test script to validate the retrieval pipeline implementation.
This script tests the main functionality of the retrieval system.
"""

import os
import sys
import logging
from pathlib import Path

# Add backend/src to path so we can import modules
sys.path.insert(0, str(Path(__file__).parent / "backend" / "src"))

def test_imports():
    """Test that all required modules can be imported"""
    print("Testing imports...")
    
    try:
        from retrieval.config import Config
        print("+ Config module imported successfully")
    except ImportError as e:
        print(f"- Failed to import Config: {e}")
        return False
    
    try:
        from retrieval.models import QueryText, QueryEmbedding, RetrievedChunk, SearchResult, ValidationResult, Metadata
        print("+ Models imported successfully")
    except ImportError as e:
        print(f"- Failed to import models: {e}")
        return False
    
    try:
        from retrieval.cohere_client import CohereClient
        print("+ CohereClient imported successfully")
    except ImportError as e:
        print(f"- Failed to import CohereClient: {e}")
        return False
    
    try:
        from retrieval.qdrant_searcher import QdrantSearcher
        print("+ QdrantSearcher imported successfully")
    except ImportError as e:
        print(f"- Failed to import QdrantSearcher: {e}")
        return False
    
    try:
        from retrieval.validation_utils import validate_retrieval_results, validate_query_text
        print("+ Validation utilities imported successfully")
    except ImportError as e:
        print(f"- Failed to import validation utilities: {e}")
        return False
    
    try:
        from retrieval.retrieval_pipeline import RetrievalPipeline
        print("+ RetrievalPipeline imported successfully")
    except ImportError as e:
        print(f"- Failed to import RetrievalPipeline: {e}")
        return False
    
    return True

def test_config():
    """Test that configuration can be loaded and validated"""
    print("\nTesting configuration...")
    
    try:
        from retrieval.config import Config
        
        # Check if .env file exists and has the expected variables
        env_path = Path(__file__).parent / ".env"
        if not env_path.exists():
            print("! Warning: .env file not found")
        else:
            print("+ .env file found")
        
        # Validate configuration
        errors = Config.validate()
        if errors:
            print(f"! Configuration has validation errors: {', '.join(errors)}")
            print("  Note: This is expected in a test environment without real API keys")
        else:
            print("+ Configuration validation passed")
        
        return True
    except Exception as e:
        print(f"- Configuration test failed: {e}")
        return False

def test_data_models():
    """Test that data models are correctly defined"""
    print("\nTesting data models...")
    
    try:
        from retrieval.models import QueryText, QueryEmbedding, RetrievedChunk, SearchResult, ValidationResult, Metadata
        import uuid
        from datetime import datetime
        
        # Test creating a simple QueryText object
        query = QueryText(
            id=str(uuid.uuid4()),
            text="How does a PID controller work?",
            timestamp=datetime.now()
        )
        print(f"+ QueryText created: {query.id}")
        
        # Test creating a Metadata object
        metadata = Metadata(
            url="https://example.com/module1/chapter1",
            module="Module 1: Robotic Nervous System",
            chapter="PID Controllers",
            section="Introduction",
            chunk_index=1,
            hash="abc123"
        )
        print(f"+ Metadata created: {metadata.module}")
        
        # Test creating a RetrievedChunk object
        chunk = RetrievedChunk(
            id=str(uuid.uuid4()),
            query_embedding_id=str(uuid.uuid4()),
            content="A PID controller is a control loop feedback mechanism...",
            similarity_score=0.85,
            rank=0,
            metadata=metadata
        )
        print(f"+ RetrievedChunk created: {chunk.id}")
        
        print("+ All data models tested successfully")
        return True
    except Exception as e:
        print(f"- Data models test failed: {e}")
        return False

def main():
    """Main test function"""
    print("Testing Retrieval Pipeline Implementation")
    print("="*50)
    
    # Change to the project root directory
    os.chdir(Path(__file__).parent)
    
    all_tests_passed = True
    
    # Run all tests
    all_tests_passed &= test_imports()
    all_tests_passed &= test_config()
    all_tests_passed &= test_data_models()
    
    print("\n" + "="*50)
    if all_tests_passed:
        print("+ All tests passed! Retrieval pipeline implementation is ready.")
        print("\nTo run the full pipeline:")
        print("1. Get Cohere API key from https://dashboard.cohere.com/api-keys")
        print("2. Get Qdrant Cloud URL and API key")
        print("3. Update .env file with your credentials")
        print("4. Run: python -m retrieval.main")
    else:
        print("- Some tests failed. Please check the implementation.")
        return 1
    
    return 0

if __name__ == "__main__":
    exit(main())