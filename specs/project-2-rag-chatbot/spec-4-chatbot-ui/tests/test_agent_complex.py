"""
Complex query tests for the agent functionality.

This module tests the complex query processing capabilities of the agent layer,
focusing on multi-step reasoning and comparative queries.
"""
import pytest
import os
from unittest.mock import Mock, patch, MagicMock
from backend.src.agent.core import Agent
from backend.src.agent.models import QueryRequest, ChatRequest
from backend.src.agent.validation_utils import validate_tool_call_count


@patch('backend.src.agent.core.OpenAI')
@patch('backend.src.agent.retrieval_tool.RetrievalTool')
def test_complex_query_decomposition(mock_retrieval_tool, mock_openai_client):
    """Test that complex queries are properly decomposed."""
    # Mock the OpenAI client and retrieval tool
    mock_assistant = Mock()
    mock_assistant.id = "test-assistant-id"
    mock_openai_client.return_value.beta.assistants.create.return_value = mock_assistant
    
    mock_retrieval_tool_instance = Mock()
    
    # Mock the call_retrieval method to return test data
    mock_retrieval_result = {
        "chunks": [
            {
                "id": "chunk1",
                "content": "Information about PID controllers",
                "similarity_score": 0.85,
                "rank": 0,
                "metadata": {
                    "url": "https://textbook.example.com/module3/chapter2",
                    "module": "Module 3: Control Systems",
                    "chapter": "PID Controllers",
                    "section": "Basic Principles",
                    "chunk_index": 1,
                    "hash": "hash123"
                }
            }
        ],
        "execution_time_ms": 100.0,
        "retrieval_timestamp": "2025-12-15T10:30:00Z",
        "total_chunks_found": 1,
        "search_parameters": {"k": 5, "threshold": 0.6}
    }
    mock_retrieval_tool_instance.call_retrieval.return_value = mock_retrieval_result
    mock_retrieval_tool.return_value = mock_retrieval_tool_instance
    
    # Set environment variable for API key
    os.environ['OPENAI_API_KEY'] = 'test-key'
    
    # Create agent instance
    agent = Agent()
    
    # Test comparison query decomposition
    comparison_query = "Compare PID and MPC control"
    decomposed = agent._decompose_query(comparison_query)
    assert len(decomposed) == 2
    assert "PID" in decomposed[0]
    assert "MPC" in decomposed[1]
    
    # Test "difference between" query decomposition
    diff_query = "What is the difference between forward and inverse kinematics?"
    decomposed = agent._decompose_query(diff_query)
    assert len(decomposed) == 2
    assert "forward" in decomposed[0].lower()
    assert "inverse" in decomposed[1].lower()
    
    # Test simple query (no decomposition needed)
    simple_query = "What is a robot?"
    decomposed = agent._decompose_query(simple_query)
    assert len(decomposed) == 1
    assert decomposed[0] == simple_query


@patch('backend.src.agent.core.OpenAI')
@patch('backend.src.agent.retrieval_tool.RetrievalTool')
def test_complex_query_processing(mock_retrieval_tool, mock_openai_client):
    """Test processing of complex queries."""
    # Mock the OpenAI client and retrieval tool
    mock_assistant = Mock()
    mock_assistant.id = "test-assistant-id"
    mock_openai_client.return_value.beta.assistants.create.return_value = mock_assistant
    
    mock_retrieval_tool_instance = Mock()
    
    # Mock the call_retrieval method to return test data
    mock_retrieval_result = {
        "chunks": [
            {
                "id": "chunk1",
                "content": "Detailed information about the first concept",
                "similarity_score": 0.90,
                "rank": 0,
                "metadata": {
                    "url": "https://textbook.example.com/module1/chapter1",
                    "module": "Module 1: Basics",
                    "chapter": "Introduction",
                    "section": "Concepts",
                    "chunk_index": 1,
                    "hash": "hash123"
                }
            },
            {
                "id": "chunk2",
                "content": "Detailed information about the second concept",
                "similarity_score": 0.85,
                "rank": 1,
                "metadata": {
                    "url": "https://textbook.example.com/module1/chapter2",
                    "module": "Module 1: Basics",
                    "chapter": "Advanced Topics",
                    "section": "Concepts",
                    "chunk_index": 2,
                    "hash": "hash456"
                }
            }
        ],
        "execution_time_ms": 150.0,
        "retrieval_timestamp": "2025-12-15T10:30:00Z",
        "total_chunks_found": 2,
        "search_parameters": {"k": 5, "threshold": 0.6}
    }
    mock_retrieval_tool_instance.call_retrieval.return_value = mock_retrieval_result
    mock_retrieval_tool.return_value = mock_retrieval_tool_instance
    
    # Set environment variable for API key
    os.environ['OPENAI_API_KEY'] = 'test-key'
    
    # Create agent instance
    agent = Agent()
    
    # Test complex query processing
    complex_query = "Compare the structure of a robotic arm and a humanoid leg"
    response = agent.process_complex_query(complex_query)
    
    assert response is not None
    assert response.content is not None
    assert len(response.sources) > 0
    assert response.confidence_score >= 0.0 and response.confidence_score <= 1.0


@patch('backend.src.agent.core.OpenAI')
@patch('backend.src.agent.retrieval_tool.RetrievalTool')
def test_synthesis_from_multiple_sources(mock_retrieval_tool, mock_openai_client):
    """Test that responses are properly synthesized from multiple sources."""
    # Mock the OpenAI client and retrieval tool
    mock_assistant = Mock()
    mock_assistant.id = "test-assistant-id"
    mock_openai_client.return_value.beta.assistants.create.return_value = mock_assistant
    
    mock_retrieval_tool_instance = Mock()
    
    # Set environment variable for API key
    os.environ['OPENAI_API_KEY'] = 'test-key'
    
    # Create agent instance
    agent = Agent()
    
    # Create test chunks from different modules
    from backend.src.agent.models import RetrievedChunk, Metadata
    chunk1 = RetrievedChunk(
        id="chunk1",
        content="Information about the first concept",
        similarity_score=0.9,
        rank=0,
        metadata=Metadata(
            url="https://textbook.example.com/module1/chapter1",
            module="Module 1: Basics",
            chapter="Introduction",
            section="Concepts",
            chunk_index=1,
            hash="hash123"
        )
    )
    
    chunk2 = RetrievedChunk(
        id="chunk2",
        content="Information about the second concept",
        similarity_score=0.8,
        rank=1,
        metadata=Metadata(
            url="https://textbook.example.com/module2/chapter1",
            module="Module 2: Advanced",
            chapter="Complex Topics",
            section="Concepts",
            chunk_index=1,
            hash="hash456"
        )
    )
    
    # Test synthesis function
    result = agent._synthesize_response_from_multiple_sources(
        "Compare concept A and concept B",
        [chunk1, chunk2],
        2
    )
    
    assert "Module 1: Basics" in result
    assert "Module 2: Advanced" in result
    assert "concept A and concept B" in result
    assert "Information about the first concept" in result
    assert "Information about the second concept" in result


def test_tool_call_limit_validation():
    """Test that tool call count is properly validated."""
    # Test with valid number of calls (under limit)
    mock_tool_calls = [Mock() for _ in range(3)]  # 3 calls
    is_valid = validate_tool_call_count(mock_tool_calls, max_calls=5)
    assert is_valid

    # Test with too many calls
    mock_tool_calls = [Mock() for _ in range(6)]  # 6 calls
    is_valid = validate_tool_call_count(mock_tool_calls, max_calls=5)
    assert not is_valid


if __name__ == "__main__":
    pytest.main([__file__])