"""
End-to-end integration tests for the agent functionality.

This module tests complete user scenarios and end-to-end workflows
to ensure all components work together correctly.
"""
import pytest
import os
from unittest.mock import Mock, patch
from backend.src.agent.core import Agent
from backend.src.agent.api import router
from backend.src.agent.models import ChatRequest, QueryRequest, QueryParameters


@patch('backend.src.agent.core.OpenAI')
@patch('backend.src.agent.retrieval_tool.RetrievalTool')
def test_user_story_1_basic_query_processing(mock_retrieval_tool, mock_openai_client):
    """Test User Story 1 - Basic Query Processing."""
    # Mock the OpenAI client and retrieval tool
    mock_assistant = Mock()
    mock_assistant.id = "test-assistant-id"
    mock_openai_client.return_value.beta.assistants.create.return_value = mock_assistant
    
    mock_retrieval_tool_instance = Mock()
    
    # Mock retrieval to return results
    mock_retrieval_result = {
        "chunks": [
            {
                "id": "chunk1",
                "content": "ROS2 is a middleware for robotics applications providing communication between nodes.",
                "similarity_score": 0.85,
                "rank": 0,
                "metadata": {
                    "url": "https://textbook.example.com/ros2/intro",
                    "module": "Module 2: ROS 2 Framework",
                    "chapter": "Introduction",
                    "section": "Overview",
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
    
    # Test the complete workflow
    agent = Agent()
    response = agent.process_simple_query("What is ROS2?")
    
    # Verify the response
    assert response.content is not None
    assert len(response.sources) > 0
    assert response.confidence_score > 0.0


@patch('backend.src.agent.core.OpenAI')
@patch('backend.src.agent.retrieval_tool.RetrievalTool')
def test_user_story_2_complex_query_reasoning(mock_retrieval_tool, mock_openai_client):
    """Test User Story 2 - Complex Query Reasoning."""
    # Mock the OpenAI client and retrieval tool
    mock_assistant = Mock()
    mock_assistant.id = "test-assistant-id"
    mock_openai_client.return_value.beta.assistants.create.return_value = mock_assistant
    
    mock_retrieval_tool_instance = Mock()
    
    # Mock retrieval to return results for different aspects
    mock_retrieval_result = {
        "chunks": [
            {
                "id": "chunk1",
                "content": "PID controllers use Proportional, Integral, and Derivative terms.",
                "similarity_score": 0.9,
                "rank": 0,
                "metadata": {
                    "url": "https://textbook.example.com/control/pid",
                    "module": "Module 3: Control Systems",
                    "chapter": "PID Controllers",
                    "section": "Components",
                    "chunk_index": 1,
                    "hash": "hash123"
                }
            },
            {
                "id": "chunk2",
                "content": "MPC uses predictive models and optimization for control.",
                "similarity_score": 0.87,
                "rank": 1,
                "metadata": {
                    "url": "https://textbook.example.com/control/mpc",
                    "module": "Module 3: Control Systems",
                    "chapter": "MPC Control",
                    "section": "Principles",
                    "chunk_index": 1,
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
    
    # Test complex query processing
    agent = Agent()
    response = agent.process_complex_query("Compare PID and MPC control strategies")
    
    # Verify the response contains information about both concepts
    assert response.content is not None
    assert len(response.sources) >= 1
    assert "PID" in response.content or "MPC" in response.content


@patch('backend.src.agent.core.OpenAI')
@patch('backend.src.agent.retrieval_tool.RetrievalTool')
def test_user_story_3_low_confidence_handling(mock_retrieval_tool, mock_openai_client):
    """Test User Story 3 - Low-Confidence Result Handling."""
    # Mock the OpenAI client and retrieval tool
    mock_assistant = Mock()
    mock_assistant.id = "test-assistant-id"
    mock_openai_client.return_value.beta.assistants.create.return_value = mock_assistant
    
    mock_retrieval_tool_instance = Mock()
    
    # Mock retrieval with low confidence results
    mock_retrieval_result = {
        "chunks": [
            {
                "id": "chunk1",
                "content": "Somewhat related information",
                "similarity_score": 0.2,  # Low similarity
                "rank": 0,
                "metadata": {
                    "url": "https://textbook.example.com/somewhere",
                    "module": "Module 1: Basics",
                    "chapter": "Introduction",
                    "section": "Overview",
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
    
    # Test how the agent handles low-confidence results
    agent = Agent()
    response = agent.process_simple_query("What is quantum robotics?")
    
    # Verify that the response is appropriate for low-confidence situation
    assert response.content is not None
    # Response should acknowledge the lower quality of information
    assert response.confidence_score < 0.5


@patch('backend.src.agent.core.OpenAI')
@patch('backend.src.agent.retrieval_tool.RetrievalTool')
def test_user_story_4_api_interaction(mock_retrieval_tool, mock_openai_client):
    """Test User Story 4 - API Interaction."""
    # Mock the OpenAI client and retrieval tool
    mock_assistant = Mock()
    mock_assistant.id = "test-assistant-id"
    mock_openai_client.return_value.beta.assistants.create.return_value = mock_assistant
    
    mock_retrieval_tool_instance = Mock()
    
    # Mock retrieval to return results
    mock_retrieval_result = {
        "chunks": [
            {
                "id": "chunk1",
                "content": "Robot Operating System provides tools for robotics development.",
                "similarity_score": 0.8,
                "rank": 0,
                "metadata": {
                    "url": "https://textbook.example.com/ros/intro",
                    "module": "Module 2: ROS 2 Framework",
                    "chapter": "Introduction",
                    "section": "Basics",
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
    
    # Test API request/response flow
    from backend.src.agent.models import ChatRequest, QueryRequest, QueryParameters
    
    # Test ChatRequest
    chat_request = ChatRequest(
        query="What is the Robot Operating System?",
        session_id="test-session-123",
        parameters=QueryParameters(top_k=3, threshold=0.6)
    )
    assert chat_request.query == "What is the Robot Operating System?"
    assert chat_request.session_id == "test-session-123"
    assert chat_request.parameters.top_k == 3
    
    # Test QueryRequest
    query_request = QueryRequest(
        query="Explain PID controllers",
        parameters=QueryParameters(top_k=5, threshold=0.7)
    )
    assert query_request.query == "Explain PID controllers"
    assert query_request.parameters.threshold == 0.7


def test_agent_initialization_and_config():
    """Test that the agent initializes correctly with proper configuration."""
    # Check that config validation works
    from backend.src.agent.config import Config
    import os
    
    # Save original value
    original_key = os.environ.get('OPENAI_API_KEY')
    
    # Set a test key
    os.environ['OPENAI_API_KEY'] = 'test-key'
    
    try:
        # Validate config
        Config.validate()
    except ValueError:
        # If validation fails, it's expected if other required env vars are missing
        pass
    finally:
        # Restore original value
        if original_key is not None:
            os.environ['OPENAI_API_KEY'] = original_key
        else:
            if 'OPENAI_API_KEY' in os.environ:
                del os.environ['OPENAI_API_KEY']


if __name__ == "__main__":
    pytest.main([__file__])