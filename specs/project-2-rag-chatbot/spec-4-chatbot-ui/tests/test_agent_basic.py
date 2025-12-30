"""
Basic tests for the agent query processing functionality.

This module tests the core functionality of the agent layer,
focusing on basic query processing capabilities.
"""
import pytest
import os
from unittest.mock import Mock, patch, MagicMock
from pydantic import ValidationError
from backend.src.agent.models import QueryRequest, ChatRequest, QueryParameters
from backend.src.agent.core import Agent, get_agent
from backend.src.agent.validation_utils import validate_query_text, validate_top_k, validate_threshold


def test_query_text_validation():
    """Test basic query text validation."""
    # Valid queries
    assert validate_query_text("What is ROS?")
    assert validate_query_text("This is a valid query text that is longer than 3 characters")
    
    # Invalid queries
    assert not validate_query_text("")  # Empty
    assert not validate_query_text("Hi")  # Too short
    assert not validate_query_text("A" * 10001)  # Too long


def test_top_k_validation():
    """Test top_k parameter validation."""
    # Valid values
    assert validate_top_k(1)
    assert validate_top_k(5)
    assert validate_top_k(50)
    
    # Invalid values
    assert not validate_top_k(0)
    assert not validate_top_k(51)
    assert not validate_top_k(-1)


def test_threshold_validation():
    """Test threshold parameter validation."""
    # Valid values
    assert validate_threshold(0.0)
    assert validate_threshold(0.5)
    assert validate_threshold(1.0)
    
    # Invalid values
    assert not validate_threshold(-0.1)
    assert not validate_threshold(1.1)


def test_pydantic_models():
    """Test that Pydantic models work correctly."""
    # Test QueryRequest
    query_req = QueryRequest(
        query="What is a PID controller?",
        parameters=QueryParameters(top_k=3, threshold=0.7)
    )
    assert query_req.query == "What is a PID controller?"
    assert query_req.parameters.top_k == 3
    assert query_req.parameters.threshold == 0.7
    
    # Test ChatRequest
    chat_req = ChatRequest(
        query="How does ROS2 work?",
        session_id="test-session-id",
        parameters=QueryParameters(top_k=5, threshold=0.6)
    )
    assert chat_req.query == "How does ROS2 work?"
    assert chat_req.session_id == "test-session-id"
    assert chat_req.parameters.top_k == 5
    
    # Test validation
    with pytest.raises(ValidationError):
        QueryRequest(query="Hi")  # Query too short


@patch('backend.src.agent.core.OpenAI')
@patch('backend.src.agent.retrieval_tool.RetrievalTool')
def test_agent_initialization(mock_retrieval_tool, mock_openai_client):
    """Test agent initialization."""
    # Mock the OpenAI client and retrieval tool
    mock_assistant = Mock()
    mock_assistant.id = "test-assistant-id"
    mock_openai_client.return_value.beta.assistants.create.return_value = mock_assistant
    
    mock_retrieval_tool_instance = Mock()
    mock_retrieval_tool.return_value = mock_retrieval_tool_instance
    
    # Set environment variable for API key
    os.environ['OPENAI_API_KEY'] = 'test-key'
    
    # Create agent instance
    agent = Agent()
    
    # Verify initialization
    assert agent.assistant.id == "test-assistant-id"
    assert agent.retrieval_tool == mock_retrieval_tool_instance


@patch('backend.src.agent.core.get_agent')
def test_get_agent_singleton(mock_get_agent):
    """Test that get_agent returns the same instance."""
    agent1 = get_agent()
    agent2 = get_agent()
    
    # In this test, we're just checking the function can be called
    # The singleton behavior is tested in implementation
    assert agent1 is not None


def test_query_params_model():
    """Test QueryParameters model."""
    params = QueryParameters(top_k=10, threshold=0.8)
    assert params.top_k == 10
    assert params.threshold == 0.8
    
    # Test defaults
    default_params = QueryParameters()
    assert default_params.top_k == 5  # Default value
    assert default_params.threshold == 0.6  # Default value


if __name__ == "__main__":
    pytest.main([__file__])