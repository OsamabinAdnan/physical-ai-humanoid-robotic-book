import pytest
from datetime import datetime
from app.schemas.chat import ChatRequest, ChatResponse, ChatSession, ChatMessage


def test_chat_request_schema():
    """Test ChatRequest schema"""
    request = ChatRequest(
        message="Hello, world!",
        session_id="session_123",
        selected_text="selected text"
    )

    assert request.message == "Hello, world!"
    assert request.session_id == "session_123"
    assert request.selected_text == "selected text"


def test_chat_response_schema():
    """Test ChatResponse schema"""
    response = ChatResponse(
        response="AI response",
        session_id="session_123",
        context_used=True,
        sources=[{"text": "source text", "metadata": {}, "score": 0.9}]
    )

    assert response.response == "AI response"
    assert response.session_id == "session_123"
    assert response.context_used is True
    assert len(response.sources) == 1


def test_chat_session_schema():
    """Test ChatSession schema"""
    session = ChatSession(
        id=1,
        title="Test Session",
        user_id=123,
        created_at=datetime.now(),
        updated_at=datetime.now(),
        messages=[]
    )

    assert session.id == 1
    assert session.title == "Test Session"
    assert session.user_id == 123
    assert session.messages == []


def test_chat_message_schema():
    """Test ChatMessage schema"""
    message = ChatMessage(
        id=1,
        session_id=1,
        role="user",
        content="Test message",
        created_at=datetime.now()
    )

    assert message.id == 1
    assert message.session_id == 1
    assert message.role == "user"
    assert message.content == "Test message"