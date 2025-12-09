import pytest
from sqlalchemy import create_engine
from sqlalchemy.orm import sessionmaker
from app.core.database import engine, Base, get_db
from app.models.user import User
from app.models.chat import ChatSession, ChatMessage


def test_database_engine_creation():
    """Test that database engine is created properly"""
    assert engine is not None
    # Check that the URL is not empty and contains the expected database connection string
    assert str(engine.url) != ""
    assert "postgresql" in str(engine.url)


def test_base_metadata():
    """Test that Base has the expected tables"""
    tables = Base.metadata.tables
    assert 'users' in tables
    assert 'chat_sessions' in tables
    assert 'chat_messages' in tables


def test_user_model():
    """Test User model structure"""
    user = User(
        email="test@example.com",
        username="testuser",
        hashed_password="hashed_password"
    )
    assert user.email == "test@example.com"
    assert user.username == "testuser"
    assert user.hashed_password == "hashed_password"


def test_chat_models():
    """Test Chat models structure"""
    session = ChatSession(id="test_session")
    assert session.id == "test_session"

    message = ChatMessage(
        session_id="test_session",
        role="user",
        content="test message"
    )
    assert message.session_id == "test_session"
    assert message.role == "user"
    assert message.content == "test message"