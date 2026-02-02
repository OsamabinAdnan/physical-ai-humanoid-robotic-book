"""
SQLAlchemy models for Neon database integration.
Defines the data models for users, chat_sessions, chat_messages, and documents.
"""
from sqlalchemy import Column, Integer, String, DateTime, Text, ForeignKey, Boolean
from sqlalchemy.dialects.postgresql import UUID
from sqlalchemy.orm import declarative_base, relationship
from sqlalchemy.sql import func
import uuid

Base = declarative_base()

class User(Base):
    """
    User model for storing user account information.
    """
    __tablename__ = "users"

    id = Column(UUID(as_uuid=True), primary_key=True, default=uuid.uuid4)
    email = Column(String(255), unique=True, nullable=False)
    password_hash = Column(String(255), nullable=False)  # Added for authentication
    name = Column(String(255), nullable=True)
    software_background = Column(String(20), nullable=False)  # Added for personalization
    hardware_background = Column(String(20), nullable=False)  # Added for personalization
    email_verified = Column(Boolean, default=False)  # Added for authentication
    created_at = Column(DateTime(timezone=True), server_default=func.now())
    updated_at = Column(DateTime(timezone=True), server_default=func.now(), onupdate=func.now())

    # Relationships
    chat_sessions = relationship("ChatSession", back_populates="user", cascade="all, delete-orphan")
    chat_messages = relationship("ChatMessage", back_populates="user")


class ChatSession(Base):
    """
    Chat session model for storing conversation metadata.
    """
    __tablename__ = "chat_sessions"

    id = Column(UUID(as_uuid=True), primary_key=True, default=uuid.uuid4)
    user_id = Column(UUID(as_uuid=True), ForeignKey("users.id"), nullable=False)
    session_id = Column(String(255), nullable=False)  # For external session identification
    title = Column(String(500), nullable=True)  # Title for the conversation
    created_at = Column(DateTime(timezone=True), server_default=func.now())
    updated_at = Column(DateTime(timezone=True), server_default=func.now(), onupdate=func.now())

    # Relationships
    user = relationship("User", back_populates="chat_sessions")
    chat_messages = relationship("ChatMessage", back_populates="chat_session", cascade="all, delete-orphan")


class ChatMessage(Base):
    """
    Chat message model for storing individual messages in conversations.
    """
    __tablename__ = "chat_messages"

    id = Column(UUID(as_uuid=True), primary_key=True, default=uuid.uuid4)
    session_id = Column(UUID(as_uuid=True), ForeignKey("chat_sessions.id"), nullable=False)
    user_id = Column(UUID(as_uuid=True), ForeignKey("users.id"), nullable=False)
    role = Column(String(50), nullable=False)  # 'user' or 'assistant'
    content = Column(Text, nullable=False)
    created_at = Column(DateTime(timezone=True), server_default=func.now())
    token_count = Column(Integer, nullable=True)  # Optional: number of tokens in the message

    # Relationships
    chat_session = relationship("ChatSession", back_populates="chat_messages")
    user = relationship("User", back_populates="chat_messages")


class Document(Base):
    """
    Document model for storing document ingestion metadata.
    """
    __tablename__ = "documents"

    id = Column(UUID(as_uuid=True), primary_key=True, default=uuid.uuid4)
    source_url = Column(Text, nullable=True)  # URL of the source document
    checksum = Column(String(255), nullable=True)  # Checksum for integrity verification
    original_filename = Column(String(500), nullable=True)  # Original filename
    ingestion_status = Column(String(50), default="pending")  # pending, success, failed
    created_at = Column(DateTime(timezone=True), server_default=func.now())

    # Additional metadata can be stored in a JSON column if needed


class PersonalizedContent(Base):
    """
    Personalized Content model for storing AI-generated personalized content summaries.
    """
    __tablename__ = "personalized_contents"

    id = Column(UUID(as_uuid=True), primary_key=True, default=uuid.uuid4)
    user_id = Column(UUID(as_uuid=True), ForeignKey("users.id"), nullable=False)
    chapter_id = Column(String(500), nullable=False)  # Identifier for the chapter that was personalized
    chapter_url = Column(String(1000), nullable=False)  # Full URL of the chapter that was personalized
    original_content_hash = Column(String(255), nullable=False)  # Hash of the original chapter content to detect changes
    personalized_summary = Column(Text, nullable=False)  # The AI-generated personalized summary/roadmap
    personalization_level = Column(String(20), nullable=False)  # Expertise level for which content was personalized
    created_at = Column(DateTime(timezone=True), server_default=func.now())
    updated_at = Column(DateTime(timezone=True), server_default=func.now(), onupdate=func.now())

    # Relationships
    user = relationship("User", back_populates="personalized_contents")


# Add relationship to User model
User.personalized_contents = relationship("PersonalizedContent", back_populates="user", cascade="all, delete-orphan")