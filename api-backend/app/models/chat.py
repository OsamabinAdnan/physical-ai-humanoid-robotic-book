from sqlalchemy import Column, Integer, String, DateTime, Text, ForeignKey
from sqlalchemy.sql import func
from app.core.database import Base


class ChatSession(Base):
    __tablename__ = "chat_sessions"

    id: int = Column(Integer, primary_key=True, index=True)
    user_id: int = Column(Integer, ForeignKey("users.id"), nullable=True)  # Optional: unauthenticated sessions
    title: str = Column(String, nullable=True)  # Optional: session title
    created_at = Column(DateTime(timezone=True), server_default=func.now())
    updated_at = Column(DateTime(timezone=True), onupdate=func.now())


class ChatMessage(Base):
    __tablename__ = "chat_messages"

    id: int = Column(Integer, primary_key=True, index=True)
    session_id: int = Column(Integer, ForeignKey("chat_sessions.id"), nullable=False)
    role: str = Column(String, nullable=False)  # "user" or "assistant"
    content: str = Column(Text, nullable=False)
    created_at = Column(DateTime(timezone=True), server_default=func.now())

    def __repr__(self):
        return f"ChatMessage(id={self.id!r}, role={self.role!r}, session_id={self.session_id!r})"