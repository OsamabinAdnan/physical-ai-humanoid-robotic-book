from pydantic import BaseModel
from typing import List, Optional
from datetime import datetime


class ChatMessageBase(BaseModel):
    role: str
    content: str


class ChatMessageCreate(ChatMessageBase):
    pass


class ChatMessage(ChatMessageBase):
    id: int
    session_id: int
    created_at: datetime

    class Config:
        from_attributes = True


class ChatSessionBase(BaseModel):
    title: Optional[str] = None


class ChatSessionCreate(ChatSessionBase):
    pass


class ChatSession(ChatSessionBase):
    id: int
    user_id: Optional[int] = None
    created_at: datetime
    updated_at: Optional[datetime] = None
    messages: List[ChatMessage] = []

    class Config:
        from_attributes = True


class ChatRequest(BaseModel):
    message: str
    session_id: str  # Changed to string to allow more flexible session IDs
    selected_text: Optional[str] = None  # For selected text mode


class ChatResponse(BaseModel):
    response: str
    session_id: str
    context_used: bool
    sources: Optional[list] = None  # List of sources used in the response