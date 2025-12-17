from pydantic import BaseModel
from datetime import datetime
from typing import List, Optional
import uuid


class ChatSessionSchema(BaseModel):
    id: str
    user_id: str
    session_id: str
    title: str
    created_at: datetime
    updated_at: datetime


class ChatMessageSchema(BaseModel):
    id: str
    session_id: str
    user_id: str
    role: str
    content: str
    token_count: Optional[int]
    created_at: datetime


class ChatHistoryResponseSchema(BaseModel):
    session: ChatSessionSchema
    messages: List[ChatMessageSchema]


class UserChatHistoryResponseSchema(BaseModel):
    sessions: List[ChatSessionSchema]


class CreateSessionRequestSchema(BaseModel):
    title: str
    user_id: str