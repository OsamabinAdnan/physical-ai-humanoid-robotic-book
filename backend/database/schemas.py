from pydantic import BaseModel, EmailStr, validator
from datetime import datetime
from typing import List, Optional
import uuid


class UserSchema(BaseModel):
    id: str
    email: EmailStr
    name: Optional[str] = None
    software_background: str
    hardware_background: str
    email_verified: bool = False
    created_at: datetime
    updated_at: datetime

    @validator('software_background')
    def validate_software_background(cls, v):
        if v not in ['beginner', 'intermediate', 'expert']:
            raise ValueError('software_background must be one of: beginner, intermediate, expert')
        return v

    @validator('hardware_background')
    def validate_hardware_background(cls, v):
        if v not in ['beginner', 'intermediate', 'expert']:
            raise ValueError('hardware_background must be one of: beginner, intermediate, expert')
        return v


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


class UserRegistrationSchema(BaseModel):
    email: EmailStr
    password: str
    name: str
    software_background: str = "beginner"
    hardware_background: str = "beginner"

    @validator('password')
    def validate_password(cls, v):
        if len(v) < 8:
            raise ValueError('Password must be at least 8 characters long')
        if not any(c.isupper() for c in v):
            raise ValueError('Password must contain at least one uppercase letter')
        if not any(c.islower() for c in v):
            raise ValueError('Password must contain at least one lowercase letter')
        if not any(c.isdigit() for c in v):
            raise ValueError('Password must contain at least one digit')
        if not any(c in "!@#$%^&*(),.?\":{}|<>" for c in v):
            raise ValueError('Password must contain at least one special character')
        return v

    @validator('software_background')
    def validate_software_background(cls, v):
        if v not in ['beginner', 'intermediate', 'expert']:
            raise ValueError('software_background must be one of: beginner, intermediate, expert')
        return v

    @validator('hardware_background')
    def validate_hardware_background(cls, v):
        if v not in ['beginner', 'intermediate', 'expert']:
            raise ValueError('hardware_background must be one of: beginner, intermediate, expert')
        return v


class UserLoginSchema(BaseModel):
    email: EmailStr
    password: str


class UserResponseSchema(BaseModel):
    id: str
    email: EmailStr
    name: Optional[str] = None
    software_background: str
    hardware_background: str
    email_verified: bool = False
    created_at: datetime
    updated_at: datetime

    class Config:
        from_attributes = True


class AuthResponseSchema(BaseModel):
    success: bool
    user_id: str
    session_token: str
    message: str


class TokenDataSchema(BaseModel):
    user_id: str
    email: str


class PersonalizedContentSchema(BaseModel):
    id: str
    user_id: str
    chapter_id: str
    chapter_url: str
    original_content_hash: str
    personalized_summary: str
    personalization_level: str
    created_at: datetime
    updated_at: datetime

    @validator('personalization_level')
    def validate_personalization_level(cls, v):
        if v not in ['beginner', 'intermediate', 'expert']:
            raise ValueError('personalization_level must be one of: beginner, intermediate, expert')
        return v

    class Config:
        from_attributes = True


class CreatePersonalizedContentRequest(BaseModel):
    chapter_url: str
    chapter_content: str

    @validator('chapter_url')
    def validate_url(cls, v):
        if not v.startswith(('http://', 'https://')):
            raise ValueError('chapter_url must be a valid URL')
        return v


class PersonalizeContentResponse(BaseModel):
    success: bool
    personalized_summary: str
    message: str = ""