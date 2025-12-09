from pydantic import BaseModel
from typing import Optional
from datetime import datetime


class UserBase(BaseModel):
    email: str
    username: Optional[str] = None
    full_name: Optional[str] = None


class UserCreate(UserBase):
    password: str
    software_background: Optional[str] = None
    hardware_background: Optional[str] = None


class UserUpdate(BaseModel):
    full_name: Optional[str] = None
    software_background: Optional[str] = None
    hardware_background: Optional[str] = None


class User(UserBase):
    id: int
    is_active: bool
    created_at: datetime
    updated_at: Optional[datetime] = None

    class Config:
        from_attributes = True