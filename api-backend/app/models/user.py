from sqlalchemy import Column, Integer, String, DateTime, Boolean
from sqlalchemy.sql import func
from app.core.database import Base


class User(Base):
    __tablename__ = "users"

    id: int = Column(Integer, primary_key=True, index=True)
    email: str = Column(String, unique=True, index=True, nullable=False)
    username: str = Column(String, unique=True, index=True, nullable=True)
    full_name: str = Column(String, nullable=True)
    hashed_password: str = Column(String, nullable=False)
    is_active: bool = Column(Boolean, default=True)
    created_at = Column(DateTime(timezone=True), server_default=func.now())
    updated_at = Column(DateTime(timezone=True), onupdate=func.now())

    # Background information for personalization
    software_background: str = Column(String, nullable=True)  # e.g., "beginner", "intermediate", "expert"
    hardware_background: str = Column(String, nullable=True)  # e.g., "beginner", "intermediate", "expert"

    def __repr__(self):
        return f"User(id={self.id!r}, email={self.email!r}, username={self.username!r})"