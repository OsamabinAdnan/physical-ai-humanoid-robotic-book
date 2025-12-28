"""
Service layer for database operations.
Implements CRUD utilities for users, chat sessions, chat messages, and documents.
"""
from typing import Optional, List, Dict, Any
from sqlalchemy import select, and_, or_, func
from sqlalchemy.ext.asyncio import AsyncSession
from sqlalchemy.orm import selectinload
from .models import User, ChatSession, ChatMessage, Document
import uuid


class BaseService:
    """
    Base service class with common database operations.
    """
    def __init__(self, db_session: AsyncSession):
        self.db_session = db_session

    async def _get_by_id(self, model_class, obj_id: uuid.UUID):
        """Get an object by its ID."""
        stmt = select(model_class).where(model_class.id == obj_id)
        result = await self.db_session.execute(stmt)
        return result.scalar_one_or_none()

    async def _create(self, model_instance):
        """Create a new object."""
        self.db_session.add(model_instance)
        await self.db_session.commit()
        await self.db_session.refresh(model_instance)
        return model_instance

    async def _update(self, model_instance):
        """Update an existing object."""
        await self.db_session.commit()
        await self.db_session.refresh(model_instance)
        return model_instance

    async def _delete(self, model_instance):
        """Delete an object."""
        await self.db_session.delete(model_instance)
        await self.db_session.commit()


class UserService(BaseService):
    """
    Service class for user operations.
    """
    async def create_user(self, email: str, password_hash: str, name: Optional[str] = None,
                         software_background: str = "beginner", hardware_background: str = "beginner") -> User:
        """Create a new user with authentication fields."""
        user = User(
            email=email,
            password_hash=password_hash,
            name=name,
            software_background=software_background,
            hardware_background=hardware_background
        )
        return await self._create(user)

    async def get_user_by_id(self, user_id: uuid.UUID) -> Optional[User]:
        """Get user by ID."""
        return await self._get_by_id(User, user_id)

    async def get_user_by_email(self, email: str) -> Optional[User]:
        """Get user by email."""
        stmt = select(User).where(User.email == email)
        result = await self.db_session.execute(stmt)
        return result.scalar_one_or_none()

    async def update_user_background(self, user_id: uuid.UUID, software_background: str = None,
                                   hardware_background: str = None) -> Optional[User]:
        """Update user background information."""
        user = await self.get_user_by_id(user_id)
        if user:
            if software_background:
                user.software_background = software_background
            if hardware_background:
                user.hardware_background = hardware_background
            return await self._update(user)
        return None

    async def update_user(self, user_id: uuid.UUID, **kwargs) -> Optional[User]:
        """Update user fields."""
        user = await self.get_user_by_id(user_id)
        if user:
            for key, value in kwargs.items():
                if hasattr(user, key):
                    setattr(user, key, value)
            return await self._update(user)
        return None

    async def delete_user(self, user_id: uuid.UUID) -> bool:
        """Delete a user."""
        user = await self.get_user_by_id(user_id)
        if user:
            await self._delete(user)
            return True
        return False


class ChatSessionService(BaseService):
    """
    Service class for chat session operations.
    """
    async def create_session(self, user_id: uuid.UUID, title: Optional[str] = None, session_id: Optional[str] = None) -> ChatSession:
        """Create a new chat session."""
        session = ChatSession(
            user_id=user_id,
            title=title or "New Chat",
            session_id=session_id or str(uuid.uuid4())
        )
        return await self._create(session)

    async def get_session_by_id(self, session_id: uuid.UUID, user_id: Optional[uuid.UUID] = None) -> Optional[ChatSession]:
        """Get chat session by ID with optional user verification."""
        stmt = select(ChatSession).where(ChatSession.id == session_id)
        if user_id:
            stmt = stmt.where(ChatSession.user_id == user_id)
        stmt = stmt.options(
            selectinload(ChatSession.user)
        )
        result = await self.db_session.execute(stmt)
        return result.scalar_one_or_none()

    async def get_sessions_by_user(self, user_id: uuid.UUID) -> List[ChatSession]:
        """Get all sessions for a user."""
        stmt = select(ChatSession).where(ChatSession.user_id == user_id).order_by(ChatSession.updated_at.desc())
        result = await self.db_session.execute(stmt)
        return result.scalars().all()

    async def update_session(self, session_id: uuid.UUID, user_id: uuid.UUID, **kwargs) -> Optional[ChatSession]:
        """Update session fields with user verification."""
        session = await self.get_session_by_id(session_id, user_id)
        if session:
            for key, value in kwargs.items():
                if hasattr(session, key):
                    setattr(session, key, value)
            return await self._update(session)
        return None

    async def delete_session(self, session_id: uuid.UUID, user_id: uuid.UUID) -> bool:
        """Delete a session with user verification."""
        session = await self.get_session_by_id(session_id, user_id)
        if session:
            await self._delete(session)
            return True
        return False


class ChatMessageService(BaseService):
    """
    Service class for chat message operations.
    """
    async def add_message(self, session_id: uuid.UUID, user_id: uuid.UUID, role: str, content: str, token_count: Optional[int] = None) -> ChatMessage:
        """Add a new message to a session."""
        message = ChatMessage(
            session_id=session_id,
            user_id=user_id,
            role=role,
            content=content,
            token_count=token_count
        )
        return await self._create(message)

    async def get_messages_by_session(self, session_id: uuid.UUID, user_id: Optional[uuid.UUID] = None) -> List[ChatMessage]:
        """Get all messages for a session with optional user verification."""
        stmt = select(ChatMessage).where(ChatMessage.session_id == session_id)
        if user_id:
            stmt = stmt.where(ChatMessage.user_id == user_id)
        stmt = stmt.order_by(ChatMessage.created_at.asc())
        result = await self.db_session.execute(stmt)
        return result.scalars().all()

    async def get_message_by_id(self, message_id: uuid.UUID, user_id: Optional[uuid.UUID] = None) -> Optional[ChatMessage]:
        """Get message by ID with optional user verification."""
        stmt = select(ChatMessage).where(ChatMessage.id == message_id)
        if user_id:
            stmt = stmt.where(ChatMessage.user_id == user_id)
        result = await self.db_session.execute(stmt)
        return result.scalar_one_or_none()

    async def update_message(self, message_id: uuid.UUID, user_id: uuid.UUID, **kwargs) -> Optional[ChatMessage]:
        """Update message fields with user verification."""
        message = await self.get_message_by_id(message_id, user_id)
        if message:
            for key, value in kwargs.items():
                if hasattr(message, key):
                    setattr(message, key, value)
            return await self._update(message)
        return None

    async def delete_message(self, message_id: uuid.UUID, user_id: uuid.UUID) -> bool:
        """Delete a message with user verification."""
        message = await self.get_message_by_id(message_id, user_id)
        if message:
            await self._delete(message)
            return True
        return False


class DocumentService(BaseService):
    """
    Service class for document operations.
    """
    async def create_document(self, source_url: Optional[str] = None, original_filename: Optional[str] = None,
                            checksum: Optional[str] = None, ingestion_status: str = "pending") -> Document:
        """Create a new document record."""
        document = Document(
            source_url=source_url,
            original_filename=original_filename,
            checksum=checksum,
            ingestion_status=ingestion_status
        )
        return await self._create(document)

    async def get_document_by_id(self, document_id: uuid.UUID) -> Optional[Document]:
        """Get document by ID."""
        return await self._get_by_id(Document, document_id)

    async def get_documents_by_status(self, status: str) -> List[Document]:
        """Get documents by ingestion status."""
        stmt = select(Document).where(Document.ingestion_status == status)
        result = await self.db_session.execute(stmt)
        return result.scalars().all()

    async def update_document(self, document_id: uuid.UUID, **kwargs) -> Optional[Document]:
        """Update document fields."""
        document = await self.get_document_by_id(document_id)
        if document:
            for key, value in kwargs.items():
                if hasattr(document, key):
                    setattr(document, key, value)
            return await self._update(document)
        return None

    async def delete_document(self, document_id: uuid.UUID) -> bool:
        """Delete a document."""
        document = await self.get_document_by_id(document_id)
        if document:
            await self._delete(document)
            return True
        return False


class PersonalizedContentService(BaseService):
    """
    Service class for personalized content operations.
    """
    async def create_personalized_content(self, user_id: uuid.UUID, chapter_id: str, chapter_url: str,
                                        original_content_hash: str, personalized_summary: str,
                                        personalization_level: str) -> 'PersonalizedContent':
        """Create a new personalized content record."""
        from .models import PersonalizedContent
        content = PersonalizedContent(
            user_id=user_id,
            chapter_id=chapter_id,
            chapter_url=chapter_url,
            original_content_hash=original_content_hash,
            personalized_summary=personalized_summary,
            personalization_level=personalization_level
        )
        return await self._create(content)

    async def get_personalized_content_by_user_and_chapter(self, user_id: uuid.UUID, chapter_id: str) -> Optional['PersonalizedContent']:
        """Get personalized content for a specific user and chapter (most recent one)."""
        from .models import PersonalizedContent
        stmt = select(PersonalizedContent).where(
            and_(PersonalizedContent.user_id == user_id, PersonalizedContent.chapter_id == chapter_id)
        ).order_by(PersonalizedContent.created_at.desc()).limit(1)  # Get the most recent one
        result = await self.db_session.execute(stmt)
        return result.scalar_one_or_none()

    async def get_personalized_content_by_id(self, content_id: uuid.UUID) -> Optional['PersonalizedContent']:
        """Get personalized content by ID."""
        from .models import PersonalizedContent
        return await self._get_by_id(PersonalizedContent, content_id)

    async def update_personalized_content(self, content_id: uuid.UUID, **kwargs) -> Optional['PersonalizedContent']:
        """Update personalized content fields."""
        from .models import PersonalizedContent
        content = await self.get_personalized_content_by_id(content_id)
        if content:
            for key, value in kwargs.items():
                if hasattr(content, key):
                    setattr(content, key, value)
            return await self._update(content)
        return None

    async def delete_personalized_content(self, content_id: uuid.UUID) -> bool:
        """Delete personalized content."""
        from .models import PersonalizedContent
        content = await self.get_personalized_content_by_id(content_id)
        if content:
            await self._delete(content)
            return True
        return False


# Combined service class for easier dependency injection
class DatabaseService:
    """
    Combined service class that provides access to all individual services.
    """
    def __init__(self, db_session: AsyncSession):
        self.db_session = db_session
        self.user = UserService(db_session)
        self.chat_session = ChatSessionService(db_session)
        self.chat_message = ChatMessageService(db_session)
        self.document = DocumentService(db_session)
        self.personalized_content = PersonalizedContentService(db_session)