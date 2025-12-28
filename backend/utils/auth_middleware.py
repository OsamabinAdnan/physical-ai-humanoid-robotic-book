"""
Authentication middleware for session validation.
"""
import logging
import traceback
import uuid
from datetime import datetime
from fastapi import HTTPException, status, Request, Depends
from fastapi.security import HTTPBearer, HTTPAuthorizationCredentials
from typing import Optional
from .token_utils import verify_token
from database.services import DatabaseService
from database import AsyncDBSession, init_db_engine
from sqlalchemy.ext.asyncio import AsyncSession

logger = logging.getLogger(__name__)


security = HTTPBearer()


async def get_current_user(
    credentials: HTTPAuthorizationCredentials = Depends(security)
) -> dict:
    """
    Get the current authenticated user from the token.

    Args:
        credentials: The authorization credentials from the request header

    Returns:
        dict: The user data if authenticated, raises HTTPException otherwise
    """
    token = credentials.credentials
    logger.info(f"Received token: {token[:20]}..." if token else "No token received")

    try:
        payload = verify_token(token)
        logger.info(f"Token verification result: {payload is not None}")
    except Exception as e:
        logger.error(f"Error during token verification: {e}")
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail="Token verification error",
            headers={"WWW-Authenticate": "Bearer"},
        )

    if payload is None:
        logger.warning("Token verification failed - payload is None")
        raise HTTPException(
            status_code=status.HTTP_401_UNAUTHORIZED,
            detail="Could not validate credentials",
            headers={"WWW-Authenticate": "Bearer"},
        )

    user_id: str = payload.get("sub")
    if user_id is None:
        raise HTTPException(
            status_code=status.HTTP_401_UNAUTHORIZED,
            detail="Could not validate credentials",
            headers={"WWW-Authenticate": "Bearer"},
        )

    # Convert user_id to UUID
    try:
        user_uuid = uuid.UUID(user_id)
        logger.info(f"Converted user ID to UUID: {user_uuid}")
    except ValueError:
        logger.error(f"Invalid user ID format: {user_id}")
        raise HTTPException(
            status_code=status.HTTP_401_UNAUTHORIZED,
            detail="Invalid user ID format",
            headers={"WWW-Authenticate": "Bearer"},
        )

    # Ensure database engine is initialized before using the session
    init_db_engine()
    logger.info("Database engine initialized")

    # Import AsyncDBSession again after initialization to ensure it's properly loaded
    from database import AsyncDBSession

    # Check if AsyncDBSession is properly initialized
    if AsyncDBSession is None:
        logger.error("Database session is still None after initialization")
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail="Database session is not initialized",
        )

    # Get user from database using a temporary session and return as dict to avoid session issues
    logger.info(f"Attempting to get user from database with UUID: {user_uuid}")
    try:
        async with AsyncDBSession() as session:
            logger.info("Database session created successfully")
            db_service = DatabaseService(session)
            user = await db_service.user.get_user_by_id(user_uuid)
            logger.info(f"Database query result: user found = {user is not None}")

            if user is None:
                logger.warning(f"User not found with ID: {user_uuid}")
                raise HTTPException(
                    status_code=status.HTTP_401_UNAUTHORIZED,
                    detail="User not found",
                    headers={"WWW-Authenticate": "Bearer"},
                )

            # Convert user object to dictionary to avoid SQLAlchemy session issues
            # Ensure all attributes are loaded before session closes
            logger.info("Converting user object to dictionary")

            # Safely convert datetime objects to ISO format strings
            def safe_datetime_format(dt_obj):
                if dt_obj is None:
                    return datetime.now().isoformat()
                elif hasattr(dt_obj, 'isoformat'):
                    return dt_obj.isoformat()
                else:
                    return str(dt_obj)

            user_dict = {
                'id': str(user.id) if user.id else '',
                'email': user.email or '',
                'name': user.name or '',
                'software_background': user.software_background or 'beginner',
                'hardware_background': user.hardware_background or 'beginner',
                'email_verified': bool(user.email_verified) if user.email_verified is not None else False,
                'created_at': safe_datetime_format(user.created_at),
                'updated_at': safe_datetime_format(user.updated_at)
            }
            logger.info(f"User dictionary created successfully: {user_dict}")
    except Exception as e:
        logger.error(f"Error during database query or user conversion: {e}")
        import traceback
        logger.error(f"Full traceback: {traceback.format_exc()}")
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail="Error retrieving user information",
            headers={"WWW-Authenticate": "Bearer"},
        )

    logger.info("Returning user dictionary from middleware")
    return user_dict


def require_auth(request: Request) -> bool:
    """
    Check if the request has valid authentication.

    Args:
        request: The incoming request

    Returns:
        bool: True if authenticated, raises HTTPException otherwise
    """
    auth_header = request.headers.get("Authorization")
    if not auth_header or not auth_header.startswith("Bearer "):
        raise HTTPException(
            status_code=status.HTTP_401_UNAUTHORIZED,
            detail="Authorization header missing or invalid",
        )

    token = auth_header.split(" ")[1]
    payload = verify_token(token)

    if payload is None:
        raise HTTPException(
            status_code=status.HTTP_401_UNAUTHORIZED,
            detail="Invalid or expired token",
        )

    return True