"""
JWT token management utilities for authentication system.
"""
from datetime import datetime, timedelta
from typing import Optional, Union
import os
from jose import JWTError, jwt
from fastapi import HTTPException, status


# Get JWT configuration from environment variables
SECRET_KEY = os.getenv("BETTER_AUTH_SECRET", "ia2r7zQKcIffRwgyPaoPDRE8d9FESdhZ")
ALGORITHM = os.getenv("JWT_ALGORITHM", "HS256")
ACCESS_TOKEN_EXPIRE_HOURS = int(os.getenv("JWT_ACCESS_TOKEN_EXPIRE_HOURS", "24"))


def create_access_token(data: dict, expires_delta: Optional[timedelta] = None):
    """
    Create a JWT access token.

    Args:
        data (dict): Data to encode in the token
        expires_delta (Optional[timedelta]): Expiration time for the token

    Returns:
        str: The encoded JWT token
    """
    to_encode = data.copy()
    if expires_delta:
        expire = datetime.utcnow() + expires_delta
    else:
        expire = datetime.utcnow() + timedelta(hours=ACCESS_TOKEN_EXPIRE_HOURS)

    to_encode.update({"exp": expire})
    encoded_jwt = jwt.encode(to_encode, SECRET_KEY, algorithm=ALGORITHM)
    return encoded_jwt


def verify_token(token: str) -> Optional[dict]:
    """
    Verify a JWT token and return the payload if valid.

    Args:
        token (str): The JWT token to verify

    Returns:
        Optional[dict]: The token payload if valid, None otherwise
    """
    import logging
    logger = logging.getLogger(__name__)

    try:
        logger.info(f"Attempting to verify token: {token[:20]}..." if token else "No token to verify")
        payload = jwt.decode(token, SECRET_KEY, algorithms=[ALGORITHM])
        logger.info(f"Token verification successful, payload: {payload}")
        return payload
    except JWTError as e:
        logger.error(f"JWT verification error: {e}")
        return None
    except Exception as e:
        logger.error(f"Unexpected error during token verification: {e}")
        return None


def decode_token(token: str) -> Optional[dict]:
    """
    Decode a JWT token without verification (for debugging purposes).

    Args:
        token (str): The JWT token to decode

    Returns:
        Optional[dict]: The token payload if decoding is successful, None otherwise
    """
    try:
        payload = jwt.decode(token, options={"verify_signature": False})
        return payload
    except JWTError:
        return None