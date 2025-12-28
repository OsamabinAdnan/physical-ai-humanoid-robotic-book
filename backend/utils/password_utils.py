"""
Password hashing utilities for authentication system.
"""
import bcrypt
import re


def verify_password(plain_password: str, hashed_password: str) -> bool:
    """
    Verify a plain password against a hashed password.

    Args:
        plain_password (str): The plain text password to verify
        hashed_password (str): The hashed password to compare against

    Returns:
        bool: True if the password matches, False otherwise
    """
    try:
        # Convert plain password to bytes if it's a string
        if isinstance(plain_password, str):
            plain_password = plain_password.encode('utf-8')
        if isinstance(hashed_password, str):
            # Remove any leading/trailing whitespace that might have been stored
            hashed_password = hashed_password.strip()
            hashed_password = hashed_password.encode('utf-8')

        return bcrypt.checkpw(plain_password, hashed_password)
    except Exception as e:
        # Log the error for debugging purposes
        print(f"Password verification error: {e}")
        return False


def get_password_hash(password: str) -> str:
    """
    Generate a hash for a plain text password.

    Args:
        password (str): The plain text password to hash

    Returns:
        str: The hashed password
    """
    # Convert password to bytes if it's a string
    if isinstance(password, str):
        password = password.encode('utf-8')

    # Generate salt and hash the password
    # Use default rounds (12) which is secure for most applications
    salt = bcrypt.gensalt()
    hashed = bcrypt.hashpw(password, salt)

    # Convert bytes back to string for storage
    return hashed.decode('utf-8')


def validate_password_strength(password: str) -> tuple[bool, str]:
    """
    Validate password strength based on security requirements.

    Args:
        password (str): The password to validate

    Returns:
        tuple[bool, str]: (is_valid, error_message)
    """
    if len(password) < 8:
        return False, "Password must be at least 8 characters long"

    if not re.search(r"[A-Z]", password):
        return False, "Password must contain at least one uppercase letter"

    if not re.search(r"[a-z]", password):
        return False, "Password must contain at least one lowercase letter"

    if not re.search(r"\d", password):
        return False, "Password must contain at least one digit"

    if not re.search(r"[!@#$%^&*(),.?\":{}|<>]", password):
        return False, "Password must contain at least one special character"

    # Check for bcrypt 72-byte limit
    if len(password.encode('utf-8')) > 72:
        return False, "Password cannot be longer than 72 bytes (bcrypt limit)"

    return True, ""