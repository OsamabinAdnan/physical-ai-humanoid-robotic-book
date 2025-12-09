from fastapi import APIRouter, Depends, HTTPException, status, Request
from sqlalchemy.orm import Session
from typing import Optional
from datetime import datetime, timedelta
import jwt
from passlib.context import CryptContext
from pydantic import BaseModel

from app.core.database import get_db
from app.schemas.user import UserCreate, User, UserUpdate
from app.models.user import User as UserModel
from app.core.settings import settings

router = APIRouter()

# Password hashing
pwd_context = CryptContext(schemes=["bcrypt"], deprecated="auto")

# JWT token creation
def create_access_token(data: dict, expires_delta: Optional[timedelta] = None):
    to_encode = data.copy()
    if expires_delta:
        expire = datetime.utcnow() + expires_delta
    else:
        expire = datetime.utcnow() + timedelta(minutes=30)
    to_encode.update({"exp": expire})
    encoded_jwt = jwt.encode(to_encode, settings.BETTER_AUTH_SECRET or "fallback_secret", algorithm="HS256")
    return encoded_jwt


def verify_password(plain_password, hashed_password):
    return pwd_context.verify(plain_password, hashed_password)


def get_password_hash(password):
    return pwd_context.hash(password)


class Token(BaseModel):
    access_token: str
    token_type: str


class LoginRequest(BaseModel):
    email: str
    password: str


@router.post("/auth/register", response_model=User)
def register_user(user: UserCreate, db: Session = Depends(get_db)):
    """
    Register a new user with background information
    """
    # Check if user already exists
    existing_user = db.query(UserModel).filter(UserModel.email == user.email).first()
    if existing_user:
        raise HTTPException(
            status_code=status.HTTP_400_BAD_REQUEST,
            detail="Email already registered"
        )

    # Hash the password
    hashed_password = get_password_hash(user.password)

    db_user = UserModel(
        email=user.email,
        username=user.username,
        full_name=user.full_name,
        hashed_password=hashed_password,
        software_background=user.software_background,
        hardware_background=user.hardware_background
    )

    db.add(db_user)
    db.commit()
    db.refresh(db_user)

    return db_user


@router.post("/auth/login", response_model=Token)
def login_user(login_request: LoginRequest, db: Session = Depends(get_db)):
    """
    Login endpoint to get JWT token
    """
    user = db.query(UserModel).filter(UserModel.email == login_request.email).first()

    # Check if user exists and password is correct
    if not user or not verify_password(login_request.password, user.hashed_password):
        raise HTTPException(
            status_code=status.HTTP_401_UNAUTHORIZED,
            detail="Incorrect email or password",
            headers={"WWW-Authenticate": "Bearer"},
        )

    access_token_expires = timedelta(minutes=30)
    access_token = create_access_token(
        data={"sub": str(user.id)}, expires_delta=access_token_expires
    )

    return {"access_token": access_token, "token_type": "bearer"}


@router.post("/auth/logout")
def logout_user():
    """
    Logout endpoint - in a real implementation this might involve token blacklisting
    """
    # In a real implementation, you might want to blacklist the token
    # For now, we just return a success message
    return {"message": "Successfully logged out"}


@router.get("/auth/me", response_model=User)
def get_current_user(request: Request, db: Session = Depends(get_db)):
    """
    Get current user profile using JWT token from Authorization header
    """
    # Extract token from Authorization header
    auth_header = request.headers.get("Authorization")
    if not auth_header or not auth_header.startswith("Bearer "):
        raise HTTPException(
            status_code=status.HTTP_401_UNAUTHORIZED,
            detail="Authorization header missing or invalid",
            headers={"WWW-Authenticate": "Bearer"},
        )

    token = auth_header.split(" ")[1]

    try:
        payload = jwt.decode(token, settings.BETTER_AUTH_SECRET or "fallback_secret", algorithms=["HS256"])
        user_id: int = payload.get("sub")
        if user_id is None:
            raise HTTPException(
                status_code=status.HTTP_401_UNAUTHORIZED,
                detail="Could not validate credentials",
                headers={"WWW-Authenticate": "Bearer"},
            )
    except jwt.PyJWTError:
        raise HTTPException(
            status_code=status.HTTP_401_UNAUTHORIZED,
            detail="Could not validate credentials",
            headers={"WWW-Authenticate": "Bearer"},
        )

    user = db.query(UserModel).filter(UserModel.id == user_id).first()
    if user is None:
        raise HTTPException(
            status_code=status.HTTP_401_UNAUTHORIZED,
            detail="User not found",
            headers={"WWW-Authenticate": "Bearer"},
        )

    return user


@router.put("/auth/me", response_model=User)
def update_user_profile(user_update: UserUpdate, request: Request, db: Session = Depends(get_db)):
    """
    Update user profile information
    """
    # Extract token from Authorization header
    auth_header = request.headers.get("Authorization")
    if not auth_header or not auth_header.startswith("Bearer "):
        raise HTTPException(
            status_code=status.HTTP_401_UNAUTHORIZED,
            detail="Authorization header missing or invalid",
            headers={"WWW-Authenticate": "Bearer"},
        )

    token = auth_header.split(" ")[1]

    try:
        payload = jwt.decode(token, settings.BETTER_AUTH_SECRET or "fallback_secret", algorithms=["HS256"])
        user_id: int = payload.get("sub")
        if user_id is None:
            raise HTTPException(
                status_code=status.HTTP_401_UNAUTHORIZED,
                detail="Could not validate credentials",
                headers={"WWW-Authenticate": "Bearer"},
            )
    except jwt.PyJWTError:
        raise HTTPException(
            status_code=status.HTTP_401_UNAUTHORIZED,
            detail="Could not validate credentials",
            headers={"WWW-Authenticate": "Bearer"},
        )

    user = db.query(UserModel).filter(UserModel.id == user_id).first()
    if user is None:
        raise HTTPException(
            status_code=status.HTTP_401_UNAUTHORIZED,
            detail="User not found",
            headers={"WWW-Authenticate": "Bearer"},
        )

    # Update user fields if provided
    if user_update.full_name is not None:
        user.full_name = user_update.full_name
    if user_update.software_background is not None:
        user.software_background = user_update.software_background
    if user_update.hardware_background is not None:
        user.hardware_background = user_update.hardware_background

    db.commit()
    db.refresh(user)

    return user