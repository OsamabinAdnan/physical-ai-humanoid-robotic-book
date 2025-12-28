import logging
import os
import time
import httpx
import traceback
from contextlib import asynccontextmanager
from typing import Any, Dict, List

from dotenv import load_dotenv
from fastapi import FastAPI, HTTPException, Depends
from fastapi.middleware.cors import CORSMiddleware
from pydantic import BaseModel
from datetime import datetime
from typing import List

from agents import Runner
from agent.agent import agent

# Import database components
from database import engine, AsyncDBSession
from database.services import DatabaseService

# Import the authentication middleware at the top to avoid circular imports
from utils.auth_middleware import get_current_user as auth_get_current_user


# Load environment variables
load_dotenv()

# Configure logging
logging.basicConfig(
    level=logging.INFO, format="%(asctime)s - %(levelname)s - %(message)s"
)
logger = logging.getLogger(__name__)


@asynccontextmanager
async def lifespan(app: FastAPI):
    """
    Lifespan event handler for startup and shutdown events.
    """
    logger.info("RAG Chatbot Agent API starting up...")

    # Initialize database connection
    logger.info("Initializing database connection...")
    from database import init_db_engine
    init_db_engine()

    yield

    # Cleanup database connection
    logger.info("Cleaning up database connection...")
    from database import engine
    # Use sync disposal in async context
    await engine.dispose()
    logger.info("RAG Chatbot Agent API shutting down...")


# Create FastAPI application
app = FastAPI(
    title="RAG Chatbot Agent API",
    description="API for the Physical AI & Humanoid Robotics textbook RAG agent",
    version="1.0.0",
    lifespan=lifespan,
)

# Add CORS middleware to allow requests from the frontend
app.add_middleware(
    CORSMiddleware,
    allow_origins=[
        "*",
        "https://osamabinadnan.github.io/physical-ai-humanoid-robotic-book/",
    ],  # In production, replace with specific origins
    allow_credentials=True,
    allow_methods=["*"],  # This allows all methods including OPTIONS
    allow_headers=["*"],  # This allows all headers
)

# Import schemas from the agent module after app is defined to avoid circular imports
from agent.schemas import Citation, QueryRequest, QueryResponse

# Import database schemas
from database.schemas import (
    ChatSessionSchema,
    ChatMessageSchema,
    ChatHistoryResponseSchema,
    UserChatHistoryResponseSchema,
    CreateSessionRequestSchema,
    UserRegistrationSchema,
    UserLoginSchema,
    UserResponseSchema,
    AuthResponseSchema,
    TokenDataSchema,
    CreatePersonalizedContentRequest,
    PersonalizeContentResponse
)


# Dependency to get database service
async def get_db_service():
    from database import init_db_engine, AsyncDBSession
    # Ensure the database is initialized
    init_db_engine()
    async with AsyncDBSession() as session:
        yield DatabaseService(session)


@app.get("/")
async def root():
    """
    Root endpoint providing API information and available endpoints.

    Returns:
        Dict: API information and available endpoints
    """
    return {
        "message": "RAG Chatbot Agent API",
        "description": "API for the Physical AI & Humanoid Robotics textbook RAG agent with authentication and personalization",
        "version": "1.0.0",
        "endpoints": {
            "/": "This message",
            "/chat": "POST endpoint to interact with the RAG agent (requires JWT token in Authorization header)",
            "/chat-history/{user_id}": "GET endpoint to retrieve user's chat sessions (requires JWT token in Authorization header)",
            "/chat-history/{user_id}/session/{session_id}": "GET endpoint to retrieve specific session messages (requires JWT token in Authorization header)",
            "/api/auth/register": "POST endpoint to register a new user with expertise levels",
            "/api/auth/login": "POST endpoint to authenticate user with email/password",
            "/api/auth/logout": "POST endpoint to end current user session",
            "/api/auth/me": "GET endpoint to get current authenticated user information (requires JWT token in Authorization header)",
            "/personalize": "POST endpoint to generate personalized content based on user background (requires JWT token in Authorization header)",
            "/health": "GET endpoint to check the health status of the agent backend",
            "/db-health": "GET endpoint to check the health status of the database connection",
            "/docs": "Interactive API documentation (Swagger UI)",
            "/redoc": "Alternative API documentation (ReDoc)",
        },
    }


@app.post("/chat", response_model=QueryResponse)
async def chat_with_agent(
    request: QueryRequest,
    current_user: dict = Depends(auth_get_current_user),  # Get authenticated user from token
    db_service: DatabaseService = Depends(get_db_service)
):
    """
    Chat endpoint for users to interact with the RAG agent.

    Args:
        request (QueryRequest): User's question and parameters
        current_user: Current authenticated user data from JWT token
        db_service: Database service for persistence

    Returns:
        QueryResponse: Agent's response with citations and metadata
    """
    try:
        start_time = time.time()

        logger.info(f"Processing chat request for user: {current_user.get('id')} - {request.question}")

        # Get the authenticated user ID from the token
        user_id_str = current_user.get('id') or current_user.get('sub')
        if not user_id_str:
            raise HTTPException(
                status_code=401,
                detail="Invalid token: no user ID found"
            )

        # Convert user ID to UUID format
        import uuid
        try:
            user_id = uuid.UUID(str(user_id_str))
        except ValueError:
            raise HTTPException(
                status_code=400,
                detail="Invalid user ID format in token"
            )

        # Get the authenticated user from the database
        user = await db_service.user.get_user_by_id(user_id)
        if not user:
            raise HTTPException(
                status_code=404,
                detail="User not found"
            )

        # Create or get chat session
        # For now, we'll create a new session for each request, but in the future we might want to
        # accept a session_id in the request to continue an existing conversation
        session = await db_service.chat_session.create_session(
            user_id=user.id,
            title=f"Chat session: {request.question[:50]}..."  # Use first 50 chars of question as title
        )

        # Store the user's question in the database
        await db_service.chat_message.add_message(
            session_id=session.id,
            user_id=user.id,
            role="user",
            content=request.question
        )

        # Import agent components inside the function to avoid startup issues


        # Prepare the input for the agent based on the request
        input_text = request.question

        # Run the agent with the user's question
        result = await Runner.run(starting_agent=agent, input=input_text)

        # Extract the response from the agent
        response_text = (
            result.final_output if hasattr(result, "final_output") else str(result)
        )

        # For now, we're not extracting citations, just return an empty array
        citations_data = []

        # Calculate processing time
        processing_time = int((time.time() - start_time) * 1000)

        # Create response with confidence based on similarity scores
        if citations_data:
            avg_similarity = sum(
                [citation.similarity_score for citation in citations_data]
            ) / len(citations_data)
        else:
            avg_similarity = 0.0

        response = QueryResponse(
            answer=response_text,
            citations=citations_data,
            confidence=round(avg_similarity, 2),
            processing_time_ms=processing_time,
        )

        # Store the agent's response in the database
        await db_service.chat_message.add_message(
            session_id=session.id,
            user_id=user.id,  # Agent responses are stored with the user ID for now
            role="assistant",
            content=response_text
        )

        logger.info(f"Chat request processed successfully in {processing_time}ms")
        return response
    except Exception as e:
        logger.error(f"Error processing chat request: {e}")
        raise HTTPException(
            status_code=500, detail=f"Error processing chat request: {str(e)}"
        )


@app.get("/chat-history/{user_id}", response_model=UserChatHistoryResponseSchema)
async def get_user_chat_history(
    user_id: str,
    db_service: DatabaseService = Depends(get_db_service)
):
    """
    Get all chat sessions for a specific user.
    """
    try:
        logger.info(f"Fetching chat history for user: {user_id}")

        # Convert user_id to UUID
        import uuid
        user_uuid = uuid.UUID(user_id)

        # Get all sessions for the user
        sessions = await db_service.chat_session.get_sessions_by_user(user_uuid)

        # Convert to response format
        session_list = []
        for session in sessions:
            session_list.append(ChatSessionSchema(
                id=str(session.id),
                user_id=str(session.user_id),
                session_id=session.session_id,
                title=session.title,
                created_at=session.created_at,
                updated_at=session.updated_at
            ))

        return UserChatHistoryResponseSchema(sessions=session_list)
    except Exception as e:
        logger.error(f"Error fetching user chat history: {e}")
        raise HTTPException(
            status_code=500, detail=f"Error fetching chat history: {str(e)}"
        )


@app.get("/chat-history/{user_id}/session/{session_id}", response_model=ChatHistoryResponseSchema)
async def get_session_history(
    user_id: str,
    session_id: str,
    db_service: DatabaseService = Depends(get_db_service)
):
    """
    Get all messages for a specific chat session.
    """
    try:
        logger.info(f"Fetching session history for user: {user_id}, session: {session_id}")

        # Convert IDs to UUIDs
        import uuid
        user_uuid = uuid.UUID(user_id)
        session_uuid = uuid.UUID(session_id)

        # Get the session
        session = await db_service.chat_session.get_session_by_id(session_uuid, user_uuid)
        if not session:
            raise HTTPException(status_code=404, detail="Session not found")

        # Get all messages for the session
        messages = await db_service.chat_message.get_messages_by_session(session_uuid, user_uuid)

        # Convert to response format
        message_list = []
        for message in messages:
            message_list.append(ChatMessageSchema(
                id=str(message.id),
                session_id=str(message.session_id),
                user_id=str(message.user_id),
                role=message.role,
                content=message.content,
                token_count=message.token_count,
                created_at=message.created_at
            ))

        session_schema = ChatSessionSchema(
            id=str(session.id),
            user_id=str(session.user_id),
            session_id=session.session_id,
            title=session.title,
            created_at=session.created_at,
            updated_at=session.updated_at
        )

        return ChatHistoryResponseSchema(
            session=session_schema,
            messages=message_list
        )
    except Exception as e:
        logger.error(f"Error fetching session history: {e}")
        raise HTTPException(
            status_code=500, detail=f"Error fetching session history: {str(e)}"
        )


@app.post("/api/auth/register", response_model=AuthResponseSchema)
async def register_user(
    user_data: UserRegistrationSchema,
    db_service: DatabaseService = Depends(get_db_service)
):
    """
    Register a new user with background information.
    """
    try:
        logger.info(f"Registering new user: {user_data.email}")

        # Check if user already exists
        existing_user = await db_service.user.get_user_by_email(user_data.email)
        if existing_user:
            raise HTTPException(
                status_code=409,
                detail="Email already registered"
            )

        # Hash the password
        from utils.password_utils import get_password_hash
        password_hash = get_password_hash(user_data.password)

        # Create the user
        user = await db_service.user.create_user(
            email=user_data.email,
            password_hash=password_hash,
            name=user_data.name,
            software_background=user_data.software_background,
            hardware_background=user_data.hardware_background
        )

        # Create access token
        from utils.token_utils import create_access_token
        access_token = create_access_token(data={"sub": str(user.id), "email": user.email})

        return AuthResponseSchema(
            success=True,
            user_id=str(user.id),
            session_token=access_token,
            message="User registered successfully"
        )
    except HTTPException:
        raise
    except Exception as e:
        logger.error(f"Error registering user: {e}")
        raise HTTPException(
            status_code=500, detail=f"Error registering user: {str(e)}"
        )


@app.post("/api/auth/login", response_model=AuthResponseSchema)
async def login_user(
    login_data: UserLoginSchema,
    db_service: DatabaseService = Depends(get_db_service)
):
    """
    Authenticate user and return session token.
    """
    try:
        logger.info(f"User login attempt: {login_data.email}")

        # Get user by email
        user = await db_service.user.get_user_by_email(login_data.email)
        if not user:
            raise HTTPException(
                status_code=401,
                detail="Invalid email or password"
            )

        # Verify password
        from utils.password_utils import verify_password
        if not verify_password(login_data.password, user.password_hash):
            raise HTTPException(
                status_code=401,
                detail="Invalid email or password"
            )

        # Create access token
        from utils.token_utils import create_access_token
        access_token = create_access_token(data={"sub": str(user.id), "email": user.email})

        return AuthResponseSchema(
            success=True,
            user_id=str(user.id),
            session_token=access_token,
            message="Login successful"
        )
    except HTTPException:
        raise
    except Exception as e:
        logger.error(f"Error during login: {e}")
        raise HTTPException(
            status_code=500, detail=f"Error during login: {str(e)}"
        )


@app.post("/api/auth/logout")
async def logout_user():
    """
    Logout user (client-side token removal is sufficient).
    """
    return {"success": True, "message": "Logout successful"}


async def get_current_user_from_token(
    current_user: dict = Depends(auth_get_current_user)
):
    """
    Get current user from token using the auth middleware.
    """
    return current_user


@app.get("/api/auth/me", response_model=UserResponseSchema)
async def get_current_user(
    current_user: dict = Depends(get_current_user_from_token)
):
    """
    Get current authenticated user information.
    """
    try:
        user_data = {
            'id': current_user.get('id'),
            'email': current_user.get('email'),
            'name': current_user.get('name'),
            'software_background': current_user.get('software_background'),
            'hardware_background': current_user.get('hardware_background'),
            'email_verified': current_user.get('email_verified'),
            'created_at': current_user.get('created_at'),
            'updated_at': current_user.get('updated_at')
        }
        logger.info(f"User data for schema: {user_data}")

        return UserResponseSchema(**user_data)
    except Exception as e:
        logger.error(f"Error getting current user: {e}")
        logger.error(f"User data that caused error: {current_user}")
        import traceback
        logger.error(f"Full traceback: {traceback.format_exc()}")
        raise HTTPException(
            status_code=500, detail=f"Error getting current user: {str(e)}"
        )


@app.post("/personalize", response_model=PersonalizeContentResponse)
async def personalize_content(
    request: CreatePersonalizedContentRequest,
    current_user: dict = Depends(auth_get_current_user),
    db_service: DatabaseService = Depends(get_db_service)
):
    """
    Generate personalized content based on user background and chapter context.
    """
    try:
        logger.info(f"Personalizing content for user: {current_user.get('id')}")
        logger.info(f"Chapter URL: {request.chapter_url}")
        logger.info(f"Chapter content length: {len(request.chapter_content)} characters")

        # Extract chapter ID from URL - using full URL path as identifier to ensure uniqueness
        import re
        from urllib.parse import urlparse

        # Use the full path after /docs/ as the chapter ID to ensure uniqueness
        parsed = urlparse(request.chapter_url)
        full_path = parsed.path

        # Extract everything after /docs/ to make each chapter unique
        if '/docs/' in full_path:
            # Get the part after /docs/ and normalize it
            chapter_path = full_path.split('/docs/', 1)[1].strip('/')
            # Remove file extensions but keep the full path structure
            chapter_path = re.sub(r'\.html$', '', chapter_path)
            # Ensure it's a valid identifier but keep path separators as underscores
            chapter_id = re.sub(r'/', '_', chapter_path)  # Replace path separators with underscores
            chapter_id = re.sub(r'[^a-zA-Z0-9_\-]', '_', chapter_id)  # Clean up any other special chars
        else:
            # Fallback: use the full path with safe characters only
            chapter_id = re.sub(r'[^a-zA-Z0-9_\-]', '_', full_path)

        # Ensure the chapter_id is not empty
        if not chapter_id or chapter_id == '':
            chapter_id = 'unknown_chapter'

        logger.info(f"Chapter ID being personalized: {chapter_id}")
        logger.info(f"User expertise - Software: {current_user.get('software_background')}, Hardware: {current_user.get('hardware_background')}")

        from database.schemas import PersonalizeContentResponse
        from hashlib import sha256
        import openai
        import uuid

        # Get user's background information
        user = await db_service.user.get_user_by_id(uuid.UUID(current_user.get('id')))
        if not user:
            raise HTTPException(
                status_code=404,
                detail="User not found"
            )

        # Calculate hash of original content
        original_content_hash = sha256(request.chapter_content.encode()).hexdigest()

        # Check if personalized content already exists and is still valid
        existing_content = await db_service.personalized_content.get_personalized_content_by_user_and_chapter(
            user.id, chapter_id
        )

        # Only return cached content if it exists, the content hasn't changed, AND it's for the same URL
        if existing_content and existing_content.original_content_hash == original_content_hash and existing_content.chapter_url == request.chapter_url:
            # Return existing personalized content
            return PersonalizeContentResponse(
                success=True,
                personalized_summary=existing_content.personalized_summary,
                message="Personalized content retrieved from cache"
            )

        # Determine the user's relevant expertise level for personalization
        # For now, we'll use the lower of the two expertise levels (software and hardware)
        # In a more sophisticated system, we'd determine which is more relevant to the content
        expertise_level = user.software_background
        if user.hardware_background == "beginner" or (user.hardware_background == "intermediate" and expertise_level != "beginner"):
            expertise_level = user.hardware_background

        # Prepare the prompt for the LLM
        prompt = f"""
You are an educational content personalization expert. Your task is to adapt the following textbook content based on the user's expertise level.

USER'S EXPERTISE:
- Software Background: {user.software_background}
- Hardware Background: {user.hardware_background}

ORIGINAL CONTENT:
{request.chapter_content}

INSTRUCTIONS:
1. Adapt the content to match the user's expertise level ({expertise_level}).
2. If the user is a beginner, explain concepts simply with analogies and clear examples.
3. If the user is intermediate, provide more detail and context while still explaining fundamentals.
4. If the user is an expert, dive deep into technical aspects, advanced concepts, and applications.
5. CRITICAL: Preserve the original content structure including headings (##, ###), subheadings, bullet points (-, *), numbered lists (1., 2.), code blocks (```), and tables.
6. Maintain the same hierarchy and formatting as the original content.
7. Only modify the explanations and examples based on the user's expertise level, keeping the educational flow intact.
8. Do not remove or change any formatting elements like markdown syntax, lists, or structural elements.
9. Ensure all headings, lists, and code blocks are properly formatted in the response.

Please provide a personalized version of the content that is tailored to the user's expertise level while preserving the original structure and formatting exactly.
"""

        # Call the OpenRouter API to generate personalized content
        openrouter_api_key = os.getenv("OPENROUTER_API_KEY")
        base_url = os.getenv("OPENROUTER_URL", "https://openrouter.ai/api/v1")
        # Ensure the URL ends with /chat/completions for the completions endpoint
        if not base_url.endswith('/chat/completions'):
            if base_url.endswith('/v1'):
                openrouter_url = base_url + '/chat/completions'
            else:
                openrouter_url = base_url.rstrip('/') + '/v1/chat/completions'
        else:
            openrouter_url = base_url

        headers = {
            "Authorization": f"Bearer {openrouter_api_key}",
            "Content-Type": "application/json"
        }

        payload = {
            "model": "mistralai/mistral-7b-instruct:free",  # Using a free model that's known to be available on OpenRouter
            "messages": [
                {"role": "user", "content": prompt}
            ],
            "temperature": 0.7,
            "max_tokens": 1000
        }

        async with httpx.AsyncClient() as client:
            response = await client.post(
                openrouter_url,
                json=payload,
                headers=headers,
                timeout=30.0  # 30 second timeout
            )

            if response.status_code != 200:
                logger.error(f"OpenRouter API error: {response.status_code} - {response.text}")
                raise HTTPException(
                    status_code=500,
                    detail=f"Error calling OpenRouter API: {response.status_code} - {response.text}"
                )

            # Check if response body is empty
            if not response.text:
                logger.error("OpenRouter API returned empty response")
                raise HTTPException(
                    status_code=500,
                    detail="OpenRouter API returned empty response"
                )

            try:
                response_data = response.json()
            except ValueError as e:
                logger.error(f"Error parsing OpenRouter API response as JSON: {e}")
                logger.error(f"Response text: {response.text}")
                raise HTTPException(
                    status_code=500,
                    detail=f"Error parsing OpenRouter API response: {str(e)}"
                )

            # Check if required fields exist in response
            if "choices" not in response_data or not response_data["choices"]:
                logger.error(f"OpenRouter API response missing choices: {response_data}")
                raise HTTPException(
                    status_code=500,
                    detail="OpenRouter API response missing choices"
                )

            personalized_summary = response_data["choices"][0]["message"]["content"].strip()

        # Save the personalized content to the database
        personalized_content = await db_service.personalized_content.create_personalized_content(
            user_id=user.id,
            chapter_id=chapter_id,
            chapter_url=request.chapter_url,
            original_content_hash=original_content_hash,
            personalized_summary=personalized_summary,
            personalization_level=expertise_level
        )

        logger.info(f"Personalization completed successfully for user {current_user.get('id')} and chapter {chapter_id}")

        return PersonalizeContentResponse(
            success=True,
            personalized_summary=personalized_summary,
            message="Content personalized successfully"
        )
    except HTTPException:
        raise
    except Exception as e:
        logger.error(f"Error personalizing content: {e}")
        raise HTTPException(
            status_code=500, detail=f"Error personalizing content: {str(e)}"
        )


@app.get("/db-health")
async def db_health_check(db_service: DatabaseService = Depends(get_db_service)):
    """
    Check the health status of the database connection.
    """
    try:
        # Test database connectivity by executing a simple query
        from sqlalchemy import text
        result = await db_service.db_session.execute(text("SELECT 1"))
        db_status = "healthy"
    except Exception as e:
        logger.error(f"Database health check failed: {e}")
        db_status = f"unhealthy: {str(e)}"

    return {
        "status": db_status,
        "timestamp": time.time(),
        "service": "neon_database"
    }


@app.get("/health")
async def health_check():
    """
    Check the health status of the agent backend.

    Returns:
        Dict: Health status with service connectivity information
    """
    try:
        # Check if environment variables are properly set
        qwen_api_key = os.getenv("QWEN_API_KEY")
        qwen_url = os.getenv("QWEN_URL")

        if not qwen_api_key or not qwen_url:
            return {
                "status": "unhealthy",
                "timestamp": time.time(),
                "services": {
                    "qwen_api": "missing_credentials",
                    "agent": "not_initialized",
                },
            }

        return {
            "status": "healthy",
            "timestamp": time.time(),
            "services": {
                "qwen_api": "configured",
                "agent": "ready",
                "qdrant_cloud": "configured",  # Assuming Qdrant is configured in retriever
            },
        }
    except Exception as e:
        logger.error(f"Health check failed: {e}")
        return {
            "status": "unhealthy",
            "timestamp": time.time(),
            "services": {"error": str(e)},
        }


# The application is ready to be served by uvicorn
