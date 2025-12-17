import logging
import os
import time
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
    CreateSessionRequestSchema
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
        "description": "API for the Physical AI & Humanoid Robotics textbook RAG agent",
        "version": "1.0.0",
        "endpoints": {
            "/": "This message",
            "/chat": "POST endpoint to interact with the RAG agent (expects JSON with 'question' field)",
            "/health": "GET endpoint to check the health status of the agent backend",
            "/docs": "Interactive API documentation (Swagger UI)",
            "/redoc": "Alternative API documentation (ReDoc)",
        },
    }


@app.post("/chat", response_model=QueryResponse)
async def chat_with_agent(
    request: QueryRequest,
    db_service: DatabaseService = Depends(get_db_service)
):
    """
    Chat endpoint for users to interact with the RAG agent.

    Args:
        request (QueryRequest): User's question and parameters
        db_service: Database service for persistence

    Returns:
        QueryResponse: Agent's response with citations and metadata
    """
    try:
        start_time = time.time()

        logger.info(f"Processing chat request: {request.question}")

        # For now, using a placeholder user ID - in the future, this will come from authentication
        # For the current implementation, we'll create a temporary user if needed
        import uuid
        user_id = uuid.uuid4()  # Placeholder - will be replaced with actual user ID from auth later

        # Create a temporary user record to satisfy the foreign key constraint
        temp_user = await db_service.user.get_user_by_id(user_id)
        if not temp_user:
            temp_user = await db_service.user.create_user(
                email=f"temp_{user_id}@example.com",  # Create a unique email
                name=f"Temp User {str(user_id)[:8]}"  # Create a unique name
            )

        # Create or get chat session
        # For now, we'll create a new session for each request, but in the future we might want to
        # accept a session_id in the request to continue an existing conversation
        session = await db_service.chat_session.create_session(
            user_id=temp_user.id,
            title=f"Chat session: {request.question[:50]}..."  # Use first 50 chars of question as title
        )

        # Store the user's question in the database
        await db_service.chat_message.add_message(
            session_id=session.id,
            user_id=temp_user.id,
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
            user_id=temp_user.id,  # Agent responses are stored with the user ID for now
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
