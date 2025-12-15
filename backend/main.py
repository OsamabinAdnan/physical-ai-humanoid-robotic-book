import logging
import os
import time
from contextlib import asynccontextmanager
from typing import Any, Dict, List

from dotenv import load_dotenv
from fastapi import FastAPI, HTTPException
from fastapi.middleware.cors import CORSMiddleware
from pydantic import BaseModel

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
    yield
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
        "http://localhost:3000/physical-ai-humanoid-robotic-book/",
        "https://osamabinadnan.github.io/physical-ai-humanoid-robotic-book/",
    ],  # In production, replace with specific origins
    allow_credentials=True,
    allow_methods=["*"],  # This allows all methods including OPTIONS
    allow_headers=["*"],  # This allows all headers
)

# Import schemas from the agent module after app is defined to avoid circular imports
from agent.schemas import Citation, QueryRequest, QueryResponse


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
async def chat_with_agent(request: QueryRequest):
    """
    Chat endpoint for users to interact with the RAG agent.

    Args:
        request (QueryRequest): User's question and parameters

    Returns:
        QueryResponse: Agent's response with citations and metadata
    """
    try:
        start_time = time.time()

        logger.info(f"Processing chat request: {request.question}")

        # Import agent components inside the function to avoid startup issues
        from agents import Runner

        from agent.agent import agent

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

        logger.info(f"Chat request processed successfully in {processing_time}ms")
        return response
    except Exception as e:
        logger.error(f"Error processing chat request: {e}")
        raise HTTPException(
            status_code=500, detail=f"Error processing chat request: {str(e)}"
        )


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
