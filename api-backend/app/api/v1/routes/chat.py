from fastapi import APIRouter, Depends, HTTPException, status
from sqlalchemy.orm import Session
from typing import List
import os
from openai import OpenAI
from agents import Agent, Runner, AsyncOpenAI, OpenAIChatCompletionsModel, ModelSettings
from agents.run import RunConfig
from app.core.database import get_db
from app.schemas.chat import ChatRequest, ChatResponse
from app.schemas.user import User
from app.models.chat import ChatSession, ChatMessage
from app.utils.embeddings import embedding_generator
from app.utils.qdrant_client import qdrant_manager
from app.utils.logging import log_api_call, log_rag_process, log_error
from app.core.settings import settings

router = APIRouter()

def get_current_user():
    """
    Placeholder for getting current user - would use JWT token in real implementation
    For now, return a dummy user or raise an exception
    """
    # This is a placeholder - in a real implementation, you would extract user info from JWT token
    raise HTTPException(
        status_code=status.HTTP_401_UNAUTHORIZED,
        detail="Authentication required",
        headers={"WWW-Authenticate": "Bearer"},
    )

@router.post("/chat", response_model=ChatResponse)
def chat_with_rag(
    chat_request: ChatRequest,
    db: Session = Depends(get_db)
):
    """
    Chat endpoint that uses RAG (Retrieval-Augmented Generation) to provide context-aware responses
    """
    try:
        # Log the API call
        log_api_call(
            endpoint="/chat",
            method="POST",
            params={
                "session_id": chat_request.session_id,
                "message_length": len(chat_request.message),
                "has_selected_text": bool(chat_request.selected_text)
            }
        )

        # Initialize the AsyncOpenAI client with Gemini API configuration
        # Reference: https://ai.google.dev/gemini-api/docs/openai
        external_client = AsyncOpenAI(
            api_key=settings.GEMINI_API_KEY,
            base_url="https://generativelanguage.googleapis.com/v1beta/openai/",
        )

        # Set up the chat completions model using Gemini-2.0-flash
        model = OpenAIChatCompletionsModel(
            model="gemini-2.0-flash",
            openai_client=external_client
        )

        # Configure the runner with model settings
        config = RunConfig(
            model=model,
            model_provider=external_client,
            tracing_disabled=True  # Disable tracing for simpler output
        )

        # Create or get existing chat session
        session = db.query(ChatSession).filter(ChatSession.id == chat_request.session_id).first()
        if not session:
            session = ChatSession(id=chat_request.session_id)
            db.add(session)
            db.commit()
            db.refresh(session)

        # Save user message to database
        user_message = ChatMessage(
            session_id=chat_request.session_id,
            role="user",
            content=chat_request.message,
            selected_text=chat_request.selected_text
        )
        db.add(user_message)
        db.commit()

        # Prepare context for the AI response
        context_text = ""
        sources = []

        # If there's selected text, search for similar content in the vector database
        if chat_request.selected_text:
            # Generate embedding for the selected text
            query_embedding = embedding_generator.generate_embedding(chat_request.selected_text)

            # Search for similar content in Qdrant
            search_results = qdrant_manager.search_similar(
                query_vector=query_embedding,
                limit=5  # Get top 5 similar chunks
            )

            # Combine the search results as context and collect sources
            context_chunks = []
            for result in search_results:
                context_chunks.append(result["text"])
                # Add source metadata to the sources list
                sources.append({
                    "text": result["text"][:100] + "..." if len(result["text"]) > 100 else result["text"],
                    "metadata": result.get("metadata", {}),
                    "score": result.get("score", 0.0)
                })

            context_text = "\n\n".join(context_chunks)

            # Log the RAG process
            log_rag_process(
                selected_text=chat_request.selected_text,
                context_chunks=context_chunks,
                query_embedding=query_embedding,
                search_results=search_results
            )

        # Prepare the agent instructions with context
        if context_text:
            agent_instructions = f"""
            You are an AI assistant for the Physical AI Humanoid Robotics Textbook.
            Use the following context to answer the user's question:

            {context_text}

            Answer the user's question based on the provided context. If the context doesn't contain relevant information, respond based on your general knowledge about AI, robotics, and related topics. Keep your responses focused on the topic of physical AI and humanoid robotics.
            - You answer should be precise and focus
            - Try to answer within 300 to 400 words
            - If user ask question other than book topics, polietly refuse him/her and told him/her than I am here to help with book topics only.
            """
        else:
            agent_instructions = """
            You are an AI assistant for the Physical AI Humanoid Robotics Textbook.
            The user has asked a question, but no specific context from the textbook was found.
            Answer based on your general knowledge about AI, robotics, and related topics.
            Keep your responses focused on the topic of physical AI and humanoid robotics.

            - You answer should be precise and focus
            - Try to answer within 300 to 400 words
            - If user ask question other than book topics, polietly refuse him/her and told him/her than I am here to help with book topics only.

            """

        # Create an AI agent with the instructions
        agent: Agent = Agent(
            name="Physical AI Humanoid Robotics Assistant",
            instructions=agent_instructions,
            model=model,
            model_settings=ModelSettings(
                temperature=0.4,
                top_p=0.9,
            )
        )

        # Run the agent with the user's message
        result = Runner.run_sync(
            agent,
            chat_request.message,
            run_config=config,
        )

        ai_response = result.final_output

        # Save AI response to database
        ai_message = ChatMessage(
            session_id=chat_request.session_id,
            role="assistant",
            content=ai_response
        )
        db.add(ai_message)
        db.commit()

        # Log successful API call
        log_api_call(
            endpoint="/chat",
            method="POST",
            params={
                "session_id": chat_request.session_id,
                "message_length": len(chat_request.message),
                "has_selected_text": bool(chat_request.selected_text)
            },
            response={
                "response_length": len(ai_response),
                "context_used": bool(context_text),
                "sources_count": len(sources)
            }
        )

        return ChatResponse(
            response=ai_response,
            session_id=chat_request.session_id,
            context_used=bool(context_text),
            sources=sources
        )

    except Exception as e:
        # Log the error
        log_error(
            error_msg=str(e),
            context={
                "session_id": chat_request.session_id if 'chat_request' in locals() else 'unknown',
                "message_length": len(chat_request.message) if 'chat_request' in locals() and hasattr(chat_request, 'message') else 0
            }
        )
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail="An error occurred while processing your request"
        )

@router.get("/chat/history/{session_id}")
def get_chat_history(
    session_id: str,
    db: Session = Depends(get_db)
):
    """
    Retrieve chat history for a specific session
    """
    session = db.query(ChatSession).filter(ChatSession.id == session_id).first()
    if not session:
        raise HTTPException(
            status_code=status.HTTP_404_NOT_FOUND,
            detail="Session not found"
        )

    messages = db.query(ChatMessage).filter(
        ChatMessage.session_id == session_id
    ).order_by(ChatMessage.timestamp).all()

    return {
        "session_id": session_id,
        "messages": [
            {
                "role": msg.role,
                "content": msg.content,
                "timestamp": msg.timestamp,
                "selected_text": msg.selected_text
            }
            for msg in messages
        ]
    }