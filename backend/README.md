---
title: Physical AI & Humanoid Robotics RAG Chatbot with Authentication
emoji: 🤖
colorFrom: purple
colorTo: red
sdk: docker
app_file: main.py
pinned: false
license: apache-2.0
---

# Physical AI & Humanoid Robotics RAG Chatbot with Authentication

This is a comprehensive educational platform that combines a textbook on Physical AI and Humanoid Robotics with an AI-powered chatbot that can answer questions about the textbook content. The system includes user authentication, content personalization, NeonDB for relational data storage, and Qdrant for vector storage.

## Overview

This application implements a RAG system that uses NeonDB for relational data storage and Qdrant for vector storage. The system features user authentication with expertise level collection, content personalization based on user background, and leverages OpenRouter API for text generation, enabling efficient similarity search, contextual question answering, and personalized learning experiences.

## Features

- **User Authentication System**: Complete registration, login, logout, and user management with JWT-based session management
- **Content Personalization**: AI-generated content adaptation based on user expertise levels (software/hardware - beginner/intermediate/advanced)
- **Vector database storage with Qdrant**: For efficient document embedding and retrieval
- **Document embedding and retrieval using sentence-transformers**: For generating text embeddings
- **Contextual question answering**: With citation support and confidence scoring
- **Integration with OpenRouter API for text generation**: Using Mistral-7B model for personalization
- **Relational data storage with NeonDB**: For user management, chat sessions, and personalized content caching
- **Chat history persistence**: With user data isolation
- **Text selection feature**: Users can select text in the textbook and ask AI about it
- **Protected API endpoints**: With JWT token validation

## Configuration

The application requires the following environment variables:

- `NEON_DATABASE_URL`: Your NeonDB connection string
- `QDRANT_API_KEY`: Your Qdrant API key
- `QDRANT_CLUSTER_ENDPOINT`: Your Qdrant cluster endpoint
- `OPENROUTER_API_KEY`: Your OpenRouter API key for text generation
- `OPENROUTER_URL`: Your OpenRouter API endpoint (should end with /v1/chat/completions or be base URL)

## How to Use

1. Set up your environment variables in the `.env` file
2. Run database migrations with `alembic upgrade head`
3. Run the application with `uvicorn main:app --reload`
4. Use the API endpoints to register users, authenticate, and interact with the chatbot

## Endpoints

### Authentication Endpoints
- `POST /api/auth/register`: Register a new user with expertise levels and email
- `POST /api/auth/login`: Authenticate user with email/password and return JWT token
- `POST /api/auth/logout`: End current user session (client-side token removal)
- `GET /api/auth/me`: Get current authenticated user information (requires JWT token)

### Chat and Personalization Endpoints
- `POST /chat`: Chat with the RAG agent (requires JWT token in Authorization header)
- `GET /chat-history/{user_id}`: Get user's chat sessions (requires JWT token in Authorization header)
- `GET /chat-history/{user_id}/session/{session_id}`: Get specific session messages (requires JWT token in Authorization header)
- `POST /personalize`: Generate personalized content based on user background (requires JWT token in Authorization header)

### Health and Documentation Endpoints
- `GET /`: API information and available endpoints
- `GET /health`: Health check endpoint for backend services
- `GET /db-health`: Health check endpoint for database connection
- `GET /docs`: Interactive API documentation (Swagger UI)
- `GET /redoc`: Alternative API documentation (ReDoc)

## Authentication and Authorization

All protected endpoints require a JWT token in the Authorization header:
```
Authorization: Bearer <your-jwt-token>
```

## Content Personalization

The system allows authenticated users to personalize textbook content based on their expertise levels. The personalization API accepts the chapter URL and content, and returns AI-generated content tailored to the user's software and hardware background levels.

## Database Schema

The system includes multiple tables:
- **Users**: Stores user account information with expertise levels
- **ChatSessions**: Stores conversation metadata
- **ChatMessages**: Stores individual messages in conversations
- **Documents**: Stores document ingestion metadata
- **PersonalizedContents**: Stores AI-generated personalized content summaries

## License

This project is licensed under the Apache 2.0 License.