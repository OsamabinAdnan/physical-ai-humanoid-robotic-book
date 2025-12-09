# Physical AI Humanoid Robotics Textbook Backend

This is the backend API for the Physical AI Humanoid Robotics Textbook project with RAG (Retrieval-Augmented Generation) capabilities.

## Features

- **RAG System**: Uses vector database (Qdrant) to store textbook content and retrieve relevant information for AI responses
- **AI Integration**: Connects to Google Gemini API via OpenAI SDK for intelligent responses
- **Authentication**: Complete JWT-based authentication with user registration, login, and profile management with background information collection
- **Chat System**: Session-based chat with history and context-aware responses
- **Content Ingestion**: APIs to ingest textbook content into the vector database
- **Selected Text Mode**: Allows users to highlight text and get explanations/summaries

## Tech Stack

- **Framework**: FastAPI
- **Database**: PostgreSQL (Neon Serverless)
- **Vector Database**: Qdrant Cloud
- **Authentication**: JWT-based with password hashing (compatible with Better-Auth.com frontend)
- **AI Model**: Google Gemini via OpenAI SDK
- **Embeddings**: sentence-transformers/all-MiniLM-L6-v2
- **Frontend Integration**: Designed for integration with Docusaurus frontend

## API Endpoints

### Chat Endpoints (`/v1/chat`)
- `POST /chat` - Main chat endpoint with RAG functionality
- `GET /chat/history/{session_id}` - Get chat history for a session

### Ingest Endpoints (`/v1/ingest`)
- `POST /ingest/textbook` - Ingest text chunks with metadata
- `POST /ingest/chapter` - Ingest a single chapter with sections
- `POST /ingest/textbook-full` - Ingest an entire textbook with multiple chapters
- `POST /ingest/reset` - Reset the vector store
- `GET /ingest/status` - Get vector store status

### Auth Endpoints (`/v1/auth`)
- `POST /auth/register` - User registration with background info and password
- `POST /auth/login` - User login with JWT token generation
- `POST /auth/logout` - User logout
- `GET /auth/me` - Get current user info (requires valid JWT)
- `PUT /auth/me` - Update user profile (requires valid JWT)

### Health Check
- `GET /health` - Health check endpoint

## Environment Variables

The application requires the following environment variables in a `.env` file:

```env
# Database settings
DATABASE_URL=postgresql://...

# Qdrant settings
QDRANT_API_KEY=...
QDRANT_CLUSTER_ENDPOINT=...
QDRANT_HOST=...

# Gemini settings
GEMINI_API_KEY=...

# Embedding model
EMBEDDING_MODEL=sentence-transformers/all-MiniLM-L6-v2

# Better Auth settings
BETTER_AUTH_SECRET=...
BETTER_AUTH_URL=...
```

## Running the Application

1. Install dependencies:
   ```bash
   uv sync
   ```

2. Run the application:
   ```bash
   uv run uvicorn main:app --reload
   ```

## Testing

Run the test suite:
```bash
pytest
```

## Architecture

The backend follows a modular architecture with:
- `app/api/v1/routes/` - API route definitions
- `app/models/` - Database models (SQLAlchemy)
- `app/schemas/` - Pydantic schemas for request/response validation
- `app/utils/` - Utility functions (embeddings, Qdrant client, logging)
- `app/core/` - Core configurations (database, settings)

## Security

- Passwords hashed using bcrypt
- JWT tokens for authentication with configurable expiration
- API keys stored in environment variables
- Input validation using Pydantic schemas
- CORS configured for GitHub Pages frontend

## Logging

The application includes comprehensive logging for:
- API calls with parameters and responses
- RAG process tracking
- Error logging with context