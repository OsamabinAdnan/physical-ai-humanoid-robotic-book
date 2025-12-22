# Physical AI & Humanoid Robotics Textbook with RAG Chatbot

## Authentication System

This project now includes a custom authentication system that follows Better-Auth API patterns, providing:

- User registration with expertise level collection (software/hardware - beginner/intermediate/advanced)
- Secure login/logout functionality
- JWT-based session management
- Protected API endpoints for chat functionality
- Data isolation between authenticated users

### API Endpoints

#### Authentication Endpoints

- `POST /api/auth/register` - Register a new user with expertise levels
- `POST /api/auth/login` - Authenticate user with email/password
- `POST /api/auth/logout` - End current user session

#### Protected Endpoints

- `POST /chat` - Chat with authenticated user (requires JWT token)
- `GET /chat-history` - Get chat history for authenticated user (requires JWT token)
- `GET /chat-history/session/{session_id}` - Get specific session history (requires JWT token)

For more details, see the backend/auth/README.md file.

## Project Overview

This is a comprehensive educational platform that combines a textbook on Physical AI and Humanoid Robotics with an AI-powered chatbot that can answer questions about the textbook content. The project consists of a Docusaurus-based frontend textbook and a FastAPI backend with RAG (Retrieval-Augmented Generation) capabilities.

## Table of Contents

- [Architecture Components](#architecture-components)
- [Key Features](#key-features)
- [Technology Stack](#technology-stack)
- [Directory Structure](#directory-structure)
- [Data Flow](#data-flow)
- [Installation](#installation)
- [Usage](#usage)
- [API Endpoints](#api-endpoints)
- [Frontend Components](#frontend-components)
- [Backend Services](#backend-services)
- [Database Schema](#database-schema)
- [Testing](#testing)
- [Contributing](#contributing)
- [License](#license)

## Architecture Components

### 1. Frontend (book-frontend)
- Built with Docusaurus, a modern static site generator
- Contains the complete textbook content in markdown format across 5 modules
- Features an integrated chatbot component with:
  - Real-time conversation interface
  - Text selection functionality (users can select text and ask AI about it)
  - Chat history management
  - Citation display with confidence scores

### 2. Backend (backend)
- FastAPI-based REST API with multiple endpoints
- RAG agent implementation using:
  - OpenRouter LLM (mistralai/devstral-2512:free model) as the language model
  - Qdrant Cloud for vector storage of textbook embeddings
  - Sentence transformers for generating text embeddings
- Neon Serverless Postgres database for:
  - User management
  - Chat session persistence
  - Message history storage
  - Document metadata

### 3. Textbook Content
- Organized into 5 comprehensive modules:
  - Module 1: The Robotic Nervous System (ROS 2)
  - Module 2: The Digital Twin (Gazebo & Unity)
  - Module 3: The AI-Robot Brain (NVIDIA Isaac™)
  - Module 4: Vision-Language-Action (VLA)
  - Module 5: Course Conclusion and Next Steps

## Key Features

1. **RAG Chatbot Functionality**
   - Users can ask questions about the textbook content
   - The system retrieves relevant text chunks from the vector database
   - Answers are generated based on the retrieved context
   - Citations are provided with confidence scores

2. **Text Selection Feature**
   - Users can highlight text in the textbook and get AI explanations
   - A tooltip appears when text is selected, allowing direct interaction with the chatbot

3. **Chat History**
   - Conversations are persisted in the Neon database
   - Users can view and continue previous chat sessions
   - Each user's data is isolated from others

4. **API Endpoints**
   - `/chat` - Main endpoint for chatbot interactions
   - `/chat-history/{user_id}` - Retrieve user's chat sessions
   - `/chat-history/{user_id}/session/{session_id}` - Retrieve specific session messages
   - `/health` and `/db-health` - Health check endpoints

## Technology Stack

### Backend:
- Python with FastAPI
- OpenRouter API for language model (using mistralai/devstral-2512:free model)
- Qdrant Cloud for vector storage
- Sentence Transformers for embeddings
- Neon Serverless Postgres with SQLAlchemy
- AsyncPG for async database operations

### Frontend:
- React with TypeScript
- Docusaurus framework
- CSS modules for styling

### Infrastructure:
- GitHub Pages for hosting the textbook
- Hugging Face Space for backend hosting

## Directory Structure

```
physical-ai-humanoid-robotic-book/
├── .claude/                 # Claude Code configuration
├── .specify/                # SpecKit Plus templates and scripts
├── backend/                 # FastAPI backend
│   ├── agent/              # Agent implementation
│   ├── database/           # Database models and services
│   ├── embedding/          # Embedding ingestion functionality
│   ├── migrations/         # Database migration files
│   ├── test/               # Backend tests
│   ├── main.py             # Main FastAPI application
│   ├── requirements.txt    # Backend dependencies
│   └── alembic.ini         # Database migration configuration
├── book-frontend/          # Docusaurus frontend
│   ├── docs/               # Textbook content in markdown
│   ├── src/                # React components
│   │   └── components/
│   │       └── Chatbot/    # Chatbot component
│   ├── static/             # Static assets
│   ├── package.json        # Frontend dependencies
│   └── docusaurus.config.ts # Docusaurus configuration
├── docs/                   # Documentation files
├── specs/                  # Project specifications
│   ├── 001-robotics-textbook/ # Textbook spec
│   ├── 1-rag-chatbot/      # RAG chatbot spec
│   └── 2-neon-integration/ # Neon database integration spec
├── history/                # Prompt History Records and ADRs
│   ├── prompts/            # Prompt history
│   └── adr/                # Architecture Decision Records
├── .env                    # Environment variables
├── .gitignore              # Git ignore rules
├── CLAUDE.md               # Claude Code rules
└── README.md               # This file
```

## Data Flow

1. Textbook content is embedded and stored in Qdrant vector database
2. User asks a question via the frontend chatbot
3. Question is sent to the backend API
4. Backend generates embeddings for the question
5. Vector search retrieves relevant textbook chunks from Qdrant
6. LLM generates response based on retrieved context
7. Response is sent back to frontend with citations
8. Conversation is stored in Neon database

## Installation

### Backend Setup

1. Navigate to the backend directory:
```bash
cd backend
```

2. Create a virtual environment:
```bash
python -m venv .venv
source .venv/bin/activate  # On Windows: .venv\Scripts\activate
```

3. Install dependencies:
```bash
pip install -r requirements.txt
```

4. Set up environment variables in `.env`:
```env
OPENROUTER_API_KEY=your_openrouter_api_key
OPENROUTER_URL=your_openrouter_url
QDRANT_CLUSTER_ENDPOINT=your_qdrant_cluster_endpoint
QDRANT_API_KEY=your_qdrant_api_key
NEON_DATABASE_URL=your_neon_database_url
```

5. Run database migrations:
```bash
alembic upgrade head
```

6. Start the backend server:
```bash
uvicorn main:app --reload
```

### Frontend Setup

1. Navigate to the frontend directory:
```bash
cd book-frontend
```

2. Install dependencies:
```bash
npm install
```

3. Set up environment variables in `.env`:
```env
REACT_APP_BACKEND_URL=your_backend_url
```

4. Start the development server:
```bash
npm start
```

## Usage

1. The textbook will be available at `http://localhost:3000`
2. The chatbot is integrated into the textbook pages
3. Users can ask questions about the textbook content directly in the chat interface
4. Text can be selected in the textbook and sent to the chatbot for explanation
5. Previous conversations are stored and can be accessed through the history panel

## API Endpoints

### Backend API

- `GET /` - Root endpoint with API information
- `POST /chat` - Chat with the RAG agent
- `GET /chat-history/{user_id}` - Get user's chat sessions
- `GET /chat-history/{user_id}/session/{session_id}` - Get specific session messages
- `GET /health` - Health check for the backend
- `GET /db-health` - Health check for the database

### Request/Response Examples

**Chat Request:**
```json
{
  "question": "What is Physical AI?",
  "top_k": 5
}
```

**Chat Response:**
```json
{
  "answer": "Physical AI is...",
  "citations": [
    {
      "text": "Physical AI definition...",
      "url": "https://...",
      "similarity_score": 0.85,
      "chunk_id": 1,
      "source_title": "Module 1 Chapter 1"
    }
  ],
  "confidence": 0.85,
  "processing_time_ms": 1200
}
```

## Frontend Components

### Chatbot Component (`book-frontend/src/components/Chatbot`)

The chatbot component includes:

- **Chatbot.tsx**: Main chatbot component with UI logic
- **api.ts**: API service for backend communication
- **types.ts**: TypeScript interfaces
- **Chatbot.module.css**: Component-specific styles
- **README.md**: Component documentation

Key features:
- Real-time messaging interface
- Text selection tooltip
- Chat history management
- Citation display
- Confidence scoring
- Error handling and retry functionality

### Textbook Structure (`book-frontend/docs`)

The textbook is organized into:

- **Module 1**: The Robotic Nervous System (ROS 2)
- **Module 2**: The Digital Twin (Gazebo & Unity)
- **Module 3**: The AI-Robot Brain (NVIDIA Isaac™)
- **Module 4**: Vision-Language-Action (VLA)
- **Module 5**: Course Conclusion and Next Steps

Each module contains multiple chapters with detailed content on Physical AI and Humanoid Robotics.

## Backend Services

### Agent Service (`backend/agent`)

- **agent.py**: Main agent implementation with RAG functionality
- **schemas.py**: Pydantic schemas for request/response validation
- **test_retriever.py**: Tests for the retrieval functionality

The agent service handles:
- Text embedding generation
- Vector search in Qdrant
- LLM integration with OpenRouter (mistralai/devstral-2512:free model)
- Tool-based retrieval for RAG

### Database Service (`backend/database`)

- **models.py**: SQLAlchemy models for database entities
- **services.py**: Service layer with CRUD operations
- **schemas.py**: Pydantic schemas for database entities

Database entities:
- **User**: User account information
- **ChatSession**: Conversation metadata
- **ChatMessage**: Individual messages in conversations
- **Document**: Document ingestion metadata

### Embedding Service (`backend/embedding`)

- **ingestor.py**: Content extraction and embedding generation
- **reset_collection.py**: Utility to reset Qdrant collections

## Database Schema

### Users Table
- `id`: UUID (Primary Key)
- `email`: String (Unique)
- `name`: String
- `created_at`: DateTime
- `updated_at`: DateTime

### Chat Sessions Table
- `id`: UUID (Primary Key)
- `user_id`: UUID (Foreign Key)
- `session_id`: String
- `title`: String
- `created_at`: DateTime
- `updated_at`: DateTime

### Chat Messages Table
- `id`: UUID (Primary Key)
- `session_id`: UUID (Foreign Key)
- `user_id`: UUID (Foreign Key)
- `role`: String ('user' or 'assistant')
- `content`: Text
- `created_at`: DateTime
- `token_count`: Integer

### Documents Table
- `id`: UUID (Primary Key)
- `source_url`: Text
- `checksum`: String
- `original_filename`: String
- `ingestion_status`: String
- `created_at`: DateTime

## Testing

The project includes comprehensive testing:

### Backend Tests (`backend/test`)

- **test_api.py**: API endpoint tests
- **test_chat_persistence.py**: Chat persistence tests
- **test_connection_stability.py**: Connection stability tests
- **test_neon_integration.py**: Neon database integration tests
- **test_real_data_flow.py**: Real data flow validation

### Frontend Tests

Tests for the chatbot component and API integration are included in the component directory.

## Deployment

### Frontend Deployment
The frontend is designed for GitHub Pages deployment:
1. Build the project: `npm run build`
2. Deploy to GitHub Pages using the configured base URL

### Backend Deployment
The backend can be deployed to Hugging Face Spaces or similar platforms:
1. Ensure all dependencies are in requirements.txt
2. Configure environment variables
3. Deploy the FastAPI application

## Environment Variables

### Backend (.env)
```
OPENROUTER_API_KEY=your_openrouter_api_key
OPENROUTER_URL=your_openrouter_url
QDRANT_CLUSTER_ENDPOINT=your_qdrant_cluster_endpoint
QDRANT_API_KEY=your_qdrant_api_key
NEON_DATABASE_URL=your_neon_database_url
```

### Frontend (.env)
```
REACT_APP_BACKEND_URL=your_backend_url
```

## Contributing

1. Fork the repository
2. Create a feature branch (`git checkout -b feature/amazing-feature`)
3. Make your changes
4. Commit your changes (`git commit -m 'Add some amazing feature'`)
5. Push to the branch (`git push origin feature/amazing-feature`)
6. Open a Pull Request

## License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.

## Acknowledgments

- GIAIC (Government Innovation and Artificial Intelligence Center) for supporting this educational initiative
- NVIDIA for Isaac™ platform support
- Open Robotics for ROS 2 development
- OpenAI for Whisper technology
- The robotics research community for continuous innovation

## Support

If you encounter any issues or have questions about the project, please open an issue in the GitHub repository.

---

**Ready to start your journey in Physical AI and Humanoid Robotics? Explore the textbook and interact with the AI assistant to enhance your learning experience!**