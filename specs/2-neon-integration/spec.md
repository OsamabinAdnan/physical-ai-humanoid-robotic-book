# Spec: Integrate Neon Serverless Postgres Database

## Objective
Integrate **Neon Serverless Postgres** into the existing FastAPI-based RAG backend to persist application data such as users, chat sessions, chat_messages, document metadata, and audit logs.

## Scope
- Database provisioning and connection
- Schema design aligned with RAG + Auth needs
- ORM / query layer integration
- Backend API updates to persist and fetch data
- Ensure compatibility with Qdrant and Agent workflow

## Functional Requirements
- Connect FastAPI backend to Neon using secure environment variables
- Store user accounts (We will make Better-auth in next phase)
- Store chat sessions and message history
- Store document ingestion metadata (URL, chunk IDs, embeddings reference)
- Support multi-user isolation (user-scoped data access)

## Technical Requirements
- Neon Serverless Postgres
- Async-compatible DB driver (`asyncpg` or SQLAlchemy async)
- Migration support (Alembic or SQL-based)
- Environment-based configuration

## Data Models (High-Level)
- **users**
  - id (UUID, PK)
  - email
  - name
  - created_at

- **chat_sessions**
  - id (UUID, PK)
  - user_id (FK)
  - session_id
  - title
  - created_at
  - updated_at

- **chat_messages**
  - id (UUID, PK)
  - session_id (FK)
  - user_id (FK)
  - role (user/assistant/system)
  - content
  - created_at
  - token_count (optional)

- **documents**
  - id (UUID, PK)
  - source_url
  - checksum
  - original_filename
  - ingestion_status
  - created_at

## Integration Steps
1. Provision Neon database and obtain connection string (Neon URL stored in .env file)
2. Add Neon credentials to backend `.env` (Added)
3. Initialize async database client in FastAPI lifecycle
4. Define schemas/models for users, chats, and documents
5. Implement CRUD utilities for chat persistence
6. Store user queries and agent responses automatically
7. Validate DB writes during RAG execution flow
8. Test database operations locally and in deployed backend (store test file in backend\test\ folder)

## Acceptance Criteria
- Backend successfully connects to Neon
- Chat history persists across sessions
- Each user sees only their own data
- No impact on existing Qdrant-based retrieval
- Backend remains stateless aside from DB

## Out of Scope
- Analytics dashboards
- Data visualization
- DB-level authorization rules

## Detailed Implementation Requirements

### 1. Database Connection Management
- Use SQLAlchemy async with asyncpg for database operations
- Implement connection pooling for performance
- Handle connection lifecycle in FastAPI lifespan events
- Add retry logic for transient connection failures

### 2. Data Models
- Use SQLAlchemy ORM with async support
- Create basic user model with email/name fields but implement session-based identification for now
- Implement proper relationships between tables
- Add proper indexing for performance
- Include created_at/updated_at timestamps with defaults

### 3. Service Layer
- Create service classes for each data model
- Implement CRUD operations with proper error handling
- Add validation for input data
- Include user isolation in all operations

### 4. API Integration
- Modify existing `/chat` endpoint to automatically store conversations while maintaining backward compatibility
- Add new endpoints for session management (list, create, delete sessions)
- Ensure backward compatibility with existing API
- Add proper authentication placeholders for future Better Auth integration

### 5. Migration Strategy
- Use Alembic with async support for database schema management
- Create proper version control for database schema changes

### 6. Data Isolation
- Implement session ID-based isolation with a basic user identifier that can be extended later
- Ensure users cannot access each other's data

### 7. Testing
- Unit tests for database operations
- Integration tests with FastAPI
- Performance tests for concurrent operations
- Test data isolation between users

### 8. Security Considerations
- Parameterized queries to prevent SQL injection
- Proper input validation
- Connection encryption
- Environment variable management

### 9. Performance Considerations
- Proper indexing strategy
- Connection pooling configuration
- Query optimization
- Caching strategy for frequently accessed data

### 10. Error Handling
- Database connection failures
- Query execution failures
- Data validation errors
- Graceful degradation when DB is unavailable

## Clarifications

### Session 2025-12-16
- Q: Database Connection Approach → A: SQLAlchemy async with asyncpg
- Q: User Authentication Implementation → A: Create basic user model with placeholder fields, use session-based identification
- Q: Migration Strategy → A: Alembic with async support
- Q: API Endpoint Changes → A: Modify existing `/chat` endpoint to automatically store conversations (maintain backward compatibility)
- Q: Data Isolation Implementation → A: Session ID-based isolation with basic user identifier

## Dependencies
- Python packages: asyncpg, sqlalchemy[asyncio], alembic
- Neon Serverless Postgres instance
- Existing FastAPI application
- Qdrant Cloud instance (for compatibility)

## Success Metrics
- Database connection success rate > 99%
- Query response time < 200ms (p95)
- Data persistence validation (data survives API restarts)
- User data isolation validation (no cross-user data access)
- No degradation in existing RAG functionality