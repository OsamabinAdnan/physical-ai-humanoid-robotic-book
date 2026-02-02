# Neon DB Integration Plan

## Overview
This plan outlines the implementation of Neon Serverless Postgres integration into the existing RAG chatbot backend, following the established spec and requirements.

## 1. Scope and Dependencies

### In Scope:
- Neon Serverless Postgres project setup and connection string configuration
- Database credentials management in backend environment variables
- Async database engine and session management within FastAPI
- Core table creation (users, chat_sessions, chat_messages, metadata)
- Schema initialization/migration implementation
- CRUD utilities for chat history and user data
- DB health-check endpoint
- Connection stability verification

### Out of Scope:
- User authentication (handled in Better Auth phase)
- Frontend integration
- Analytics features
- Advanced DB optimization beyond basic indexing

### External Dependencies:
- Neon Serverless Postgres instance (already provisioned)
- FastAPI application framework
- SQLAlchemy async with asyncpg
- Alembic for migrations
- Existing Qdrant integration (maintain compatibility)

## 2. Key Decisions and Rationale

### Database Connection:
- **Decision**: Use SQLAlchemy async with asyncpg
- **Rationale**: Provides async support required for FastAPI, ORM capabilities for easier development, and connection pooling for performance
- **Trade-offs**: Slight overhead vs raw asyncpg, but significant development benefits

### Schema Management:
- **Decision**: Use Alembic for migrations
- **Rationale**: Provides version control for database schema, safe deployment of schema changes
- **Trade-offs**: Additional complexity vs auto-create, but much safer for production

### User Isolation:
- **Decision**: Session ID-based isolation with basic user identifier
- **Rationale**: Enables data separation that can be extended when Better Auth is implemented
- **Trade-offs**: Temporary approach until full authentication

## 3. Interfaces and API Contracts

### Database Models:
- `users`: id (UUID), email, name, created_at
- `chat_sessions`: id (UUID), user_id (FK), session_id, title, created_at, updated_at
- `chat_messages`: id (UUID), session_id (FK), user_id (FK), role, content, created_at, token_count
- `documents`: id (UUID), source_url, checksum, original_filename, ingestion_status, created_at

### Public APIs (Internal):
- `get_db_session()`: Get async database session
- `user_service.create_user()`: Create new user record
- `chat_service.create_session()`: Create new chat session
- `chat_service.add_message()`: Add message to session
- `chat_service.get_session_messages()`: Retrieve session messages
- `health_check_db()`: Verify database connectivity

### API Endpoints:
- `/db-health`: Database health check endpoint

## 4. Non-Functional Requirements (NFRs) and Budgets

### Performance:
- p95 latency: <200ms for DB operations
- Connection pooling: Max 10 connections
- Query optimization: Proper indexing on foreign keys

### Reliability:
- SLOs: 99.9% uptime for DB operations
- Error handling: Graceful degradation if DB unavailable
- Connection resilience: Automatic reconnection

### Security:
- Connection encryption: Always use SSL (already configured in Neon URL)
- Credential management: Environment variables only
- SQL injection prevention: Parameterized queries

### Cost:
- Serverless pricing: Pay-per-use model
- Connection limits: Stay within reasonable limits

## 5. Data Management and Migration

### Source of Truth:
- Alembic migration files in `backend/migrations/`
- SQLAlchemy models as code representation
- Neon database as runtime state

### Schema Evolution:
- Use Alembic for versioned migrations
- Follow forward-only migration strategy
- Include rollback scripts where possible

### Migration and Rollback:
- Initial schema migration for all tables
- Add indexes for performance
- Plan for future schema changes with safe migration patterns

## 6. Operational Readiness

### Observability:
- Log all database operations with timing
- Monitor connection pool metrics
- Track query performance

### Alerting:
- Database connection failures
- Slow query alerts (>500ms)
- Connection pool exhaustion

### Runbooks:
- Database connection troubleshooting
- Migration execution procedures
- Performance optimization steps

### Deployment:
- Environment-specific configurations
- Migration execution during deployment
- Health check for database connectivity

## 7. Risk Analysis and Mitigation

### Top 3 Risks:
1. **Connection Pool Exhaustion** (High)
   - Blast radius: All DB operations fail
   - Mitigation: Proper connection management, monitoring
   - Kill switch: Circuit breaker for DB operations

2. **Data Isolation Failure** (High)
   - Blast radius: Users see other users' data
   - Mitigation: Strict user_id filtering in queries
   - Kill switch: Disable multi-user functionality

3. **Performance Degradation** (Medium)
   - Blast radius: Slow API responses
   - Mitigation: Proper indexing, query optimization
   - Kill switch: Disable DB persistence temporarily

## 8. Evaluation and Validation

### Definition of Done:
- [ ] Neon database project created and connection string obtained
- [ ] DB credentials added to backend environment variables
- [ ] Async database engine and session management set up in FastAPI
- [ ] Core tables created (users, chat_sessions, chat_messages, metadata)
- [ ] Schema initialization/migration script implemented
- [ ] CRUD utilities for chat history and user data added
- [ ] DB health-check endpoint added
- [ ] Persistence and connection stability verified
- [ ] Existing Qdrant functionality unaffected
- [ ] All tests pass successfully

### Output Validation:
- Schema validation against requirements
- Data integrity checks
- Security validation (no SQL injection)
- Performance validation
- Connection stability under load

## 9. Implementation Steps

### Step 1: Infrastructure Setup
- Verify Neon database connection string
- Add DB credentials to environment variables
- Set up SQLAlchemy async engine
- Implement FastAPI lifespan events for DB connection

### Step 2: Data Models and Schema
- Define SQLAlchemy models for all required tables
- Set up Alembic for migrations
- Create initial migration
- Apply migration to Neon database

### Step 3: Service Layer
- Implement CRUD utilities for each model
- Add proper error handling
- Implement user isolation logic

### Step 4: API Integration
- Add DB health-check endpoint
- Integrate with existing chat functionality
- Verify persistence works correctly

### Step 5: Testing and Validation
- Unit tests for database operations
- Integration tests
- Performance testing
- Connection stability verification