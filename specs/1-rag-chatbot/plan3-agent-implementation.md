# RAG Chatbot – Spec-3 Implementation Plan

## Technical Context

**Feature**: RAG Chatbot - Phase 3: Agent-Based Backend with Retrieval
**Implementation Approach**: Python-based FastAPI server with OpenAI Agent SDK integration and Qwen API for LLM functionality
**Target URLs**: https://osamabinadnan.github.io/physical-ai-humanoid-robotic-book/
**Primary LLM Provider**: Qwen API (at https://portal.qwen.ai/v1/) accessed through OpenAI-compatible client
**Vector Database**: Qdrant Cloud Free Tier
**Collection Name**: ai_book_embedding
**Retrieval Tool**: backend/agent/retriever.py (retrieve function as function_tool)

**Key Components**:
- FastAPI backend server
- OpenAI Agent SDK with Qwen as underlying LLM
- Integration with retrieval tool from Phase 2
- API endpoints for user queries
- Response structuring with citations

## Constitution Check

Based on the project constitution, this implementation plan aligns with:
- **VII. Chunkable, RAG-Ready Content**: Enhancing retrieval capabilities with agent-based interaction
- **XIV. Deployment Guarantee**: Ensuring content integrates with the RAG backend and API
- **V. Engineering Reproducibility**: Creating runnable code with verifiable examples

## Gates

**✅ Ready to Proceed** - All clarifications from the specification have been resolved:
- Security: Comprehensive security measures implemented
- Timeout: 10-second timeout for Qwen API requests with retry mechanism
- Concurrency: Support for up to 100 concurrent requests with proper queuing

## Phase 0: Research & Resolution

### Research Findings

**Decision**: Use Qwen API at https://portal.qwen.ai/v1/ as the LLM provider for the agent
**Rationale**:
- User specifically requested Qwen API usage
- Qwen API key is already available in .env file
- Appropriate for the project requirements
**Alternatives considered**: OpenAI GPT models, Anthropic Claude, local models

**Decision**: Integrate retrieve function from backend/agent/retriever.py as function_tool for the agent
**Rationale**:
- Leverages existing retrieval functionality from Phase 2
- Maintains consistency with the embedding approach (sentence-transformers)
- Enables agent to access book content through semantic search
**Alternatives considered**: Direct Qdrant access from agent, reimplementing retrieval in agent module

**Decision**: Implement FastAPI with async support for handling concurrent requests
**Rationale**:
- FastAPI provides excellent performance for API endpoints
- Built-in async support for handling concurrent requests
- Good integration with OpenAI Agent SDK
- Provides automatic API documentation
**Alternatives considered**: Flask, Django REST Framework, Starlette

## Phase 1: Design & Architecture

### Data Model

**UserQuery Entity**:
- `query_text` (String): The original user question
- `timestamp` (DateTime): When the query was submitted
- `user_id` (Optional String): Identifier for the user (if needed later)

**AgentResponse Entity**:
- `answer_text` (String): The agent's response to the user query
- `confidence_level` (Float): Confidence score of the response (0.0-1.0)
- `source_citations` (Array of Citation objects): References to retrieved content
- `processing_time_ms` (Integer): Time taken to generate the response

**Citation Entity**:
- `text` (String): The text content that was cited
- `url` (String): Source URL of the content
- `similarity_score` (Float): Relevance score of the citation
- `chunk_id` (Integer): ID of the chunk in Qdrant

### System Architecture

```
[FastAPI endpoint] -> [OpenAI Agent] -> [function_tool: retrieve] -> [Qdrant Cloud] -> [Response formatting] -> [Structured output]
```

**Module Structure**:
- `backend/agent/main.py`: FastAPI application entry point
- `backend/agent/agent_handler.py`: Agent initialization and processing logic
- `backend/agent/schemas.py`: Pydantic schemas for request/response validation

### API Contract

The system will implement a FastAPI endpoint with the following interface:
- `POST /chat`: Accepts a user question and returns an AI-generated response with citations
- Request body: `{"question": "string"}`
- Response: `{"answer": "string", "citations": [{"text": "string", "url": "string", "similarity_score": "float", "chunk_id": "int"}]}`

## Phase 2: Implementation Plan

### Task Breakdown

1. **Environment Setup**
   - Configure environment variables loading for Qwen API
   - Install required dependencies (fastapi, uvicorn, openai-agents, etc.)

2. **Create Agent Handler**
   - Initialize OpenAI Agent with Qwen API connection
   - Register the retrieve function as a function_tool
   - Implement agent processing logic

3. **Design API Schemas**
   - Create Pydantic models for request/response validation
   - Define proper data structures for queries and responses

4. **Implement FastAPI Endpoints**
   - Create the main query endpoint
   - Add proper error handling and validation

5. **Integrate Agent with API**
   - Connect the agent processing to the API endpoint
   - Handle response formatting with citations

6. **Add Security Measures**
   - Implement API rate limiting
   - Add request validation
   - Secure sensitive data handling

7. **Add Concurrency Handling**
   - Configure FastAPI for concurrent request handling
   - Implement proper queuing mechanism for agent requests

8. **Testing & Validation**
   - Test API endpoint with sample queries
   - Verify agent responses are grounded in retrieved content
   - Confirm concurrent request handling works properly

9. **Documentation**
   - Create API documentation
   - Document how to run and configure the agent system

## Phase 3: Integration Strategy

### Prerequisites
- Python 3.8+ installed
- Valid Qwen API key in .env file
- Working Qdrant Cloud connection
- Completed retrieval functionality from Phase 2

### Dependencies
- fastapi
- uvicorn
- python-dotenv
- openai-agents (or appropriate Qwen SDK)
- pydantic

### Error Handling
- Qwen API unavailability
- Qdrant connection issues
- Invalid user queries
- Rate limiting scenarios

## Risk Assessment

- **API Availability**: Both Qwen and Qdrant APIs could be temporarily unavailable
- **Concurrency**: Handling 100 concurrent requests may require proper queuing mechanisms
- **Response Quality**: Ensuring agent responses are properly grounded in retrieved content
- **Latency**: Multiple API calls could increase response times

## Success Criteria

- FastAPI backend starts and runs reliably
- OpenAI Agent correctly initialized with Qwen API
- Agent can retrieve relevant content from Qdrant when needed
- Agent generates answers grounded only in retrieved content
- API endpoint responds to queries within 10 seconds
- System handles 100 concurrent requests properly
- Response quality maintains low hallucination rate (<5%)