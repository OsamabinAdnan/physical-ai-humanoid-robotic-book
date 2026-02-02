# API Contract: Agent-Based RAG Chatbot Backend

## Overview
This document defines the API contract for the RAG Chatbot agent-based backend that accepts user questions and returns AI-generated responses grounded in book content.

## API Endpoints

### POST /query
Process a user question and return an AI-generated response with citations.

**Request**:
```json
{
  "question": "string",
  "top_k": "integer (optional, default: 5)"
}
```

**Response** (Success - 200 OK):
```json
{
  "answer": "string",
  "citations": [
    {
      "text": "string",
      "url": "string",
      "similarity_score": "float",
      "chunk_id": "integer",
      "source_title": "string"
    }
  ],
  "confidence": "float",
  "processing_time_ms": "integer"
}
```

**Response** (Error - 400/500):
```json
{
  "error": "string",
  "details": "string (optional)",
  "request_id": "string"
}
```

### GET /health
Check the health status of the agent backend.

**Response** (Success - 200 OK):
```json
{
  "status": "healthy",
  "timestamp": "datetime string",
  "services": {
    "qwen_api": "connected/disconnected",
    "qdrant_cloud": "connected/disconnected",
    "agent": "ready/not_ready"
  }
}
```

## Agent Interface Contract

### Agent Initialization
```
initialize_agent(qwen_api_key: str, qwen_endpoint: str) -> Agent
```

**Description**: Initializes the OpenAI Agent with Qwen API configuration.

**Parameters**:
- `qwen_api_key` (str): Valid Qwen API key
- `qwen_endpoint` (str): Qwen API endpoint URL

**Returns**: Initialized Agent object with function tools registered

**Errors**: Throws exception if API key is invalid or endpoint unreachable

### Agent Processing
```
process_query(agent: Agent, query: str, top_k: int = 5) -> Dict
```

**Description**: Processes a user query through the agent, including retrieval and response generation.

**Parameters**:
- `agent` (Agent): Initialized agent instance
- `query` (str): User's question
- `top_k` (int): Number of top results to retrieve (default: 5)

**Returns**: Dictionary containing answer and citations

**Errors**: Throws exception if agent processing fails

## Function Tool Contract

### Retrieval Tool Integration
The `retrieve` function from `backend/agent/retriever.py` will be registered as a function_tool with the agent.

**Signature**: `retrieve(query: str, top_k: int = 5) -> Dict[str, Any]`

**Purpose**: Allow the agent to retrieve relevant content chunks from Qdrant when answering questions.

**Input**: User query string and number of results to retrieve.

**Output**: Dictionary with `retrieved_texts` and `citations` arrays.

## Data Validation Contract

### Request Validation
- `question` field must be a non-empty string (1-1000 characters)
- `top_k` field must be an integer between 1 and 20 (if provided)
- Request body must be valid JSON

### Response Validation
- `answer` field must be a non-empty string
- `citations` array must contain properly structured citation objects
- `confidence` field must be a float between 0.0 and 1.0
- `processing_time_ms` must be a positive integer

## Performance Contract

### Expected Response Times
- Simple queries: < 3 seconds
- Complex queries requiring retrieval: < 10 seconds
- API endpoint overall: < 15 seconds

### Concurrency
- Support up to 100 concurrent requests
- Proper queuing mechanism for agent requests
- Maintain response quality under load

## Error Handling Contract

### Recoverable Errors
- Qwen API timeout: Retry with exponential backoff (up to 3 attempts)
- Qdrant connection failure: Return appropriate error message with status 503
- Empty retrieval results: Agent should acknowledge and provide best possible answer

### Non-Recoverable Errors
- Invalid Qwen API key: Return 500 error with connection failure details
- Malformed request: Return 400 error with validation details
- Agent initialization failure: Return 500 error with details

## Security Contract

### API Protection
- Input sanitization to prevent injection attacks
- Rate limiting to prevent abuse
- Proper error message sanitization (no internal details leaked)

### Data Handling
- No sensitive data logged in plain text
- Secure handling of API keys and credentials
- Proper validation of user inputs

## Configuration Contract

### Required Environment Variables
- `QWEN_API_KEY`: Valid Qwen API key for agent
- `QWEN_URL`: Qwen API endpoint (default: https://portal.qwen.ai/v1/)
- `QDRANT_CLUSTER_ENDPOINT`: Qdrant Cloud endpoint URL
- `QDRANT_API_KEY`: Qdrant API key

### Optional Configuration
- `CONCURRENT_REQUESTS_LIMIT`: Maximum concurrent requests (default: 100)
- `AGENT_TIMEOUT_SECONDS`: Request timeout for agent processing (default: 10)
- `DEFAULT_TOP_K`: Default number of results to retrieve (default: 5)