# Data Model: RAG Chatbot Agent Implementation

## Entities

### UserQuery
Represents a user's question submitted to the agent system.

**Attributes**:
- `query_text` (String): The original user question text
- `timestamp` (DateTime): When the query was submitted
- `request_id` (String): Unique identifier for the request
- `user_context` (Optional Object): Additional context about the user (if implemented later)

**Relationships**:
- Generates one AgentResponse
- May trigger multiple RetrievalResult calls

### AgentResponse
Contains the agent's answer to the user query with source citations.

**Attributes**:
- `response_text` (String): The agent's generated answer
- `confidence_level` (Float): Confidence score of the response (0.0-1.0)
- `source_citations` (Array of Citation objects): References to retrieved content
- `processing_time_ms` (Integer): Time taken to generate the response
- `request_id` (String): Links to the original UserQuery
- `timestamp` (DateTime): When the response was generated

**Relationships**:
- Belongs to one UserQuery
- Contains multiple Citation objects

### Citation
Contains information about a specific piece of retrieved content used in the agent's response.

**Attributes**:
- `text` (String): The actual text content that was cited
- `url` (String): Source URL of the content chunk
- `similarity_score` (Float): Relevance score of the citation (0.0-1.0)
- `chunk_id` (Integer): ID of the chunk in Qdrant
- `source_title` (String): Title of the source document (if available)

**Relationships**:
- Belongs to one AgentResponse
- References content in Qdrant collection

### AgentSession
Tracks the interaction between the user and the agent (optional for future enhancement).

**Attributes**:
- `session_id` (String): Unique identifier for the session
- `start_time` (DateTime): When the session began
- `end_time` (Optional DateTime): When the session ended
- `query_count` (Integer): Number of queries in the session

**Relationships**:
- Contains multiple UserQuery objects
- Contains multiple AgentResponse objects

## API Schema Definitions

### QueryRequest
Schema for the incoming API request.

```json
{
  "question": "string",
  "top_k": "integer (optional, default: 5)"
}
```

### QueryResponse
Schema for the outgoing API response.

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

## Collection Schema (Reference)

### Qdrant Collection: ai_book_embedding (external reference)

**Vector Configuration**:
- Size: 384 (dimension of sentence-transformers embeddings)
- Distance: Cosine (for similarity search)

**Payload Structure**:
```json
{
  "url": "string",
  "text": "string",
  "chunk_id": "integer",
  "source_title": "string"
}
```

## Validation Rules

1. **UserQuery.query_text** must not be empty or exceed 1000 characters
2. **Citation.similarity_score** must be between 0.0 and 1.0
3. **Citation.chunk_id** must be a positive integer
4. **AgentResponse.processing_time_ms** must be positive
5. **AgentResponse.confidence_level** must be between 0.0 and 1.0
6. **QueryRequest.question** must be a non-empty string
7. **QueryRequest.top_k** must be between 1 and 20 if provided

## State Transitions

### AgentResponse Lifecycle
1. **Received**: User query received by API
2. **Processing**: Agent retrieving relevant content and generating response
3. **Validated**: Response validated against retrieved content
4. **Formatted**: Response formatted with proper citations
5. **Delivered**: Response sent back to user

## Relationships

- One UserQuery generates one AgentResponse
- One AgentResponse contains many Citations
- Many Citations reference the Qdrant collection
- One AgentSession contains many UserQueries and AgentResponses (future enhancement)