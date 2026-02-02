# Data Model: RAG Chatbot Retrieval Functionality

## Entities

### Query
Represents a user's text input for semantic search with its embedding vector.

**Attributes**:
- `query_text` (String): The original user query text
- `embedding_vector` (Array of Floats): 384-dimensional embedding vector generated from the query text
- `timestamp` (DateTime): When the query was processed

**Relationships**:
- Produces many SearchResult objects
- Associated with the retrieval session

### SearchResult
Contains the retrieved text chunks with metadata and similarity scores for a given query.

**Attributes**:
- `query_id` (String): Identifier linking to the original query
- `retrieved_chunks` (Array of RetrievedChunk): The top-K chunks retrieved for the query
- `search_timestamp` (DateTime): When the search was performed
- `execution_time_ms` (Integer): Time taken to perform the search

**Relationships**:
- Belongs to a single Query
- Contains many RetrievedChunk objects

### RetrievedChunk
A text chunk retrieved from the vector database with associated metadata and similarity score.

**Attributes**:
- `chunk_id` (Integer): Unique identifier from Qdrant collection
- `text` (String): The actual content of the retrieved chunk
- `url` (String): Source URL where the content originated
- `similarity_score` (Float): Cosine similarity score relative to the query
- `position_in_results` (Integer): Position in the ranked results (0 to K-1)

**Relationships**:
- Belongs to a single SearchResult
- References original content in the Qdrant collection

## Collection Schema

### Qdrant Collection: ai_book_embedding (reference)

**Vector Configuration**:
- Size: 384 (dimension of sentence-transformers embeddings)
- Distance: Cosine (for similarity search)

**Payload Structure** (existing in Qdrant):
```json
{
  "url": "string",
  "text": "string",
  "chunk_id": "integer"
}
```

## Validation Rules

1. **Query.query_text** must not be empty or null
2. **Query.embedding_vector** must be exactly 384 dimensions
3. **RetrievedChunk.similarity_score** must be between 0 and 1
4. **RetrievedChunk.url** must match the source URL format from the textbook site
5. **SearchResult.retrieved_chunks** must not exceed K items (default K=5)
6. **RetrievedChunk.text** must not be empty
7. **SearchResult.execution_time_ms** must be positive

## State Transitions

### RetrievedChunk Lifecycle
1. **Queued**: Query received, chunk is identified in vector database
2. **Retrieved**: Chunk successfully fetched from Qdrant with metadata
3. **Ranked**: Chunk positioned based on similarity score
4. **Returned**: Chunk delivered to client with complete metadata

## Relationships

- One Query produces one SearchResult
- One SearchResult contains many RetrievedChunks
- Many RetrievedChunks map to the original content in Qdrant collection
- Each RetrievedChunk maintains a reference to its original URL and chunk ID