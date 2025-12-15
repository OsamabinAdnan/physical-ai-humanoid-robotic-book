# Data Model: RAG Chatbot Content Ingestion

## Entities

### ContentChunk
Represents a segment of extracted text content that will be embedded and stored in the vector database.

**Attributes**:
- `chunk_id` (Integer): Unique identifier for the chunk within the ingestion process
- `url` (String): Source URL where the content was extracted from
- `text` (String): The actual text content of the chunk
- `vector` (Array of Floats): 1024-dimensional embedding vector from Cohere
- `created_at` (DateTime): Timestamp when the chunk was created

**Relationships**:
- Belongs to a single source URL
- Stored as a single point in the Qdrant collection

### EmbeddingRecord
Represents the vector representation stored in Qdrant Cloud with associated metadata.

**Attributes**:
- `id` (Integer): Qdrant point ID for the vector
- `vector` (Array of Floats): 1024-dimensional embedding from Cohere model
- `payload` (Object): Contains metadata for retrieval
  - `url` (String): Source URL
  - `text` (String): Original text content
  - `chunk_id` (Integer): Internal chunk identifier

### URLConfiguration
Defines the GitHub Pages URLs to be crawled and ingested.

**Attributes**:
- `url` (String): The GitHub Pages URL to be processed
- `sitemap_url` (String): The sitemap.xml URL for discovering all pages
- `last_crawled` (DateTime): Timestamp of last successful crawl

## Collection Schema

### Qdrant Collection: ai_book_embedding

**Vector Configuration**:
- Size: 1024 (dimension of Cohere embeddings)
- Distance: Cosine (for similarity search)

**Payload Structure**:
```json
{
  "url": "string",
  "text": "string",
  "chunk_id": "integer"
}
```

## Validation Rules

1. **ContentChunk.text** must not be empty or null
2. **ContentChunk.url** must be a valid GitHub Pages URL
3. **ContentChunk.vector** must be exactly 1024 dimensions
4. **EmbeddingRecord.id** must be unique within the collection
5. **EmbeddingRecord.payload.url** must match the source URL format
6. **URLConfiguration.url** must be accessible and return valid HTML content

## State Transitions

### ContentChunk Lifecycle
1. **Pending**: URL extracted from sitemap, text not yet extracted
2. **Processing**: Text extracted, chunking in progress
3. **Embedded**: Text chunked and embedding generated
4. **Stored**: Vector stored in Qdrant with metadata
5. **Indexed**: Available for retrieval in the RAG system

## Relationships

- One URLConfiguration can produce many ContentChunks
- One ContentChunk maps to one EmbeddingRecord
- Many EmbeddingRecords are stored in one Qdrant collection