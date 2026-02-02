# API Contract: Content Ingestion Service

## Overview
This document defines the contract for the content ingestion service that handles crawling, embedding, and storing textbook content in the vector database.

## Model Selection Decision

Based on research and your requirements:
- **Selected Model**: `embed-multilingual-v3.0`
- **Rationale**: The multilingual model is chosen for its 1024-dimensional embeddings and 512 max input tokens, which provides good performance for the project while supporting multiple languages if needed in the future.
- **Model Specifications**:
  - Vector dimension: 1024
  - Max input tokens: 512
  - Supports 100+ languages
- **Chunk Size**: 1200 characters (approximately equivalent to 512 tokens for typical English text)

## Service Interface

### Main Ingestion Function
```
ingest_book() -> None
```

**Description**: Main entry point that orchestrates the entire ingestion pipeline from URL extraction to vector storage.

**Parameters**: None (uses configuration from environment variables)

**Return**: None (prints progress and completion status)

**Side Effects**: Creates or recreates the Qdrant collection and populates it with embedded content chunks.

## Core Functions

### URL Extraction
```
get_all_urls(sitemap_url: str) -> List[str]
```

**Description**: Extracts all URLs from the provided sitemap.xml file.

**Parameters**:
- `sitemap_url` (str): URL to the sitemap.xml file

**Return**: List of all discovered URLs

**Errors**: Raises exception if sitemap cannot be accessed or parsed

### Text Extraction
```
extract_text_from_url(url: str) -> str
```

**Description**: Extracts clean, readable text from a web page.

**Parameters**:
- `url` (str): URL of the page to extract text from

**Return**: Extracted text content, or empty string if extraction fails

**Errors**: Returns empty string if text extraction fails

### Text Chunking
```
chunk_text(text: str, max_chars: int = 1200) -> List[str]
```

**Description**: Splits a text string into smaller chunks of specified maximum size.

**Parameters**:
- `text` (str): Text to be chunked
- `max_chars` (int): Maximum character count per chunk (default: 1200)

**Return**: List of text chunks

**Behavior**: Attempts to split at sentence boundaries when possible

### Embedding Generation
```
embedding(text: str) -> List[float]
```

**Description**: Generates a 1024-dimensional embedding vector for the input text using Cohere's embed-multilingual-v3.0 model.

**Parameters**:
- `text` (str): Text to generate embedding for

**Return**: 1024-dimensional embedding vector as a list of floats

**Errors**: Raises exception if Cohere API call fails

### Collection Management
```
create_collection(collection_name: str) -> None
```

**Description**: Creates or recreates a Qdrant collection with the specified name and configuration.

**Parameters**:
- `collection_name` (str): Name of the collection to create

**Return**: None

**Side Effects**: Creates or recreates the specified collection in Qdrant Cloud

### Vector Storage
```
save_chunk_to_qdrant(chunk: str, chunk_id: int, chunk_url: str) -> None
```

**Description**: Generates an embedding for the text chunk using embed-multilingual-v3.0 and saves it to Qdrant with metadata.

**Parameters**:
- `chunk` (str): Text chunk to embed and store
- `chunk_id` (int): Unique identifier for the chunk
- `chunk_url` (str): Source URL of the chunk

**Return**: None

**Side Effects**: Adds a new point to the Qdrant collection

## Data Contracts

### Input Data
- URLs must be valid, accessible web pages
- Text content should be within the model's token limit (max ~512 tokens, approx. 1200 characters)
- Sitemap.xml must be in standard XML sitemap format

### Output Data
- Embedding vectors are 1024-dimensional arrays of floats
- Qdrant collection uses cosine distance metric
- Payload includes: url (string), text (string), chunk_id (integer)

## Error Contracts

### Recoverable Errors
- Network timeouts: Retry with exponential backoff
- Individual URL failures: Log error and continue with other URLs
- Cohere rate limits: Wait and retry with exponential backoff

### Non-Recoverable Errors
- Invalid API keys: Stop execution and raise exception
- Invalid Qdrant connection: Stop execution and raise exception
- Invalid sitemap format: Stop execution and raise exception

## Performance Contracts

### Expected Performance
- Embedding generation: < 1 second per chunk under normal conditions
- Qdrant storage: < 0.5 seconds per chunk under normal conditions
- Text extraction: < 2 seconds per page under normal conditions

### Resource Usage
- Memory usage should remain under 512MB during normal operation
- Network connections should be properly closed after use
- No local persistence of embeddings beyond temporary processing

## Configuration Contracts

### Required Environment Variables
- `COHERE_API_KEY`: Valid Cohere API key
- `QDRANT_CLUSTER_ENDPOINT`: Valid Qdrant Cloud endpoint URL
- `QDRANT_API_KEY`: Valid Qdrant API key

### Default Values
- Collection name: "ai_book_embedding"
- Chunk size: 1200 characters maximum (to align with 512 token limit)
- Cohere model: "embed-multilingual-v3.0" (selected for 1024 dimensions and 512 max input tokens)