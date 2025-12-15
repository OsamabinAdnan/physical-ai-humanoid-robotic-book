# API Contract: Retrieval Service for RAG Chatbot

## Overview
This document defines the contract for the content retrieval service that handles semantic search in the vector database and returns relevant content chunks with metadata.

## Service Interface

### Main Retrieval Function
```
retrieve(query: str, top_k: int = 5) -> dict
```

**Description**: Performs semantic search on the vector database and returns the top-K most relevant content chunks with their metadata.

**Parameters**:
- `query` (str): User's text query for semantic search
- `top_k` (int): Number of top results to return (default: 5)

**Return**: Dictionary containing retrieved texts and citations

**Side Effects**: Makes a query to the Qdrant vector database and retrieves matching vectors with metadata.

## Core Functions

### Query Embedding Generation
```
generate_query_embedding(query_text: str) -> List[float]
```

**Description**: Generates a 384-dimensional embedding vector for the input query text using sentence-transformers model.

**Parameters**:
- `query_text` (str): Text to generate embedding for

**Return**: 384-dimensional embedding vector as a list of floats

**Errors**: Raises exception if embedding generation fails

### Qdrant Search
```
perform_qdrant_search(query_embedding: List[float], top_k: int = 5) -> List[dict]
```

**Description**: Performs vector similarity search in Qdrant collection and returns top-K results with metadata.

**Parameters**:
- `query_embedding` (List[float]): 384-dimensional query embedding vector
- `top_k` (int): Number of top results to retrieve (default: 5)

**Return**: List of result dictionaries containing text, URL, chunk_id, and similarity_score

**Errors**: Raises exception if Qdrant search fails

### Result Formatting
```
format_search_results(results: List[dict]) -> dict
```

**Description**: Formats raw Qdrant results into a structured response with clean metadata.

**Parameters**:
- `results` (List[dict]): Raw results from Qdrant search

**Return**: Structured dictionary with retrieved_texts and citations

## Data Contracts

### Input Data
- Query text must be a non-empty string
- Query text should be in a language supported by the sentence-transformers model
- top_k parameter must be a positive integer (1-100 range recommended)

### Output Data
- Embedding vectors are 384-dimensional arrays of floats
- Qdrant collection uses cosine distance metric for similarity
- Result payload includes: text (string), url (string), chunk_id (integer), similarity_score (float)

## Error Contracts

### Recoverable Errors
- Qdrant connection timeouts: Retry with exponential backoff
- Empty search results: Return empty results list with appropriate message
- Query embedding failures: Log error and return with error indication

### Non-Recoverable Errors
- Invalid Qdrant credentials: Stop execution and raise exception
- Invalid Qdrant collection name: Stop execution and raise exception
- Invalid embedding dimensions: Stop execution and raise exception

## Performance Contracts

### Expected Performance
- Query embedding generation: < 0.5 seconds per query under normal conditions
- Qdrant search: < 2-4 seconds per search under normal conditions
- Overall retrieval: < 4 seconds per request with top_k=5

### Resource Usage
- Memory usage should remain under 256MB during normal operation
- Network connections should be properly managed
- No persistent storage of query embeddings beyond temporary processing

## Configuration Contracts

### Required Environment Variables
- `QDRANT_CLUSTER_ENDPOINT`: Valid Qdrant Cloud endpoint URL
- `QDRANT_API_KEY`: Valid Qdrant API key
- `MODEL_PATH`: Path to sentence-transformers model (default: "paraphrase-multilingual-MiniLM-L12-v2")

### Default Values
- Collection name: "ai_book_embedding"
- Top-K results: 5
- Model: "paraphrase-multilingual-MiniLM-L12-v2"
- Vector dimension: 384