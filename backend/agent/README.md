# RAG Chatbot Retrieval Agent

This module implements the retrieval functionality for the RAG Chatbot project. It extracts content from the Physical AI & Humanoid Robotics textbook, generates embeddings, and stores them in Qdrant Cloud for semantic search.

## Overview

The retrieval module provides functionality to:
- Extract content from GitHub Pages URLs (specifically from https://osamabinadnan.github.io/physical-ai-humanoid-robotic-book/)
- Generate embeddings using sentence-transformers' `paraphrase-multilingual-MiniLM-L12-v2` model
- Store embeddings in Qdrant Cloud with proper metadata
- Retrieve relevant content chunks based on semantic similarity

## Architecture

The module implements the following functions:
- `get_embedding(text)`: Generates 384-dimensional embeddings for input text
- `perform_qdrant_search(query_embedding, top_k)`: Performs semantic search in Qdrant
- `format_search_results(results)`: Structures search results with proper metadata
- `retrieve(query, top_k)`: Main function_tool for the agent to retrieve relevant content

## Configuration

Create a `.env` file with the following environment variables:
```
QDRANT_CLUSTER_ENDPOINT=your_qdrant_cluster_endpoint_here
QDRANT_API_KEY=your_qdrant_api_key_here
```

## Usage

The `retrieve` function is designed to be used as a function_tool for the agent:
```python
from retriever import retrieve

# Example usage
results = retrieve("What is Physical AI?", top_k=5)
print(results)
```

The function returns a dictionary with:
- `retrieved_texts`: List of text chunks that match the query
- `citations`: List of citation objects with metadata (text, url, source_title, chunk_id, similarity_score)

## Dependencies

- sentence-transformers
- qdrant-client
- python-dotenv
- requests
- trafilatura

## Testing

To test the module, run:
```
uv run test_retriever.py
```

## Integration

This module is designed to be used as part of the RAG chatbot agent system. The `@function_tool` decorator allows it to be registered as an available tool for the agent.