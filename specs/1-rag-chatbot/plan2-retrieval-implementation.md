# RAG Chatbot – Spec-2 Implementation Plan

## Technical Context

**Feature**: RAG Chatbot - Phase 2: Retrieval Functionality
**Implementation Approach**: Python-based semantic search using sentence-transformers for query embeddings and Qdrant Cloud for vector similarity search
**Target Collection**: ai_book_embedding (contains embeddings generated with sentence-transformers)
**Primary Embedding Provider**: sentence-transformers (paraphrase-multilingual-MiniLM-L12-v2 model producing 384-dimensional vectors)
**Vector Database**: Qdrant Cloud Free Tier
**Target URLs**: https://osamabinadnan.github.io/physical-ai-humanoid-robotic-book/

**Key Components**:
- Query embedding generation with sentence-transformers
- Vector similarity search in Qdrant
- Result retrieval with metadata
- Error handling for service failures

## Constitution Check

Based on the project constitution, this implementation plan aligns with:
- **VII. Chunkable, RAG-Ready Content**: Enhancing retrieval capabilities for embedded content
- **XIV. Deployment Guarantee**: Ensuring content integrates with the RAG backend
- **V. Engineering Reproducibility**: Creating runnable code with verifiable examples

## Gates

**✅ Ready to Proceed** - All clarifications from the specification have been resolved:
- Model selection: sentence-transformers with 384-dimensional vectors
- Error handling: Comprehensive error handling for Qdrant failures
- Implementation location: backend/agent directory
- Compatibility: Same model for ingestion and retrieval

## Phase 0: Research & Resolution

### Research Findings

**Decision**: Use sentence-transformers paraphrase-multilingual-MiniLM-L12-v2 model for query embeddings
**Rationale**: This model produces 384-dimensional vectors that are compatible with the stored embeddings from Phase 1, and supports multiple languages
**Alternatives considered**: all-MiniLM-L6-v2, all-mpnet-base-v2, other sentence-transformers models

**Decision**: Implement comprehensive error handling for Qdrant connection failures
**Rationale**: Essential for system reliability when the vector database is unavailable
**Alternatives considered**: Basic error logging, skipping failed queries

**Decision**: Store implementation in backend/agent directory as specified
**Rationale**: Follows the architectural decision to separate retrieval functionality
**Alternatives considered**: Keeping in backend/embedding, creating new directory structure

## Phase 1: Design & Architecture

### Data Model

**Query Entity**:
- `query_text`: String (the original user query)
- `embedding_vector`: Array of floats (384-dimensional embedding vector)

**SearchResult Entity**:
- `retrieved_chunks`: Array of RetrievedChunk objects
- `query_similarity_scores`: Array of floats (similarity scores for each result)

**RetrievedChunk Entity**:
- `text`: String (the actual content chunk)
- `url`: String (source URL of the content)
- `chunk_id`: Integer (unique identifier from Qdrant)
- `similarity_score`: Float (cosine similarity score from the search)

### System Architecture

```
[accept_user_query] -> [generate_query_embedding] -> [perform_qdrant_search] -> [retrieve_top_k_results] -> [return_with_metadata]
```

**Module Structure**:
- `backend/agent/retriever.py`: Main retrieval module containing all functions
- Environment variables for Qdrant configuration
- Single-file implementation as requested

### API Contract

The system will implement a retrieval function that can be used as a function_tool:
- `retrieve(query: str, top_k: int = 5)`: Function that accepts a user query and returns top-K relevant chunks with metadata

## Phase 2: Implementation Plan

### Task Breakdown

1. **Environment Setup**
   - Configure Qdrant connection parameters from environment variables
   - Install required dependencies (sentence-transformers, qdrant-client)

2. **Create Agent Structure**
   - Create `backend/agent/` directory
   - Create `retriever.py` file with all required functions

3. **Implement Query Embedding Generation**
   - `generate_query_embedding(query_text: str)` → Generate 384-dimensional embedding vector

4. **Implement Qdrant Connection**
   - Establish connection to existing ai_book_embedding collection
   - Verify collection exists and is accessible

5. **Implement Vector Search**
   - `perform_semantic_search(query_embedding: list, top_k: int = 5)` → Find similar vectors in Qdrant

6. **Implement Result Processing**
   - Format retrieved results with complete metadata
   - Handle empty or low-confidence results appropriately

7. **Implement Main Retrieval Function**
   - `retrieve(query: str, top_k: int = 5)` → Complete retrieval pipeline function
   - Include error handling and logging

8. **Testing & Validation**
   - Test retrieval with sample queries
   - Verify results include correct text and metadata
   - Confirm retrieval behavior and measure response times

9. **Documentation**
   - Document retrieval behavior and limitations
   - Create usage instructions

## Phase 3: Integration Strategy

### Prerequisites
- Python 3.8+ installed
- Existing embeddings in Qdrant collection "ai_book_embedding"
- Valid Qdrant Cloud credentials

### Dependencies
- sentence-transformers
- qdrant-client
- python-dotenv

### Error Handling
- Network connectivity issues
- Qdrant service unavailability
- Empty query results
- Invalid query inputs

## Risk Assessment

- **Model Compatibility**: Ensuring query embeddings match stored embeddings format
- **Performance**: Maintaining response times within 2-4 second range
- **Qdrant Availability**: Handling vector database downtime gracefully
- **Result Quality**: Ensuring retrieved content is relevant to user queries

## Success Criteria

- Successfully connect to existing Qdrant collection
- Generate compatible query embeddings (384 dimensions)
- Retrieve top-5 relevant chunks with proper metadata
- Handle edge cases (empty results, connection failures)
- Meet performance targets (2-4 second response time)
- Ready for agent integration in Phase 3