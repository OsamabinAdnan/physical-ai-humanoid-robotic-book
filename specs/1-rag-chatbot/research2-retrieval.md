# Research Document: RAG Chatbot Retrieval Implementation

## Overview
This document details the research and decisions made for implementing the content retrieval system for the RAG Chatbot project, specifically for Phase 2.

## Embedding Model Selection

**Decision**: Use sentence-transformers paraphrase-multilingual-MiniLM-L12-v2 model
**Rationale**:
- Produces 384-dimensional embeddings which match the stored embeddings from Phase 1
- Supports 50+ languages, providing flexibility for multilingual queries
- Well-optimized for sentence similarity tasks
- Good performance for semantic search applications
- Compatible with the previously stored embeddings in Qdrant

**Alternatives Considered**:
1. all-MiniLM-L6-v2
   - Pros: Faster processing, smaller model
   - Cons: May not perform as well with multilingual content
2. all-mpnet-base-v2
   - Pros: Higher quality embeddings
   - Cons: Larger model, slower processing
3. Other sentence-transformers models
   - Pros: Various specialized capabilities
   - Cons: Potential compatibility issues with existing embeddings

## Qdrant Integration

**Decision**: Use Qdrant Cloud Free Tier with existing collection
**Rationale**:
- Existing "ai_book_embedding" collection already contains the embeddings from Phase 1
- Cloud service provides managed infrastructure
- Good performance for vector similarity search
- Supports metadata storage and retrieval
- Cosine distance metric already configured

**Collection Configuration**:
- Vector size: 384 (matching sentence-transformers model)
- Distance metric: Cosine
- Metadata: url, text, chunk_id

## Error Handling Strategy

**Decision**: Implement comprehensive error handling for service failures
**Rationale**:
- External dependencies (Qdrant) can fail
- Need to maintain pipeline resilience
- Should provide graceful degradation when possible

**Error Handling Components**:
- Qdrant connection verification before search
- Timeout handling for search requests
- Empty result handling with appropriate user feedback
- Individual query failure isolation

## Semantic Search Parameters

**Decision**: Use top-k=5 for result retrieval
**Rationale**:
- Provides sufficient context for downstream agent processing
- Balances relevance with performance
- Aligns with success criteria from specification
- Standard value for RAG systems

**Search Configuration**:
- Top-K: 5 (configurable parameter)
- Distance metric: Cosine similarity (already configured in Qdrant)
- Score threshold: Optional, to filter low-quality results

## Result Formatting

**Decision**: Return rich metadata with each result
**Rationale**:
- Enables downstream agent to properly cite sources
- Facilitates content verification
- Supports user experience requirements
- Maintains provenance information

**Metadata Fields**:
- Text content
- Source URL
- Chunk ID
- Similarity score