# Research Document: RAG Chatbot Embedding Implementation

## Overview
This document details the research and decisions made for implementing the content ingestion and embedding generation system for the RAG Chatbot project.

## Embedding Model Selection

**Decision**: Use Cohere's embed-multilingual-v3.0 model
**Rationale**:
- Provides 1024-dimensional embeddings which are efficient for similarity search
- Supports 512 max input tokens, appropriate for chunk size
- Supports 100+ languages, providing flexibility for future use
- Sufficient rate limits for the project scope
- Well-documented API with good Python SDK support
- Recommended for search and retrieval use cases

**Rate Limits Considerations**:
- Free tier has limited requests per minute/day
- Need to implement proper rate limiting and retry logic
- Consider exponential backoff for handling rate limit errors

## Text Extraction Methods

**Decision**: Use trafilatura for content extraction
**Rationale**:
- Specifically designed for extracting main content from web pages
- Handles various HTML structures and formats
- Filters out navigation, ads, and boilerplate content
- Better than generic HTML parsers for this use case

**Alternatives Considered**:
1. BeautifulSoup + custom logic
   - Pros: Full control over extraction process
   - Cons: Requires significant custom logic to handle various page structures
2. newspaper3k
   - Pros: Good for news articles
   - Cons: May not work optimally for documentation sites
3. readability (Mozilla's library)
   - Pros: Used by browsers for reader view
   - Cons: May be overkill for simple content extraction

## Chunking Strategy

**Decision**: Character-based chunking with 512-1024 token equivalent (approx. 1200 characters)
**Rationale**:
- Balances context preservation with retrieval efficiency
- Cohere embeddings work well with this size range
- Allows for semantic coherence within chunks
- Prevents exceeding model input limits

**Chunking Algorithm**:
- Split on sentence boundaries when possible
- Look for natural breaks (periods, question marks, exclamation points)
- Maintain semantic coherence
- Add overlap if needed for context preservation

## Qdrant Integration

**Decision**: Use Qdrant Cloud Free Tier with cosine distance
**Rationale**:
- Managed service reduces operational overhead
- Good performance for vector similarity search
- Supports metadata storage for retrieval context
- Cosine distance is appropriate for embedding similarity

**Collection Configuration**:
- Vector size: 1024 (matching Cohere embedding dimensions)
- Distance metric: Cosine
- Metadata: URL, text content, chunk ID

## Error Handling Strategy

**Decision**: Comprehensive error handling with fallbacks
**Rationale**:
- External dependencies (Cohere, Qdrant, web pages) can fail
- Need to maintain pipeline resilience
- Should log errors for debugging while continuing processing

**Error Handling Components**:
- API rate limit handling with exponential backoff
- Network timeout handling with retries
- Qdrant connection verification before ingestion
- Individual URL failure isolation (don't stop entire pipeline for single URL failure)

## URL Extraction from Sitemap

**Decision**: Extract URLs from sitemap.xml
**Rationale**:
- GitHub Pages sites typically have sitemaps
- Provides complete list of all pages
- More reliable than web crawling
- Avoids issues with JavaScript-rendered content

**Sitemap Processing**:
- Handle XML namespaces properly
- Validate URLs before processing
- Filter out non-content pages if needed