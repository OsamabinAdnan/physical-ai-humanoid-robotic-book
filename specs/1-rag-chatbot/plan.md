# RAG Chatbot – Spec-1 Implementation Plan

## Technical Context

**Feature**: RAG Chatbot - Phase 1: Content Ingestion and Embedding Generation
**Implementation Approach**: Python-based web scraping, text extraction, embedding generation, and vector storage
**Target URLs**: https://osamabinadnan.github.io/physical-ai-humanoid-robotic-book/
**Sitemap URLs**: https://osamabinadnan.github.io/physical-ai-humanoid-robotic-book/sitemap.xml
**Primary Embedding Provider**: Cohere (embed-multilingual-v3.0 model)
**Vector Database**: Qdrant Cloud Free Tier
**Collection Name**: ai_book_embedding

**Key Components**:
- URL extraction from sitemap
- Text extraction from web pages
- Text chunking
- Embedding generation with Cohere
- Vector storage in Qdrant

## Constitution Check

Based on the project constitution, this implementation plan aligns with:
- **VII. Chunkable, RAG-Ready Content**: Creating content optimized for embeddings and RAG systems
- **XIV. Deployment Guarantee**: Ensuring content integrates with the RAG backend
- **V. Engineering Reproducibility**: Creating runnable code with verifiable examples

## Gates

**✅ Ready to Proceed** - All clarifications from the specification have been resolved:
- Security: API key handling is implemented
- Error handling: Fallback strategies defined
- Observability: Basic logging included
- Chunking: 512-1024 token size specified
- Embedding: Cohere as primary provider

## Phase 0: Research & Resolution

### Research Findings

**Decision**: Use Cohere's embed-multilingual-v3.0 model with 1024-dimensional embeddings and 512 max input tokens
**Rationale**: This model provides 1024-dimensional embeddings with 512 max input tokens, which is optimal for the project requirements and supports multiple languages if needed
**Alternatives considered**: embed-english-v3.0, other Cohere models, Hugging Face open-source models

**Decision**: Use trafilatura for text extraction from web pages
**Rationale**: Trafilatura is specifically designed for extracting main content from web pages while filtering out navigation and boilerplate
**Alternatives considered**: BeautifulSoup, newspaper3k, readability

**Decision**: Implement URL extraction from sitemap.xml
**Rationale**: Sitemap provides a structured list of all pages in the GitHub Pages site
**Alternatives considered**: Web crawling, manual URL list

## Phase 1: Design & Architecture

### Data Model

**Content Chunk Entity**:
- `chunk_id`: Integer (unique identifier)
- `url`: String (source URL of the content)
- `text`: String (the actual content chunk)
- `vector`: Array of floats (1024-dimensional embedding vector)

**Embedding Record**:
- `id`: Integer (Qdrant point ID)
- `vector`: Array of floats (embedding from Cohere model)
- `payload`: Object containing metadata (url, text, chunk_id)

### System Architecture

```
[get_all_urls] -> [extract_text_from_url] -> [chunk_text] -> [embedding] -> [create_collection] -> [save_chunk_to_qdrant] -> [ingest_book]
```

**Module Structure**:
- `backend/embedding/ingestor.py`: Main ingestion module containing all functions
- Environment variables for API keys and configuration
- Single-file implementation as requested

### API Contract

The system will implement a command-line interface with the following main function:
- `ingest_book()`: Main entry point that orchestrates the entire ingestion pipeline

## Phase 2: Implementation Plan

### Task Breakdown

1. **Environment Setup**
   - Configure .env file with Cohere API key and Qdrant Cloud credentials
   - Install required dependencies (cohere, qdrant-client, trafilatura, python-dotenv)

2. **Create Backend Structure**
   - Create `backend/embedding/` directory
   - Create `ingestor.py` file with all required functions

3. **Implement URL Extraction**
   - `get_all_urls()`: Extract all URLs from sitemap.xml
   - Handle XML parsing and namespace issues

4. **Implement Text Extraction**
   - `extract_text_from_url()`: Extract clean text from web pages
   - Use trafilatura for reliable content extraction

5. **Implement Text Chunking**
   - `chunk_text()`: Split text into meaningful chunks with metadata
   - Target chunk size: 512-1024 tokens (approximately 1200 characters)

6. **Implement Embedding Generation**
   - `embedding()`: Generate embeddings using Cohere model
   - Handle API rate limits and errors

7. **Implement Qdrant Integration**
   - `create_collection()`: Create or verify vector collection
   - `save_chunk_to_qdrant()`: Store embeddings with metadata

8. **Implement Main Pipeline**
   - `ingest_book()`: Main function orchestrating the entire process
   - Include error handling and logging

9. **Testing & Validation**
   - Run basic checks to confirm data is stored correctly
   - Verify collection exists in Qdrant Cloud

10. **Documentation**
    - Create README with instructions for running the pipeline
    - Document environment variable requirements

## Phase 3: Deployment Strategy

### Prerequisites
- Python 3.8+ installed
- Valid Cohere API key
- Qdrant Cloud account and credentials

### Configuration
- Environment variables in `.env` file:
  - `COHERE_API_KEY`: Cohere API key
  - `QDRANT_CLUSTER_ENDPOINT`: Qdrant Cloud endpoint URL
  - `QDRANT_API_KEY`: Qdrant API key

### Execution
- Run `python backend/embedding/ingestor.py` to start the ingestion process
- Monitor logs for progress and errors
- Verify data in Qdrant Cloud after completion

## Risk Assessment

- **API Rate Limits**: Implement proper error handling and retry logic
- **Network Issues**: Add timeouts and retry mechanisms for HTTP requests
- **Qdrant Availability**: Verify connection before starting ingestion
- **Data Duplication**: Implement checks to prevent duplicate ingestion on re-runs

## Success Criteria

- All pages from the GitHub Pages site are successfully ingested
- Embeddings are stored in Qdrant Cloud with proper metadata
- Process completes without errors and provides progress feedback
- Vector collection is queryable for downstream RAG operations