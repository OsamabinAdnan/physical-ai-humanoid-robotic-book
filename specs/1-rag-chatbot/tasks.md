# RAG Chatbot – Spec-1 Task List

## Feature: RAG Chatbot - Phase 1: Content Ingestion and Embedding Generation

## Dependencies

User stories must be completed in this order:
- User Story 1 (P1) must be completed before User Story 2 (P2)
- User Story 2 (P2) must be completed before User Story 3 (P3)

## Parallel Execution Examples

- Tasks T003-T006 can run in parallel (different dependency installations)
- Tasks T012-T015 can run in parallel (different functions in the same file)

## Implementation Strategy

Start with User Story 1 (P1) as MVP - implement the basic content ingestion functionality with URL extraction and text extraction. Then incrementally add chunking (US2) and embedding/storage (US3) capabilities.

---

## Phase 1: Setup

### Goal
Initialize the project structure and install dependencies needed for the RAG chatbot ingestion pipeline.

- [X] T001 Create backend/embedding directory structure
- [X] T002 [P] Install cohere package using UV package manager
- [X] T003 [P] Install qdrant-client package using UV package manager
- [X] T004 [P] Install trafilatura package using UV package manager
- [X] T005 [P] Install python-dotenv package using UV package manager
- [X] T006 [P] Install requests package using UV package manager
- [X] T007 Create .env file with placeholder values for API keys
- [X] T008 Create backend/embedding/ingestor.py file with proper imports

---

## Phase 2: Foundational

### Goal
Implement the foundational components that all user stories depend on, including configuration loading and core utility functions.

- [X] T009 Configure environment variables loading in ingestor.py
- [X] T010 Initialize Cohere client with API key from environment
- [X] T011 Initialize Qdrant client with cluster endpoint and API key from environment
- [X] T012 [P] [US1] Implement get_all_urls function to extract URLs from sitemap.xml
- [X] T013 [P] [US1] Implement extract_text_from_url function using trafilatura
- [X] T014 [US2] Implement chunk_text function with 1200 character max chunk size
- [X] T015 [US3] Implement embedding function using Cohere's embed-multilingual-v3.0 model
- [X] T016 [US3] Implement create_collection function to create ai_book_embedding collection in Qdrant
- [X] T017 [US3] Implement save_chunk_to_qdrant function to store embeddings with metadata

---

## Phase 3: User Story 1 - Content Ingestion from GitHub Pages URLs (Priority: P1)

### Goal
Implement the ability to configure GitHub Pages URLs so that the system can automatically crawl and extract clean textual content from published documentation sites.

### Independent Test Criteria
The system can be tested by providing a GitHub Pages URL and verifying that content is successfully extracted in clean text format without HTML tags or navigation elements.

- [X] T018 [US1] Implement error handling for invalid or inaccessible URLs
- [X] T019 [US1] Add logging for URL extraction process
- [X] T020 [US1] Test URL extraction with the target site: https://osamabinadnan.github.io/physical-ai-humanoid-robotic-book/
- [X] T021 [US1] Test text extraction from a sample page
- [X] T022 [US1] Verify clean text extraction without HTML elements or navigation components

---

## Phase 4: User Story 2 - Content Chunking and Metadata Generation (Priority: P2)

### Goal
Implement automatic splitting of extracted content into semantically meaningful chunks with consistent metadata so that embeddings can be generated effectively.

### Independent Test Criteria
The system can be tested by providing a large text document and verifying that it's split into appropriately sized chunks with proper metadata.

- [X] T023 [US2] Test chunking function with a large text input
- [X] T024 [US2] Verify chunks are within 512-1024 token range (approx. 1200 characters)
- [X] T025 [US2] Verify metadata is consistently added to each chunk
- [X] T026 [US2] Ensure chunking preserves semantic meaning at boundaries
- [X] T027 [US2] Add logging for chunking process

---

## Phase 5: User Story 3 - Embedding Generation and Storage (Priority: P3)

### Goal
Implement generation of embeddings using Cohere model and store them in Qdrant Cloud for efficient retrieval in the RAG system.

### Independent Test Criteria
The system can be tested by verifying that text chunks are converted to embeddings and stored in the vector database with appropriate metadata.

- [ ] T028 [US3] Test embedding generation with sample text chunks
- [ ] T029 [US3] Verify 1024-dimensional embeddings are generated
- [ ] T030 [US3] Test connection to Qdrant Cloud and collection creation
- [ ] T031 [US3] Test storing embeddings with metadata in Qdrant
- [ ] T032 [US3] Verify stored embeddings can be retrieved from Qdrant

---

## Phase 6: Integration and Main Pipeline

### Goal
Implement the main ingestion pipeline that orchestrates all components from URL extraction to vector storage.

- [X] T033 Implement ingest_book function that orchestrates the entire pipeline
- [X] T034 Add comprehensive error handling throughout the pipeline
- [X] T035 Implement rate limiting and retry logic for Cohere API calls
- [X] T036 Implement idempotent re-runs to prevent duplicate vector insertions
- [X] T037 Add progress logging and monitoring to the main pipeline

---

## Phase 7: Testing & Validation

### Goal
Run basic checks to confirm data is stored correctly and verify all functionality works as expected.

- [X] T038 Test complete pipeline with the target GitHub Pages site
- [X] T039 Verify all pages from the site are successfully ingested
- [X] T040 Verify embeddings are stored in Qdrant Cloud with proper metadata
- [X] T041 Confirm process completes without errors and provides progress feedback
- [X] T042 Verify vector collection is queryable for downstream RAG operations

---

## Phase 8: Polish & Cross-Cutting Concerns

### Goal
Add finishing touches, documentation, and handle edge cases.

- [ ] T043 Add comprehensive error handling for external service failures
- [ ] T044 Implement fallback strategies for Cohere rate limits
- [ ] T045 Add handling for Qdrant Cloud unavailability
- [ ] T046 Document how to run and configure the pipeline
- [ ] T047 Create README with setup and execution instructions
- [ ] T048 Handle edge case: URL returns 404 or is temporarily unavailable during crawling
- [ ] T049 Handle edge case: Duplicate content or idempotent re-runs
- [ ] T050 Final integration test of complete system