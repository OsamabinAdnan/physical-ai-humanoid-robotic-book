# RAG Chatbot – Spec-2 Task List

## Feature: RAG Chatbot - Phase 2: Retrieval Functionality

## Dependencies

User stories must be completed in this order:
- User Story 1 (P1) must be completed before User Story 2 (P2)
- User Story 2 (P2) must be completed before User Story 3 (P3)

## Parallel Execution Examples

- Tasks T002-T006 can run in parallel (different dependency installations)
- Tasks T012-T017 can run in parallel (different functions in the same file)

## Implementation Strategy

Start with User Story 1 (P1) as MVP - implement the basic retrieval functionality with Qdrant connection and query embedding generation. Then incrementally add chunking validation (US2) and result processing (US3) capabilities.

---

## Phase 1: Setup

### Goal
Initialize the project structure and install dependencies needed for the RAG chatbot retrieval pipeline.

- [X] T001 Create backend/agent directory structure
- [ ] T002 [P] Install sentence-transformers package using UV package manager
- [ ] T003 [P] Install qdrant-client package using UV package manager
- [ ] T004 [P] Install python-dotenv package using UV package manager
- [ ] T005 [P] Install requests package using UV package manager
- [ ] T006 [P] Install trafilatura package using UV package manager
- [ ] T007 Create .env file with placeholder values for API keys
- [X] T008 Create backend/agent/retriever.py file with proper imports for retrieval pipeline

---

## Phase 2: Foundational

### Goal
Implement the foundational components that all user stories depend on, including configuration loading and core utility functions.

- [X] T009 Configure environment variables loading in retriever.py
- [X] T010 Initialize sentence-transformers paraphrase-multilingual-MiniLM-L12-v2 model with appropriate configuration
- [X] T011 Initialize Qdrant client with cluster endpoint and API key from environment
- [X] T012 [P] [US1] Implement generate_query_embedding function to create 384-dimensional embeddings using paraphrase-multilingual-MiniLM-L12-v2 model
- [X] T013 [P] [US1] Implement perform_qdrant_search function to connect to existing ai_book_embedding collection
- [X] T014 [US2] Implement format_search_results function to structure retrieval output
- [X] T015 [US3] Implement retrieve function that orchestrates the entire retrieval pipeline
- [X] T016 [US3] Implement error handling for Qdrant connection failures
- [X] T017 [US3] Implement handling for empty or low-confidence search results

---

## Phase 3: User Story 1 - Semantic Search from Embedded Content (Priority: P1)

### Goal
Implement the ability to perform semantic search using user queries against the existing embedded textbook content.

### Independent Test Criteria
The system can be tested by providing a query and verifying that relevant text chunks are returned from the vector database with proper metadata.

- [X] T018 [US1] Test query embedding generation with sample text inputs
- [X] T019 [US1] Verify embeddings are 384-dimensional and compatible with stored embeddings
- [X] T020 [US1] Test connection to existing Qdrant collection "ai_book_embedding"
- [X] T021 [US1] Test semantic search with a sample query against the vector database
- [X] T022 [US1] Verify top-5 results are returned with proper similarity scores

---

## Phase 4: User Story 2 - Query Embedding Generation (Priority: P2)

### Goal
Implement generation of embeddings for user queries using the same sentence-transformers model as the content embeddings.

### Independent Test Criteria
The system can be tested by providing a query text and verifying that it generates a 384-dimensional embedding vector compatible with the stored embeddings.

- [X] T023 [US2] Test embedding generation with various query lengths using paraphrase-multilingual-MiniLM-L12-v2 model
- [X] T024 [US2] Verify embedding dimensions match stored content embeddings (384 dimensions)
- [X] T025 [US2] Test embedding generation with multilingual queries
- [X] T026 [US2] Validate embedding consistency for same input across multiple calls
- [X] T027 [US2] Add logging for query embedding generation process

---

## Phase 5: User Story 3 - Result Validation and Metadata Retrieval (Priority: P3)

### Goal
Implement validation that the retrieved results include correct text and metadata so that the system can be verified as working properly.

### Independent Test Criteria
The system can be tested by comparing retrieved results with the original source content to verify accuracy.

- [X] T028 [US3] Test result formatting with various query types
- [X] T029 [US3] Verify all required metadata is included in results (text, URL, chunk_id, similarity_score)
- [X] T030 [US3] Test result validation against original source content
- [X] T031 [US3] Validate metadata accuracy for retrieved chunks
- [X] T032 [US3] Add comprehensive logging for retrieval process

---

## Phase 6: Integration and Main Pipeline

### Goal
Implement the main retrieval pipeline that orchestrates all components from query processing to result delivery.

- [X] T033 Implement retrieve function that orchestrates the entire pipeline
- [X] T034 Add comprehensive error handling throughout the pipeline
- [X] T035 Implement proper timeout handling for Qdrant requests
- [X] T036 Implement idempotent retrieval behavior for repeated queries
- [X] T037 Add progress logging and monitoring to the main pipeline

---

## Phase 7: Testing & Validation

### Goal
Run basic checks to confirm retrieval functionality works as expected and verify all functionality works correctly.

- [X] T038 Test complete retrieval pipeline with sample queries from the textbook domain
- [X] T039 Verify all retrieval results match source book content accurately
- [X] T040 Verify embeddings are stored in Qdrant Cloud with proper metadata
- [X] T041 Confirm retrieval process completes without errors and provides feedback
- [X] T042 Test edge cases: very short/long queries, empty results, connection failures

---

## Phase 8: Polish & Cross-Cutting Concerns

### Goal
Add finishing touches, documentation, and handle edge cases.

- [X] T043 Add comprehensive error handling for external service failures
- [X] T044 Implement fallback strategies for Qdrant rate limits
- [X] T045 Add handling for Qdrant Cloud unavailability
- [X] T046 Document how to run and configure the retrieval pipeline
- [X] T047 Create README with setup and execution instructions
- [X] T048 Handle edge case: Query returns no relevant results in vector database
- [X] T049 Handle edge case: Queries in languages other than English
- [X] T050 Final integration test of complete retrieval system