# Quickstart Guide: RAG Chatbot Retrieval System

## Prerequisites

- Python 3.8 or higher
- pip package manager
- Git (if cloning the repository)
- Access to Qdrant Cloud with existing "ai_book_embedding" collection
- Valid sentence-transformers compatible embeddings in the collection

## Setup

### 1. Clone the Repository (if needed)
```bash
git clone <repository-url>
cd <repository-name>
```

### 2. Create Virtual Environment
```bash
python -m venv venv
source venv/bin/activate  # On Windows: venv\Scripts\activate
```

### 3. Install Dependencies
```bash
pip install sentence-transformers qdrant-client python-dotenv
```

### 4. Configure Environment Variables
Create a `.env` file in the project root with the following variables:

```env
QDRANT_CLUSTER_ENDPOINT=your_qdrant_cluster_endpoint_here
QDRANT_API_KEY=your_qdrant_api_key_here
```

## Running the Retrieval System

### 1. Navigate to the Backend Directory
```bash
cd backend
```

### 2. Use the Retrieval Function
The retrieval functionality is available as a function that can be imported and used:

```python
from agent.retriever import retrieve

# Example usage
results = retrieve("What is Physical AI?", top_k=5)
print(results)
```

## Expected Output

When you run the retrieval function, you should see output similar to:

```
{
  "retrieved_texts": [
    "Physical AI refers to the intersection of artificial intelligence and physical systems...",
    "The concept of Physical AI encompasses robots that can learn and adapt to their environment...",
    ...
  ],
  "citations": [
    {
      "text": "Physical AI refers to the intersection of artificial intelligence and physical systems...",
      "url": "https://osamabinadnan.github.io/physical-ai-humanoid-robotic-book/docs/module-1/chapter-1/",
      "chunk_id": 123,
      "similarity_score": 0.87
    },
    ...
  ]
}
```

## Model Configuration

The system uses sentence-transformers' `paraphrase-multilingual-MiniLM-L12-v2` model which provides:
- 384-dimensional embeddings
- Support for multiple languages
- Compatibility with the embeddings generated in Phase 1

## Verification

To verify that the retrieval system is working correctly:

1. Check that you can connect to the Qdrant collection "ai_book_embedding"
2. Verify that query embeddings are generated with 384 dimensions
3. Confirm that search results include proper metadata (text, URL, chunk_id, similarity_score)
4. Test with various sample queries to ensure relevance

## Troubleshooting

### Common Issues

**Connection Errors**:
- Ensure your Qdrant endpoint and API key are correct
- Check that your Qdrant Cloud instance is running

**Empty Results**:
- Verify that the "ai_book_embedding" collection contains data
- Check that your query is in a format similar to the stored content

**Dimension Mismatch**:
- Ensure the query embedding dimensions match the stored embeddings (384 dimensions)
- Verify the same sentence-transformers model is used for both ingestion and retrieval

## Next Steps

After successful retrieval implementation:

1. The system will be ready for agent integration in Phase 3
2. You can proceed to implement the RAG agent that uses this retrieval function
3. Monitor the Qdrant Cloud instance for performance and usage metrics