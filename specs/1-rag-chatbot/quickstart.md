# Quickstart Guide: RAG Chatbot Content Ingestion

## Prerequisites

- Python 3.8 or higher
- pip package manager
- Git (if cloning the repository)

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
pip install cohere qdrant-client trafilatura python-dotenv requests
```

### 4. Configure Environment Variables
Create a `.env` file in the project root with the following variables:

```env
COHERE_API_KEY=your_cohere_api_key_here
QDRANT_CLUSTER_ENDPOINT=your_qdrant_cluster_endpoint_here
QDRANT_API_KEY=your_qdrant_api_key_here
```

## Model Configuration

The system uses Cohere's `embed-multilingual-v3.0` model which provides:
- 1024-dimensional embeddings
- Support for up to 512 input tokens per request
- Support for 100+ languages

## Running the Ingestion Pipeline

### 1. Navigate to the Backend Directory
```bash
cd backend
```

### 2. Run the Ingestion Script
```bash
python embedding/ingestor.py
```

## Expected Output

When you run the ingestion script, you should see output similar to:

```
Found URLs:
- https://osamabinadnan.github.io/physical-ai-humanoid-robotic-book/
- https://osamabinadnan.github.io/physical-ai-humanoid-robotic-book/module-1/
...
X URLs found
Creating collection in Qdrant Cloud: ai_book_embedding
Extracting text from https://osamabinadnan.github.io/physical-ai-humanoid-robotic-book/
Saved chunk 0 to Qdrant Cloud
Saved chunk 1 to Qdrant Cloud
...
Ingestion complete!
Total chunks ingested/stored: X
```

## Verification

To verify that the ingestion was successful:

1. Check your Qdrant Cloud dashboard to confirm the `ai_book_embedding` collection exists
2. Verify that the collection contains the expected number of vectors
3. Check that the vectors have the correct dimensionality (1024)
4. Confirm that the payload contains the expected metadata (url, text, chunk_id)

## Troubleshooting

### Common Issues

**API Key Errors**:
- Ensure your API keys are valid and have the necessary permissions
- Check for typos in your `.env` file

**Network Issues**:
- Verify that the target URLs are accessible
- Check your internet connection

**Rate Limiting**:
- If you encounter rate limit errors, the system should automatically retry with exponential backoff
- Consider reducing the number of concurrent requests if needed

**Qdrant Connection**:
- Verify that your Qdrant endpoint and API key are correct
- Check that your Qdrant Cloud instance is running

## Next Steps

After successful ingestion:

1. The embedded content will be available for retrieval in the RAG system
2. You can proceed to implement the retrieval and query components
3. Monitor the Qdrant Cloud instance for performance and usage metrics