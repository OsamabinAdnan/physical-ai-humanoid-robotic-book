# Quickstart Guide: RAG Chatbot Agent-Based Backend

## Prerequisites

- Python 3.8 or higher
- pip package manager
- Git (if cloning the repository)
- Valid Qwen API key
- Qdrant Cloud account with the ai_book_embedding collection populated

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
uv pip install fastapi uvicorn python-dotenv openai-agents qdrant-client sentence-transformers
```

### 4. Configure Environment Variables
Ensure your `.env` file in the backend directory contains the following variables:

```env
QWEN_API_KEY=your_qwen_api_key_here
QWEN_URL=https://portal.qwen.ai/v1/
QDRANT_CLUSTER_ENDPOINT=your_qdrant_cluster_endpoint_here
QDRANT_API_KEY=your_qdrant_api_key_here
```

## Running the Agent Backend

### 1. Navigate to the Backend Directory
```bash
cd backend
```

### 2. Start the FastAPI Server
```bash
uvicorn agent.main:app --reload --port 8000
```

### 3. Alternative: Run with Production Settings
```bash
uvicorn agent.main:app --host 0.0.0.0 --port 8000 --workers 4
```

## API Usage

### Send a Query to the Agent
```bash
curl -X POST "http://localhost:8000/query" \
  -H "Content-Type: application/json" \
  -d '{
    "question": "What is Physical AI?",
    "top_k": 5
  }'
```

### Expected Response Format
```json
{
  "answer": "Physical AI refers to the intersection of artificial intelligence and physical systems...",
  "citations": [
    {
      "text": "Physical AI refers to the intersection of artificial intelligence and physical systems...",
      "url": "https://osamabinadnan.github.io/physical-ai-humanoid-robotic-book/docs/module-1/chapter-1/",
      "similarity_score": 0.87,
      "chunk_id": 123,
      "source_title": "Chapter 1: Introduction to Physical AI"
    }
  ],
  "confidence": 0.92,
  "processing_time_ms": 2450
}
```

## Verification

To verify that the agent backend is working correctly:

1. Check that the server starts without errors
2. Test the health endpoint: `GET /health`
3. Submit a test query to ensure the agent responds appropriately
4. Verify that citations are properly included in responses
5. Confirm that the agent uses retrieved content for answers

## Testing the Agent

### Manual Testing
Send various types of questions to the agent:
- Questions that should have matches in the book content
- Questions that might not have matches
- Very short and very long queries

### Automated Testing
The system includes test endpoints for validation:
- Health checks to verify service availability
- Performance metrics to monitor response times

## Troubleshooting

### Common Issues

**Qwen API Connection Errors**:
- Verify that your QWEN_API_KEY is valid
- Check that the QWEN_URL is correct
- Ensure your account has sufficient credits/quotas

**Qdrant Connection Issues**:
- Verify that your QDRANT_CLUSTER_ENDPOINT and QDRANT_API_KEY are correct
- Confirm that the ai_book_embedding collection exists and has content
- Check that your Qdrant Cloud instance is running

**Slow Response Times**:
- Check for network latency between your server and external APIs
- Monitor resource usage during concurrent requests
- Verify that the retrieval function is working efficiently

**Agent Hallucinations**:
- Verify that the retrieval tool is properly integrated
- Ensure the agent is being prompted to use only retrieved content
- Check that the retrieved content is relevant to the query