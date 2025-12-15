from pydantic import BaseModel
from typing import List, Optional


class QueryRequest(BaseModel):
    """
    Schema for the incoming API request containing user question.
    """
    question: str
    top_k: Optional[int] = 5


class Citation(BaseModel):
    """
    Schema for citation information in the response.
    """
    text: str
    url: str
    similarity_score: float
    chunk_id: int
    source_title: str


class QueryResponse(BaseModel):
    """
    Schema for the API response containing agent answer and citations.
    """
    answer: str
    citations: List[Citation]
    confidence: float
    processing_time_ms: int