from sentence_transformers import SentenceTransformer
from typing import List
import numpy as np
from app.core.settings import settings


class EmbeddingGenerator:
    def __init__(self):
        self.model = SentenceTransformer(settings.EMBEDDING_MODEL)

    def generate_embedding(self, text: str) -> List[float]:
        """
        Generate embedding for a single text
        """
        embedding = self.model.encode([text])
        return embedding[0].tolist()  # Convert to list for JSON serialization

    def generate_embeddings(self, texts: List[str]) -> List[List[float]]:
        """
        Generate embeddings for multiple texts
        """
        embeddings = self.model.encode(texts)
        return [embedding.tolist() for embedding in embeddings]


# Global instance
embedding_generator = EmbeddingGenerator()