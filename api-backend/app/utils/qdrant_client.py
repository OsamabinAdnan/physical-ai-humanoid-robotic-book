from qdrant_client import QdrantClient
from qdrant_client.http import models
from typing import List, Dict, Optional
from app.core.settings import settings
import uuid


class QdrantManager:
    def __init__(self):
        # Initialize Qdrant client with cloud endpoint
        self.client = QdrantClient(
            url=settings.QDRANT_CLUSTER_ENDPOINT,
            api_key=settings.QDRANT_API_KEY,
            # prefer_grpc=True  # Uncomment for better performance if needed
        )
        self.collection_name = "textbook_content"

    def create_collection(self, vector_size: int = 384):  # all-MiniLM-L6-v2 outputs 384-dim vectors
        """
        Create a collection for storing textbook content embeddings
        """
        try:
            # Check if collection already exists
            self.client.get_collection(self.collection_name)
            print(f"Collection '{self.collection_name}' already exists")
        except:
            # Create new collection
            self.client.create_collection(
                collection_name=self.collection_name,
                vectors_config=models.VectorParams(
                    size=vector_size,
                    distance=models.Distance.COSINE
                )
            )
            print(f"Created collection '{self.collection_name}'")

    def store_text_chunk(self, text: str, metadata: Dict, vector: List[float], point_id: Optional[str] = None):
        """
        Store a text chunk with its embedding in Qdrant
        """
        if point_id is None:
            point_id = str(uuid.uuid4())

        self.client.upsert(
            collection_name=self.collection_name,
            points=[
                models.PointStruct(
                    id=point_id,
                    vector=vector,
                    payload={
                        "text": text,
                        "metadata": metadata
                    }
                )
            ]
        )
        return point_id

    def search_similar(self, query_vector: List[float], limit: int = 5) -> List[Dict]:
        """
        Search for similar text chunks based on the query vector
        """
        search_result = self.client.search(
            collection_name=self.collection_name,
            query_vector=query_vector,
            limit=limit
        )

        results = []
        for hit in search_result:
            results.append({
                "id": hit.id,
                "text": hit.payload.get("text", ""),
                "metadata": hit.payload.get("metadata", {}),
                "score": hit.score
            })

        return results

    def delete_collection(self):
        """
        Delete the collection (useful for re-indexing)
        """
        try:
            self.client.delete_collection(self.collection_name)
            print(f"Deleted collection '{self.collection_name}'")
        except Exception as e:
            print(f"Error deleting collection: {e}")


# Global instance
qdrant_manager = QdrantManager()