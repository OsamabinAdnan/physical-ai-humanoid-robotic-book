import os
from qdrant_client import QdrantClient
from qdrant_client.http.models import Distance, VectorParams
from dotenv import load_dotenv

# Load environment variables
load_dotenv()

# Get Qdrant configuration
qdrant_url = os.getenv("QDRANT_CLUSTER_ENDPOINT")
qdrant_api_key = os.getenv("QDRANT_API_KEY")
collection_name = "ai_book_embedding"

# Connect to Qdrant Cloud
qdrant = QdrantClient(
    url=qdrant_url,
    api_key=qdrant_api_key
)

print(f"Connected to Qdrant at {qdrant_url}")

# Check if collection exists
if qdrant.collection_exists(collection_name):
    print(f"Collection '{collection_name}' exists. Deleting it...")
    qdrant.delete_collection(collection_name)
    print(f"Collection '{collection_name}' deleted.")
else:
    print(f"Collection '{collection_name}' does not exist.")

# Create a new collection with 384-dimensional vectors (for SentenceTransformer embeddings)
# Using cosine distance which is appropriate for embedding similarity
qdrant.create_collection(
    collection_name=collection_name,
    vectors_config=VectorParams(
        size=384,  # SentenceTransformer's paraphrase-multilingual-MiniLM-L12-v2 produces 384-dimensional vectors
        distance=Distance.COSINE
    )
)

print(f"Created collection '{collection_name}' with 384-dimensional vectors and cosine distance")
print("Collection reset complete!")