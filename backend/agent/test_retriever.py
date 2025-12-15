import os
from dotenv import load_dotenv

# Load environment variables
load_dotenv()

# Test that the required environment variables are available
qdrant_url = os.getenv("QDRANT_CLUSTER_ENDPOINT")
qdrant_api_key = os.getenv("QDRANT_API_KEY")

if not qdrant_url:
    print("ERROR: QDRANT_CLUSTER_ENDPOINT environment variable not found")
    exit(1)

if not qdrant_api_key:
    print("ERROR: QDRANT_API_KEY environment variable not found")
    exit(1)

print("Environment variables loaded successfully")

# Test importing the necessary modules
try:
    from sentence_transformers import SentenceTransformer
    from qdrant_client import QdrantClient
    print("Dependencies imported successfully")
except ImportError as e:
    print(f"ERROR: Failed to import dependencies: {e}")
    exit(1)

# Test initializing the model
try:
    EMBEDDING_MODEL_NAME = "paraphrase-multilingual-MiniLM-L12-v2"
    embedding_model = SentenceTransformer(EMBEDDING_MODEL_NAME)
    print("Sentence transformer model initialized successfully")
except Exception as e:
    print(f"ERROR: Failed to initialize sentence transformer model: {e}")
    exit(1)

# Test connecting to Qdrant (without actually making a call to preserve rate limits)
try:
    qdrant = QdrantClient(
        url=qdrant_url,
        api_key=qdrant_api_key
    )
    print("Qdrant client initialized successfully")
except Exception as e:
    print(f"ERROR: Failed to initialize Qdrant client: {e}")
    exit(1)

# Test importing the retrieve function from the module
try:
    from retriever import retrieve, get_embedding
    print("Retrieve and get_embedding functions imported successfully")
except Exception as e:
    print(f"ERROR: Failed to import retrieve function: {e}")
    exit(1)

print("All basic initialization and import tests passed. The retriever module is ready for use.")