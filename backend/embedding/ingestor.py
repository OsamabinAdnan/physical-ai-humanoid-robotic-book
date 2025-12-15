import requests
import xml.etree.ElementTree as ET
import trafilatura
from qdrant_client import QdrantClient
from qdrant_client.http.models import Distance, VectorParams, PointStruct
import os
from dotenv import load_dotenv
import logging
from sentence_transformers import SentenceTransformer

# Load environment variables
load_dotenv()

# Configure logging
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')
logger = logging.getLogger(__name__)

# -------------------------------
# Configuration
# -------------------------------

qdrant_url = os.getenv("QDRANT_CLUSTER_ENDPOINT")
if not qdrant_url:
    raise Exception("Missing QDRANT_CLUSTER_ENDPOINT environment variable")

qdrant_api_key = os.getenv("QDRANT_API_KEY")
if not qdrant_api_key:
    raise Exception("Missing QDRANT_API_KEY environment variable")

SITEMAP_URL = "https://osamabinadnan.github.io/physical-ai-humanoid-robotic-book/sitemap.xml"
COLLECTION_NAME = "ai_book_embedding"

# Initialize the sentence transformer model for multilingual embeddings
EMBEDDING_MODEL_NAME = "paraphrase-multilingual-MiniLM-L12-v2"
embedding_model = SentenceTransformer(EMBEDDING_MODEL_NAME)

# Connect to Qdrant Cloud
qdrant = QdrantClient(
    url=qdrant_url,
    api_key=qdrant_api_key
)

logger.info("Configuration loaded successfully")

# -------------------------------
# Step 01: Extract URLs from sitemap
# -------------------------------

def get_all_urls(sitemap_url):
    """
    Extract all URLs from the provided sitemap.xml file.

    Args:
        sitemap_url (str): URL to the sitemap.xml file

    Returns:
        List[str]: List of all discovered URLs
    """
    try:
        response = requests.get(sitemap_url)
        response.raise_for_status()
        xml_content = response.text
        root = ET.fromstring(xml_content)

        # Handle XML namespaces properly
        ns = {"ns": "http://www.sitemaps.org/schemas/sitemap/0.9"}

        urls = []
        for url_tag in root.findall("ns:url", ns):
            loc_tag = url_tag.find("ns:loc", ns)
            if loc_tag is not None:
                urls.append(loc_tag.text)

        logger.info(f"Found {len(urls)} URLs in sitemap")
        for url in urls:
            logger.info(f"- {url}")

        return urls
    except requests.exceptions.RequestException as e:
        logger.error(f"Error fetching sitemap: {e}")
        raise
    except ET.ParseError as e:
        logger.error(f"Error parsing sitemap XML: {e}")
        raise

# -------------------------------
# Step 02: Extract text from URLs
# -------------------------------

def extract_text_from_url(url):
    """
    Extract clean text from a web page using trafilatura.

    Args:
        url (str): URL of the page to extract text from

    Returns:
        str: Extracted text content, or empty string if extraction fails
    """
    try:
        response = requests.get(url)
        response.raise_for_status()
        html_content = response.text

        # Use trafilatura to extract main content, filtering out boilerplate
        text = trafilatura.extract(html_content,
                                  include_tables=True,
                                  include_formatting=True,
                                  no_fallback=False)

        if not text:
            logger.warning(f"Failed to extract text from {url}")
            return ""

        logger.info(f"Successfully extracted {len(text)} characters from {url}")
        return text
    except requests.exceptions.RequestException as e:
        logger.error(f"Error fetching URL {url}: {e}")
        return ""
    except Exception as e:
        logger.error(f"Error extracting text from {url}: {e}")
        return ""

# -------------------------------
# Step 03: Chunk the text
# -------------------------------

def chunk_text(text, max_chars=1200):
    """
    Split a text string into smaller chunks of specified maximum size.

    Args:
        text (str): Text to be chunked
        max_chars (int): Maximum character count per chunk (default: 1200)

    Returns:
        List[str]: List of text chunks
    """
    chunks = []

    # Split the text into sentences to maintain semantic coherence
    sentences = text.split('. ')

    current_chunk = ""
    for sentence in sentences:
        # Add the period back to the sentence
        sentence_with_period = sentence + '. ' if not sentence.endswith('.') else sentence

        # Check if adding this sentence would exceed the max character limit
        if len(current_chunk) + len(sentence_with_period) <= max_chars:
            current_chunk += sentence_with_period
        else:
            # If the current chunk is not empty, save it
            if current_chunk.strip():
                chunks.append(current_chunk.strip())

            # If the sentence itself is longer than max_chars, split it
            if len(sentence_with_period) > max_chars:
                # Split the long sentence into smaller parts
                for i in range(0, len(sentence_with_period), max_chars):
                    sub_sentence = sentence_with_period[i:i+max_chars]
                    if sub_sentence.strip():
                        chunks.append(sub_sentence.strip())
                current_chunk = ""
            else:
                current_chunk = sentence_with_period

    # Add the last chunk if it's not empty
    if current_chunk.strip():
        chunks.append(current_chunk.strip())

    logger.info(f"Text split into {len(chunks)} chunks")
    return chunks

# -------------------------------
# Step 04: Embed the text
# -------------------------------

def embedding(text):
    """
    Generate an embedding vector for the input text using SentenceTransformer model.

    Args:
        text (str): Text to generate embedding for

    Returns:
        List[float]: Embedding vector as a list of floats
    """
    try:
        # Use the SentenceTransformer model to generate embeddings
        embedding_vector = embedding_model.encode([text])[0].tolist()  # Encode and convert to list

        logger.info(f"Generated embedding with {len(embedding_vector)} dimensions for text of {len(text)} characters")
        return embedding_vector
    except Exception as e:
        logger.error(f"Error generating embedding for text: {e}")
        raise

# -------------------------------
# Step 05: Create or verify Qdrant collection
# -------------------------------

def create_collection(collection_name=COLLECTION_NAME):
    """
    Create or verify the Qdrant collection with the specified name and configuration.

    Args:
        collection_name (str): Name of the collection to create
    """
    try:
        # Check if the collection already exists
        if qdrant.collection_exists(collection_name):
            logger.info(f"Collection '{collection_name}' already exists, using existing collection")
            return

        # Create a new collection with 384-dimensional vectors (for SentenceTransformer embeddings)
        # Using cosine distance which is appropriate for embedding similarity
        qdrant.create_collection(
            collection_name=collection_name,
            vectors_config=VectorParams(
                size=384,  # SentenceTransformer's paraphrase-multilingual-MiniLM-L12-v2 produces 384-dimensional vectors
                distance=Distance.COSINE
            )
        )

        logger.info(f"Created collection '{collection_name}' with 384-dimensional vectors and cosine distance")
    except Exception as e:
        logger.error(f"Error creating collection '{collection_name}': {e}")
        raise

# -------------------------------
# Step 06: Store the embeddings in Qdrant Cloud
# -------------------------------

def save_chunk_to_qdrant(chunk, chunk_id, chunk_url):
    """
    Generate an embedding for the text chunk and save it to Qdrant with metadata.

    Args:
        chunk (str): Text chunk to embed and store
        chunk_id (int): Unique identifier for the chunk
        chunk_url (str): Source URL of the chunk
    """
    try:
        # Generate embedding for the chunk
        vector = embedding(chunk)

        # Upsert the point to Qdrant with metadata
        qdrant.upload_points(
            collection_name=COLLECTION_NAME,
            points=[
                PointStruct(
                    id=chunk_id,
                    vector=vector,
                    payload={
                        "url": chunk_url,
                        "text": chunk,
                        "chunk_id": chunk_id
                    }
                )
            ]
        )

        logger.info(f"Saved chunk {chunk_id} from {chunk_url} to Qdrant collection '{COLLECTION_NAME}'")
    except Exception as e:
        logger.error(f"Error saving chunk {chunk_id} to Qdrant: {e}")
        raise

# -------------------------------
# Step 07: Main Ingestion Pipeline
# -------------------------------

def ingest_book():
    """
    Main function that orchestrates the entire ingestion pipeline from URL extraction to vector storage.
    """
    logger.info("Starting book ingestion process...")

    try:
        # Get all URLs from the sitemap
        urls = get_all_urls(SITEMAP_URL)

        # Create or verify the Qdrant collection
        create_collection()

        # Process each URL
        global_id = 0
        for url in urls:
            logger.info(f"Processing URL: {url}")

            # Extract text from the URL
            text = extract_text_from_url(url)
            if not text:
                logger.warning(f"Could not extract text from {url}, skipping...")
                continue

            # Chunk the text
            chunks = chunk_text(text)
            logger.info(f"Split content from {url} into {len(chunks)} chunks")

            # Process each chunk
            for chunk in chunks:
                if chunk.strip():  # Only process non-empty chunks
                    save_chunk_to_qdrant(chunk, global_id, url)
                    global_id += 1

        logger.info(f"Ingestion complete! Total chunks processed and stored: {global_id}")
        return global_id
    except Exception as e:
        logger.error(f"Error during ingestion process: {e}")
        raise

# Execute the ingestion if this script is run directly
if __name__ == "__main__":
    ingest_book()