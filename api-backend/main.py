import requests
import xml.etree.ElementTree as ET
import trafilatura
from qdrant_client import QdrantClient
from qdrant_client.http.models import Distance, VectorParams, PointStruct
import os
from cohere import Client
from dotenv import load_dotenv

load_dotenv()

# -------------------------------
# Configuration
# -------------------------------

cohere_api_key = os.getenv("COHERE_EMBEDDING_API")
if not cohere_api_key:
    raise Exception("Missing COHERE_EMBEDDING_API environment variable")
qdrant_url = os.getenv("QDRANT_CLUSTER_ENDPOINT")
if not qdrant_url:
    raise Exception("Missing QDRANT_CLUSTER_ENDPOINT environment variable")
qdrant_api_key = os.getenv("QDRANT_API_KEY")
if not qdrant_api_key:
    raise Exception("Missing QDRANT_API_KEY environment variable")

SITEMAP_URL ="https://osamabinadnan.github.io/physical-ai-humanoid-robotic-book/sitemap.xml"
COLLECTION_NAME = "physical-ai-humanoid-robotics-textbook"

cohere_client = Client(cohere_api_key)
EMBEDDING_MODEL = "embed-english-v3.0"

# Connect to Qdrant Cloud

qdrant = QdrantClient(
    url=qdrant_url,
    api_key=qdrant_api_key
)

# -------------------------------
# Step 01: Extract URLs from sitemap
# -------------------------------

def get_all_urls_from_sitemap(sitemap_url):
    xml = requests.get(sitemap_url).text
    root = ET.fromstring(xml)

    ns = {"ns": "http://www.sitemaps.org/schemas/sitemap/0.9"} # This properly handles namespaces using

    urls = []
    for url_tag in root.findall("ns:url", ns):
        loc_tag = url_tag.find("ns:loc", ns)
        if loc_tag is not None:
            urls.append(loc_tag.text)

    print("\nFound URLs:")
    for u in urls:
        print("-", u)
    print(len(urls), "URLs found")

    return urls

# -------------------------------
# Step 02: Extract text from URLs
# -------------------------------

def extract_text_from_url(url):
    html = requests.get(url).text
    text = trafilatura.extract(html)

    if not text:
        print(f"[ERROR]Failed to extract text from {url}")
    
    return text

# -------------------------------
# Step 03: Chunk the text
# -------------------------------

def chunk_text(text, max_chars = 1200):
    chunks = []
    while len(text) > max_chars:
        split_pos = text[:max_chars].rfind(". ")
        if split_pos == -1:
            split_pos = max_chars
        chunks.append(text[:split_pos])
        text = text[split_pos:]
    chunks.append(text)
    return chunks

# -------------------------------
# Step 04: Embed the text
# -------------------------------

def embed_text(text):
    response = cohere_client.embed(
        model=EMBEDDING_MODEL,
        input_type="search_query", # Use search query for queries
        texts=[text]
    )
    return response.embeddings[0] # Return the first embedding

# -------------------------------
# Step 05: Store the embeddings Collection in Qdrant Cloud
# -------------------------------

def creat_collection():
    print(f"Creating collection in Qdrant Cloud: {COLLECTION_NAME}")
    qdrant.recreate_collection(
        collection_name=COLLECTION_NAME,
        vectors_config=VectorParams(
            size=1024, # Embedding size of Cohere model
            distance=Distance.COSINE
        )
    )

# -------------------------------
# Step 06: Store the embeddings in Qdrant Cloud
# -------------------------------

def save_chunk_to_qdrant(chunk, chunk_id, chunk_url):
    vector = embed_text(chunk)

    qdrant.upsert(
        collection_name=COLLECTION_NAME,
        points = [
            PointStruct(
                id=chunk_id,
                vector=vector,
                payload= {
                    "url": chunk_url,
                    "text": chunk,
                    "chunk_id": chunk_id
                }
            )
        ]
    )
    
# -------------------------------
# Step 07: Main Ingestion Pipeline
# -------------------------------

def ingest_book():
    urls = get_all_urls_from_sitemap(SITEMAP_URL)

    creat_collection()

    global_id = 0

    for url in urls:
        print(f"Extracting text from {url}")
        text = extract_text_from_url(url)

        if not text:
            continue

        chunks = chunk_text(text)

        for chunk in chunks:
            save_chunk_to_qdrant(chunk, global_id, url)
            print(f"Saved chunk {global_id} to Qdrant Cloud")
            global_id += 1
    
    print("\nIngestion complete!")
    print("Total chunks ingested/stored:", global_id)

if __name__ == "__main__":
    ingest_book()


