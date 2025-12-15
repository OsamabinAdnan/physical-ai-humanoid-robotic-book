import os
from dotenv import load_dotenv
from agents import (
    Agent,
    Runner,
    OpenAIChatCompletionsModel,
    set_tracing_disabled,
    ModelSettings,
    function_tool
)
from openai import AsyncOpenAI
import logging
from typing import List, Dict, Any

from sentence_transformers import SentenceTransformer
from qdrant_client import QdrantClient

# Load environment variables
load_dotenv()
set_tracing_disabled(True)

# Configure logging
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')
logger = logging.getLogger(__name__)

qdrant_url = os.getenv("QDRANT_CLUSTER_ENDPOINT")
qdrant_api_key = os.getenv("QDRANT_API_KEY")

# Initialize the sentence transformer model for multilingual embeddings
EMBEDDING_MODEL_NAME = "paraphrase-multilingual-MiniLM-L12-v2"
embedding_model = SentenceTransformer(EMBEDDING_MODEL_NAME)

# Connect to Qdrant
qdrant = QdrantClient(
    url=qdrant_url,
    api_key=qdrant_api_key
)

# Get Qwen API credentials from environment
QWEN_API_KEY = os.getenv("QWEN_API_KEY")
if not QWEN_API_KEY:
    raise Exception("Missing QWEN_API_KEY environment variable")

QWEN_URL = os.getenv("QWEN_URL")
if not QWEN_URL:
    raise Exception("Missing QWEN_URL environment variable")

def get_embedding(text: str) -> List[float]:
    """
    Generate a 384-dimensional embedding vector for the input text using sentence-transformers model.

    Args:
        text (str): Text to generate embedding for

    Returns:
        List[float]: 384-dimensional embedding vector as a list of floats
    """
    try:
        embedding_vector = embedding_model.encode([text])[0].tolist()  # Encode and convert to list
        logger.info(f"Generated embedding with {len(embedding_vector)} dimensions for text of {len(text)} characters")
        return embedding_vector
    except Exception as e:
        logger.error(f"Error generating embedding for text: {e}")
        raise


def perform_qdrant_search(query_embedding: List[float], top_k: int = 5) -> List[Dict[str, Any]]:
    """
    Perform semantic search in Qdrant collection to retrieve top-K relevant chunks.

    Args:
        query_embedding (List[float]): 384-dimensional query embedding vector
        top_k (int): Number of top results to retrieve (default: 5)

    Returns:
        List[Dict[str, Any]]: List of result dictionaries containing text, URL, chunk_id, and similarity_score
    """
    try:
        result = qdrant.query_points(
            collection_name="ai_book_embedding",  # Using the correct collection name from the spec
            query=query_embedding,
            limit=top_k,
        )

        # Return structured citation data
        citations = []
        for point in result.points:
            citation = {
                "text": point.payload.get("text", ""),
                "url": point.payload.get("url", ""),
                "source_title": point.payload.get("source_title", ""),
                "chunk_id": point.payload.get("chunk_id", ""),
                "similarity_score": getattr(point, 'score', 0.0)  # Use getattr to safely get score
            }
            citations.append(citation)

        logger.info(f"Retrieved {len(citations)} results from Qdrant collection")
        return citations
    except Exception as e:
        logger.error(f"Error performing Qdrant search: {e}")
        raise


def format_search_results(results: List[Dict[str, Any]]) -> Dict[str, Any]:
    """
    Format raw Qdrant results into a structured response with clean metadata.

    Args:
        results (List[Dict[str, Any]]): Raw results from Qdrant search

    Returns:
        Dict[str, Any]: Structured dictionary with retrieved_texts and citations
    """
    try:
        retrieved_texts = [result["text"] for result in results]

        formatted_results = {
            "retrieved_texts": retrieved_texts,
            "citations": results
        }

        logger.info(f"Formatted {len(results)} search results")
        return formatted_results
    except Exception as e:
        logger.error(f"Error formatting search results: {e}")
        raise


# Initialize the AsyncOpenAI-compatible client with Qwen details
external_client: AsyncOpenAI = AsyncOpenAI(
    api_key=QWEN_API_KEY,
    base_url=QWEN_URL,
)


# Model Initialization - using Qwen model with OpenAI-compatible interface
model: OpenAIChatCompletionsModel = OpenAIChatCompletionsModel(
    model="qwen3-coder-plus",  # Using Qwen model
    openai_client=external_client
)


@function_tool
def retrieve(query: str, top_k: int = 2) -> Dict[str, Any]:
    """
    Retrieve relevant chunks with citations from the Qdrant vector database.
    Works for both regular queries and section-selected queries (explain/summarize).
    Returns structured data with text and citation information.

    Args:
        query (str): User's text query for semantic search
        top_k (int): Number of top results to retrieve (default: 5)

    Returns:
        Dict[str, Any]: Dictionary containing retrieved texts and citations
    """
    try:
        logger.info(f"Processing retrieval query: {query}")

        # Generate embedding for the query
        query_embedding = get_embedding(query)

        # Perform search in Qdrant
        raw_results = perform_qdrant_search(query_embedding, top_k)

        # Format the results
        formatted_results = format_search_results(raw_results)

        logger.info(f"Retrieval completed successfully, returning {len(formatted_results['retrieved_texts'])} results")
        return formatted_results
    except Exception as e:
        logger.error(f"Error during retrieval process: {e}")
        raise



# Create agent and register tools
agent = Agent(
    name="Physical AI & Humanoid Robotics Tutor",  # Agent's identity
    instructions=(
        """
    - You are an expert AI tutor for the Physical AI & Humanoid Robotics textbook.
    - When a user asks a question, always first call the `retrieve` tool with the user query,
    then use only the return content from retrieve to answer.
    - If the answer is not in the retrieved content via tool, say `I'm sorry. I am here to help you about Physical AI & Humanoid-robotics textbook, so I can only entertain you with information from the textbook.`
    - For section-selected queries where a user wants to know about specific text, use the same retrieval process but focus your response on the selected text.
        * When user highlight task, a dialog box or dropdown menu will appear having option to `Ask AI`, when user click on it, it will directly go to Chatbot as a query and agent will response accordingly.
    - You can add addition information from your own knowledge base to your response but it will be strictly relevant to the book topic and user query about Physical AI & Humanoid-robotics.
    - User ask via `chatbot query` or ask via `selected text`, you will answer within 80 to 150 words.
    """
    ),
    model=model,
    tools=[retrieve],  # Register the retrieval tool
    model_settings=ModelSettings(
        tool_choice="required"
    )
)

logger.info("Agent initialized successfully with Qwen API as LLM provider")
logger.info(f"Agent name: {agent.name}")



# Execute the ingestion if this script is run directly
# if __name__ == "__main__":
#     print("RAG Chatbot Retriever module loaded successfully")
#     print("Environment variables and clients initialized")
#     print("Ready to use as function_tool for agent integration")


# result = Runner.run_sync(
#     starting_agent=agent,
#     input="What is Humanoid Robotics?",
# )

# print(result.final_output)