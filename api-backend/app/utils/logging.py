import logging
from datetime import datetime
import json

# Create a custom logger
logger = logging.getLogger(__name__)
logger.setLevel(logging.DEBUG)

# Create handlers
console_handler = logging.StreamHandler()
console_handler.setLevel(logging.DEBUG)

# Create formatters and add them to handlers
formatter = logging.Formatter(
    '%(asctime)s - %(name)s - %(levelname)s - %(message)s'
)
console_handler.setFormatter(formatter)

# Add handlers to the logger
if not logger.handlers:
    logger.addHandler(console_handler)

def log_api_call(endpoint: str, method: str, params: dict = None, response: dict = None, error: str = None):
    """
    Log API calls with relevant information
    """
    log_data = {
        "timestamp": datetime.utcnow().isoformat(),
        "endpoint": endpoint,
        "method": method,
        "params": params,
        "response": response,
        "error": error
    }

    if error:
        logger.error(f"API Call Error: {json.dumps(log_data, default=str)}")
    else:
        logger.info(f"API Call: {json.dumps(log_data, default=str)}")

def log_rag_process(selected_text: str, context_chunks: list, query_embedding: list = None, search_results: list = None):
    """
    Log the RAG process with relevant information
    """
    log_data = {
        "timestamp": datetime.utcnow().isoformat(),
        "process": "RAG",
        "selected_text": selected_text[:100] + "..." if selected_text and len(selected_text) > 100 else selected_text,
        "context_chunks_count": len(context_chunks) if context_chunks else 0,
        "query_embedding_length": len(query_embedding) if query_embedding else 0,
        "search_results_count": len(search_results) if search_results else 0
    }

    logger.info(f"RAG Process: {json.dumps(log_data, default=str)}")

def log_error(error_msg: str, context: dict = None):
    """
    Log errors with optional context
    """
    log_data = {
        "timestamp": datetime.utcnow().isoformat(),
        "error": error_msg,
        "context": context
    }

    logger.error(f"Error: {json.dumps(log_data, default=str)}")