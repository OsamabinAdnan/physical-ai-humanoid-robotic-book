#!/usr/bin/env python3
"""
Test script to verify all imports work correctly before starting the server
"""
import os
import sys

# Add backend to path
sys.path.insert(0, os.path.join(os.path.dirname(__file__)))

def test_imports():
    print("Testing imports...")

    # Test basic imports
    try:
        from agent.schemas import QueryRequest, QueryResponse, Citation
        print("✓ Schemas imported successfully")
    except Exception as e:
        print(f"✗ Schemas import failed: {e}")
        return False

    # Test retriever import
    try:
        from agent.retriever import retrieve, get_embedding, perform_qdrant_search, format_search_results
        print("✓ Retriever imported successfully")
    except Exception as e:
        print(f"✗ Retriever import failed: {e}")
        return False

    # Test main app import
    try:
        from main import app
        print("✓ Main app imported successfully")
    except Exception as e:
        print(f"✗ Main app import failed: {e}")
        return False

    # Test agent import (with environment variables set to avoid errors)
    try:
        os.environ.setdefault("QWEN_API_KEY", "dummy_key")
        os.environ.setdefault("QWEN_URL", "https://dummy.url")
        os.environ.setdefault("QDRANT_CLUSTER_ENDPOINT", "https://dummy.qdrant.io")
        os.environ.setdefault("QDRANT_API_KEY", "dummy_key")

        from agent.agent import agent
        print("✓ Agent imported successfully")
    except Exception as e:
        print(f"✗ Agent import failed: {e}")
        return False

    print("\nAll imports successful! ✓")
    return True

if __name__ == "__main__":
    success = test_imports()
    if success:
        print("\nYou can now run the server with: uvicorn main:app --reload")
    else:
        print("\nSome imports failed. Please fix the errors before running the server.")
        sys.exit(1)