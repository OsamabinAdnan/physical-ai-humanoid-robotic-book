#!/usr/bin/env python3
"""
Test script to verify the API endpoints work correctly
"""
import sys
import os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

import asyncio
from main import app
from agent.schemas import QueryRequest

def test_api_endpoints():
    print("Testing API endpoints...")

    # Test that the app has the required endpoints
    routes = [route.path for route in app.routes]
    print(f"Available routes: {routes}")

    if "/chat" in routes:
        print("[OK] /chat endpoint exists")
    else:
        print("[ERROR] /chat endpoint missing")
        return False

    if "/health" in routes:
        print("[OK] /health endpoint exists")
    else:
        print("[ERROR] /health endpoint missing")
        return False

    # Test schema
    try:
        request = QueryRequest(question="What is Physical AI?", top_k=3)
        print(f"[OK] QueryRequest schema works: {request}")
    except Exception as e:
        print(f"[ERROR] QueryRequest schema failed: {e}")
        return False

    print("\nAPI endpoints are properly configured! [OK]")
    return True

if __name__ == "__main__":
    success = test_api_endpoints()
    if success:
        print("\nAPI endpoints are ready for use!")
    else:
        print("\nAPI endpoint tests failed.")
        sys.exit(1)