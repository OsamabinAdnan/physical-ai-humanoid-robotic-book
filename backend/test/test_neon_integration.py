"""
Test script to verify Neon database connection and integration.
"""
import os
import asyncio
import sys
from dotenv import load_dotenv
from sqlalchemy.ext.asyncio import create_async_engine

# Add backend directory to path to import modules
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

# Load environment variables
load_dotenv()

def test_database_url():
    """Test if the database URL is properly loaded from environment."""
    database_url = os.getenv("NEON_DATABASE_URL")
    print(f"NEON_DATABASE_URL: {database_url}")

    if database_url:
        print("[OK] Database URL loaded successfully")
        return True
    else:
        print("[ERROR] Database URL not found in environment")
        return False

async def test_database_connection():
    """Test the actual database connection."""
    from database import init_db_engine, engine
    try:
        # Initialize the database engine
        init_db_engine()

        # Import the engine again after initialization to ensure it's updated
        from database import engine

        # Test the connection
        from sqlalchemy import text
        async with engine.connect() as conn:
            result = await conn.execute(text("SELECT 1"))
            row = result.fetchone()
            print(f"[OK] Database connection successful! Result: {row}")
            return True

    except Exception as e:
        print(f"[ERROR] Database connection failed: {e}")
        return False
    finally:
        # Import the engine again to make sure we have the global one
        from database import engine
        # Clean up the engine if it was created
        if engine is not None:
            await engine.dispose()

def test_models_import():
    """Test if database models can be imported."""
    try:
        from database.models import User, ChatSession, ChatMessage, Document
        print("[OK] Database models imported successfully")
        print(f"  - User model: {User.__tablename__}")
        print(f"  - ChatSession model: {ChatSession.__tablename__}")
        print(f"  - ChatMessage model: {ChatMessage.__tablename__}")
        print(f"  - Document model: {Document.__tablename__}")
        return True
    except ImportError as e:
        print(f"[ERROR] Failed to import database models: {e}")
        return False

def test_services_import():
    """Test if database services can be imported."""
    try:
        from database.services import UserService, ChatSessionService, ChatMessageService, DocumentService, DatabaseService
        print("[OK] Database services imported successfully")
        return True
    except ImportError as e:
        print(f"[ERROR] Failed to import database services: {e}")
        return False

async def main():
    """Main test function."""
    print("Starting Neon Database Integration Tests...\n")

    # Test 1: Environment variables
    env_ok = test_database_url()
    print()

    # Test 2: Models import
    models_ok = test_models_import()
    print()

    # Test 3: Services import
    services_ok = test_services_import()
    print()

    # Test 4: Database connection (only if URL is available)
    if env_ok:
        conn_ok = await test_database_connection()
        print()
    else:
        conn_ok = False

    # Summary
    print("Test Summary:")
    print(f"  Environment: {'[OK]' if env_ok else '[FAIL]'}")
    print(f"  Models import: {'[OK]' if models_ok else '[FAIL]'}")
    print(f"  Services import: {'[OK]' if services_ok else '[FAIL]'}")
    print(f"  Database connection: {'[OK]' if conn_ok else '[FAIL]'}")

    all_passed = env_ok and models_ok and services_ok and conn_ok
    print(f"\nOverall result: {'[OK] ALL TESTS PASSED' if all_passed else '[FAIL] SOME TESTS FAILED'}")

    return all_passed

if __name__ == "__main__":
    asyncio.run(main())