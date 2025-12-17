"""
Final comprehensive integration test for Neon database integration.

This test suite validates all aspects of the Neon database integration:
- Database connection and configuration
- Model definitions and relationships
- Service layer CRUD operations
- API integration and chat persistence
- Data isolation and security
"""
import asyncio
import sys
import os
from dotenv import load_dotenv

# Load environment variables first
load_dotenv()

# Add the backend directory to the path to import modules
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

async def test_full_integration():
    """Test the complete Neon database integration."""
    print("Starting full Neon database integration test...\n")

    # Test 1: Initialize database engine
    print("1. Testing database engine initialization...")
    from database import init_db_engine
    init_db_engine()

    # Re-import the engine after initialization to ensure we get the updated value
    from database import engine
    if engine is not None:
        print("   [OK] Database engine initialized successfully")
    else:
        print("   [ERROR] Database engine initialization failed")
        return False

    # Test 2: Test database connection
    print("\n2. Testing database connection...")
    from sqlalchemy import text
    try:
        async with engine.connect() as conn:
            result = await conn.execute(text("SELECT 1"))
            row = result.fetchone()
            if row and row[0] == 1:
                print("   [OK] Database connection successful")
            else:
                print("   [ERROR] Database connection failed")
                return False
    except Exception as e:
        print(f"   [ERROR] Database connection failed: {e}")
        return False

    # Test 3: Test model imports
    print("\n3. Testing model imports...")
    try:
        from database.models import User, ChatSession, ChatMessage, Document
        print("   [OK] All models imported successfully")
        print(f"   [OK] User table: {User.__tablename__}")
        print(f"   [OK] ChatSession table: {ChatSession.__tablename__}")
        print(f"   [OK] ChatMessage table: {ChatMessage.__tablename__}")
        print(f"   [OK] Document table: {Document.__tablename__}")
    except Exception as e:
        print(f"   [ERROR] Model import failed: {e}")
        return False

    # Test 4: Test service imports
    print("\n4. Testing service imports...")
    try:
        from database.services import UserService, ChatSessionService, ChatMessageService, DocumentService, DatabaseService
        print("   [OK] All services imported successfully")
    except Exception as e:
        print(f"   [ERROR] Service import failed: {e}")
        return False

    # Test 5: Test actual CRUD operations
    print("\n5. Testing CRUD operations...")
    try:
        from database import AsyncDBSession
        from database.services import DatabaseService
        import uuid

        async with AsyncDBSession() as session:
            db_service = DatabaseService(session)

            # Create a test user
            user = await db_service.user.create_user(
                email="integration@test.com",
                name="Integration Test User"
            )
            if not user:
                print("   [ERROR] User creation failed")
                return False
            print(f"   [OK] User created: {user.id}")

            # Create a chat session
            chat_session = await db_service.chat_session.create_session(
                user_id=user.id,
                title="Integration Test Session"
            )
            if not chat_session:
                print("   [ERROR] Chat session creation failed")
                return False
            print(f"   [OK] Chat session created: {chat_session.id}")

            # Add a message
            message = await db_service.chat_message.add_message(
                session_id=chat_session.id,
                user_id=user.id,
                role="user",
                content="This is a test message for integration testing."
            )
            if not message:
                print("   [ERROR] Message creation failed")
                return False
            print(f"   [OK] Message created: {message.id}")

            # Retrieve the message
            messages = await db_service.chat_message.get_messages_by_session(chat_session.id, user.id)
            if len(messages) != 1 or messages[0].content != "This is a test message for integration testing.":
                print("   [ERROR] Message retrieval failed")
                return False
            print("   [OK] Message retrieved successfully")

            # Test data isolation by trying to access with wrong user ID
            fake_user_id = uuid.uuid4()
            isolated_messages = await db_service.chat_message.get_messages_by_session(chat_session.id, fake_user_id)
            if len(isolated_messages) != 0:
                print("   [ERROR] Data isolation failed - other user could access session")
                return False
            print("   [OK] Data isolation working correctly")

            # Clean up
            await db_service.chat_session.delete_session(chat_session.id, user.id)
            await db_service.user.delete_user(user.id)
            print("   [OK] Test data cleaned up successfully")

    except Exception as e:
        print(f"   [ERROR] CRUD operations failed: {e}")
        import traceback
        traceback.print_exc()
        return False

    # Test 6: Test API integration (health check)
    print("\n6. Testing API integration...")
    try:
        from main import db_health_check
        from database import AsyncDBSession
        from database.services import DatabaseService

        # Create a mock test for the health check endpoint
        async def mock_test():
            async with AsyncDBSession() as session:
                db_service = DatabaseService(session)
                result = await db_health_check(db_service)
                if isinstance(result, dict) and "status" in result:
                    if result["status"] == "healthy":
                        print("   [OK] DB health check endpoint working")
                        return True
                    else:
                        print(f"   [ERROR] DB health check failed: {result['status']}")
                        return False
                else:
                    print(f"   [ERROR] Unexpected health check result: {result}")
                    return False

        health_ok = await mock_test()
        if not health_ok:
            return False
    except Exception as e:
        print(f"   [ERROR] API integration test failed: {e}")
        import traceback
        traceback.print_exc()
        return False

    print("\n" + "="*50)
    print("SUCCESS! ALL INTEGRATION TESTS PASSED!")
    print("="*50)
    print("Neon Serverless Postgres integration is working correctly!")
    print("- Database connection: [OK]")
    print("- Model definitions: [OK]")
    print("- Service layer: [OK]")
    print("- CRUD operations: [OK]")
    print("- Data isolation: [OK]")
    print("- API integration: [OK]")
    print("="*50)

    return True

async def run_all_tests():
    """Run all tests and report results."""
    print("Starting Neon Database Integration Tests\n")

    success = await test_full_integration()

    if success:
        print("\n[SUCCESS] All Neon database integration tests passed!")
        return True
    else:
        print(f"\n[FAILURE] Integration tests failed!")
        return False


if __name__ == "__main__":
    success = asyncio.run(run_all_tests())
    if success:
        print("\n[OVERALL SUCCESS] Neon database integration is working correctly!")
    else:
        print("\n[OVERALL FAILURE] Neon database integration has issues!")
        sys.exit(1)