"""
Test connection stability and resilience for Neon database integration.
"""
import asyncio
import time
import sys
import os
from dotenv import load_dotenv

# Load environment variables first
load_dotenv()

# Add the backend directory to the path to import modules
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

async def test_connection_stability():
    """Test database connection stability under various conditions."""
    print("Starting connection stability tests...\n")

    # Initialize database first
    from database import init_db_engine
    init_db_engine()

    # Test 1: Multiple concurrent connections
    print("1. Testing concurrent connections...")
    from database import AsyncDBSession
    from database.services import DatabaseService
    import uuid

    # Create multiple concurrent sessions
    async def create_session_with_user(session_num):
        async with AsyncDBSession() as session:
            db_service = DatabaseService(session)

            # Create a test user
            user = await db_service.user.create_user(
                email=f"test{session_num}@stability.com",
                name=f"Stability Test User {session_num}"
            )

            # Create a chat session
            chat_session = await db_service.chat_session.create_session(
                user_id=user.id,
                title=f"Stability Test Session {session_num}"
            )

            # Add a message
            message = await db_service.chat_message.add_message(
                session_id=chat_session.id,
                user_id=user.id,
                role="user",
                content=f"This is test message {session_num} for stability testing."
            )

            # Retrieve the message to verify
            messages = await db_service.chat_message.get_messages_by_session(chat_session.id, user.id)

            # Clean up
            await db_service.chat_session.delete_session(chat_session.id, user.id)
            await db_service.user.delete_user(user.id)

            return len(messages)

    # Run multiple concurrent operations
    start_time = time.time()
    tasks = [create_session_with_user(i) for i in range(5)]
    results = await asyncio.gather(*tasks)
    end_time = time.time()

    if all(r == 1 for r in results):
        print(f"   [OK] All {len(results)} concurrent operations completed successfully in {end_time - start_time:.2f}s")
    else:
        print(f"   [ERROR] Some operations failed: {results}")
        return False

    # Test 2: Connection persistence over time
    print("\n2. Testing connection persistence...")
    start_time = time.time()

    async with AsyncDBSession() as session:
        db_service = DatabaseService(session)

        # Perform a simple operation
        from sqlalchemy import text
        result = await db_service.db_session.execute(text("SELECT 1"))
        row = result.fetchone()

        if row and row[0] == 1:
            print("   [OK] Connection remained stable over time")
        else:
            print("   [ERROR] Connection failed after time delay")
            return False

    # Wait a bit and test again
    await asyncio.sleep(1)
    async with AsyncDBSession() as session:
        db_service = DatabaseService(session)

        # Perform another operation
        from sqlalchemy import text
        result = await db_service.db_session.execute(text("SELECT 1"))
        row = result.fetchone()

        if row and row[0] == 1:
            print("   [OK] Connection remained stable after delay")
        else:
            print("   [ERROR] Connection failed after delay")
            return False

    # Test 3: Error handling and recovery
    print("\n3. Testing error handling and recovery...")

    try:
        async with AsyncDBSession() as session:
            db_service = DatabaseService(session)

            # Test proper error handling by attempting to retrieve non-existent data
            fake_user_id = uuid.uuid4()
            fake_session_id = uuid.uuid4()

            # This should return empty results rather than crash
            messages = await db_service.chat_message.get_messages_by_session(fake_session_id, fake_user_id)

            if len(messages) == 0:
                print("   [OK] Proper error handling for non-existent data")
            else:
                print(f"   [ERROR] Unexpected results for non-existent data: {len(messages)}")
                return False

    except Exception as e:
        print(f"   [ERROR] Exception during error handling test: {e}")
        return False

    print("\n" + "="*50)
    print("SUCCESS! CONNECTION STABILITY TESTS PASSED!")
    print("="*50)
    print("Connection resilience features working correctly:")
    print("- Concurrent connections: [OK]")
    print("- Connection persistence: [OK]")
    print("- Error handling: [OK]")
    print("="*50)

    return True

async def run_stability_tests():
    """Run all stability tests."""
    print("Starting Connection Stability Tests\n")

    success = await test_connection_stability()

    if success:
        print("\n[SUCCESS] All connection stability tests passed!")
        return True
    else:
        print(f"\n[FAILURE] Stability tests failed!")
        return False

if __name__ == "__main__":
    success = asyncio.run(run_stability_tests())
    if success:
        print("\n[OVERALL SUCCESS] Connection stability verified!")
    else:
        print("\n[OVERALL FAILURE] Connection stability issues found!")
        sys.exit(1)