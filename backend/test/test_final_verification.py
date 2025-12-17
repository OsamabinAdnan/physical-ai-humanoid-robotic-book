"""
Final verification test for the Neon database integration.
"""
import asyncio
import sys
import os
from dotenv import load_dotenv

# Load environment variables first
load_dotenv()

# Add the backend directory to the path to import modules
sys.path.insert(0, os.path.dirname(os.path.dirname(__file__)))

async def final_verification_test():
    """Perform final verification that the Neon integration is working."""
    print("Starting final verification test...\n")

    # Import and initialize database engine first
    from database import init_db_engine
    print("1. Initializing database engine...")
    init_db_engine()
    print("   [OK] Database engine initialized\n")

    # Now import the session after initialization
    from database import AsyncDBSession
    from database.services import DatabaseService

    print("2. Testing database connectivity...")
    async with AsyncDBSession() as session:
        # Test basic database connectivity
        from sqlalchemy import text
        result = await session.execute(text("SELECT 1"))
        row = result.fetchone()

        if row and row[0] == 1:
            print("   [OK] Database connectivity test passed")
        else:
            print("   [ERROR] Database connectivity test failed")
            return False
    print()

    print("3. Testing basic CRUD operations...")
    async with AsyncDBSession() as session:
        db_service = DatabaseService(session)

        # Create a test user
        user = await db_service.user.create_user(
            email="final.test@example.com",
            name="Final Test User"
        )
        print(f"   [OK] User created: {user.id}")

        # Create a chat session
        session_obj = await db_service.chat_session.create_session(
            user_id=user.id,
            title="Final Verification Test"
        )
        print(f"   [OK] Chat session created: {session_obj.id}")

        # Add a message
        message = await db_service.chat_message.add_message(
            session_id=session_obj.id,
            user_id=user.id,
            role="user",
            content="This is a final verification test message."
        )
        print(f"   [OK] Message created: {message.id}")

        # Retrieve the message
        messages = await db_service.chat_message.get_messages_by_session(session_obj.id, user.id)
        if len(messages) == 1 and messages[0].content == "This is a final verification test message.":
            print("   [OK] Message retrieved successfully")
        else:
            print(f"   [ERROR] Message retrieval failed: {len(messages)} messages retrieved")
            return False

        # Clean up
        await db_service.chat_session.delete_session(session_obj.id, user.id)
        await db_service.user.delete_user(user.id)
        print("   [OK] Test data cleaned up")

    print("\n" + "="*60)
    print("SUCCESS! FINAL VERIFICATION TEST PASSED!")
    print("="*60)
    print("Neon database integration is fully functional:")
    print("- Database engine initialization: [OK]")
    print("- Database connectivity: [OK]")
    print("- CRUD operations: [OK]")
    print("- Data retrieval: [OK]")
    print("- Cleanup: [OK]")
    print("="*60)

    return True

if __name__ == "__main__":
    success = asyncio.run(final_verification_test())

    if success:
        print("\n[SUCCESS] Neon database integration verification completed successfully!")
        print("The RAG chatbot now has persistent storage with Neon Serverless Postgres!")
    else:
        print("\n[FAILURE] Final verification test failed!")
        sys.exit(1)