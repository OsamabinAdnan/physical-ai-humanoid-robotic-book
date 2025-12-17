"""
Test to verify that chat messages are persisted correctly during RAG execution flow.
"""
import asyncio
import sys
import os
from dotenv import load_dotenv

# Load environment variables first
load_dotenv()

# Add the backend directory to the path to import modules
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from sqlalchemy import select
from database import init_db_engine, AsyncDBSession
from database.models import ChatSession, ChatMessage
from database.services import DatabaseService


async def test_chat_persistence():
    """Test that chat messages are stored and retrieved correctly."""
    # Import the database module to access initialized components
    from database import init_db_engine, AsyncDBSession, engine

    # Initialize the database engine
    init_db_engine()

    # Check if engine was properly initialized
    if engine is None:
        print("[ERROR] Database engine failed to initialize")
        return False

    # Create a test user ID
    import uuid
    user_id = uuid.uuid4()

    async with AsyncDBSession() as session:
        db_service = DatabaseService(session)

        # Test 1: Create a chat session
        print("Testing chat session creation...")
        chat_session = await db_service.chat_session.create_session(
            user_id=user_id,
            title="Test session for validation"
        )
        print(f"[OK] Created session: {chat_session.id}")

        # Test 2: Add a user message
        print("Testing user message storage...")
        user_message = await db_service.chat_message.add_message(
            session_id=chat_session.id,
            user_id=user_id,
            role="user",
            content="Hello, this is a test message."
        )
        print(f"[OK] Stored user message: {user_message.id}")

        # Test 3: Add an assistant message
        print("Testing assistant message storage...")
        assistant_message = await db_service.chat_message.add_message(
            session_id=chat_session.id,
            user_id=user_id,
            role="assistant",
            content="Hello, this is a test response."
        )
        print(f"[OK] Stored assistant message: {assistant_message.id}")

        # Test 4: Retrieve messages from the session
        print("Testing message retrieval...")
        retrieved_messages = await db_service.chat_message.get_messages_by_session(chat_session.id, user_id)
        print(f"[OK] Retrieved {len(retrieved_messages)} messages from session")

        # Verify the messages
        if len(retrieved_messages) == 2:
            print("[OK] Correct number of messages retrieved")
        else:
            print(f"[ERROR] Expected 2 messages, got {len(retrieved_messages)}")
            return False

        # Check content
        message_contents = [msg.content for msg in retrieved_messages]
        expected_contents = ["Hello, this is a test message.", "Hello, this is a test response."]

        if all(content in message_contents for content in expected_contents):
            print("[OK] Message contents match expected values")
        else:
            print(f"[ERROR] Message contents don't match. Got: {message_contents}")
            return False

        # Test 5: Verify session information
        retrieved_session = await db_service.chat_session.get_session_by_id(chat_session.id, user_id)
        if retrieved_session and retrieved_session.title == "Test session for validation":
            print("[OK] Session retrieved correctly")
        else:
            print("[ERROR] Session retrieval failed")
            return False

        # Clean up - delete the test session and its messages
        print("Cleaning up test data...")
        await db_service.chat_session.delete_session(chat_session.id, user_id)
        print("[OK] Test data cleaned up")

    print("\n[SUCCESS] All chat persistence tests passed!")
    return True


if __name__ == "__main__":
    result = asyncio.run(test_chat_persistence())
    if result:
        print("\n[SUCCESS] Chat persistence validation successful!")
    else:
        print("\n[FAILURE] Chat persistence validation failed!")