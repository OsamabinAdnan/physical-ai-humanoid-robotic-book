"""
Test with real dummy data to verify complete data flow:
- Create user with real data
- Create chat session
- Store real chat messages
- Store document metadata
- Verify retrieval works correctly
"""
import asyncio
import uuid
import sys
import os
from dotenv import load_dotenv

# Add backend to path to import modules
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

# Load environment variables
load_dotenv()

async def test_real_data_flow():
    print("Testing real data flow with dummy data...\n")

    # Initialize database
    from database import init_db_engine
    print("1. Initializing database...")
    init_db_engine()
    print("   [OK] Database initialized\n")

    # Import services
    from database import AsyncDBSession
    from database.services import DatabaseService
    print("2. Importing services...")
    print("   [OK] Services imported\n")

    # Test with actual data flow
    async with AsyncDBSession() as session:
        db_service = DatabaseService(session)

        print("3. Testing complete data flow...")

        # Create a real user with dummy data
        print("   Creating user with dummy data...")
        user = await db_service.user.create_user(
            email="test.user@example.com",
            name="Test User Name"
        )
        print(f"   [OK] User created: {user.email}, ID: {user.id}")

        # Create a chat session with real data
        print("   Creating chat session...")
        chat_session = await db_service.chat_session.create_session(
            user_id=user.id,
            title="Test Conversation about AI Robotics"
        )
        print(f"   [OK] Chat session created: '{chat_session.title}', ID: {chat_session.id}")

        # Add real user message
        print("   Adding user message...")
        user_msg = await db_service.chat_message.add_message(
            session_id=chat_session.id,
            user_id=user.id,
            role="user",
            content="How does reinforcement learning apply to humanoid robot locomotion?"
        )
        print(f"   [OK] User message stored: '{user_msg.content[:50]}...'")

        # Add real assistant response
        print("   Adding assistant response...")
        assistant_msg = await db_service.chat_message.add_message(
            session_id=chat_session.id,
            user_id=user.id,  # For assistant responses, using user ID as placeholder
            role="assistant",
            content="Reinforcement learning can be applied to humanoid robot locomotion through..."
        )
        print(f"   [OK] Assistant response stored: '{assistant_msg.content[:50]}...'")

        # Add document metadata
        print("   Adding document metadata...")
        document = await db_service.document.create_document(
            source_url="https://example.com/humanoid-locomotion-paper.pdf",
            original_filename="humanoid-locomotion-paper.pdf",
            checksum="abc123def456ghi789",
            ingestion_status="completed"
        )
        print(f"   [OK] Document metadata stored: '{document.original_filename}'")

        # Verify retrieval of all data
        print("\n4. Verifying data retrieval...")

        # Get user by ID
        retrieved_user = await db_service.user.get_user_by_id(user.id)
        if retrieved_user and retrieved_user.email == "test.user@example.com":
            print("   [OK] User retrieved correctly")
        else:
            print("   [ERROR] User retrieval failed")
            return False

        # Get chat session
        retrieved_session = await db_service.chat_session.get_session_by_id(chat_session.id, user.id)
        if retrieved_session and retrieved_session.title == "Test Conversation about AI Robotics":
            print("   [OK] Chat session retrieved correctly")
        else:
            print("   [ERROR] Chat session retrieval failed")
            return False

        # Get all messages from session
        messages = await db_service.chat_message.get_messages_by_session(chat_session.id, user.id)
        if len(messages) == 2:
            print(f"   [OK] {len(messages)} messages retrieved from session")

            # Check message contents
            message_contents = [msg.content for msg in messages]
            expected_user_content = "How does reinforcement learning apply to humanoid robot locomotion?"
            expected_assistant_content = "Reinforcement learning can be applied to humanoid robot locomotion through..."

            if expected_user_content in message_contents and expected_assistant_content in message_contents:
                print("   [OK] Message contents match expected values")
            else:
                print(f"   [ERROR] Message contents don't match. Got: {message_contents}")
                return False
        else:
            print(f"   [ERROR] Expected 2 messages, got {len(messages)}")
            return False

        # Get document by ID
        retrieved_doc = await db_service.document.get_document_by_id(document.id)
        if retrieved_doc and retrieved_doc.original_filename == "humanoid-locomotion-paper.pdf":
            print("   [OK] Document retrieved correctly")
        else:
            print("   [ERROR] Document retrieval failed")
            return False

        # Test data isolation - try to access with wrong user ID
        fake_user_id = uuid.uuid4()
        isolated_messages = await db_service.chat_message.get_messages_by_session(chat_session.id, fake_user_id)
        if len(isolated_messages) == 0:
            print("   [OK] Data isolation working - other users cannot access session")
        else:
            print(f"   [ERROR] Data isolation failed - got {len(isolated_messages)} messages for wrong user")
            return False

        # Clean up test data
        print("\n5. Cleaning up test data...")
        await db_service.chat_session.delete_session(chat_session.id, user.id)
        await db_service.user.delete_user(user.id)
        await db_service.document.delete_document(document.id)
        print("   [OK] Test data cleaned up")

    print("\n" + "="*60)
    print("SUCCESS! REAL DATA FLOW TEST PASSED!")
    print("="*60)
    print("Complete data flow is working correctly:")
    print("- User creation and retrieval: [OK]")
    print("- Chat session creation: [OK]")
    print("- Message storage and retrieval: [OK]")
    print("- Document metadata storage: [OK]")
    print("- Data isolation: [OK]")
    print("- Cleanup: [OK]")
    print("="*60)

    return True

if __name__ == "__main__":
    import sys
    print("Starting Real Data Flow Test\n")

    success = asyncio.run(test_real_data_flow())

    if success:
        print("\n[SUCCESS] FINAL RESULT: Real data flow test PASSED!")
        print("Neon database is fully functional with real data!")
    else:
        print("\n[ERROR] FINAL RESULT: Real data flow test FAILED!")
        sys.exit(1)