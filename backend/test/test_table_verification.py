"""
Test to verify that all tables were created correctly in the Neon database.
"""
import asyncio
import sys
import os
from dotenv import load_dotenv

# Load environment variables first
load_dotenv()

# Add the backend directory to the path to import modules
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from sqlalchemy import inspect, text
from database import init_db_engine, engine


async def verify_tables():
    """Verify that all expected tables exist in the database."""
    # Initialize the database engine
    init_db_engine()

    # Now the engine should be initialized
    if engine is None:
        print("[ERROR] Database engine failed to initialize")
        return False

    async with engine.connect() as conn:
        # Get the inspector to check for tables
        inspector = inspect(engine)
        tables = inspector.get_table_names()

        print(f"Tables in database: {tables}")

        # Expected tables
        expected_tables = {'users', 'chat_sessions', 'chat_messages', 'documents'}
        existing_tables = set(tables)

        print(f"Expected tables: {expected_tables}")
        print(f"Existing tables: {existing_tables}")

        missing_tables = expected_tables - existing_tables
        extra_tables = existing_tables - expected_tables

        if missing_tables:
            print(f"[ERROR] Missing tables: {missing_tables}")
            return False

        if extra_tables:
            print(f"[INFO] Extra tables (not expected): {extra_tables}")

        # Verify each table structure
        for table_name in expected_tables:
            print(f"\nVerifying table: {table_name}")
            columns = inspector.get_columns(table_name)
            print(f"  Columns: {[col['name'] for col in columns]}")

            # Print detailed column info for verification
            for col in columns:
                print(f"    {col['name']}: {col['type']} (nullable: {col['nullable']})")

        print(f"\n[OK] All expected tables exist: {expected_tables.issubset(existing_tables)}")
        return True


if __name__ == "__main__":
    result = asyncio.run(verify_tables())
    if result:
        print("\n[SUCCESS] All tables verified successfully!")
    else:
        print("\n[FAILURE] Table verification failed!")