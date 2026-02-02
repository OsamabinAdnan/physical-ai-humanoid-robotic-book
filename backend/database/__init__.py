"""
Database configuration module for Neon Serverless Postgres integration.
"""
import os
from dotenv import load_dotenv
from sqlalchemy.ext.asyncio import create_async_engine, AsyncSession
from sqlalchemy.orm import sessionmaker

# Load environment variables
load_dotenv()

# Global variables that will be initialized when needed
engine = None
AsyncDBSession = None

def init_db_engine():
    """
    Initialize the database engine with proper asyncpg driver.
    This function should be called when the database connection is needed.
    """
    global engine, AsyncDBSession

    if engine is not None:
        return  # Already initialized

    # Get database URL from environment
    database_url = os.getenv("NEON_DATABASE_URL")

    if not database_url:
        print("ERROR: NEON_DATABASE_URL environment variable not set")
        return

    print(f"Using database URL: {database_url[:50]}...")  # Just print first 50 chars for security

    # Update the database URL to use asyncpg driver if it's not already configured
    if database_url and database_url.startswith("postgresql://"):
        # Replace 'postgresql://' with 'postgresql+asyncpg://' to use the async driver
        database_url = database_url.replace("postgresql://", "postgresql+asyncpg://", 1)

        # Remove psycopg2-specific parameters that asyncpg doesn't support
        # channel_binding is specific to psycopg2 and not supported by asyncpg
        if "channel_binding" in database_url:
            import re
            database_url = re.sub(r'&?channel_binding=[^&]*', '', database_url)
            # Clean up any double ampersands or trailing ampersands
            database_url = database_url.replace('&&', '&').rstrip('&')

        # Also remove sslmode from URL as SQLAlchemy async engine may have issues with it
        import re
        database_url = re.sub(r'&?sslmode=[^&]*', '', database_url)
        # Clean up any double ampersands or trailing ampersands
        database_url = database_url.replace('&&', '&').rstrip('&')

    print(f"Final database URL: {database_url[:50]}...")

    try:
        # Create async engine
        engine = create_async_engine(
            database_url,
            echo=False,  # Set to True for SQL query logging
            pool_size=5,
            max_overflow=10,
            pool_pre_ping=True,  # Verify connections before use
            pool_recycle=300,    # Recycle connections after 5 minutes
            # For Neon, we need to handle SSL properly
            connect_args={
                "server_settings": {
                    "application_name": "RAG-Chatbot-App",
                }
            }
        )

        # Create async session maker
        AsyncDBSession = sessionmaker(
            engine,
            class_=AsyncSession,
            expire_on_commit=False
        )

        print("Database engine initialized successfully!")
    except Exception as e:
        print(f"ERROR creating database engine: {e}")
        engine = None
        AsyncDBSession = None

async def get_db_session():
    """
    Dependency function to get database session.
    """
    # Initialize the engine if not already done
    init_db_engine()

    async with AsyncDBSession() as session:
        yield session