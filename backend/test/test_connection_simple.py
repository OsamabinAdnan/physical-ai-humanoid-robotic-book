"""
Simple test to verify asyncpg connection to Neon.
"""
import asyncio
import asyncpg
import os
from dotenv import load_dotenv

# Load environment variables
load_dotenv()

# Get the original database URL
database_url = os.getenv("NEON_DATABASE_URL")
print(f"Original URL: {database_url}")

# Convert to asyncpg format
if database_url and database_url.startswith("postgresql://"):
    # Replace with asyncpg format
    asyncpg_url = database_url.replace("postgresql://", "postgresql+asyncpg://", 1)
    print(f"Asyncpg URL: {asyncpg_url}")

    # Try to connect directly with asyncpg
    try:
        # Parse the URL manually to extract components
        from urllib.parse import urlparse
        parsed = urlparse(database_url)

        # Create connection parameters
        conn_params = {
            'host': parsed.hostname,
            'port': parsed.port,
            'user': parsed.username,
            'password': parsed.password,
            'database': parsed.path[1:],  # Remove leading '/'
            'ssl': 'require'  # Use SSL as required by Neon
        }

        print(f"Connection params: host={conn_params['host']}, port={conn_params['port']}, user={conn_params['user']}, db={conn_params['database']}")

        async def test_connection():
            conn = await asyncpg.connect(**conn_params)
            result = await conn.fetchval('SELECT 1')
            print(f"Connection successful! Result: {result}")
            await conn.close()

        asyncio.run(test_connection())
        print("Direct asyncpg connection successful!")

    except Exception as e:
        print(f"Direct asyncpg connection failed: {e}")

        # Try with the modified URL approach
        try:
            print(f"Trying with modified URL approach...")
            # Remove problematic parameters
            import re
            clean_url = re.sub(r'&?channel_binding=[^&]*', '', asyncpg_url)
            clean_url = clean_url.replace('&&', '&').rstrip('&')
            print(f"Clean URL: {clean_url}")

            async def test_with_url():
                conn = await asyncpg.connect(clean_url)
                result = await conn.fetchval('SELECT 1')
                print(f"URL-based connection successful! Result: {result}")
                await conn.close()

            asyncio.run(test_with_url())
            print("URL-based asyncpg connection successful!")

        except Exception as e2:
            print(f"URL-based asyncpg connection also failed: {e2}")
else:
    print("No database URL found")