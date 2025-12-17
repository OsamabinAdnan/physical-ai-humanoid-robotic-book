"""
Debug script to check environment variables and database connection.
"""
import os
from dotenv import load_dotenv

# Load environment variables
load_dotenv()

# Check if the environment variable is available
database_url = os.getenv("NEON_DATABASE_URL")
print(f"NEON_DATABASE_URL: {database_url}")
print(f"NEON_DATABASE_URL exists: {database_url is not None}")

if database_url:
    print("✓ Database URL is available")
else:
    print("✗ Database URL is not available")