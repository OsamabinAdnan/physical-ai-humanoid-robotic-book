from sqlalchemy import create_engine
from sqlalchemy.ext.declarative import declarative_base
from sqlalchemy.orm import sessionmaker
from app.core.settings import settings

# Create the database engine
engine = create_engine(
    settings.DATABASE_URL,
    pool_pre_ping=True,  # Verify connections before using them
    pool_recycle=300,    # Recycle connections after 5 minutes
    echo=False,          # Set to True for SQL debugging
)

# Create a configured "Session" class
SessionLocal = sessionmaker(autocommit=False, autoflush=False, bind=engine)

# Create a Base class for declarative models
Base = declarative_base()


def get_db():
    """
    Dependency function that provides database sessions for FastAPI endpoints.
    """
    db = SessionLocal()
    try:
        yield db
    finally:
        db.close()