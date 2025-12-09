import pytest
from sqlalchemy import create_engine
from sqlalchemy.orm import sessionmaker
from app.core.database import Base, get_db
from app.models.user import User
from app.models.chat import ChatSession, ChatMessage
from main import app
from fastapi.testclient import TestClient


# Create test database
SQLALCHEMY_DATABASE_URL = "sqlite:///./test.db"
engine = create_engine(SQLALCHEMY_DATABASE_URL, connect_args={"check_same_thread": False})
TestingSessionLocal = sessionmaker(autocommit=False, autoflush=False, bind=engine)


@pytest.fixture(scope="module")
def test_db():
    """Create test database"""
    Base.metadata.create_all(bind=engine)
    yield engine
    Base.metadata.drop_all(bind=engine)


@pytest.fixture(scope="function")
def db_session(test_db):
    """Create a new database session for each test"""
    connection = test_db.connect()
    transaction = connection.begin()
    session = TestingSessionLocal(bind=connection)

    yield session

    session.close()
    transaction.rollback()
    connection.close()


@pytest.fixture(scope="module")
def client():
    """Create a test client for the FastAPI app"""
    # Create the test database tables
    Base.metadata.create_all(bind=engine)

    with TestClient(app) as c:
        yield c

    # Clean up the test database tables
    Base.metadata.drop_all(bind=engine)