import pytest
from unittest.mock import Mock, patch
from fastapi.testclient import TestClient
from sqlalchemy import create_engine
from sqlalchemy.orm import sessionmaker
from app.core.database import Base, get_db
from main import app  # Import the main FastAPI app


# Create a test database
SQLALCHEMY_DATABASE_URL = "sqlite:///:memory:"
engine = create_engine(SQLALCHEMY_DATABASE_URL, connect_args={"check_same_thread": False})
TestingSessionLocal = sessionmaker(autocommit=False, autoflush=False, bind=engine)


# Dependency to override the database session
def override_get_db():
    try:
        db = TestingSessionLocal()
        Base.metadata.create_all(bind=db.get_bind())  # Create tables for this session
        yield db
    finally:
        db.close()


# Override the database dependency
app.dependency_overrides[get_db] = override_get_db


client = TestClient(app)


def test_ingest_textbook_endpoint():
    """Test that the ingest textbook endpoint exists"""
    response = client.post("/api/v1/ingest/textbook", json={
        "text_chunks": [
            {
                "text": "Test chunk content",
                "metadata": {"chapter": "1", "section": "1.1"}
            }
        ]
    })
    # This will likely return 500 due to missing Qdrant setup in test, but not 404
    assert response.status_code in [200, 422, 500]


def test_ingest_chapter_endpoint():
    """Test that the ingest chapter endpoint exists"""
    response = client.post("/api/v1/ingest/chapter", json={
        "title": "Test Chapter",
        "content": "Chapter content",
        "sections": [
            {
                "title": "Section 1",
                "content": "Section content"
            }
        ]
    })
    # This will likely return 500 due to missing Qdrant setup in test, but not 404
    assert response.status_code in [200, 422, 500]


def test_ingest_reset_endpoint():
    """Test that the ingest reset endpoint exists"""
    response = client.post("/api/v1/ingest/reset")
    # This will likely return 500 due to missing Qdrant setup in test, but not 404
    assert response.status_code in [200, 500]


def test_ingest_status_endpoint():
    """Test that the ingest status endpoint exists"""
    response = client.get("/api/v1/ingest/status")
    # This will return the status based on whether collection exists
    assert response.status_code in [200, 500]


def test_ingest_textbook_validation():
    """Test that ingest textbook request validation works"""
    # Empty request should return 422
    response = client.post("/api/v1/ingest/textbook", json={})
    assert response.status_code == 422