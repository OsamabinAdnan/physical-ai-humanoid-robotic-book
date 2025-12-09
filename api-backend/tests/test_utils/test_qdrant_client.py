import pytest
from unittest.mock import Mock, patch
from app.utils.qdrant_client import QdrantManager, qdrant_manager


def test_qdrant_manager_initialization():
    """Test that Qdrant manager is initialized properly"""
    assert qdrant_manager is not None
    assert isinstance(qdrant_manager, QdrantManager)


def test_collection_name():
    """Test that collection name is set correctly"""
    assert qdrant_manager.collection_name == "textbook_content"


@patch('qdrant_client.QdrantClient')
def test_create_collection(mock_client):
    """Test creating a collection"""
    # Mock the client methods
    mock_client_instance = Mock()
    qdrant_manager.client = mock_client_instance

    # Mock the get_collection to raise an exception (simulating collection doesn't exist)
    mock_client_instance.get_collection.side_effect = Exception("Collection does not exist")

    # Call the method
    result = qdrant_manager.create_collection()

    # Verify the method was called correctly
    mock_client_instance.create_collection.assert_called_once()
    # The create_collection method in Qdrant doesn't return a value (returns None)
    assert result is None


@patch('qdrant_client.QdrantClient')
def test_store_text_chunk(mock_client):
    """Test storing a text chunk"""
    # Mock the client methods
    mock_client_instance = Mock()
    qdrant_manager.client = mock_client_instance
    mock_client_instance.upsert.return_value = True

    # Test data
    text = "Test text content"
    metadata = {"chapter": "1", "section": "1.1"}
    vector = [0.1, 0.2, 0.3]

    # Call the method
    result = qdrant_manager.store_text_chunk(text, metadata, vector)

    # Verify the method was called correctly
    mock_client_instance.upsert.assert_called_once()
    assert result is not None


@patch('qdrant_client.QdrantClient')
def test_search_similar(mock_client):
    """Test searching for similar content"""
    # Mock the client methods
    mock_client_instance = Mock()
    qdrant_manager.client = mock_client_instance

    # Mock search results
    mock_search_result = [
        Mock(payload={"text": "test", "metadata": {}}, score=0.9)
    ]
    mock_client_instance.search.return_value = mock_search_result

    # Test data
    query_vector = [0.1, 0.2, 0.3]

    # Call the method
    results = qdrant_manager.search_similar(query_vector, limit=5)

    # Verify the method was called correctly
    mock_client_instance.search.assert_called_once()
    assert len(results) == 1
    assert results[0]["text"] == "test"