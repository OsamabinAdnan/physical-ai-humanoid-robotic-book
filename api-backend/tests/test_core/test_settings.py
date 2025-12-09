import pytest
from app.core.settings import settings


def test_settings_loaded():
    """Test that settings are loaded properly"""
    assert settings is not None
    assert hasattr(settings, 'DATABASE_URL')
    assert hasattr(settings, 'QDRANT_API_KEY')
    assert hasattr(settings, 'QDRANT_CLUSTER_ENDPOINT')
    assert hasattr(settings, 'GEMINI_API_KEY')
    assert hasattr(settings, 'EMBEDDING_MODEL')
    assert hasattr(settings, 'BETTER_AUTH_SECRET')
    assert hasattr(settings, 'BETTER_AUTH_URL')
    assert hasattr(settings, 'QDRANT_CLUSTER_ID')


def test_settings_values():
    """Test that settings have expected values"""
    # Check that embedding model has the expected default value
    assert settings.EMBEDDING_MODEL == "sentence-transformers/all-MiniLM-L6-v2"