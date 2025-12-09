import pytest
import numpy as np
from app.utils.embeddings import EmbeddingGenerator, embedding_generator


def test_embedding_generator_initialization():
    """Test that embedding generator is initialized properly"""
    assert embedding_generator is not None
    assert isinstance(embedding_generator, EmbeddingGenerator)


def test_generate_single_embedding():
    """Test generating a single embedding"""
    text = "This is a test sentence."
    embedding = embedding_generator.generate_embedding(text)

    # Check that embedding is a list of floats
    assert isinstance(embedding, list)
    assert len(embedding) > 0  # Should have some dimensions
    assert all(isinstance(val, float) for val in embedding)


def test_generate_multiple_embeddings():
    """Test generating multiple embeddings"""
    texts = ["First sentence.", "Second sentence.", "Third sentence."]
    embeddings = embedding_generator.generate_embeddings(texts)

    # Check that we have embeddings for each text
    assert len(embeddings) == len(texts)
    for embedding in embeddings:
        assert isinstance(embedding, list)
        assert len(embedding) > 0
        assert all(isinstance(val, float) for val in embedding)


def test_embedding_dimension_consistency():
    """Test that all embeddings have the same dimension"""
    texts = ["First sentence.", "Second sentence."]
    embeddings = embedding_generator.generate_embeddings(texts)

    # All embeddings should have the same length
    first_len = len(embeddings[0])
    for embedding in embeddings[1:]:
        assert len(embedding) == first_len