"""
Integration tests for the authentication and personalization features.
"""
import pytest
import asyncio
from fastapi.testclient import TestClient
from unittest.mock import AsyncMock, patch
import uuid

from main import app


@pytest.fixture
def client():
    """Create a test client for the FastAPI app."""
    return TestClient(app)


@pytest.mark.asyncio
async def test_auth_and_personalization_flow(client):
    """Test the complete flow: register -> login -> personalize -> logout."""

    # Step 1: Register a new user
    register_data = {
        "email": "test@example.com",
        "password": "SecurePassword123!",
        "name": "Test User",
        "software_background": "beginner",
        "hardware_background": "intermediate"
    }

    response = client.post("/api/auth/register", json=register_data)
    assert response.status_code == 200

    register_response = response.json()
    assert register_response["success"] is True
    assert "session_token" in register_response
    assert register_response["message"] == "User registered successfully"

    token = register_response["session_token"]
    user_id = register_response["user_id"]

    # Step 2: Login with the same credentials (should work)
    login_data = {
        "email": "test@example.com",
        "password": "SecurePassword123!"
    }

    response = client.post("/api/auth/login", json=login_data)
    assert response.status_code == 200

    login_response = response.json()
    assert login_response["success"] is True
    assert "session_token" in login_response
    assert login_response["message"] == "Login successful"

    token = login_response["session_token"]

    # Step 3: Get current user info
    response = client.get("/api/auth/me", headers={"Authorization": f"Bearer {token}"})
    assert response.status_code == 200

    user_info = response.json()
    assert user_info["email"] == "test@example.com"
    assert user_info["software_background"] == "beginner"
    assert user_info["hardware_background"] == "intermediate"

    # Step 4: Try to personalize content (mock the OpenRouter call)
    personalize_data = {
        "chapter_url": "https://example.com/docs/test-chapter",
        "chapter_content": "This is a test chapter about robotics and AI."
    }

    # Mock the OpenRouter API call to avoid actual API calls during testing
    with patch('httpx.AsyncClient.post') as mock_post:
        mock_response = AsyncMock()
        mock_response.status_code = 200
        mock_response.json.return_value = {
            "choices": [
                {
                    "message": {
                        "content": "This is personalized content for a beginner in software and intermediate in hardware."
                    }
                }
            ]
        }
        mock_post.return_value = mock_response

        response = client.post(
            "/personalize",
            json=personalize_data,
            headers={"Authorization": f"Bearer {token}"}
        )

        assert response.status_code == 200

        personalize_response = response.json()
        assert personalize_response["success"] is True
        assert "personalized_summary" in personalize_response
        assert "Content personalized successfully" in personalize_response["message"]

    # Step 5: Logout
    response = client.post("/api/auth/logout")
    assert response.status_code == 200

    logout_response = response.json()
    assert logout_response["success"] is True
    assert logout_response["message"] == "Logout successful"


def test_registration_validation(client):
    """Test registration with invalid data."""

    # Test with weak password
    weak_password_data = {
        "email": "weak@example.com",
        "password": "123",  # Too weak
        "name": "Test User",
        "software_background": "beginner",
        "hardware_background": "intermediate"
    }

    response = client.post("/api/auth/register", json=weak_password_data)
    assert response.status_code == 422  # Validation error

    # Test with invalid expertise level
    invalid_expertise_data = {
        "email": "invalid@example.com",
        "password": "SecurePassword123!",
        "name": "Test User",
        "software_background": "invalid_level",  # Invalid
        "hardware_background": "beginner"
    }

    response = client.post("/api/auth/register", json=invalid_expertise_data)
    assert response.status_code == 422  # Validation error


def test_login_with_nonexistent_user(client):
    """Test login with a non-existent user."""

    login_data = {
        "email": "nonexistent@example.com",
        "password": "somepassword"
    }

    response = client.post("/api/auth/login", json=login_data)
    assert response.status_code == 401  # Unauthorized


def test_unauthorized_access_to_protected_endpoints(client):
    """Test accessing protected endpoints without authentication."""

    # Try to access /api/auth/me without token
    response = client.get("/api/auth/me")
    assert response.status_code == 401  # Unauthorized

    # Try to access /personalize without token
    personalize_data = {
        "chapter_url": "https://example.com/docs/test-chapter",
        "chapter_content": "This is a test chapter."
    }
    response = client.post("/personalize", json=personalize_data)
    assert response.status_code == 401  # Unauthorized


if __name__ == "__main__":
    pytest.main([__file__])