# Better-Auth Integration Implementation Plan

## Overview
This plan outlines the step-by-step approach to integrate Better-Auth into the Physical AI & Humanoid Robotics textbook project, following the established specification and requirements.

## Phase 1: Database Setup

### Task 1.1: Update SQLAlchemy Models
- **File**: `backend/database/models.py`
- **Action**: Update User model to include authentication fields that match the existing database schema
- **Fields to add**: `hashed_password`, `email_verified`, `software_knowledge`, `hardware_knowledge`
- **Dependencies**: None
- **Acceptance Criteria**: Model reflects actual database schema with authentication fields

### Task 1.2: Create Pydantic Schemas for Authentication
- **File**: `backend/database/schemas.py`
- **Action**: Add authentication-related schemas for user registration and login
- **Schemas to create**: `UserCreate`, `UserRegister`, `UserLogin`, `UserResponse`, `Token`, `UserProfile`
- **Dependencies**: Task 1.1
- **Acceptance Criteria**: Schemas support all required authentication data including expertise fields

## Phase 2: Backend Setup

### Task 2.1: Create Authentication Utilities
- **File**: `backend/auth/utils.py`
- **Action**: Implement password hashing and JWT utilities
- **Functions to implement**: `hash_password()`, `verify_password()`, `create_access_token()`, `verify_token()`, `get_current_user()`
- **Dependencies**: None
- **Acceptance Criteria**: Secure password handling and JWT token functions work correctly

### Task 2.2: Create Authentication Service
- **File**: `backend/auth/service.py`
- **Action**: Implement user registration, login, and session management using patterns similar to Better-Auth
- **Functions to implement**: `register_user()`, `authenticate_user()`, `get_user_by_email()`, `update_user_profile()`
- **Dependencies**: Task 2.1, Task 1.1, Task 1.2
- **Acceptance Criteria**: Authentication service handles all user authentication operations

### Task 2.3: Create Better-Auth Compatible Configuration
- **File**: `backend/auth/auth.py`
- **Action**: Create a FastAPI-compatible authentication system that follows Better-Auth patterns
- **Components to create**: `Auth` class with email/password functionality, session management, JWT generation
- **Dependencies**: Task 2.1, Task 2.2
- **Acceptance Criteria**: Authentication system follows Better-Auth API patterns and can be used with React components

### Task 2.4: Add Authentication Endpoints
- **File**: `backend/main.py`
- **Action**: Add `/api/auth/register`, `/api/auth/login`, `/api/auth/logout`, `/api/auth/me` endpoints
- **Additional endpoints**: `/api/auth/register-with-background` for collecting expertise during registration
- **Dependencies**: Task 2.3
- **Acceptance Criteria**: Authentication endpoints are available and functional, with expertise collection

### Task 2.5: Protect Existing Endpoints
- **File**: `backend/main.py`
- **Action**: Add authentication middleware to existing `/chat` and chat history endpoints
- **Dependencies**: Task 2.3
- **Acceptance Criteria**: Protected endpoints require valid authentication, user data is properly associated

## Phase 3: Frontend Components

### Task 3.1: Create Authentication Client
- **File**: `book-frontend/src/lib/auth-client.ts`
- **Action**: Create a client-side authentication API client that follows Better-Auth patterns
- **Functions to implement**: `signIn()`, `signUp()`, `signOut()`, `getSession()`, `updateSession()`
- **Dependencies**: None
- **Acceptance Criteria**: Client can communicate with backend authentication endpoints

### Task 3.2: Create Authentication Context
- **File**: `book-frontend/src/contexts/AuthContext.tsx`
- **Action**: Implement React context for authentication state management
- **Components to create**: `AuthContext`, `AuthProvider`, `useAuth` hook
- **Dependencies**: Task 3.1
- **Acceptance Criteria**: Authentication state can be shared across components

### Task 3.3: Create Signup Form with Background Questions
- **File**: `book-frontend/src/components/Auth/SignupForm.tsx`
- **Action**: Create a signup form that collects email, password, and expertise background
- **Features to implement**:
  - Email and password fields
  - Software background questions (programming experience, familiar languages)
  - Hardware background questions (robotics experience, available hardware)
  - Form validation
- **Dependencies**: Task 3.1, Task 3.2
- **Acceptance Criteria**: Form collects all required information and sends to backend

### Task 3.4: Create Signin Form
- **File**: `book-frontend/src/components/Auth/SigninForm.tsx`
- **Action**: Create a standard login form
- **Features to implement**: Email and password fields, form validation
- **Dependencies**: Task 3.1, Task 3.2
- **Acceptance Criteria**: Form authenticates users and updates context

### Task 3.5: Create User Menu Component
- **File**: `book-frontend/src/components/Auth/UserMenu.tsx`
- **Action**: Create a component to display user information and logout option
- **Features to implement**: User name/email display, logout button, user profile access
- **Dependencies**: Task 3.1, Task 3.2
- **Acceptance Criteria**: Component displays user info when authenticated

### Task 3.6: Create Authentication Pages
- **Files**:
  - `book-frontend/src/pages/auth/signin.tsx`
  - `book-frontend/src/pages/auth/signup.tsx`
- **Action**: Create full-page authentication forms
- **Dependencies**: Task 3.3, Task 3.4, Task 3.5
- **Acceptance Criteria**: Complete authentication pages are available

## Phase 4: Integration

### Task 4.1: Update Chatbot API Service
- **File**: `book-frontend/src/components/Chatbot/api.ts`
- **Action**: Update API calls to include authentication tokens
- **Changes to implement**: Add authorization headers to all API calls, handle token expiration
- **Dependencies**: Task 3.1
- **Acceptance Criteria**: API calls include proper authentication headers

### Task 4.2: Update Chatbot Component for Auth
- **File**: `book-frontend/src/components/Chatbot/Chatbot.tsx`
- **Action**: Integrate authentication state with chatbot functionality
- **Changes to implement**:
  - Check authentication before allowing chat
  - Display login prompt for unauthenticated users
  - Associate chat sessions with authenticated user
- **Dependencies**: Task 3.2, Task 4.1
- **Acceptance Criteria**: Chatbot works seamlessly with authenticated users

### Task 4.3: Add Protected Route Components
- **File**: `book-frontend/src/components/Auth/ProtectedRoute.tsx`
- **Action**: Create components to protect routes/pages based on authentication status
- **Components to create**: `RequireAuth`, `RedirectIfAuthenticated`
- **Dependencies**: Task 3.2
- **Acceptance Criteria**: Components properly redirect based on auth status

### Task 4.4: Update Navigation with Auth State
- **File**: `book-frontend/src/theme/Navbar/index.tsx` (or similar Docusaurus navbar)
- **Action**: Update navigation to show/hide items based on authentication state
- **Changes to implement**: Show login/signup when not authenticated, show user menu when authenticated
- **Dependencies**: Task 3.2, Task 3.5
- **Acceptance Criteria**: Navigation adapts based on user authentication status

## Phase 5: Testing

### Task 5.1: Create Backend Authentication Tests
- **File**: `backend/test/test_authentication.py`
- **Action**: Write tests for authentication endpoints and functionality
- **Tests to implement**:
  - User registration with expertise collection
  - User login and token generation
  - Protected endpoint access
  - Token validation and refresh
- **Dependencies**: All backend tasks (Phase 2)
- **Acceptance Criteria**: All authentication functionality is properly tested

### Task 5.2: Create Frontend Authentication Tests
- **File**: `book-frontend/src/components/Auth/__tests__/*`
- **Action**: Write tests for authentication components
- **Tests to implement**:
  - Signup form with background questions
  - Signin form
  - Auth context functionality
  - Protected route components
- **Dependencies**: All frontend tasks (Phase 3)
- **Acceptance Criteria**: All authentication components are properly tested

### Task 5.3: End-to-End Flow Testing
- **Action**: Test complete authentication flow from registration to protected content access
- **Scenarios to test**:
  - New user registration with background questions
  - Login and access to chatbot
  - Session management and persistence
  - Logout functionality
- **Dependencies**: All previous tasks
- **Acceptance Criteria**: Complete flow works seamlessly

## Implementation Order
1. Phase 1: Database Setup
2. Phase 2: Backend Setup
3. Phase 3: Frontend Components
4. Phase 4: Integration
5. Phase 5: Testing

## Risk Mitigation
- Maintain backward compatibility during development using feature flags if needed
- Test authentication features in isolation before integration
- Preserve existing RAG chatbot functionality during implementation
- Implement proper error handling for authentication failures
- Ensure database migrations are properly handled

## Integration Points with Existing Code
- **Database**: Update existing User model to include auth fields
- **API Endpoints**: Add new auth endpoints alongside existing chat endpoints
- **Chatbot**: Integrate auth state with existing chat functionality
- **Frontend**: Add auth components to existing Docusaurus structure
- **Environment**: Add auth-related environment variables to existing .env files