# Better-Auth Integration Tasks

## Phase 1: Setup
- [ ] T001 Create auth directory structure in backend/auth/
- [ ] T002 Update requirements.txt to include authentication dependencies

## Phase 2: Foundational
- [ ] T003 Update User model with authentication fields in backend/database/models.py
- [ ] T004 Create authentication-related Pydantic schemas in backend/database/schemas.py
- [ ] T005 Create authentication utilities in backend/auth/utils.py

## Phase 3: [US1] User Registration with Expertise Collection
- [ ] T006 Create authentication service in backend/auth/service.py
- [ ] T007 Create Better-Auth compatible auth configuration in backend/auth/auth.py
- [ ] T008 Add registration endpoint with background questions in backend/main.py
- [ ] T009 Create signup form component in book-frontend/src/components/Auth/SignupForm.tsx
- [ ] T010 Create authentication client in book-frontend/src/lib/auth-client.ts
- [ ] T011 Create authentication context in book-frontend/src/contexts/AuthContext.tsx

## Phase 4: [US2] User Authentication and Session Management
- [ ] T012 Add login/logout endpoints in backend/main.py
- [ ] T013 Create signin form component in book-frontend/src/components/Auth/SigninForm.tsx
- [ ] T014 Create user menu component in book-frontend/src/components/Auth/UserMenu.tsx
- [ ] T015 Update chatbot API to include auth headers in book-frontend/src/components/Chatbot/api.ts

## Phase 5: [US3] Protected Chatbot Access
- [ ] T016 Add authentication middleware to chat endpoints in backend/main.py
- [ ] T017 Update chatbot component for auth integration in book-frontend/src/components/Chatbot/Chatbot.tsx
  - Ensure chatbot prompts user to sign up/sign in before allowing chat
  - Associate authenticated user with chat sessions and messages
- [ ] T017a [P] Add user-specific chat session creation in backend/main.py
  - Ensure new chat sessions are properly linked to authenticated user ID
- [ ] T017b [P] Add user-specific message storage in backend/main.py
  - Ensure chat messages are properly linked to authenticated user ID
- [ ] T018 Update chat history functionality to be user-specific in book-frontend/src/components/Chatbot/Chatbot.tsx
  - Ensure chat history shows only authenticated user's conversations
- [ ] T018a [P] Add user-specific chat history retrieval in book-frontend/src/components/Chatbot/api.ts
  - Ensure API calls fetch only the current user's chat history
- [ ] T019 Update navigation to show auth state in book-frontend/src/theme/Navbar/index.tsx

## Phase 6: [US4] User Profile and Session Management
- [ ] T020 Add user profile endpoint in backend/main.py
- [ ] T021 Create authentication pages in book-frontend/src/pages/auth/
- [ ] T022 Test complete authentication flow

## Phase 7: Polish & Cross-Cutting Concerns
- [ ] T023 Add error handling for authentication failures
- [ ] T024 Add loading states for auth operations
- [ ] T025 Update environment variables for auth configuration
- [ ] T026 Document authentication API endpoints
- [ ] T027 Write comprehensive tests for authentication functionality