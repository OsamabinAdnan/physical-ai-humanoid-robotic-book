# Better Auth Integration and Content Personalization Implementation Plan

## Executive Summary

This plan outlines the implementation of user authentication and content personalization for the Docusaurus book and FastAPI backend. The feature will integrate Better-Auth for user management and provide personalized learning experiences based on user expertise levels.

## Technical Context

- **Frontend**: Docusaurus-based textbook application
- **Backend**: FastAPI application with Neon Postgres database
- **Authentication**: Better-Auth integration
- **Personalization Engine**: OpenRouter LLM for content adaptation
- **Database**: Neon Postgres for user profiles and personalized content
- **Development URLs**:
  - Local: `http://localhost:3000/physical-ai-humanoid-robotic-book/`
  - Production: `https://osamabinadnan.github.io/physical-ai-humanoid-robotic-book/`

## Architecture Overview

The system will consist of:
1. Backend authentication service using Better-Auth
2. Frontend authentication UI components
3. Content personalization engine
4. Database schema updates for user profiles

## Phase 0: Research & Analysis

### Research Tasks

1. **Better-Auth Integration Research**
   - Decision: Integrate Better-Auth with FastAPI backend
   - Rationale: Better-Auth provides robust authentication with session management and is compatible with FastAPI
   - Alternatives considered: Custom JWT implementation, Auth0, Firebase Auth

2. **Database Schema Research**
   - Decision: Extend existing Neon Postgres schema to include user background fields
   - Rationale: Building on existing database infrastructure reduces complexity
   - Alternatives considered: Separate user database, NoSQL solutions

3. **Frontend Component Research**
   - Decision: Create React components for authentication flows within Docusaurus
   - Rationale: Docusaurus supports custom React components and this maintains consistency
   - Alternatives considered: External authentication pages, iframe integration

4. **Personalization Algorithm Research**
   - Decision: Use OpenRouter LLM to generate personalized content summaries
   - Rationale: LLMs can effectively adapt content based on user expertise levels
   - Alternatives considered: Rule-based systems, pre-generated content levels

## Phase 1: Design & Contracts

### Data Model

#### User Profile Entity
- `id`: UUID (Primary Key)
- `email`: String (Unique, Required)
- `password_hash`: String (Required, bcrypt hashed)
- `name`: String (Optional)
- `created_at`: DateTime
- `updated_at`: DateTime
- `software_background`: Enum (beginner, intermediate, expert)
- `hardware_background`: Enum (beginner, intermediate, expert)
- `auth_token`: String (Optional, for session management)

#### Personalized Content Entity
- `id`: UUID (Primary Key)
- `user_id`: UUID (Foreign Key to users)
- `chapter_id`: String (Reference to chapter)
- `original_content_hash`: String (To detect changes)
- `personalized_summary`: Text (Generated personalized content)
- `personalization_level`: Enum (beginner, intermediate, expert)
- `created_at`: DateTime
- `updated_at`: DateTime

### API Contracts

#### Authentication Endpoints
```
POST /api/auth/register
- Request: {email, password, software_background, hardware_background}
- Response: {user_id, session_token, success}

POST /api/auth/login
- Request: {email, password}
- Response: {user_id, session_token, success}

POST /api/auth/logout
- Request: {session_token}
- Response: {success}

GET /api/auth/me
- Request: {session_token}
- Response: {user, is_authenticated}
```

#### Personalization Endpoints
```
POST /personalize
- Request: {user_id, chapter_url, chapter_content}
- Response: {personalized_summary, success}
```

## Phase 2: Implementation Plan

### Phase 2.1: Backend Auth Infrastructure

**Task 1: Setup Better-Auth Schema**
- Update Neon Postgres schema to include authentication fields
- Add password_hash, software_background, hardware_background fields to users table
- Ensure proper indexing for authentication queries

**Task 2: Implement Auth Endpoints**
- Create `/api/auth/register` endpoint
- Create `/api/auth/login` endpoint
- Create `/api/auth/logout` endpoint
- Implement session validation middleware
- Add password hashing with bcrypt

**Task 3: Update User Profile Schema**
- Modify existing User model to include background fields
- Add validation for expertise level enums
- Update database migration files

### Phase 2.2: Frontend Auth UI

**Task 4: Create Authentication Components**
- Create Signin/Signup React components in Docusaurus
- Add routing for `/signin` and `/signup` paths
- Implement form validation and error handling
- Use CSS from `book-frontend/src/theme/Layout.tsx` for styling

**Task 5: Implement Onboarding Modal**
- Create modal component that appears after signup
- Collect software and hardware background information
- Submit data to update user profile
- Integrate with Better-Auth session management

**Task 6: URL Configuration**
- Configure routes for both localhost and production URLs
- Add `/signin` and `/signup` paths to both environments
- Ensure no conflicts with existing routes

### Phase 2.3: Personalization Logic

**Task 7: Understand Current Backend Structure**
- Analyze `backend/database` folder files
- Understand current data models and services
- Identify integration points for personalization

**Task 8: Create Personalization API Endpoint**
- Implement `/personalize` endpoint in FastAPI
- Extract chapter ID from URL formats (localhost and production)
- Retrieve user profile from Neon Postgres
- Use OpenRouter LLM to generate personalized content
- Store personalized content in database

**Task 9: Implement Frontend Personalization Button**
- Add "Personalize for my background" button to chapter pages
- Position at bottom left, opposite to chatbot button
- Implement state management for original/personalized content toggle
- Add loading and error states

## Phase 3: Integration & Testing

### Phase 3.1: System Integration
- Connect frontend authentication to backend endpoints
- Integrate personalization engine with frontend components
- Test end-to-end user flows

### Phase 3.2: Testing
- Unit tests for authentication functions
- Integration tests for personalization API
- User acceptance testing for complete flows

## Risk Assessment

- **Authentication Integration Risk**: Better-Auth may have compatibility issues with existing FastAPI setup
- **Performance Risk**: Personalization API may be slow due to LLM processing
- **Data Consistency Risk**: User background information may not align with actual expertise

## Success Metrics

- Authentication flows complete with 95% success rate
- Personalization API responds within 10 seconds for chapters up to 5000 words under normal server load
- User satisfaction with personalized content above 80%
- Zero unauthorized access to protected features