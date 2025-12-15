# RAG Chatbot – Plan-4: Frontend and Backend Integration

## Feature: Frontend and Backend Integration

Connecting the Docusaurus frontend with the FastAPI backend to enable real-time chat-based interaction with the RAG chatbot.

## Technical Context

### System Architecture
- Frontend: Docusaurus static site (React-based)
- Backend: FastAPI API server
- Communication: HTTP POST with JSON payloads
- No streaming required

### Existing Components
- Backend API endpoints: `/chat`, `/health`
- API request/response schemas: `QueryRequest`, `QueryResponse`, `Citation`
- Frontend structure: `/src/components`, `/src/pages`, `/src/css`

### Integration Points
- Floating chat button that expands to full interface
- Selected text functionality with contextual tooltips
- API communication layer

## Architecture Decisions

### A1: Frontend Component Structure
**Decision**: Implement chat functionality as a React component using Docusaurus component architecture
**Rationale**: Docusaurus uses React components, so we'll create a reusable chat component that can be integrated globally
**Alternative Considered**: Adding to global layout vs. specific pages
**Chosen Approach**: Global floating component accessible from any page

### A2: API Communication Layer
**Decision**: Use Fetch API for HTTP communication with the backend
**Rationale**: Fetch API is modern, well-supported, and doesn't require additional dependencies
**Alternative Considered**: Axios library
**Chosen Approach**: Native Fetch API to minimize dependencies

### A3: State Management
**Decision**: Use React hooks (useState, useEffect) for local component state
**Rationale**: For a chat interface, React's built-in state management is sufficient
**Alternative Considered**: Redux or Context API for global state
**Chosen Approach**: Local component state for simplicity

### A4: Global Integration Method
**Decision**: Add chat component to root layout using React Portal or by wrapping the main layout
**Rationale**: Chat should be accessible from any page in the documentation
**Alternative Considered**: Adding to each page individually
**Chosen Approach**: Global integration via layout wrapper

### A5: Selected Text Handling
**Decision**: Implement via event listeners for text selection with DOM positioning for tooltips
**Rationale**: Native browser APIs provide good text selection detection
**Alternative Considered**: Third-party libraries
**Chosen Approach**: Native browser APIs for simplicity and performance

## Implementation Strategy

### Phase 1: Component Development
1. Create Chatbot component with floating button and expandable interface
2. Implement basic UI with message display area
3. Add input field and send button functionality
4. Style to match Docusaurus theme

### Phase 2: Backend Communication
1. Implement API service module for backend communication
2. Connect chat interface to `/chat` endpoint
3. Handle request/response mapping to/from API schemas
4. Implement loading states

### Phase 3: Selected Text Feature
1. Add event listeners for text selection
2. Position contextual tooltip near selected text
3. Connect tooltip to chat functionality
4. Pass selected text as context to backend

### Phase 4: Error Handling and Polish
1. Implement error handling with user-friendly messages
2. Add retry functionality for failed requests
3. Optimize performance and accessibility
4. Cross-browser testing

## System Design

### Component Structure
```
Chatbot/
├── Chatbot.tsx (main component)
├── Chatbot.module.css (component styles)
├── ChatWindow.tsx (chat interface)
├── Message.tsx (individual message display)
├── Tooltip.tsx (selected text tooltip)
└── api.ts (API communication service)
```

### API Service Interface
- `sendMessage(question: string, selectedText?: string)`: Send query to backend
- `healthCheck()`: Check backend availability
- Return Promise with `QueryResponse` or error

### Data Flow
1. User types question OR selects text and clicks tooltip
2. Frontend sends POST request to `/chat` endpoint
3. Backend processes query with RAG agent
4. Backend returns response with citations
5. Frontend displays response in chat window

## Research & Discovery

### Docusaurus Integration Options
- Layout wrapper: Modify main layout to include chat component
- Root component: Add to main App or Layout component
- Global plugin: Create Docusaurus plugin for chat functionality

### Selected Text Implementation
- `window.getSelection()` API for detecting text selection
- `getBoundingClientRect()` for positioning tooltip
- Event listeners for 'mouseup' to detect selection completion

### API Communication Requirements
- Backend URL configuration (local vs production)
- Error handling for network failures
- Timeout handling for long-running requests
- Request/response validation

## Risk Analysis

### R1: Backend Availability
- **Risk**: Backend API may be unavailable during user sessions
- **Impact**: Users cannot interact with chatbot
- **Mitigation**: Implement clear error messages and retry functionality
- **Contingency**: Provide offline indicator and graceful degradation

### R2: Performance Impact
- **Risk**: Chat component may slow down page load or interaction
- **Impact**: Poor user experience
- **Mitigation**: Lazy load chat component, optimize rendering
- **Contingency**: Provide option to disable chat functionality

### R3: Cross-browser Compatibility
- **Risk**: Selected text functionality may not work consistently across browsers
- **Impact**: Feature inconsistency
- **Mitigation**: Thorough testing across major browsers
- **Contingency**: Fallback to basic chat interface

## Success Criteria

### SC1: Technical Integration
- [X] Chat component loads without affecting page performance
- [X] API communication works with backend endpoints
- [X] Component renders correctly across supported browsers

### SC2: User Experience
- [X] Floating chat button is accessible and intuitive
- [X] Selected text tooltip appears near selected text
- [X] Chat interface is responsive and user-friendly

### SC3: Error Handling
- [X] Clear error messages when backend is unavailable
- [X] Retry functionality for failed requests
- [X] Graceful degradation when features fail

### SC4: Styling Consistency
- [X] Chat interface matches Docusaurus theme
- [X] Component respects dark/light mode preferences
- [X] Responsive design works on mobile and desktop

## Implementation Notes

### Environment Configuration
- Development: Backend runs on localhost:8000
- Production: Backend URL from environment configuration
- CORS settings properly configured for frontend domain

### Security Considerations
- No authentication required for Phase 4
- Input sanitization handled by backend
- No sensitive data stored in frontend

### Performance Considerations
- Optimize component rendering with React.memo where appropriate
- Implement virtual scrolling if message history becomes large
- Lazy load chat component until user interacts with it