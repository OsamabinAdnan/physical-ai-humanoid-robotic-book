# RAG Chatbot – Spec-4: Frontend and Backend Integration

## Feature: Frontend and Backend Integration

Connecting the Docusaurus frontend with the FastAPI backend to enable real-time chat-based interaction with the RAG chatbot.

## Clarifications

### Session 2025-12-15

- Q: How should the selected text functionality work exactly? → A: User selects/highlights text, a dialog appears with "Ask AI" option, clicking it sends the selected text as a focused query to the agent
- Q: Where exactly should the chat interface be placed on the Docusaurus site? → A: Floating chat button that expands to full chat interface
- Q: What specific fallback behavior should occur when the backend is temporarily unavailable? → A: Show user-friendly error message with retry option
- Q: How should the selected text UI element appear and function? → A: Contextual tooltip that appears near the selected text
- Q: Should we implement any specific communication protocol features? → A: Standard HTTP POST with JSON, no streaming

## User Scenarios & Testing

### Primary User Flow
1. User visits the Docusaurus-based book website
2. User sees a floating chat button that expands to full chat interface
3. User types a question about the Physical AI & Humanoid Robotics content
4. User submits the question to the backend API
5. User receives a response with citations to relevant content
6. User can continue the conversation or ask follow-up questions

### Selected Text User Flow
1. User selects/highlights text on the book page
2. A contextual tooltip appears near the selected text
3. User clicks "Ask AI" option in the tooltip
4. The selected text is sent as a focused query to the backend API
5. User receives a response focused on the selected text content

### Edge Cases
- User submits very long questions
- Network connection fails during request
- Backend is temporarily unavailable
- User selects text and asks about it specifically
- Multiple users using the chat simultaneously

## Functional Requirements

### FR1: API Communication
The frontend must be able to send user queries to the backend API using standard HTTP POST with JSON and receive structured responses.

### FR2: Real-time Interaction
The chat interface must provide a responsive user experience with appropriate loading states and error handling.

### FR3: Selected Text Queries
The system must allow users to select text on the page, showing a contextual tooltip with an "Ask AI" option that sends the selected text as a focused query when clicked.

### FR4: Response Display
The frontend must display responses with proper formatting, including citations and source information.

### FR5: Error Handling
The system must gracefully handle backend failures by showing user-friendly error messages with retry options.

### FR6: Styling and Theme Consistency
The chat interface must match the overall theme and styling of the book application.

## Key Entities

### QueryRequest
- question: The user's question
- top_k: Number of results to retrieve (optional)
- selected_text: Text selected by user for context (optional)

### QueryResponse
- answer: The agent's response
- citations: List of source citations
- confidence: Confidence score of the response
- processing_time_ms: Time taken to process the request

## Success Criteria

### SC1: Communication Reliability
- 99% of user queries successfully reach the backend
- Backend responses are received and displayed within 10 seconds for 95% of requests

### SC2: User Experience
- Users can initiate and maintain conversations with the chatbot
- Response time is perceived as fast and responsive by users
- Error messages are clear and helpful when failures occur

### SC3: Integration Quality
- The chat interface is seamlessly integrated into the Docusaurus site as a floating button that expands
- Styling matches the book application theme consistently
- Selected text functionality works as expected with contextual tooltips appearing near selected text

### SC4: Development Environment
- The integration works properly in local development environment
- No conflicts with existing Docusaurus functionality

### SC5: Cross-browser Compatibility
- The chat interface works across major browsers (Chrome, Firefox, Safari, Edge)

## Assumptions

- The FastAPI backend endpoints are available and functional
- The Docusaurus site has a suitable location for the chat interface
- Users have standard web browser capabilities
- Network connectivity is available during user sessions
- Better Auth integration will be handled in a future phase
- Neon DB for chat history will be implemented in a future phase

## Constraints

- Frontend: Docusaurus (static site)
- Backend: FastAPI (local or hosted)
- Communication: HTTP POST with JSON (Fetch API or Axios), no streaming
- No authentication required (for now)
- Styling must match the theme of Book app
- Production-grade deployment

## Not Building

- User accounts or chat history storage (for next step when we will use Neon DB for it)
- Analytics or monitoring