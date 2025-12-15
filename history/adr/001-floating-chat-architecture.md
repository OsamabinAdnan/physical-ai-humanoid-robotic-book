# ADR-001: Floating Chat Interface Architecture for Docusaurus Integration

## Context

For the RAG Chatbot frontend-backend integration, we needed to decide how to integrate the chat functionality into the Docusaurus-based textbook website. The solution needed to be accessible from any page while maintaining good user experience and not interfering with the existing content.

## Decision

We decided to implement a floating chat button that expands to a full chat interface. This approach provides:

- Accessibility from any page in the documentation
- Non-intrusive design that doesn't interfere with content reading
- Familiar UX pattern for users
- Efficient use of screen space
- Support for both direct chat interaction and selected text functionality

## Status

Proposed

## Consequences

### Positive
- Users can access chat functionality without navigating away from content
- Minimal impact on page layout and existing functionality
- Consistent experience across all pages of the textbook
- Ability to support contextual interactions (selected text feature)
- Good performance with proper implementation techniques

### Negative
- Potential for UI clutter if not designed carefully
- Requires careful positioning to avoid conflicts with existing elements
- Additional complexity in the frontend codebase
- Possible accessibility considerations that need to be addressed

## Alternatives Considered

### Alternative 1: Dedicated Chat Page
- Pro: Simpler implementation, isolated functionality
- Con: Users need to navigate away from content, breaks reading flow

### Alternative 2: Sidebar Integration
- Pro: Consistent location, doesn't overlay content
- Con: Takes up valuable sidebar space, may conflict with navigation

### Alternative 3: Embedded Page Components
- Pro: Context-aware responses based on current page
- Con: Inconsistent experience across pages, complex routing

## Technical Approach

- Implement as a React component compatible with Docusaurus
- Use CSS modules for styling to avoid conflicts
- Implement global integration via layout wrapper
- Add event listeners for text selection detection
- Use Fetch API for backend communication
- Implement proper error handling and loading states

## Assumptions

- Docusaurus supports global component integration
- Users will appreciate the convenience of always-accessible chat
- Performance impact will be minimal with proper implementation
- The design will be responsive and accessible