# Chatbot Component

The Chatbot component provides an AI-powered assistant for users to ask questions about the Physical AI & Humanoid Robotics textbook content.

## Features

- **Floating Chat Button**: A persistent chat button that appears on all pages of the textbook
- **Expandable Interface**: Click the button to expand the full chat interface
- **Selected Text Functionality**: Select text on any page to ask specific questions about it
- **Citation Display**: Responses include citations to relevant textbook content
- **Confidence Scores**: Each response includes a confidence score
- **Error Handling**: Clear error messages and retry functionality
- **Responsive Design**: Works on mobile and desktop devices
- **Dark/Light Mode Support**: Respects the Docusaurus theme preferences

## Architecture

The component follows the structure outlined in the plan:

```
Chatbot/
├── Chatbot.tsx (main component)
├── Chatbot.module.css (component styles)
├── types.ts (TypeScript interfaces)
├── api.ts (API service module)
└── README.md (this file)
```

## API Integration

The component communicates with the backend RAG agent through the API service module:

- `/chat` endpoint for sending questions and receiving responses
- `/health` endpoint for checking backend availability
- Uses HTTP POST with JSON payloads
- Implements proper error handling and retry logic

## Usage

The component is integrated globally through Docusaurus theme customization in `src/theme/Layout.tsx`.

## Configuration

The backend URL can be configured using the `REACT_APP_BACKEND_URL` environment variable. By default, it connects to `http://localhost:8000`.

## Error Handling

- Network errors are caught and displayed to the user
- Retry functionality is available for failed requests
- Loading states provide feedback during API calls
- Graceful degradation when backend is unavailable

## Styling

The component uses CSS modules for styling and follows the Docusaurus theme:
- Uses the primary color gradient (`#1a73e8` to `#0d47a1`)
- Respects dark/light mode preferences
- Responsive design for all screen sizes
- Smooth animations and transitions