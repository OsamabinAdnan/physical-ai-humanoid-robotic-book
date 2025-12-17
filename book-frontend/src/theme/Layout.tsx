import React, { useState, useEffect, useRef } from 'react';
import OriginalLayout from '@theme-original/Layout';

// Text selection handler as a React component
const TextSelectionHandler = () => {
  useEffect(() => {
    if (typeof window === 'undefined') return;

    let modalElement: HTMLElement | null = null;
    let hideTimeout: number | null = null;

    // Debounce function to prevent rapid firing
    const debounce = (func: (...args: any[]) => void, wait: number) => {
      let timeout: number;
      return (...args: any[]) => {
        clearTimeout(timeout);
        timeout = window.setTimeout(() => func.apply(this, args), wait);
      };
    };

    const handleTextSelection = debounce(() => {
      const selection = window.getSelection();
      if (selection && selection.toString().trim() !== '' && selection.rangeCount > 0) {
        const selectedText = selection.toString().trim();
        const range = selection.getRangeAt(0);
        const rect = range.getBoundingClientRect();

        // Remove any existing context menu
        if (modalElement && document.body.contains(modalElement)) {
          document.body.removeChild(modalElement);
        }

        // Create a new context menu
        modalElement = document.createElement('div');
        modalElement.className = 'selected-text-modal';
        modalElement.style.position = 'fixed';

        // Position the menu near the selection, ensuring it stays within viewport
        // Since we're using position: fixed, rect.top and rect.left are already relative to the viewport
        let topPos = rect.top - 45; // Position slightly above the selection
        let leftPos = rect.left + (rect.width / 2) - 70; // Center horizontally relative to selection

        // Ensure the menu stays within the viewport
        topPos = Math.max(10, topPos);
        leftPos = Math.max(10, Math.min(window.innerWidth - 160, leftPos));

        modalElement.style.top = `${topPos}px`;
        modalElement.style.left = `${leftPos}px`;
        modalElement.style.zIndex = '1000000';
        modalElement.style.backgroundColor = 'white';
        modalElement.style.border = '1px solid #e5e7eb';
        modalElement.style.borderRadius = '8px';
        modalElement.style.boxShadow = '0 10px 15px -3px rgba(0, 0, 0, 0.1), 0 4px 6px -2px rgba(0, 0, 0, 0.05)';
        modalElement.style.display = 'flex';
        modalElement.style.flexDirection = 'column';
        modalElement.style.minWidth = '140px';
        modalElement.style.maxWidth = '200px';
        modalElement.style.overflow = 'hidden';
        modalElement.style.fontFamily = '-apple-system, BlinkMacSystemFont, "Segoe UI", Roboto, Oxygen, Ubuntu, Cantarell, "Open Sans", "Helvetica Neue", sans-serif';
        modalElement.style.pointerEvents = 'auto';
        modalElement.style.userSelect = 'none';
        modalElement.style.opacity = '0';
        modalElement.style.transform = 'scale(0.8)';
        modalElement.style.transition = 'opacity 0.2s ease, transform 0.2s ease';

        // Create "Ask AI" button with theme styling
        const askAIButton = document.createElement('button');
        askAIButton.className = 'selected-text-modal-button';
        askAIButton.textContent = 'Ask AI';
        askAIButton.style.padding = '12px 16px';
        askAIButton.style.border = 'none';
        askAIButton.style.background = 'white';
        askAIButton.style.textAlign = 'center';
        askAIButton.style.cursor = 'pointer';
        askAIButton.style.fontSize = '15px';
        askAIButton.style.color = '#1f2937';
        askAIButton.style.fontWeight = '500';
        askAIButton.style.transition = 'all 0.2s ease';
        askAIButton.style.outline = 'none';

        // Hover effects matching the theme
        askAIButton.addEventListener('mouseenter', () => {
          askAIButton.style.backgroundColor = '#4f46e5';
          askAIButton.style.color = 'white';
        });

        askAIButton.addEventListener('mouseleave', () => {
          askAIButton.style.backgroundColor = 'white';
          askAIButton.style.color = '#1f2937';
        });

        askAIButton.addEventListener('click', (e) => {
          e.stopPropagation();
          // Dispatch a custom event that the chatbot can listen to
          const event = new CustomEvent('selectedTextAction', {
            detail: {
              selectedText: selectedText,
              action: 'askAI' // Use 'askAI' action instead of 'explain' or 'summarize'
            }
          });
          document.dispatchEvent(event);

          // Remove the context menu with fade out effect
          modalElement!.style.transition = 'opacity 0.15s ease, transform 0.15s ease';
          modalElement!.style.opacity = '0';
          modalElement!.style.transform = 'scale(0.8)';
          setTimeout(() => {
            if (modalElement && document.body.contains(modalElement)) {
              document.body.removeChild(modalElement);
              modalElement = null;
            }
          }, 150);
        });

        // Add button to the modal
        modalElement.appendChild(askAIButton);

        // Add modal to the document
        document.body.appendChild(modalElement);

        // Trigger the animation
        setTimeout(() => {
          if (modalElement) {
            modalElement.style.opacity = '1';
            modalElement.style.transform = 'scale(1)';
          }
        }, 0);

        // Add event listener to close modal when clicking outside
        const handleOutsideClick = (event: MouseEvent) => {
          if (modalElement && !modalElement.contains(event.target as Node)) {
            // Fade out before removing
            modalElement.style.transition = 'opacity 0.15s ease, transform 0.15s ease';
            modalElement.style.opacity = '0';
            modalElement.style.transform = 'scale(0.8)';
            setTimeout(() => {
              if (modalElement && document.body.contains(modalElement)) {
                document.body.removeChild(modalElement);
                modalElement = null;
              }
            }, 150);
          }
        };

        // Add the event listener with capture phase
        document.addEventListener('click', handleOutsideClick, { capture: true });

        // Set up timeout to remove menu after period of inactivity
        if (hideTimeout) {
          window.clearTimeout(hideTimeout);
        }
        hideTimeout = window.setTimeout(() => {
          if (modalElement && document.body.contains(modalElement)) {
            modalElement.style.transition = 'opacity 0.15s ease, transform 0.15s ease';
            modalElement.style.opacity = '0';
            modalElement.style.transform = 'scale(0.8)';
            setTimeout(() => {
              if (modalElement && document.body.contains(modalElement)) {
                document.body.removeChild(modalElement);
                modalElement = null;
              }
            }, 150);
          }
        }, 5000); // Auto-hide after 5 seconds

        // Clean up the timeout if the element is removed by other means
        const observer = new MutationObserver((mutations) => {
          mutations.forEach((mutation) => {
            if (mutation.removedNodes) {
              mutation.removedNodes.forEach((node) => {
                if (node === modalElement) {
                  if (hideTimeout) {
                    window.clearTimeout(hideTimeout);
                    hideTimeout = null;
                  }
                  document.removeEventListener('click', handleOutsideClick, { capture: true });
                }
              });
            }
          });
        });

        observer.observe(document.body, { childList: true, subtree: true });
      } else {
        // If no text is selected, remove the context menu if it exists
        if (modalElement && document.body.contains(modalElement)) {
          modalElement.style.transition = 'opacity 0.15s ease, transform 0.15s ease';
          modalElement.style.opacity = '0';
          modalElement.style.transform = 'scale(0.8)';
          setTimeout(() => {
            if (modalElement && document.body.contains(modalElement)) {
              document.body.removeChild(modalElement);
              modalElement = null;
            }
          }, 150);
        }
      }
    }, 150); // Debounce for 150ms to prevent multiple rapid calls

    document.addEventListener('mouseup', handleTextSelection);

    // Clean up
    return () => {
      document.removeEventListener('mouseup', handleTextSelection);

      // Remove any existing context menu
      if (modalElement && document.body.contains(modalElement)) {
        document.body.removeChild(modalElement);
        modalElement = null;
      }

      // Clear timeout if exists
      if (hideTimeout) {
        window.clearTimeout(hideTimeout);
      }

      console.log('Text selection handler cleanup');
    };
  }, []);

  return null; // This component doesn't render anything itself
};

// Define TypeScript interfaces
interface IChatMessage {
  id: string;
  role: 'user' | 'assistant';
  content: string;
  timestamp: Date;
  metadata?: {
    selectedText?: string;
    citations?: ICitation[];
  };
}

interface ICitation {
  text: string;
  url: string;
  similarity_score: number;
  chunk_id: number;
  source_title: string;
}

interface IApiRequest {
  query: string;
  selected_text?: string;
  action?: 'explain' | 'summarize' | 'askAI';
}

interface IApiResponse {
  answer: string;
  citations: ICitation[];
}

// API service with fallback and CORS handling
const apiService = {
  async ask(request: IApiRequest): Promise<IApiResponse> {
    try {
      // Use environment variable or default to localhost, but make it configurable
      // Check if we're in a browser environment (where process might not be defined)
      let API_BASE_URL = process.env.NODE_ENV === 'production' ? 'https://osamabinadnan-rag-with-neondb.hf.space': 'http://127.0.0.1:8000';  // Local development
      if (typeof process !== 'undefined' && process.env && process.env.REACT_APP_API_BASE_URL) {
        API_BASE_URL = process.env.REACT_APP_API_BASE_URL;
      }

      // Prepare the request body according to the backend schema
      let finalQuestion = request.query;
      // If there's selected text and an action, prepend it to the question
      if (request.selected_text && request.action) {
        if (request.action === 'explain') {
          finalQuestion = `Please explain the following text: "${request.selected_text}". ${request.query}`;
        } else if (request.action === 'summarize') {
          finalQuestion = `Please summarize the following text: "${request.selected_text}". ${request.query}`;
        } else if (request.action === 'askAI') {
          finalQuestion = `Regarding the text "${request.selected_text}": ${request.query}`;
        } else {
          finalQuestion = `Regarding the text "${request.selected_text}": ${request.query}`;
        }
      }

      const requestBody = {
        question: finalQuestion,
        top_k: request.action === 'explain' ? 5 : 3 // Adjust based on action
      };

      console.log('Making API request to:', `${API_BASE_URL}/chat`);
      console.log('Request payload:', requestBody);

      const controller = new AbortController();
      const timeoutId = setTimeout(() => controller.abort(), 30000); // 30 second timeout to allow for agent processing

      const response = await fetch(`${API_BASE_URL}/chat`, {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify(requestBody),
        signal: controller.signal, // Support for cancellation
        mode: 'cors', // Enable CORS
        credentials: 'omit' // Don't include credentials
      });

      clearTimeout(timeoutId);

      console.log('Response status:', response.status);
      console.log('Response ok:', response.ok);

      if (!response.ok) {
        throw new Error(`HTTP error! status: ${response.status} - ${response.statusText}`);
      }

      // The backend response has a different structure than expected
      const backendResponse = await response.json();

      // Transform backend response to match our expected format
      const transformedResponse: IApiResponse = {
        answer: backendResponse.answer,
        citations: backendResponse.citations || []
      };

      console.log('API response:', transformedResponse);
      return transformedResponse;
    } catch (error) {
      console.error('API call failed:', error);

      // Provide a fallback response when the backend is not available
      // This allows the UI to continue working for testing purposes
      // Check if we're in development mode
      const isDev = typeof process !== 'undefined' && process.env && process.env.NODE_ENV === 'development';
      if (isDev) {
        console.warn('Using mock response for development purposes');
        console.warn('Error details:', error);
        return {
          answer: `I received your query: "${request.query}". The backend server is not currently running. To get actual responses, please start the backend server at http://127.0.0.1:8000. Error: ${(error as Error).message}`,
          citations: []
        };
      } else {
        // In production, throw the error
        if (error instanceof TypeError) {
          // Network error - likely server is not running
          throw new Error('Network error: Unable to connect to the server. Please make sure the backend API server is running on http://127.0.0.1:8000');
        } else if ((error as Error).name === 'AbortError') {
          // Timeout error
          throw new Error('Request timeout: The server took too long to respond. Please check if the backend API server is running');
        } else {
          // Other errors (e.g., HTTP errors)
          throw error;
        }
      }
    }
  }
};

// ChatInput component
const ChatInput = ({ onSubmit, disabled }: { onSubmit: (message: string, selectedText?: string, action?: 'explain' | 'summarize') => void, disabled: boolean }) => {
  const [inputValue, setInputValue] = useState<string>('');
  const textareaRef = useRef<HTMLTextAreaElement>(null);

  const handleSubmit = () => {
    if (inputValue.trim() && !disabled) {
      onSubmit(inputValue);
      setInputValue('');
    }
  };

  const handleKeyDown = (e: React.KeyboardEvent<HTMLTextAreaElement>) => {
    if (e.key === 'Enter' && !e.shiftKey) {
      e.preventDefault();
      handleSubmit();
    }
  };

  // Auto-resize textarea based on content
  useEffect(() => {
    if (textareaRef.current) {
      textareaRef.current.style.height = 'auto';
      textareaRef.current.style.height = Math.min(textareaRef.current.scrollHeight, 150) + 'px';
    }
  }, [inputValue]);

  return (
    <div style={{
      padding: '16px',
      backgroundColor: 'var(--ifm-background-surface-color)',
      borderTop: '1px solid var(--ifm-color-emphasis-300)',
      display: 'flex',
      gap: '8px'
    }}>
      <textarea
        ref={textareaRef}
        value={inputValue}
        onChange={(e) => setInputValue(e.target.value)}
        onKeyDown={handleKeyDown}
        placeholder="Ask me..."
        style={{
          flex: 1,
          padding: '14px 16px',
          border: '1px solid var(--ifm-color-emphasis-300)',
          borderRadius: '24px',
          fontSize: '15px',
          resize: 'none',
          maxHeight: '150px',
          minHeight: '50px',
          outline: 'none',
          overflow: 'hidden',
          backgroundColor: 'var(--ifm-background-surface-color)',
          color: 'var(--ifm-font-color-base)'
        }}
        disabled={disabled}
      />
      <button
        onClick={handleSubmit}
        disabled={disabled || !inputValue.trim()}
        style={{
          backgroundColor: disabled || !inputValue.trim() ? 'var(--ifm-color-emphasis-200)' : 'var(--ifm-color-primary)',
          color: 'white',
          border: 'none',
          borderRadius: '50%',
          width: '40px',
          height: '40px',
          display: 'flex',
          alignItems: 'center',
          justifyContent: 'center',
          cursor: disabled || !inputValue.trim() ? 'not-allowed' : 'pointer'
        }}
      >
        ➤
      </button>
    </div>
  );
};

// ChatResponse component
const ChatResponse = ({ messages }: { messages: IChatMessage[] }) => {
  const messagesEndRef = useRef<HTMLDivElement>(null);

  // Scroll to the bottom whenever messages change
  useEffect(() => {
    messagesEndRef.current?.scrollIntoView({ behavior: 'smooth' });
  }, [messages]);

  return (
    <div style={{
      flex: 1,
      padding: '16px',
      overflowY: 'auto',
      display: 'flex',
      flexDirection: 'column',
      gap: '12px',
      backgroundColor: 'var(--ifm-background-surface-color)',
      minHeight: '450px' // Ensure minimum height for the messages area
    }}>
      {messages.map((message) => (
        <div
          key={message.id}
          style={{
            maxWidth: '85%',
            padding: '10px 14px',
            borderRadius: '18px',
            backgroundColor: message.role === 'user' ? 'var(--ifm-color-primary)' : '#AEDEFC', // Changed assistant color to #AEDEFC, user uses theme primary color
            color: message.role === 'user' ? 'white' : '#1e293b',
            alignSelf: message.role === 'user' ? 'flex-end' : 'flex-start',
            borderBottomLeftRadius: message.role === 'assistant' ? '4px' : '18px',
            borderBottomRightRadius: message.role === 'user' ? '4px' : '18px',
            fontSize: '15px' // Increased text size for both bubbles
          }}
        >
          <div style={{ fontSize: '15px' }}>{message.content}</div>
          {message.metadata?.citations && message.metadata.citations.length > 0 && (
            <div style={{ marginTop: '12px' }}>
              {message.metadata.citations.map((citation, index) => (
                <div
                  key={citation.id || `citation-${index}`}
                  style={{
                    backgroundColor: 'var(--ifm-color-emphasis-100)', // Theme background
                    border: '1px solid var(--ifm-color-emphasis-300)',
                    borderRadius: '6px',
                    padding: '8px 10px',
                    marginTop: '8px',
                    fontSize: '12px', // Smaller font size
                    cursor: 'pointer',
                    color: 'var(--ifm-color-primary)', // Theme color
                    fontWeight: '500'
                  }}
                  onClick={() => window.open(citation.url, '_blank', 'noopener,noreferrer')}
                >
                  <strong style={{ color: 'var(--ifm-color-primary)', fontSize: '12px' }}>Source {index + 1}:</strong> {citation.source_title}
                </div>
              ))}
            </div>
          )}
        </div>
      ))}
      {/* Invisible div to anchor scroll to */}
      <div ref={messagesEndRef} />
    </div>
  );
};

// Main chatbot component
const FunctionalChatbot = () => {
  const [isOpen, setIsOpen] = useState(false);
  const [messages, setMessages] = useState<IChatMessage[]>([]);
  const [isLoading, setIsLoading] = useState(false);
  const [showHistory, setShowHistory] = useState(false);
  const [chatSessions, setChatSessions] = useState<any[]>([]);
  const [selectedSession, setSelectedSession] = useState<any>(null);
  const [loadingHistory, setLoadingHistory] = useState(false);

  // Generate a temporary user ID for the session
  const getOrCreateUserId = () => {
    let userId = localStorage.getItem('chat_user_id');
    if (!userId) {
      userId = `temp_${Date.now()}_${Math.random().toString(36).substr(2, 9)}`;
      localStorage.setItem('chat_user_id', userId);
    }
    return userId;
  };

  // Fetch user chat history
  const fetchChatHistory = async () => {
    try {
      const userId = getOrCreateUserId();
      setLoadingHistory(true);
      const response = await fetch(`https://osamabinadnan-rag-with-neondb.hf.space/chat-history/${userId}`);
      if (!response.ok) {
        throw new Error(`Failed to fetch chat history: ${response.status} ${response.statusText}`);
      }
      const data = await response.json();
      setChatSessions(data.sessions || []);
    } catch (err) {
      console.error('Error fetching chat history:', err);
    } finally {
      setLoadingHistory(false);
    }
  };

  // Load a specific session history
  const loadSessionHistory = async (session: any) => {
    try {
      const userId = getOrCreateUserId();
      const response = await fetch(`https://osamabinadnan-rag-with-neondb.hf.space/chat-history/${userId}/session/${session.id}`);
      if (!response.ok) {
        throw new Error(`Failed to fetch session history: ${response.status} ${response.statusText}`);
      }
      const data = await response.json();

      // Convert backend messages to frontend message format
      const convertedMessages: IChatMessage[] = data.messages.map((msg: any) => ({
        id: msg.id,
        role: msg.role === 'assistant' ? 'assistant' : 'user',
        content: msg.content,
        timestamp: new Date(msg.created_at),
        metadata: msg.citations ? { citations: msg.citations } : undefined
      }));

      setMessages(convertedMessages);
      setSelectedSession(session);
      setShowHistory(false); // Close the history panel after loading
    } catch (err) {
      console.error('Error loading session history:', err);
    }
  };

  // Start a new chat session
  const startNewChat = () => {
    setMessages([]);
    setSelectedSession(null);
  };

  // Load chat history when history panel is opened
  useEffect(() => {
    if (showHistory && isOpen) {
      fetchChatHistory();
    }
  }, [showHistory, isOpen]);

  // Add initial message when component mounts and chat is opened
  useEffect(() => {
    if (isOpen && messages.length === 0) {
      const welcomeMessage: IChatMessage = {
        id: 'welcome-' + Date.now(),
        role: 'assistant',
        content: 'Hello! I\'m your Physical AI & Humanoid Robotics assistant. Ask me anything about the textbook content.',
        timestamp: new Date(),
      };
      setMessages([welcomeMessage]);
    }
  }, [isOpen, messages.length]);

  // Listen for selected text events
  useEffect(() => {
    const handleSelectedTextAction = (event: Event) => {
      console.log('Received selectedTextAction event:', event);
      const customEvent = event as CustomEvent;
      const { selectedText, action } = customEvent.detail;
      console.log('Event detail - selectedText:', selectedText, 'action:', action);

      // Create a contextual query based on the selected text and action
      let query = selectedText;
      if (action === 'explain') {
        query = `Explain the following text: ${selectedText}`;
      } else if (action === 'summarize') {
        query = `Summarize the following text: ${selectedText}`;
      } else if (action === 'askAI') {
        query = selectedText; // For 'askAI', just use the selected text as the query
      }

      // Open the chatbot if it's not already open
      setIsOpen(true);
      console.log('Opening chatbot for selected text action');

      // Add user message to chat
      const userMessage: IChatMessage = {
        id: 'user-' + Date.now(),
        role: 'user',
        content: `Ask AI: "${selectedText}"`,
        timestamp: new Date(),
        metadata: { selectedText }
      };

      setMessages(prev => [...prev, userMessage]);
      handleSubmit(query, selectedText, action);
    };

    // Handle clicks outside the chatbot
    const handleClickOutside = (event: MouseEvent) => {
      if (isOpen) {
        const chatbotPanel = document.querySelector('.chatbot-panel');
        if (chatbotPanel && !chatbotPanel.contains(event.target as Node)) {
          setIsOpen(false);
        }
      }
    };

    document.addEventListener('selectedTextAction', handleSelectedTextAction);
    document.addEventListener('mousedown', handleClickOutside);
    console.log('Listening for selectedTextAction events and outside clicks');

    return () => {
      document.removeEventListener('selectedTextAction', handleSelectedTextAction);
      document.removeEventListener('mousedown', handleClickOutside);
      console.log('Stopped listening for selectedTextAction events and outside clicks');
    };
  }, [isOpen]); // Added isOpen to dependencies to ensure proper cleanup

  const handleSubmit = async (message: string, selectedText?: string, action?: 'explain' | 'summarize') => {
    if (!message.trim() || isLoading) return;

    try {
      setIsLoading(true);

      // Prepare API request
      const request: IApiRequest = {
        query: message,
        selected_text: selectedText,
        action: action,
      };

      // Add a temporary loading message
      const loadingMessage: IChatMessage = {
        id: 'loading-' + Date.now(),
        role: 'assistant',
        content: '●●●', // More attractive loading indicator
        timestamp: new Date(),
      };

      setMessages(prev => [...prev, loadingMessage]);

      // Call the backend API
      const response = await apiService.ask(request);

      // Remove the loading message and add the actual response
      setMessages(prev => {
        // Filter out the loading message
        const updatedMessages = prev.filter(msg => !msg.id.startsWith('loading-'));

        // Add the actual response
        const assistantMessage: IChatMessage = {
          id: 'assistant-' + Date.now(),
          role: 'assistant',
          content: response.answer,
          timestamp: new Date(),
          metadata: {
            citations: response.citations
          }
        };

        return [...updatedMessages, assistantMessage];
      });
    } catch (error) {
      // Remove the loading message and add the error message
      setMessages(prev => {
        // Filter out the loading message
        const updatedMessages = prev.filter(msg => !msg.id.startsWith('loading-'));

        const errorMessage: IChatMessage = {
          id: 'error-' + Date.now(),
          role: 'assistant',
          content: `Sorry, I encountered an error: ${(error as Error).message}`,
          timestamp: new Date(),
        };

        return [...updatedMessages, errorMessage];
      });
    } finally {
      setIsLoading(false);
    }
  };

  const handleChatSubmit = (message: string, selectedText?: string, action?: 'explain' | 'summarize' | 'askAI') => {
    if (!message.trim()) return;

    // Add user message to chat
    const userMessage: IChatMessage = {
      id: 'user-' + Date.now(),
      role: 'user',
      content: selectedText && action
        ? (action === 'askAI' ? `Ask AI: "${selectedText}"` : `${action.charAt(0).toUpperCase() + action.slice(1)}: "${selectedText}"`)
        : message,
      timestamp: new Date(),
      metadata: selectedText ? { selectedText } : undefined,
    };

    setMessages(prev => [...prev, userMessage]);
    handleSubmit(message, selectedText, action);
  };

  if (!isOpen) {
    return (
      <div className="chatbot-container">
        <button
          onClick={() => setIsOpen(true)}
          className="chatbot-float-button"
          style={{
            width: '60px',
            height: '60px',
            borderRadius: '50%',
            backgroundColor: 'var(--ifm-color-primary)',
            color: 'white',
            border: 'none',
            fontSize: '24px',
            display: 'flex',
            alignItems: 'center',
            justifyContent: 'center',
            cursor: 'pointer',
            boxShadow: '0 4px 6px -1px rgba(0, 0, 0, 0.1), 0 2px 4px -1px rgba(0, 0, 0, 0.06)',
            position: 'fixed',
            bottom: '20px',
            right: '20px',
            zIndex: '100000',
            fontWeight: 'bold'
          }}
        >
          💬
        </button>
      </div>
    );
  }

  return (
    <div
      className="chatbot-panel"
      style={{
        position: 'fixed',
        bottom: '20px',
        right: '20px',
        width: '400px',
        height: '600px',
        display: 'flex',
        flexDirection: 'column',
        backgroundColor: 'var(--ifm-background-surface-color)',
        borderRadius: '16px',
        border: '1px solid var(--ifm-color-emphasis-300)',
        boxShadow: '0 20px 25px -5px rgba(0, 0, 0, 0.2), 0 10px 10px -5px rgba(0, 0, 0, 0.1)',
        zIndex: '99999',
        fontFamily: '-apple-system, BlinkMacSystemFont, "Segoe UI", Roboto, Oxygen, Ubuntu, Cantarell, "Open Sans", "Helvetica Neue", sans-serif'
      }}
      onClick={(e) => e.stopPropagation()} // Prevent clicks inside the chatbot from closing it
    >
      <div style={{
        backgroundColor: 'var(--ifm-color-primary)',
        color: 'white',
        padding: '16px',
        display: 'flex',
        justifyContent: 'space-between',
        alignItems: 'center',
        borderTopLeftRadius: '16px',
        borderTopRightRadius: '16px'
      }}>
        <div style={{ display: 'flex', alignItems: 'center', gap: '8px' }}>
          <h3 style={{ fontSize: '18px', fontWeight: '600', margin: 0, color: 'white' }}>
            {selectedSession ? selectedSession.title : 'Physical AI Assistant'}
          </h3>
          {selectedSession && (
            <button
              onClick={startNewChat}
              style={{
                background: 'rgba(255, 255, 255, 0.2)',
                border: '1px solid rgba(255, 255, 255, 0.3)',
                color: 'white',
                padding: '4px 8px',
                borderRadius: '4px',
                fontSize: '12px',
                cursor: 'pointer',
                transition: 'all 0.2s ease'
              }}
              title="Start new chat"
            >
              ✚ New
            </button>
          )}
        </div>
        <div style={{ display: 'flex', alignItems: 'center', gap: '8px' }}>
          <button
            onClick={() => setShowHistory(!showHistory)}
            style={{
              background: 'none',
              border: 'none',
              color: 'white',
              cursor: 'pointer',
              fontSize: '16px',
              padding: '4px',
              borderRadius: '4px',
              transition: 'background-color 0.2s ease'
            }}
            title="Chat history"
          >
            📜
          </button>
          <button
            onClick={() => setIsOpen(false)}
            style={{
              background: 'none',
              border: 'none',
              color: 'white',
              cursor: 'pointer',
              fontSize: '28px',
              width: '40px',
              height: '40px',
              borderRadius: '50%',
              display: 'flex',
              alignItems: 'center',
              justifyContent: 'center'
            }}
          >
            ×
          </button>
        </div>
      </div>

      {/* Chat history panel */}
      {showHistory ? (
        <div style={{
          flex: 1,
          display: 'flex',
          flexDirection: 'column',
          overflow: 'hidden',
          backgroundColor: 'white',
          border: '1px solid var(--ifm-color-emphasis-300)',
          borderBottomLeftRadius: '16px',
          borderBottomRightRadius: '16px',
          margin: '0 16px 16px 16px'
        }}>
          <div style={{
            display: 'flex',
            justifyContent: 'space-between',
            alignItems: 'center',
            padding: '12px 16px',
            backgroundColor: '#f5f5f5',
            borderBottom: '1px solid #e0e0e0'
          }}>
            <h3 style={{ margin: 0, fontSize: '16px', fontWeight: '600', color: '#333' }}>Chat History</h3>
            <button
              onClick={() => setShowHistory(false)}
              style={{
                background: 'none',
                border: 'none',
                color: '#666',
                cursor: 'pointer',
                fontSize: '18px',
                padding: 0,
                width: '24px',
                height: '24px',
                display: 'flex',
                alignItems: 'center',
                justifyContent: 'center'
              }}
            >
              ×
            </button>
          </div>

          {loadingHistory ? (
            <div style={{
              flex: 1,
              display: 'flex',
              alignItems: 'center',
              justifyContent: 'center'
            }}>
              <div style={{
                display: 'flex',
                gap: '4px'
              }}>
                <span style={{ backgroundColor: '#666', borderRadius: '50%', width: '8px', height: '8px', display: 'inline-block', animation: 'loading 1.4s infinite ease-in-out both' }}></span>
                <span style={{ backgroundColor: '#666', borderRadius: '50%', width: '8px', height: '8px', display: 'inline-block', animation: 'loading 1.4s infinite ease-in-out both', animationDelay: '-0.32s' }}></span>
                <span style={{ backgroundColor: '#666', borderRadius: '50%', width: '8px', height: '8px', display: 'inline-block', animation: 'loading 1.4s infinite ease-in-out both', animationDelay: '-0.16s' }}></span>
                <style>{`
                  @keyframes loading {
                    0%, 80%, 100% { transform: scale(0); }
                    40% { transform: scale(1); }
                  }
                `}</style>
              </div>
            </div>
          ) : chatSessions.length === 0 ? (
            <div style={{
              flex: 1,
              display: 'flex',
              alignItems: 'center',
              justifyContent: 'center',
              color: '#666',
              fontStyle: 'italic'
            }}>
              No chat history available
            </div>
          ) : (
            <div style={{
              flex: 1,
              overflowY: 'auto',
              padding: '8px',
              display: 'flex',
              flexDirection: 'column',
              gap: '8px'
            }}>
              {chatSessions.map((session) => (
                <div
                  key={session.id}
                  style={{
                    padding: '12px',
                    borderRadius: '8px',
                    backgroundColor: selectedSession?.id === session.id ? '#e3f2fd' : '#f9f9f9',
                    cursor: 'pointer',
                    transition: 'background-color 0.2s ease',
                    border: selectedSession?.id === session.id ? '1px solid #1a73e8' : '1px solid #e0e0e0',
                    ':hover': {
                      backgroundColor: '#f0f0f0'
                    }
                  }}
                  onClick={() => loadSessionHistory(session)}
                >
                  <div style={{ fontWeight: '500', marginBottom: '4px', color: '#333' }}>
                    {session.title}
                  </div>
                  <div style={{ fontSize: '12px', color: '#666' }}>
                    {new Date(session.created_at).toLocaleDateString()}
                  </div>
                </div>
              ))}
            </div>
          )}
        </div>
      ) : (
        <>
          <ChatResponse messages={messages} />
          <ChatInput onSubmit={handleChatSubmit} disabled={isLoading} />
        </>
      )}
    </div>
  );
};

export default function Layout(props) {
  return (
    <>
      <OriginalLayout {...props} />
      <TextSelectionHandler />
      <FunctionalChatbot />
    </>
  );
}