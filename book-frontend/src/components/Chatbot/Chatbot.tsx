import React, { useState, useEffect, useRef } from 'react';
import clsx from 'clsx';
import styles from './Chatbot.module.css';
import { QueryRequest, QueryResponse, Message } from './types';
import { sendMessage, getUserChatHistory, getSessionHistory, ChatSession, ChatMessage, UserChatHistoryResponse } from './api';
import { useAuth } from '../Auth/AuthContext';

const Chatbot: React.FC = () => {
  const [isOpen, setIsOpen] = useState(false);
  const [messages, setMessages] = useState<Message[]>([]);
  const [inputValue, setInputValue] = useState('');
  const [isLoading, setIsLoading] = useState(false);
  const [error, setError] = useState<string | null>(null);
  const [selectedText, setSelectedText] = useState<string | null>(null);
  const [showHistory, setShowHistory] = useState(false);
  const [chatSessions, setChatSessions] = useState<ChatSession[]>([]);
  const [selectedSession, setSelectedSession] = useState<ChatSession | null>(null);
  const [loadingHistory, setLoadingHistory] = useState(false);
  const [currentSessionId, setCurrentSessionId] = useState<string | null>(null);

  const { user, isAuthenticated } = useAuth();

  const messagesEndRef = useRef<HTMLDivElement>(null);
  const chatInputRef = useRef<HTMLTextAreaElement>(null);

  // Scroll to bottom of messages when new messages are added
  useEffect(() => {
    scrollToBottom();
  }, [messages]);

  // Focus input when chat is opened
  useEffect(() => {
    if (isOpen && chatInputRef.current) {
      setTimeout(() => {
        chatInputRef.current?.focus();
      }, 100);
    }
  }, [isOpen]);

  const scrollToBottom = () => {
    messagesEndRef.current?.scrollIntoView({ behavior: 'smooth' });
  };

  // Generate a user ID for the session (use authenticated user ID if available, otherwise temporary)
  const getOrCreateUserId = () => {
    // If user is authenticated, use their ID
    if (isAuthenticated && user && user.id) {
      return user.id;
    }

    // Otherwise, use temporary user ID
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
      console.log('Fetching chat history for user:', userId);

      const response = await getUserChatHistory(userId);
      console.log('Raw chat history response:', response);

      // Ensure the response has the expected structure
      const sessions = Array.isArray(response) ? response : (response && response.sessions) ? response.sessions : [];
      console.log('Processed chat sessions:', sessions);

      setChatSessions(sessions);
    } catch (err) {
      console.error('Error fetching chat history:', err);
      // Don't set error state for history loading as it shouldn't break the main chat functionality
      // setError('Failed to load chat history');
    } finally {
      setLoadingHistory(false);
    }
  };

  // Load chat history when history panel is opened
  useEffect(() => {
    if (showHistory && isOpen) {
      fetchChatHistory();
    }
  }, [showHistory, isOpen]);

  // Load chat history when user authenticates
  useEffect(() => {
    if (isAuthenticated && isOpen && messages.length === 0) {
      fetchChatHistory();
    }
  }, [isAuthenticated, isOpen, messages.length]);

  // Load a specific session history
  const loadSessionHistory = async (session: ChatSession) => {
    const userId = getOrCreateUserId();
    try {
      const response = await getSessionHistory(userId, session.id);
      // Convert backend messages to frontend message format
      const convertedMessages: Message[] = response.messages.map(msg => ({
        id: msg.id,
        text: msg.content,
        sender: msg.role as 'user' | 'bot',
        timestamp: new Date(msg.created_at),
      }));
      setMessages(convertedMessages);
      setSelectedSession(session);
      setShowHistory(false); // Close the history panel after loading
    } catch (err) {
      console.error('Error loading session history:', err);
      setError('Failed to load session history');
    }
  };

  // Start a new chat session
  const startNewChat = () => {
    setMessages([]);
    setSelectedSession(null);
    setCurrentSessionId(null);
  };

  // Handle sending a message to the backend
  const handleSendMessage = async () => {
    if (!inputValue.trim() && !selectedText) return;

    const question = inputValue.trim() || (selectedText || '').trim();
    if (!question) return;

    // Check if this is the first message in a new conversation
    const isNewConversation = messages.length === 0;

    // Add user message to chat
    const userMessage: Message = {
      id: Date.now().toString(),
      text: question,
      sender: 'user',
      timestamp: new Date(),
    };

    setMessages(prev => [...prev, userMessage]);
    setInputValue('');
    setSelectedText(null); // Clear selected text after using it
    setIsLoading(true);
    setError(null);

    try {
      // Prepare the request payload
      const requestBody: QueryRequest = {
        question: question,
        top_k: 5,
        ...(selectedText && { selected_text: selectedText })
      };

      // Call the backend API using our API service
      const data: QueryResponse = await sendMessage(requestBody);

      // Add bot response to chat
      const botMessage: Message = {
        id: Date.now().toString(),
        text: data.answer,
        sender: 'bot',
        timestamp: new Date(),
        citations: data.citations,
        confidence: data.confidence,
      };

      setMessages(prev => [...prev, botMessage]);

      // If this was the first message in a new conversation, refresh the chat history
      // to load the new session that was created on the backend
      if (isNewConversation) {
        // Small delay to ensure the session is created on the backend before fetching history
        setTimeout(() => {
          if (showHistory) {
            fetchChatHistory();
          }
        }, 1000);
      }
    } catch (err) {
      console.error('Error sending message:', err);
      setError(err instanceof Error ? err.message : 'An unknown error occurred');

      // Add error message to chat
      const errorMessage: Message = {
        id: Date.now().toString(),
        text: 'Sorry, I encountered an error processing your request. Please try again.',
        sender: 'bot',
        timestamp: new Date(),
      };
      setMessages(prev => [...prev, errorMessage]);
    } finally {
      setIsLoading(false);
    }
  };

  // Handle retry functionality for failed requests
  const handleRetry = async (messageId: string) => {
    // Find the failed message
    const messageToRetry = messages.find(msg => msg.id === messageId);
    if (!messageToRetry || messageToRetry.sender !== 'user') return;

    // Remove the error message that followed the failed request
    setMessages(prev => prev.filter(msg =>
      !(msg.sender === 'bot' &&
        msg.text.includes('Sorry, I encountered an error') &&
        prev.indexOf(messageToRetry as Message) === prev.indexOf(msg) - 1)
    ));

    // Resend the message
    setInputValue(messageToRetry.text);
    handleSendMessage();
  };

  // Handle Enter key press for sending message
  const handleKeyPress = (e: React.KeyboardEvent) => {
    if (e.key === 'Enter' && !e.shiftKey) {
      e.preventDefault();
      handleSendMessage();
    }
  };

  // Toggle chat window open/closed
  const toggleChat = () => {
    setIsOpen(!isOpen);
  };

  // Close chat window
  const closeChat = () => {
    setIsOpen(false);
  };

  // Handle text selection for the selected text feature
  useEffect(() => {
    let tooltip: HTMLElement | null = null;

    const handleTextSelection = () => {
      const selection = window.getSelection();
      if (selection && selection.toString().trim() !== '') {
        const selectedText = selection.toString().trim();
        if (selectedText.length > 0 && selectedText.length < 500) { // Reasonable length limit
          // Get the position of the selection
          const range = selection.getRangeAt(0);
          const rect = range.getBoundingClientRect();

          // Create or update the tooltip
          tooltip = document.getElementById('chatbot-tooltip');
          if (!tooltip) {
            tooltip = document.createElement('div');
            tooltip.id = 'chatbot-tooltip';
            tooltip.className = styles.tooltip;
            document.body.appendChild(tooltip);
          }

          // Position the tooltip near the selection
          tooltip.style.top = `${rect.top + window.scrollY - 40}px`;
          tooltip.style.left = `${rect.left + window.scrollX + (rect.width / 2)}px`;
          tooltip.style.display = 'block';

          // Add click handler to the tooltip
          const handleClick = () => {
            if (selectedText) {
              setInputValue(selectedText);
              setIsOpen(true);
              selection?.removeAllRanges(); // Clear selection
              if (tooltip) {
                tooltip.style.display = 'none';
              }
            }
          };

          // Remove any existing event listener to avoid duplicates
          if (tooltip.hasAttribute('data-listener-added')) {
            tooltip.removeEventListener('click', handleClick);
          }

          tooltip.addEventListener('click', handleClick);
          tooltip.setAttribute('data-listener-added', 'true');
        }
      } else {
        // Hide tooltip when no text is selected
        tooltip = document.getElementById('chatbot-tooltip');
        if (tooltip) {
          tooltip.style.display = 'none';
        }
      }
    };

    document.addEventListener('mouseup', handleTextSelection);
    return () => {
      document.removeEventListener('mouseup', handleTextSelection);
      // Clean up tooltip when component unmounts
      const tooltip = document.getElementById('chatbot-tooltip');
      if (tooltip) {
        tooltip.remove();
      }
    };
  }, []);


  return (
    <>
      {/* Floating chat button */}
      {!isOpen && (
        <button
          className={clsx(styles.chatContainer, styles.floatingButton)}
          onClick={toggleChat}
          aria-label="Open chat"
        >
          💬
        </button>
      )}

      {/* Chat window */}
      {isOpen && (
        <div className={styles.chatContainer}>
          <div className={styles.chatWindow}>
            {/* Chat header with history button */}
            <div className={styles.chatHeader}>
              <div className={styles.chatHeaderLeft}>
                <span>{selectedSession ? selectedSession.title : 'AI Assistant'}</span>
              </div>
              <div className={styles.chatHeaderRight}>
                {selectedSession && (
                  <button
                    className={styles.newChatButton}
                    onClick={startNewChat}
                    title="Start new chat"
                  >
                    ✚ New
                  </button>
                )}
                <button
                  className={styles.historyButton}
                  onClick={() => setShowHistory(!showHistory)}
                  title="Chat history"
                >
                  📜
                </button>
                <button
                  className={styles.chatCloseButton}
                  onClick={closeChat}
                  aria-label="Close chat"
                >
                  ×
                </button>
              </div>
            </div>

            {/* Chat history panel */}
            {showHistory && (
              <div className={styles.historyPanel}>
                <div className={styles.historyHeader}>
                  <h3>Chat History</h3>
                  <button
                    className={styles.closeHistoryButton}
                    onClick={() => setShowHistory(false)}
                    aria-label="Close history"
                  >
                    ×
                  </button>
                </div>
                {loadingHistory ? (
                  <div className={styles.loadingHistory}>
                    <div className={styles.loadingDots}>
                      <span></span>
                      <span></span>
                      <span></span>
                    </div>
                  </div>
                ) : chatSessions.length === 0 ? (
                  <div className={styles.noHistory}>
                    No chat history available
                  </div>
                ) : (
                  <div className={styles.historyList}>
                    {chatSessions.map((session) => (
                      <div
                        key={session.id}
                        className={clsx(
                          styles.historyItem,
                          selectedSession?.id === session.id ? styles.historyItemSelected : ''
                        )}
                        onClick={() => loadSessionHistory(session)}
                      >
                        <div className={styles.historyTitle}>{session.title || `Chat ${new Date(session.created_at).toLocaleString()}`}</div>
                        <div className={styles.historyDate}>
                          {new Date(session.created_at).toLocaleDateString()}
                        </div>
                      </div>
                    ))}
                  </div>
                )}
              </div>
            )}

            {/* Messages area */}
            {!showHistory && (
              <div className={styles.chatMessages}>
                {messages.length === 0 ? (
                  <div className={clsx(styles.message, styles.botMessage)}>
                    Hello! I'm your AI assistant for the Physical AI & Humanoid Robotics textbook. How can I help you today?
                  </div>
                ) : (
                  messages.map((message) => (
                    <div
                      key={message.id}
                      className={clsx(
                        styles.message,
                        message.sender === 'user' ? styles.userMessage : styles.botMessage
                      )}
                    >
                      {message.text}
                      {message.citations && message.citations.length > 0 && (
                        <div className={styles.citation}>
                          <strong>Citations:</strong>
                          <ul style={{ margin: '5px 0', paddingLeft: '20px' }}>
                            {message.citations.slice(0, 3).map((citation, index) => (
                              <li key={index}>
                                <a
                                  href={citation.url}
                                  target="_blank"
                                  rel="noopener noreferrer"
                                  style={{ color: 'inherit', textDecoration: 'underline' }}
                                >
                                  {citation.source_title}
                                </a>
                              </li>
                            ))}
                          </ul>
                        </div>
                      ))}
                      {message.confidence !== undefined && (
                        <div className={styles.confidenceScore}>
                          Confidence: {(message.confidence * 100).toFixed(1)}%
                        </div>
                      )}
                      {message.text.includes('Sorry, I encountered an error') && message.sender === 'bot' && (
                        <div style={{ marginTop: '8px', textAlign: 'right' }}>
                          <button
                            onClick={() => handleRetry(message.id)}
                            className={styles.sendButton}
                            style={{ padding: '4px 8px', fontSize: '12px', width: 'auto', height: 'auto' }}
                          >
                            Retry
                          </button>
                        </div>
                      )}
                    </div>
                  ))
                )}
                {isLoading && (
                  <div className={clsx(styles.message, styles.botMessage)}>
                    <div className={styles.loadingIndicator}>
                      <div className={styles.loadingDots}>
                        <span></span>
                        <span></span>
                        <span></span>
                      </div>
                    </div>
                  </div>
                )}
                {error && (
                  <div className={clsx(styles.message, styles.botMessage)}>
                    Error: {error}
                  </div>
                )}
                <div ref={messagesEndRef} />
              </div>
            )}

            {/* Input area */}
            {!showHistory && (
              <div className={styles.chatInputArea}>
                <textarea
                  ref={chatInputRef}
                  className={styles.chatInput}
                  value={inputValue}
                  onChange={(e) => setInputValue(e.target.value)}
                  onKeyPress={handleKeyPress}
                  placeholder={selectedText
                    ? `Ask about: "${selectedText.substring(0, 30)}${selectedText.length > 30 ? '...' : ''}"`
                    : "Ask about the textbook content..."}
                  rows={1}
                  disabled={isLoading}
                />
                <button
                  className={styles.sendButton}
                  onClick={handleSendMessage}
                  disabled={isLoading || (!inputValue.trim() && !selectedText)}
                  aria-label="Send message"
                >
                  <span>➤</span>
                </button>
              </div>
            )}
          </div>
        </div>
      )}
    </>
  );
};

export default Chatbot;