// API service module for backend communication
import { QueryRequest, QueryResponse } from './types';

const BACKEND_URL = process.env.REACT_APP_BACKEND_URL || 'https://osamabinadnan-rag-with-neondb.hf.space';

// Define types for chat history
export interface ChatSession {
  id: string;
  user_id: string;
  session_id: string;
  title: string;
  created_at: string;
  updated_at: string;
}

export interface ChatMessage {
  id: string;
  session_id: string;
  user_id: string;
  role: string;
  content: string;
  token_count: number | null;
  created_at: string;
}

export interface ChatHistoryResponse {
  session: ChatSession;
  messages: ChatMessage[];
}

export interface UserChatHistoryResponse {
  sessions: ChatSession[];
}

/**
 * Send a message to the backend RAG agent
 * @param request - The query request containing the question and optional parameters
 * @returns Promise resolving to the query response from the backend
 */
export const sendMessage = async (request: QueryRequest): Promise<QueryResponse> => {
  try {
    // Get auth token if available
    const authToken = localStorage.getItem('authToken');

    const headers: Record<string, string> = {
      'Content-Type': 'application/json',
    };

    if (authToken) {
      headers['Authorization'] = `Bearer ${authToken}`;
    }

    const response = await fetch(`${BACKEND_URL}/chat`, {
      method: 'POST',
      headers: headers,
      body: JSON.stringify(request),
    });

    if (!response.ok) {
      throw new Error(`Backend error: ${response.status} ${response.statusText}`);
    }

    const data: QueryResponse = await response.json();
    return data;
  } catch (error) {
    console.error('Error communicating with backend:', error);
    throw error;
  }
};

/**
 * Check the health status of the backend
 * @returns Promise resolving to the health check response
 */
export const healthCheck = async (): Promise<any> => {
  try {
    // Get auth token if available
    const authToken = localStorage.getItem('authToken');

    const headers: Record<string, string> = {
      'Content-Type': 'application/json',
    };

    if (authToken) {
      headers['Authorization'] = `Bearer ${authToken}`;
    }

    const response = await fetch(`${BACKEND_URL}/health`, {
      headers: headers,
    });

    if (!response.ok) {
      throw new Error(`Health check failed: ${response.status} ${response.statusText}`);
    }

    const data = await response.json();
    return data;
  } catch (error) {
    console.error('Health check failed:', error);
    throw error;
  }
};

/**
 * Get all chat sessions for a specific user
 * @param userId - The user ID to fetch chat history for
 * @returns Promise resolving to the user's chat sessions
 */
export const getUserChatHistory = async (userId: string): Promise<UserChatHistoryResponse> => {
  try {
    // Get auth token if available
    const authToken = localStorage.getItem('authToken');

    const headers: Record<string, string> = {
      'Content-Type': 'application/json',
    };

    if (authToken) {
      headers['Authorization'] = `Bearer ${authToken}`;
    }

    const response = await fetch(`${BACKEND_URL}/chat-history/${userId}`, {
      headers: headers,
    });

    if (!response.ok) {
      throw new Error(`Failed to fetch chat history: ${response.status} ${response.statusText}`);
    }

    const rawData = await response.json();
    console.log('Raw data from chat history API:', rawData);

    // Handle different possible response structures
    let data: UserChatHistoryResponse;

    // If rawData already has the expected structure
    if (rawData && typeof rawData === 'object' && Array.isArray(rawData.sessions)) {
      data = rawData as UserChatHistoryResponse;
    } else if (Array.isArray(rawData)) {
      // If the response is directly an array of sessions
      data = { sessions: rawData };
    } else {
      // If the response structure is unexpected, create a default structure
      console.warn('Unexpected chat history response structure, using empty sessions');
      data = { sessions: [] };
    }

    console.log('Processed chat history data:', data);
    return data;
  } catch (error) {
    console.error('Error fetching user chat history:', error);
    throw error;
  }
};

/**
 * Get all messages for a specific chat session
 * @param userId - The user ID
 * @param sessionId - The session ID to fetch messages for
 * @returns Promise resolving to the session's messages
 */
export const getSessionHistory = async (userId: string, sessionId: string): Promise<ChatHistoryResponse> => {
  try {
    // Get auth token if available
    const authToken = localStorage.getItem('authToken');

    const headers: Record<string, string> = {
      'Content-Type': 'application/json',
    };

    if (authToken) {
      headers['Authorization'] = `Bearer ${authToken}`;
    }

    const response = await fetch(`${BACKEND_URL}/chat-history/${userId}/session/${sessionId}`, {
      headers: headers,
    });

    if (!response.ok) {
      throw new Error(`Failed to fetch session history: ${response.status} ${response.statusText}`);
    }

    const data: ChatHistoryResponse = await response.json();
    return data;
  } catch (error) {
    console.error('Error fetching session history:', error);
    throw error;
  }
};