// API service module for backend communication
import { QueryRequest, QueryResponse } from './types';

const BACKEND_URL = process.env.REACT_APP_BACKEND_URL || 'http://localhost:8000';

/**
 * Send a message to the backend RAG agent
 * @param request - The query request containing the question and optional parameters
 * @returns Promise resolving to the query response from the backend
 */
export const sendMessage = async (request: QueryRequest): Promise<QueryResponse> => {
  try {
    const response = await fetch(`${BACKEND_URL}/chat`, {
      method: 'POST',
      headers: {
        'Content-Type': 'application/json',
      },
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
    const response = await fetch(`${BACKEND_URL}/health`);

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