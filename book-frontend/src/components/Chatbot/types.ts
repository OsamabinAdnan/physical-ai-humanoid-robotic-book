// TypeScript interfaces matching the backend schemas

export interface Citation {
  text: string;
  url: string;
  similarity_score: number;
  chunk_id: number;
  source_title: string;
}

export interface QueryRequest {
  question: string;
  top_k?: number;
  selected_text?: string;
}

export interface QueryResponse {
  answer: string;
  citations: Citation[];
  confidence: number;
  processing_time_ms: number;
}

// Define message interface
export interface Message {
  id: string;
  text: string;
  sender: 'user' | 'bot';
  timestamp: Date;
  citations?: Citation[];
  confidence?: number;
}