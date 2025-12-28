import React, { useState, useEffect, useRef } from 'react';
import { useAuth } from './AuthContext';
import Link from '@docusaurus/Link';
import ReactMarkdown from 'react-markdown';
import remarkGfm from 'remark-gfm';
import LoadingSpinner from '../Loading/LoadingSpinner';

interface PersonalizationButtonProps {
  chapterUrl: string;
  chapterContent: string;
}

const PersonalizationButton: React.FC<PersonalizationButtonProps> = ({ chapterUrl, chapterContent }) => {
  const { isAuthenticated } = useAuth();
  const [isPersonalizing, setIsPersonalizing] = useState(false);
  const [personalizedContent, setPersonalizedContent] = useState<string | null>(null);
  const abortControllerRef = React.useRef<AbortController | null>(null);
  const [showOriginal, setShowOriginal] = useState(true);
  const [error, setError] = useState<string | null>(null);

  // Reset when chapter changes - ensure we don't show old personalized content for new chapter
  useEffect(() => {
    setPersonalizedContent(null);
    setShowOriginal(true);
    setError(null);
  }, [chapterUrl]);

  const handlePersonalize = async () => {
    if (!isAuthenticated) {
      alert('Please log in to access content personalization');
      return;
    }

    setIsPersonalizing(true);
    setError(null);
    // Clear the previous personalized content to ensure we're loading fresh content for current page
    setPersonalizedContent(null);

    // Create an AbortController to allow cancellation
    const abortController = new AbortController();
    abortControllerRef.current = abortController;

    try {
      // Get the token from the auth context instead of localStorage
      const token = localStorage.getItem('authToken');
      if (!token) {
        throw new Error('Authentication token not found');
      }

      const BACKEND_URL = process.env.NODE_ENV === 'production'
        ? 'https://osamabinadnan-rag-with-neondb.hf.space'
        : (process.env.REACT_APP_BACKEND_URL || 'http://127.0.0.1:8000');
      const response = await fetch(`${BACKEND_URL}/personalize`, {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
          'Authorization': `Bearer ${token}`,
        },
        body: JSON.stringify({
          chapter_url: chapterUrl,
          chapter_content: chapterContent,
        }),
        signal: abortController.signal, // Add signal for cancellation
      });

      if (!response.ok) {
        // Check if the request was cancelled
        if (abortController.signal.aborted) {
          // Request was cancelled, don't show an error
          return;
        }
        throw new Error(`Personalization failed: ${response.status} ${response.statusText}`);
      }

      const data = await response.json();
      // Only update state if not cancelled
      if (!abortController.signal.aborted) {
        setPersonalizedContent(data.personalized_summary);
        setShowOriginal(false);
      }
    } catch (err) {
      // Check if the error was due to cancellation
      if (err instanceof Error && err.name === 'AbortError') {
        // Request was cancelled, don't show an error
        return;
      }
      console.error('Personalization error:', err);
      setError(err instanceof Error ? err.message : 'An error occurred during personalization');
    } finally {
      // Only update state if not cancelled
      if (!abortController.signal.aborted) {
        setIsPersonalizing(false);
      }
      // Clean up the ref
      if (abortControllerRef.current === abortController) {
        abortControllerRef.current = null;
      }
    }
  };

  const toggleContent = () => {
    setShowOriginal(!showOriginal);
  };

  return (
    <div style={{
      position: 'fixed',
      bottom: '20px', // Same height as chatbot (which is 20px from bottom)
      left: '20px', // On the left side, opposite to chatbot
      zIndex: '99998',
    }}>
      {isAuthenticated ? (
        <>
          {isPersonalizing && (
            <div
              style={{
                position: 'fixed',
                top: 0,
                left: 0,
                width: '100vw',
                height: '100vh',
                backgroundColor: 'rgba(0, 0, 0, 0.7)',
                display: 'flex',
                justifyContent: 'center',
                alignItems: 'center',
                zIndex: '100002',
              }}
              className="personalization-loading-overlay"
              onClick={() => {
                // Allow user to cancel by clicking outside
                if (abortControllerRef.current) {
                  abortControllerRef.current.abort();
                  abortControllerRef.current = null;
                }
                setIsPersonalizing(false);
                setPersonalizedContent(null);
              }}
              onKeyDown={(e) => {
                if (e.key === 'Escape') {
                  // Allow user to cancel with ESC key
                  if (abortControllerRef.current) {
                    abortControllerRef.current.abort();
                    abortControllerRef.current = null;
                  }
                  setIsPersonalizing(false);
                  setPersonalizedContent(null);
                }
              }}
              tabIndex={0} // Make div focusable to handle key events
            >
              <div style={{
                width: '40vw',
                height: '40vh',
                backgroundColor: 'white',
                borderRadius: '12px',
                display: 'flex',
                flexDirection: 'column',
                justifyContent: 'center',
                alignItems: 'center',
                gap: '1.5rem',
                padding: '2rem',
                boxShadow: '0 20px 25px -5px rgba(0, 0, 0, 0.1), 0 10px 10px -5px rgba(0, 0, 0, 0.04)',
                position: 'relative',
              }}
              onClick={(e) => e.stopPropagation()} // Prevent click from bubbling to overlay
              >
                <div style={{
                  width: '60px',
                  height: '60px',
                  border: '4px solid #e0e0e0',
                  borderTop: '4px solid var(--ifm-color-primary)',
                  borderRadius: '50%',
                  animation: 'spin 1s linear infinite',
                }} />
                <div style={{
                  textAlign: 'center',
                }}>
                  <h3 style={{
                    margin: '0 0 0.5rem 0',
                    color: 'var(--ifm-font-color-base)',
                    fontSize: '1.5rem',
                  }}>
                    Personalizing Content...
                  </h3>
                  <p style={{
                    margin: '0.5rem 0',
                    color: 'var(--ifm-font-color-base)',
                    textAlign: 'center',
                  }}>
                    Please wait while we customize the content based on your expertise level.
                  </p>
                  <p style={{
                    margin: '0.5rem 0 0 0',
                    color: '#666',
                    fontSize: '0.875rem',
                    fontStyle: 'italic',
                  }}>
                    Press ESC or click outside to cancel
                  </p>
                </div>
              </div>
            </div>
          )}
          <button
            onClick={personalizedContent ? toggleContent : handlePersonalize}
            disabled={isPersonalizing}
            style={{
              background: isPersonalizing ? 'linear-gradient(135deg, #4F46E5 0%, #9333EA 100%)' : 'var(--ifm-color-primary)',
              color: 'white',
              border: 'none',
              borderRadius: '30px', // More rounded
              padding: '14px 24px',
              fontSize: '15px',
              fontWeight: '600',
              cursor: isPersonalizing ? 'not-allowed' : 'pointer',
              boxShadow: isPersonalizing
                ? '0 6px 12px rgba(79, 70, 229, 0.3), 0 2px 4px rgba(0, 0, 0, 0.1)'
                : '0 6px 12px rgba(79, 70, 229, 0.3), 0 2px 4px rgba(0, 0, 0, 0.1)',
              display: 'flex',
              alignItems: 'center',
              gap: '10px',
              transition: 'all 0.3s ease', // Smooth transitions
              transform: 'scale(1)',
            }}
            onMouseEnter={(e) => {
              if (!isPersonalizing) {
                (e.target as HTMLElement).style.transform = 'scale(1.05)';
                (e.target as HTMLElement).style.boxShadow = '0 8px 16px rgba(79, 70, 229, 0.4), 0 4px 8px rgba(0, 0, 0, 0.15)';
              }
            }}
            onMouseLeave={(e) => {
              if (!isPersonalizing) {
                (e.target as HTMLElement).style.transform = 'scale(1)';
                (e.target as HTMLElement).style.boxShadow = '0 6px 12px rgba(79, 70, 229, 0.3), 0 2px 4px rgba(0, 0, 0, 0.1)';
              }
            }}
          >
            {isPersonalizing ? (
              <>
                <span style={{
                  display: 'inline-block',
                  width: '14px',
                  height: '14px',
                  borderRadius: '50%',
                  backgroundColor: 'currentColor',
                  animation: 'loading 1.4s infinite ease-in-out both'
                }} />
                <span>Personalizing...</span>
              </>
            ) : personalizedContent && !showOriginal ? (
              <>
                <span>🔄</span> {/* Refresh icon */}
                <span>Show Original</span>
              </>
            ) : (
              <>
                <span>✨</span> {/* Sparkle icon */}
                <span>Personalize Content</span>
              </>
            )}
          </button>
        </>
      ) : (
        <Link
          to="/signin"
          style={{
            backgroundColor: 'var(--ifm-color-primary)',
            color: 'white',
            border: 'none',
            borderRadius: '30px',
            padding: '14px 24px',
            fontSize: '15px',
            fontWeight: '600',
            cursor: 'pointer',
            boxShadow: '0 6px 12px rgba(79, 70, 229, 0.3), 0 2px 4px rgba(0, 0, 0, 0.1)',
            display: 'flex',
            alignItems: 'center',
            gap: '10px',
            transition: 'all 0.3s ease',
            textDecoration: 'none', // Remove default link styling
          }}
          onMouseEnter={(e) => {
            (e.target as HTMLElement).style.transform = 'scale(1.05)';
            (e.target as HTMLElement).style.boxShadow = '0 8px 16px rgba(79, 70, 229, 0.4), 0 4px 8px rgba(0, 0, 0, 0.15)';
          }}
          onMouseLeave={(e) => {
            (e.target as HTMLElement).style.transform = 'scale(1)';
            (e.target as HTMLElement).style.boxShadow = '0 6px 12px rgba(79, 70, 229, 0.3), 0 2px 4px rgba(0, 0, 0, 0.1)';
          }}
        >
          <span>🔒</span> {/* Lock icon */}
          <span>Sign In to Personalize</span>
        </Link>
      )}

      {/* Add the loading animation style to the document */}
      <style>{`
        @keyframes loading {
          0%, 80%, 100% { transform: scale(0); }
          40% { transform: scale(1); }
        }

        @keyframes spin {
          0% { transform: rotate(0deg); }
          100% { transform: rotate(360deg); }
        }

        /* Loading overlay styles */
        .personalization-loading-overlay {
          background-color: rgba(0, 0, 0, 0.7) !important;
        }

        [data-theme="dark"] .personalization-loading-overlay {
          background-color: rgba(0, 0, 0, 0.7) !important;
        }

        [data-theme="dark"] .personalization-loading-overlay > div {
          background-color: #1e1e2e !important;
          color: white !important;
        }

        [data-theme="dark"] .personalization-loading-overlay h3 {
          color: white !important;
        }

        [data-theme="dark"] .personalization-loading-overlay p {
          color: #e6e6e6 !important;
        }

        /* Glassmorphism styles for light and dark themes */
        [data-theme="light"] .personalization-modal {
          background: rgba(255, 255, 255, 0.85);
          backdrop-filter: blur(10px);
          -webkit-backdrop-filter: blur(10px);
          border: 1px solid rgba(255, 255, 255, 0.5);
        }

        [data-theme="light"] .personalization-modal-header {
          background: rgba(255, 255, 255, 0.7);
          border-bottom: 1px solid rgba(0, 0, 0, 0.1);
        }

        [data-theme="light"] .toggle-button {
          background: rgba(0, 0, 0, 0.05);
          border: 1px solid rgba(0, 0, 0, 0.1);
          color: #000;
        }

        [data-theme="dark"] .personalization-modal {
          background: rgba(30, 30, 30, 0.85);
          backdrop-filter: blur(10px);
          -webkit-backdrop-filter: blur(10px);
          border: 1px solid rgba(255, 255, 255, 0.1);
        }

        [data-theme="dark"] .personalization-modal-header {
          background: rgba(30, 30, 30, 0.7);
          border-bottom: 1px solid rgba(255, 255, 255, 0.1);
        }

        [data-theme="dark"] .toggle-button {
          background: rgba(255, 255, 255, 0.1);
          border: 1px solid rgba(255, 255, 255, 0.2);
          color: #fff;
        }

        /* Markdown content styling */
        .personalization-content h1,
        .personalization-content h2,
        .personalization-content h3,
        .personalization-content h4,
        .personalization-content h5,
        .personalization-content h6 {
          margin-top: 1rem;
          margin-bottom: 0.5rem;
          font-weight: 600;
          line-height: 1.3;
        }

        .personalization-content p {
          margin-bottom: 1rem;
          line-height: 1.6;
        }

        .personalization-content ul,
        .personalization-content ol {
          margin-bottom: 1rem;
          padding-left: 1.5rem;
        }

        .personalization-content li {
          margin-bottom: 0.25rem;
        }

        .personalization-content code {
          background-color: rgba(0, 0, 0, 0.05);
          padding: 0.2rem 0.4rem;
          border-radius: 4px;
          font-family: monospace;
          font-size: 0.9em;
        }

        .personalization-content pre {
          background-color: rgba(0, 0, 0, 0.05);
          padding: 1rem;
          border-radius: 6px;
          overflow-x: auto;
          margin-bottom: 1rem;
          border: 1px solid rgba(0, 0, 0, 0.1);
        }

        .personalization-content pre code {
          background: none;
          padding: 0;
          display: block;
          line-height: 1.5;
        }

        .personalization-content blockquote {
          border-left: 3px solid rgba(0, 0, 0, 0.2);
          padding-left: 1rem;
          margin: 1rem 0;
          color: rgba(0, 0, 0, 0.7);
          background-color: rgba(0, 0, 0, 0.02);
          padding: 1rem 1rem 1rem 1.5rem;
          border-radius: 0 4px 4px 0;
        }

        .personalization-content a {
          color: var(--ifm-color-primary);
          text-decoration: underline;
        }

        .personalization-content strong {
          font-weight: 600;
        }

        .personalization-content em {
          font-style: italic;
        }

        .personalization-content hr {
          border: none;
          height: 1px;
          background-color: rgba(0, 0, 0, 0.1);
          margin: 1.5rem 0;
        }

        .personalization-content table {
          border-collapse: collapse;
          width: 100%;
          margin-bottom: 1rem;
        }

        .personalization-content table th,
        .personalization-content table td {
          padding: 0.5rem;
          border: 1px solid rgba(0, 0, 0, 0.1);
        }

        .personalization-content table th {
          background-color: rgba(0, 0, 0, 0.05);
          font-weight: 600;
        }

        /* Dark theme specific styles for markdown */
        [data-theme="dark"] .personalization-content code {
          background-color: rgba(255, 255, 255, 0.1);
        }

        [data-theme="dark"] .personalization-content pre {
          background-color: rgba(255, 255, 255, 0.1);
          border-color: rgba(255, 255, 255, 0.15);
        }

        [data-theme="dark"] .personalization-content blockquote {
          border-left: 3px solid rgba(255, 255, 255, 0.3);
          color: rgba(255, 255, 255, 0.8);
          background-color: rgba(255, 255, 255, 0.05);
        }

        [data-theme="dark"] .personalization-content table th {
          background-color: rgba(255, 255, 255, 0.1);
        }
      `}</style>

      {error && (
        <div style={{
          position: 'absolute',
          top: 'calc(100% + 8px)',
          left: '0',
          width: '300px',
          padding: '12px',
          backgroundColor: 'var(--ifm-color-danger-background)',
          color: 'var(--ifm-color-danger)',
          border: '1px solid var(--ifm-color-danger-border)',
          borderRadius: '8px',
          fontSize: '13px',
          boxShadow: '0 4px 6px rgba(0, 0, 0, 0.1)',
        }}>
          <div style={{ fontWeight: '500', marginBottom: '4px', color: 'var(--ifm-color-danger)' }}>Error:</div>
          <div>{error}</div>
        </div>
      )}

      {/* Modal backdrop */}
      {personalizedContent && (
        <div style={{
          position: 'fixed',
          top: 0,
          left: 0,
          width: '100vw',
          height: '100vh',
          backgroundColor: 'rgba(0, 0, 0, 0.6)',
          display: 'flex',
          justifyContent: 'center',
          alignItems: 'center',
          zIndex: '100000',
        }}
        onClick={() => setShowOriginal(true)} // Close modal when clicking on backdrop
        >
          {/* Modal content */}
          <div
            className="personalization-modal"
            style={{
              borderRadius: '12px',
              width: '80%',
              maxWidth: '800px',
              maxHeight: '80vh',
              overflow: 'hidden',
              boxShadow: '0 20px 25px -5px rgba(0, 0, 0, 0.1), 0 10px 10px -5px rgba(0, 0, 0, 0.04)',
              display: 'flex',
              flexDirection: 'column',
            }}
            onClick={(e) => e.stopPropagation()} // Prevent closing when clicking inside modal
          >
            {/* Modal header */}
            <div className="personalization-modal-header"
              style={{
                padding: '20px',
                display: 'flex',
                justifyContent: 'space-between',
                alignItems: 'center',
              }}>
              <h3 style={{
                margin: '0',
                color: 'var(--ifm-heading-color)',
                fontSize: '18px',
                fontWeight: '600'
              }}>
                Personalized Content
              </h3>
              <div style={{ display: 'flex', gap: '10px' }}>
                <button
                  className="toggle-button"
                  onClick={() => setShowOriginal(!showOriginal)}
                  style={{
                    borderRadius: '6px',
                    padding: '8px 12px',
                    fontSize: '14px',
                    cursor: 'pointer',
                    fontWeight: '500'
                  }}
                >
                  {showOriginal ? 'Show Personalized' : 'Show Original'}
                </button>
                <button
                  onClick={() => {
                    setShowOriginal(true);
                    setPersonalizedContent(null);
                  }}
                  style={{
                    backgroundColor: '#EF4444',
                    border: 'none',
                    borderRadius: '6px',
                    padding: '8px 12px',
                    fontSize: '14px',
                    cursor: 'pointer',
                    color: 'white',
                    fontWeight: '500'
                  }}
                >
                  Close
                </button>
              </div>
            </div>

            {/* Modal body */}
            <div style={{
              padding: '20px',
              overflowY: 'auto',
              maxHeight: 'calc(80vh - 120px)',
            }}>
              <div className="personalization-content" style={{
                color: 'var(--ifm-font-color-base)',
                lineHeight: '1.6',
                fontSize: '16px'
              }}>
                {showOriginal
                  ? (
                    <div>
                      <h4 style={{ color: 'var(--ifm-heading-color)', marginBottom: '12px' }}>Original Content</h4>
                      <ReactMarkdown remarkPlugins={[remarkGfm]}>{chapterContent}</ReactMarkdown>
                    </div>
                  )
                  : (
                    <div>
                      <h4 style={{ color: 'var(--ifm-heading-color)', marginBottom: '12px' }}>Personalized Content</h4>
                      <ReactMarkdown remarkPlugins={[remarkGfm]}>{personalizedContent}</ReactMarkdown>
                    </div>
                  )
                }
              </div>
            </div>
          </div>
        </div>
      )}
    </div>
  );
};

export default PersonalizationButton;