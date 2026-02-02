import React, { useState } from 'react';
import Layout from '@theme/Layout';
import Link from '@docusaurus/Link';

function ForgotPasswordPage() {
  const [email, setEmail] = useState('');
  const [submitted, setSubmitted] = useState(false);
  const [error, setError] = useState('');
  const [isLoading, setIsLoading] = useState(false);

  const handleSubmit = async (e: React.FormEvent) => {
    e.preventDefault();
    setError('');
    setIsLoading(true);

    try {
      // In a real implementation, you would call your backend API here
      // For now, we'll simulate the API call
      await new Promise(resolve => setTimeout(resolve, 1000)); // Simulate API call

      // In a real implementation, this would be your backend API call
      // const response = await fetch('/api/auth/forgot-password', {
      //   method: 'POST',
      //   headers: {
      //     'Content-Type': 'application/json',
      //   },
      //   body: JSON.stringify({ email }),
      // });

      // if (!response.ok) {
      //   throw new Error('Failed to send password reset email');
      // }

      setSubmitted(true);
    } catch (err) {
      setError(err instanceof Error ? err.message : 'An error occurred while sending the reset email');
    } finally {
      setIsLoading(false);
    }
  };

  return (
    <Layout title="Forgot Password" description="Reset your password">
      <div className="auth-page">
        <div className="auth-content">
          {submitted ? (
            <div className="auth-form-container">
              <h2>Password Reset Email Sent</h2>
              <p>
                If an account exists for <strong>{email}</strong>, you will receive a password reset link shortly.
              </p>
              <p>
                Please check your email and follow the instructions to reset your password.
              </p>
              <div className="auth-switch">
                <Link to="/signin" className="switch-button">Back to Sign In</Link>
              </div>
            </div>
          ) : (
            <div className="auth-form-container">
              <h2>Forgot Your Password?</h2>
              <p>Enter your email address and we'll send you a link to reset your password.</p>

              {error && <div className="error-message">{error}</div>}

              <form onSubmit={handleSubmit} className="auth-form">
                <div className="form-group">
                  <label htmlFor="email">Email</label>
                  <input
                    type="email"
                    id="email"
                    value={email}
                    onChange={(e) => setEmail(e.target.value)}
                    onKeyDown={(e) => e.stopPropagation()} // Prevent search plugin from intercepting keyboard events
                    required
                    className="form-input"
                    placeholder="Enter your email"
                  />
                </div>

                <button
                  type="submit"
                  disabled={isLoading}
                  className="auth-button"
                >
                  {isLoading ? 'Sending...' : 'Send Reset Link'}
                </button>
              </form>

              <div className="auth-switch">
                <Link to="/signin" className="switch-button">Back to Sign In</Link>
              </div>
            </div>
          )}
        </div>
      </div>
    </Layout>
  );
}

export default ForgotPasswordPage;