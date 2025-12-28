import React, { useState, useEffect } from 'react';
import Layout from '@theme/Layout';
import Link from '@docusaurus/Link';
import { useLocation } from '@docusaurus/router';

function ResetPasswordPage() {
  const [password, setPassword] = useState('');
  const [confirmPassword, setConfirmPassword] = useState('');
  const [error, setError] = useState('');
  const [success, setSuccess] = useState(false);
  const [isLoading, setIsLoading] = useState(false);
  const [token, setToken] = useState('');
  const location = useLocation();

  // Extract token from URL query parameters
  useEffect(() => {
    const searchParams = new URLSearchParams(location.search);
    const tokenParam = searchParams.get('token');
    if (tokenParam) {
      setToken(tokenParam);
    } else {
      setError('Invalid reset token. Please try resetting your password again.');
    }
  }, [location]);

  const handleSubmit = async (e: React.FormEvent) => {
    e.preventDefault();
    setError('');

    if (password !== confirmPassword) {
      setError('Passwords do not match');
      return;
    }

    // Validate password requirements
    if (password.length < 8) {
      setError('Password must be at least 8 characters long');
      return;
    }

    if (!/(?=.*[a-z])(?=.*[A-Z])(?=.*\d)(?=.*[@$!%*?&])[A-Za-z\d@$!%*?&]/.test(password)) {
      setError('Password must contain at least one uppercase letter, one lowercase letter, one number, and one special character');
      return;
    }

    setIsLoading(true);

    try {
      // In a real implementation, you would call your backend API here
      // For now, we'll simulate the API call
      await new Promise(resolve => setTimeout(resolve, 1000)); // Simulate API call

      // In a real implementation, this would be your backend API call
      // const response = await fetch('/api/auth/reset-password', {
      //   method: 'POST',
      //   headers: {
      //     'Content-Type': 'application/json',
      //   },
      //   body: JSON.stringify({ token, password }),
      // });

      // if (!response.ok) {
      //   throw new Error('Failed to reset password');
      // }

      setSuccess(true);
    } catch (err) {
      setError(err instanceof Error ? err.message : 'An error occurred while resetting your password');
    } finally {
      setIsLoading(false);
    }
  };

  return (
    <Layout title="Reset Password" description="Create a new password">
      <div className="auth-page">
        <div className="auth-content">
          {success ? (
            <div className="auth-form-container">
              <h2>Password Reset Successful</h2>
              <p>Your password has been successfully reset.</p>
              <p>You can now sign in with your new password.</p>
              <div className="auth-switch">
                <Link to="/signin" className="switch-button">Go to Sign In</Link>
              </div>
            </div>
          ) : (
            <div className="auth-form-container">
              <h2>Reset Your Password</h2>
              <p>Create a new password for your account.</p>

              {error && <div className="error-message">{error}</div>}

              <form onSubmit={handleSubmit} className="auth-form">
                <div className="form-group">
                  <label htmlFor="password">New Password</label>
                  <input
                    type="password"
                    id="password"
                    value={password}
                    onChange={(e) => setPassword(e.target.value)}
                    onKeyDown={(e) => e.stopPropagation()} // Prevent search plugin from intercepting keyboard events
                    required
                    className="form-input"
                    placeholder="Enter new password"
                  />
                </div>

                <div className="form-group">
                  <label htmlFor="confirm-password">Confirm Password</label>
                  <input
                    type="password"
                    id="confirm-password"
                    value={confirmPassword}
                    onChange={(e) => setConfirmPassword(e.target.value)}
                    onKeyDown={(e) => e.stopPropagation()} // Prevent search plugin from intercepting keyboard events
                    required
                    className="form-input"
                    placeholder="Confirm new password"
                  />
                </div>

                <button
                  type="submit"
                  disabled={isLoading}
                  className="auth-button"
                >
                  {isLoading ? 'Resetting...' : 'Reset Password'}
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

export default ResetPasswordPage;