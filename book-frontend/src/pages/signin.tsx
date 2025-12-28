import React, { useEffect, useState } from 'react';
import Layout from '@theme/Layout';
import LoginForm from '../components/Auth/LoginForm';
import Link from '@docusaurus/Link';

function SigninPage() {
  const [isAuthenticated, setIsAuthenticated] = useState(false);

  // Check if user is already authenticated
  useEffect(() => {
    const token = localStorage.getItem('authToken');
    const user = localStorage.getItem('user');
    if (token && user) {
      setIsAuthenticated(true);
    }
  }, []);

  if (isAuthenticated) {
    return (
      <Layout title="Already Signed In" description="You are already signed in">
        <div className="auth-page">
          <div className="auth-already-signed-in">
            <h1>You're Already Signed In</h1>
            <p>You are currently signed in to your account.</p>
            <Link to="/physical-ai-humanoid-robotic-book/" className="button button--primary">
              Go to Home
            </Link>
          </div>
        </div>
      </Layout>
    );
  }

  return (
    <Layout title="Sign In" description="Sign in to your account to access personalized features">
      <div className="auth-page">
        <div className="auth-content">
          <h1 className="auth-title">Sign in to your account</h1>
          <LoginForm onLoginSuccess={() => window.location.href = '/physical-ai-humanoid-robotic-book/'} />
        </div>
      </div>
    </Layout>
  );
}

export default SigninPage;