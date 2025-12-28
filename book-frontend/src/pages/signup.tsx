import React, { useEffect, useState } from 'react';
import Layout from '@theme/Layout';
import RegisterForm from '../components/Auth/RegisterForm';
import Link from '@docusaurus/Link';

function SignupPage() {
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
    <Layout title="Sign Up" description="Create an account to access personalized features">
      <div className="auth-page">
        <div className="auth-content">
          <h1 className="auth-title">Sign up for an account</h1>
          <RegisterForm onRegisterSuccess={() => window.location.href = '/physical-ai-humanoid-robotic-book/'} />
        </div>
      </div>
    </Layout>
  );
}

export default SignupPage;