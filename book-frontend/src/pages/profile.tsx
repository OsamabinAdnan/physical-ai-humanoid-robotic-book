import React from 'react';
import { useAuth } from '../components/Auth/AuthContext';
import Layout from '@theme/Layout';

const ProfilePage = () => {
  const { user, isAuthenticated, logout } = useAuth();

  if (!isAuthenticated || !user) {
    return (
      <Layout title="Profile" description="User profile page">
        <div style={{ padding: '2rem' }}>
          <h1>Please Sign In</h1>
          <p>You need to be signed in to view your profile.</p>
          <a href="/physical-ai-humanoid-robotic-book/signin" className="button button--primary">Sign In</a>
        </div>
      </Layout>
    );
  }

  return (
    <Layout title="Profile" description="User profile page">
      <div style={{ padding: '2rem' }}>
        <div style={{ display: 'flex', justifyContent: 'space-between', alignItems: 'center', marginBottom: '2rem' }}>
          <h1>User Profile</h1>
          <button
            onClick={async () => {
              await logout();
              window.location.href = '/physical-ai-humanoid-robotic-book/';
            }}
            style={{
              backgroundColor: '#ef4444',
              color: 'white',
              border: 'none',
              padding: '8px 16px',
              borderRadius: '4px',
              cursor: 'pointer',
              fontSize: '14px'
            }}
          >
            Sign Out
          </button>
        </div>
        <div style={{
          display: 'grid',
          gridTemplateColumns: 'repeat(auto-fit, minmax(250px, 1fr))',
          gap: '1rem',
          marginTop: '1rem'
        }}>
          <div>
            <strong>Name:</strong> {user.name || 'Not provided'}
          </div>
          <div>
            <strong>Email:</strong> {user.email}
          </div>
          <div>
            <strong>Software Background:</strong> {user.software_background}
          </div>
          <div>
            <strong>Hardware Background:</strong> {user.hardware_background}
          </div>
          <div>
            <strong>Account Created:</strong> {user.created_at ? new Date(user.created_at).toLocaleDateString() : 'Unknown'}
          </div>
        </div>
      </div>
    </Layout>
  );
};

export default ProfilePage;