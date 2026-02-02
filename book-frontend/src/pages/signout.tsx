import React, { useEffect } from 'react';
import { useAuth } from '../components/Auth/AuthContext';
import { useHistory } from '@docusaurus/router';

const SignOutPage = () => {
  const { logout, isAuthenticated } = useAuth();
  const history = useHistory();

  useEffect(() => {
    const handleLogout = async () => {
      if (isAuthenticated) {
        await logout();
      }
      // Redirect to home page after logout
      history.replace('/');
    };

    handleLogout();
  }, [logout, isAuthenticated, history]);

  return (
    <div style={{
      display: 'flex',
      justifyContent: 'center',
      alignItems: 'center',
      height: '100vh',
      fontSize: '18px'
    }}>
      <p>Signing out...</p>
    </div>
  );
};

export default SignOutPage;