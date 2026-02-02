import React from 'react';
import { useAuth } from '../../components/Auth/AuthContext';
import OriginalNavbar from '@theme-original/Navbar';

const Navbar = (props) => {
  const { user, isAuthenticated, logout } = useAuth();

  // Create navbar items based on auth status
  const getAuthItems = () => {
    if (isAuthenticated && user) {
      // User is authenticated - show user dropdown
      return [
        {
          type: 'dropdown',
          label: `${user.name || user.email.split('@')[0]} ▼`,
          position: 'right' as const,
          items: [
            {
              label: 'Profile',
              to: '/profile',
            },
            {
              label: 'Sign Out',
              to: '#',
              onClick: async (e) => {
                e.preventDefault();
                try {
                  await logout();
                  // Reload page to update navbar state
                  window.location.reload();
                } catch (error) {
                  console.error('Logout error:', error);
                }
              }
            }
          ]
        }
      ];
    } else {
      // User is not authenticated - show sign in/up links
      return [
        {
          to: '/signin',
          label: 'Sign In',
          position: 'right' as const,
        },
        {
          to: '/signup',
          label: 'Sign Up',
          position: 'right' as const,
        }
      ];
    }
  };

  // Get original navbar items (excluding any existing auth links)
  const originalItems = props.items || [];
  const filteredItems = originalItems.filter(item =>
    !(item && item.to && (item.to === '/signin' || item.to === '/signup')) &&
    !(item && item.label && (item.label === 'Sign In' || item.label === 'Sign Up'))
  );

  // Combine original items with auth items
  const updatedItems = [...filteredItems, ...getAuthItems()];

  // Return the original navbar with updated items
  return <OriginalNavbar {...props} items={updatedItems} />;
};

export default Navbar;