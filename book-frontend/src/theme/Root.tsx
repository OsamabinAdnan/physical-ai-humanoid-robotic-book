import React from 'react';
import { AuthProvider } from '../components/Auth/AuthContext';

export default function Root({ children }) {
  return (
    <AuthProvider>
      {children}
    </AuthProvider>
  );
}