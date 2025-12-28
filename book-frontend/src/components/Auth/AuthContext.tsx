import React, { createContext, useContext, useState, useEffect, ReactNode } from 'react';

interface User {
  id: string;
  email: string;
  name: string;
  software_background: string;
  hardware_background: string;
  email_verified: boolean;
  created_at: string;
  updated_at: string;
}

interface AuthContextType {
  user: User | null;
  token: string | null;
  login: (email: string, password: string) => Promise<void>;
  register: (userData: {
    email: string;
    password: string;
    name: string;
    software_background: string;
    hardware_background: string;
  }) => Promise<void>;
  logout: () => void;
  isAuthenticated: boolean;
}

const AuthContext = createContext<AuthContextType | undefined>(undefined);

interface AuthProviderProps {
  children: ReactNode;
}

export const AuthProvider: React.FC<AuthProviderProps> = ({ children }) => {
  const [user, setUser] = useState<User | null>(null);
  const [token, setToken] = useState<string | null>(null);
  const [loading, setLoading] = useState(true);

  // Check for existing token on app load
  useEffect(() => {
    const storedToken = localStorage.getItem('authToken');
    const storedUser = localStorage.getItem('user');

    if (storedToken && storedUser) {
      setToken(storedToken);
      try {
        const parsedUser = JSON.parse(storedUser);
        setUser(parsedUser);
      } catch (error) {
        console.error('Error parsing stored user:', error);
        // If there's an error parsing, clear the stored data
        localStorage.removeItem('authToken');
        localStorage.removeItem('user');
      }
    }

    setLoading(false);
  }, []);

  const login = async (email: string, password: string) => {
    try {
      const BACKEND_URL = typeof process !== 'undefined' && process.env && process.env.NODE_ENV === 'production'
        ? 'https://osamabinadnan-rag-with-neondb.hf.space'
        : (typeof process !== 'undefined' && process.env && process.env.REACT_APP_BACKEND_URL ? process.env.REACT_APP_BACKEND_URL : 'http://127.0.0.1:8000');
      const response = await fetch(`${BACKEND_URL}/api/auth/login`, {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify({ email, password }),
      });

      if (!response.ok) {
        const errorData = await response.json().catch(() => ({}));
        console.error('Login API error:', errorData);
        throw new Error(errorData.detail || errorData.message || 'Login failed with status: ' + response.status);
      }

      const data = await response.json();

      if (data.session_token) {
        setToken(data.session_token);
        localStorage.setItem('authToken', data.session_token);

        // Get user details
        const userResponse = await fetch(`${BACKEND_URL}/api/auth/me`, {
          headers: {
            'Authorization': `Bearer ${data.session_token}`,
          },
        });

        if (userResponse.ok) {
          const userData = await userResponse.json();
          setUser(userData);
          localStorage.setItem('user', JSON.stringify(userData));
        }
      }
    } catch (error) {
      console.error('Login error:', error);
      throw error;
    }
  };

  const register = async (userData: {
    email: string;
    password: string;
    name: string;
    software_background: string;
    hardware_background: string;
  }) => {
    try {
      const BACKEND_URL = typeof process !== 'undefined' && process.env && process.env.NODE_ENV === 'production'
        ? 'https://osamabinadnan-rag-with-neondb.hf.space'
        : (typeof process !== 'undefined' && process.env && process.env.REACT_APP_BACKEND_URL ? process.env.REACT_APP_BACKEND_URL : 'http://127.0.0.1:8000');
      const response = await fetch(`${BACKEND_URL}/api/auth/register`, {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify(userData),
      });

      if (!response.ok) {
        const errorData = await response.json().catch(() => ({}));
        console.error('Registration API error:', errorData);
        throw new Error(errorData.detail || errorData.message || 'Registration failed with status: ' + response.status);
      }

      const data = await response.json();

      if (data.session_token) {
        setToken(data.session_token);
        localStorage.setItem('authToken', data.session_token);

        // Get user details
        const userResponse = await fetch(`${BACKEND_URL}/api/auth/me`, {
          headers: {
            'Authorization': `Bearer ${data.session_token}`,
          },
        });

        if (userResponse.ok) {
          const userData = await userResponse.json();
          setUser(userData);
          localStorage.setItem('user', JSON.stringify(userData));
        }
      }
    } catch (error) {
      console.error('Registration error:', error);
      throw error;
    }
  };

  const logout = () => {
    setToken(null);
    setUser(null);
    localStorage.removeItem('authToken');
    localStorage.removeItem('user');
  };

  const isAuthenticated = !!token && !!user;

  const value = {
    user,
    token,
    login,
    register,
    logout,
    isAuthenticated,
  };

  if (loading) {
    return <div>Loading...</div>; // Or a proper loading component
  }

  return <AuthContext.Provider value={value}>{children}</AuthContext.Provider>;
};

export const useAuth = () => {
  const context = useContext(AuthContext);
  if (context === undefined) {
    throw new Error('useAuth must be used within an AuthProvider');
  }
  return context;
};