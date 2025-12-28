import React, { useState } from 'react';
import { useAuth } from './AuthContext';

interface RegisterFormProps {
  onSwitchToLogin?: () => void;
  onRegisterSuccess?: () => void;
}

const RegisterForm: React.FC<RegisterFormProps> = ({ onSwitchToLogin, onRegisterSuccess }) => {
  const [email, setEmail] = useState('');
  const [password, setPassword] = useState('');
  const [confirmPassword, setConfirmPassword] = useState('');
  const [showPassword, setShowPassword] = useState(false);
  const [showConfirmPassword, setShowConfirmPassword] = useState(false);
  const [name, setName] = useState('');
  const [softwareBackground, setSoftwareBackground] = useState('beginner');
  const [hardwareBackground, setHardwareBackground] = useState('beginner');
  const [error, setError] = useState('');
  const [isLoading, setIsLoading] = useState(false);

  const { register } = useAuth();

  const handleSubmit = async (e: React.FormEvent) => {
    e.preventDefault();
    setError('');
    setIsLoading(true);

    // Validate password confirmation
    if (password !== confirmPassword) {
      setError('Passwords do not match');
      setIsLoading(false);
      return;
    }

    // Validate password requirements
    const passwordErrors = [];
    if (password.length < 8) {
      passwordErrors.push('at least 8 characters');
    }
    if (!/[A-Z]/.test(password)) {
      passwordErrors.push('one uppercase letter');
    }
    if (!/[a-z]/.test(password)) {
      passwordErrors.push('one lowercase letter');
    }
    if (!/\d/.test(password)) {
      passwordErrors.push('one digit');
    }
    if (!/[!@#$%^&*(),.?":{}|<>]/.test(password)) {
      passwordErrors.push('one special character');
    }

    if (passwordErrors.length > 0) {
      setError(`Password must contain ${passwordErrors.join(', ')}`);
      setIsLoading(false);
      return;
    }

    try {
      await register({
        email,
        password,
        name,
        software_background: softwareBackground,
        hardware_background: hardwareBackground,
      });
      if (onRegisterSuccess) {
        onRegisterSuccess();
      }
    } catch (err) {
      setError(err instanceof Error ? err.message : 'An error occurred during registration');
    } finally {
      setIsLoading(false);
    }
  };

  return (
    <div className="auth-form-container">
      <h2>Register Your Account</h2>
      {error && <div className="error-message">{error}</div>}

      <form onSubmit={handleSubmit} className="auth-form">
        <div className="form-group">
          <label htmlFor="name">Full Name</label>
          <input
            type="text"
            id="name"
            value={name}
            onChange={(e) => setName(e.target.value)}
            onKeyDown={(e) => e.stopPropagation()} // Prevent search plugin from intercepting keyboard events
            required
            className="form-input"
            placeholder="Enter your full name"
          />
        </div>

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

        <div className="form-group">
          <label htmlFor="password">Password</label>
          <div className="password-input-container">
            <input
              type={showPassword ? "text" : "password"}
              id="password"
              value={password}
              onChange={(e) => setPassword(e.target.value)}
              onKeyDown={(e) => e.stopPropagation()} // Prevent search plugin from intercepting keyboard events
              required
              className="form-input"
              placeholder="Enter your password"
            />
            <button
              type="button"
              className="password-toggle-button"
              onClick={() => setShowPassword(!showPassword)}
              aria-label={showPassword ? "Hide password" : "Show password"}
            >
              {showPassword ? "🙈" : "👁️"} {/* Eye emoji for visibility toggle */}
            </button>
          </div>
        </div>

        <div className="form-group">
          <label htmlFor="confirm-password">Confirm Password</label>
          <div className="password-input-container">
            <input
              type={showConfirmPassword ? "text" : "password"}
              id="confirm-password"
              value={confirmPassword}
              onChange={(e) => setConfirmPassword(e.target.value)}
              onKeyDown={(e) => e.stopPropagation()} // Prevent search plugin from intercepting keyboard events
              required
              className="form-input"
              placeholder="Confirm your password"
            />
            <button
              type="button"
              className="password-toggle-button"
              onClick={() => setShowConfirmPassword(!showConfirmPassword)}
              aria-label={showConfirmPassword ? "Hide confirm password" : "Show confirm password"}
            >
              {showConfirmPassword ? "🙈" : "👁️"} {/* Eye emoji for visibility toggle */}
            </button>
          </div>
        </div>

        {/* Password requirements */}
        <div className="password-requirements">
          <p className="password-requirement-text">
            <strong>Password must contain:</strong>
          </p>
          <ul className="password-requirements-list">
            <li className={password.length >= 8 ? "requirement-met" : "requirement-pending"}>
              At least 8 characters
            </li>
            <li className={/[A-Z]/.test(password) ? "requirement-met" : "requirement-pending"}>
              One uppercase letter
            </li>
            <li className={/[a-z]/.test(password) ? "requirement-met" : "requirement-pending"}>
              One lowercase letter
            </li>
            <li className={/\d/.test(password) ? "requirement-met" : "requirement-pending"}>
              One number
            </li>
            <li className={/[!@#$%^&*(),.?":{}|<>]/.test(password) ? "requirement-met" : "requirement-pending"}>
              One special character
            </li>
          </ul>
        </div>

        <div className="form-group">
          <label htmlFor="software-background">Software Background</label>
          <select
            id="software-background"
            value={softwareBackground}
            onChange={(e) => setSoftwareBackground(e.target.value)}
            className="form-select"
            required
          >
            <option value="" disabled>Select your software background</option>
            <option value="beginner">Beginner</option>
            <option value="intermediate">Intermediate</option>
            <option value="expert">Expert</option>
          </select>
        </div>

        <div className="form-group">
          <label htmlFor="hardware-background">Hardware Background</label>
          <select
            id="hardware-background"
            value={hardwareBackground}
            onChange={(e) => setHardwareBackground(e.target.value)}
            className="form-select"
            required
          >
            <option value="" disabled>Select your hardware background</option>
            <option value="beginner">Beginner</option>
            <option value="intermediate">Intermediate</option>
            <option value="expert">Expert</option>
          </select>
        </div>

        <button
          type="submit"
          disabled={isLoading}
          className="auth-button"
        >
          {isLoading ? 'Registering...' : 'Register'}
        </button>
      </form>

      <div className="auth-switch">
        Already have an account?{' '}
        <button
          type="button"
          onClick={onSwitchToLogin}
          className="switch-button"
        >
          Login
        </button>
      </div>
    </div>
  );
};

export default RegisterForm;