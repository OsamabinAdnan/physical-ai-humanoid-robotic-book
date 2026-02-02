import React, { useState, useEffect } from 'react';
import './PageNavigationLoader.css';

const PageNavigationLoader: React.FC = () => {
  const [isLoading, setIsLoading] = useState(false);

  useEffect(() => {
    let timeoutId: NodeJS.Timeout;

    // Handle Docusaurus navigation events
    const handleRouteChangeStart = () => {
      setIsLoading(true);
    };

    const handleRouteChangeComplete = () => {
      // Clear any existing timeout
      if (timeoutId) clearTimeout(timeoutId);
      // Set a brief timeout to ensure content is loaded before hiding the loader
      timeoutId = setTimeout(() => {
        setIsLoading(false);
      }, 300);
    };

    // Listen for Docusaurus-specific events
    // These are the standard events Docusaurus emits during navigation
    window.addEventListener('docusaurus:route-change', handleRouteChangeStart);
    window.addEventListener('docusaurus:route-change-complete', handleRouteChangeComplete);

    // Also listen for React Router events which Docusaurus is built on
    window.addEventListener('routeChangeStart', handleRouteChangeStart);
    window.addEventListener('routeChangeComplete', handleRouteChangeComplete);

    // Listen for popstate events (browser back/forward)
    const handlePopState = () => {
      setIsLoading(true);
    };

    window.addEventListener('popstate', handlePopState);

    // Cleanup function
    return () => {
      window.removeEventListener('docusaurus:route-change', handleRouteChangeStart);
      window.removeEventListener('docusaurus:route-change-complete', handleRouteChangeComplete);
      window.removeEventListener('routeChangeStart', handleRouteChangeStart);
      window.removeEventListener('routeChangeComplete', handleRouteChangeComplete);
      window.removeEventListener('popstate', handlePopState);

      if (timeoutId) clearTimeout(timeoutId);
    };
  }, []);

  if (!isLoading) return null;

  return (
    <div className="page-navigation-loader-overlay">
      <div className="page-navigation-loader-content">
        <div className="page-navigation-loader-spinner"></div>
        <h3 className="page-navigation-loader-message">Loading Page...</h3>
        <p className="page-navigation-loader-subtext">Please wait while we load your content</p>
      </div>
    </div>
  );
};

export default PageNavigationLoader;