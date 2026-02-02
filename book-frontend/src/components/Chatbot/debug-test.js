// Debug script to test if the client module is being loaded
console.log('Client module loaded at:', new Date().toISOString());

// Check if DOM is available
console.log('ExecutionEnvironment.canUseDOM would check:', typeof window !== 'undefined' && typeof document !== 'undefined');

// Test dynamic imports
if (typeof window !== 'undefined') {
  console.log('Window object is available');

  // Check if container exists or will be created
  setTimeout(() => {
    const container = document.getElementById('chatbot-container');
    console.log('Chatbot container exists:', !!container);

    // Check for any error in console that might prevent rendering
    console.log('Checking for errors that might prevent chatbot rendering...');
  }, 1000);
}