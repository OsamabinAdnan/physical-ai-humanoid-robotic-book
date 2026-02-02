/**
 * Integration test script for the Chatbot component
 * This script tests the frontend-backend integration functionality
 */

const { spawn } = require('child_process');
const axios = require('axios');

async function testBackendConnection() {
  try {
    console.log('Testing backend health check...');
    const response = await axios.get('http://localhost:8000/health');
    console.log('✓ Backend health check passed:', response.data.status);
    return true;
  } catch (error) {
    console.error('✗ Backend health check failed:', error.message);
    return false;
  }
}

async function testChatEndpoint() {
  try {
    console.log('Testing chat endpoint...');
    const response = await axios.post('http://localhost:8000/chat', {
      question: 'What is Physical AI?',
      top_k: 3
    });

    console.log('✓ Chat endpoint test passed');
    console.log('Response keys:', Object.keys(response.data));
    console.log('Answer preview:', response.data.answer.substring(0, 100) + '...');

    if (response.data.citations && response.data.citations.length > 0) {
      console.log('✓ Citations included in response');
    } else {
      console.log('⚠ No citations in response');
    }

    return true;
  } catch (error) {
    console.error('✗ Chat endpoint test failed:', error.message);
    return false;
  }
}

async function runIntegrationTests() {
  console.log('Starting frontend-backend integration tests...\n');

  const backendOk = await testBackendConnection();
  if (!backendOk) {
    console.log('\n✗ Backend is not accessible. Please start the backend server on http://localhost:8000');
    process.exit(1);
  }

  const chatOk = await testChatEndpoint();

  if (chatOk) {
    console.log('\n✓ All integration tests passed!');
    console.log('The frontend can successfully communicate with the backend.');
  } else {
    console.log('\n✗ Some integration tests failed.');
    process.exit(1);
  }
}

// Run the tests
runIntegrationTests().catch(error => {
  console.error('Test execution error:', error);
  process.exit(1);
});