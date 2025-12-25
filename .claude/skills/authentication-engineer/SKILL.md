---
name: authentication-engineer
description: Authentication engineer specializing in Better-Auth integration in any project. Expert in implementing, configuring, and troubleshooting Better-Auth for various authentication scenarios.
tool: context7 MCP server
---

# Authentication Engineer Skill

## Overview
This skill provides expert-level assistance with Better-Auth integration and authentication systems. It helps users implement, configure, troubleshoot, and optimize authentication flows using Better-Auth and related technologies.

## When to Use This Skill
Use this skill when:
- Integrating Better-Auth into a new project
- Troubleshooting authentication issues
- Configuring providers (OAuth, email/password, etc.)
- Implementing custom authentication flows
- Securing API routes with authentication
- Setting up user sessions and permissions
- Migrating from other auth systems to Better-Auth

## Core Capabilities

### 1. Better-Auth Integration
- Install and configure Better-Auth in various frameworks (Next.js, Express, etc.)
- Set up database adapters for user persistence
- Configure authentication providers (Google, GitHub, email/password)
- Implement custom user models and schemas

### 2. Authentication Flow Implementation
- Create login and registration pages
- Implement protected routes and middleware
- Handle session management
- Set up password reset and email verification

### 3. Security Best Practices
- Implement secure token handling
- Configure proper CORS and security headers
- Set up rate limiting for auth endpoints
- Implement proper error handling without exposing sensitive info

### 4. Troubleshooting and Debugging
- Diagnose common authentication issues
- Debug session and token problems
- Fix provider configuration issues
- Resolve database and adapter problems

## Usage Patterns

### For Better-Auth Setup
When users ask for:
- "How to integrate Better-Auth with my Next.js app?"
- "Configure Google OAuth with Better-Auth"
- "Set up email/password authentication"
- "Fix session expiration issues"

### For Authentication Flows
When users need to:
- "Create a protected route in Next.js"
- "Implement a custom login page"
- "Add multi-factor authentication"
- "Handle user roles and permissions"

### For Troubleshooting
When users encounter:
- "Why is my session not persisting?"
- "Better-Auth provider not working"
- "Database connection issues with Better-Auth"
- "CORS errors with authentication endpoints"

## Available Tools

### Scripts
- `setup-better-auth.js`: Initialize Better-Auth in a project
- `configure-providers.js`: Set up authentication providers
- `secure-endpoints.js`: Add authentication to API routes
- `debug-auth.js`: Diagnose authentication issues

### Assets
- Pre-configured auth templates
- Security best practices checklists
- Common error resolution guides
- Provider-specific configuration examples

## Workflow

1. **Analyze Requirements**: Understand the authentication needs and current setup
2. **Select Approach**: Choose between basic setup, provider integration, or troubleshooting
3. **Apply Best Practices**: Implement security measures and proper configuration
4. **Test Implementation**: Verify authentication flows work correctly
5. **Optimize**: Fine-tune performance and security as needed

## Best Practices

### Security Considerations
- Always use HTTPS in production
- Implement proper CSRF protection
- Use secure, httpOnly cookies for tokens
- Implement proper input validation
- Follow OWASP authentication guidelines

### Configuration
- Use environment variables for sensitive data
- Configure proper session expiration times
- Set up secure cookie settings (secure, sameSite)
- Implement proper error handling
- Use strong password requirements when applicable

### Performance
- Optimize database queries for user lookup
- Implement proper caching strategies
- Minimize token payload size
- Consider token refresh strategies