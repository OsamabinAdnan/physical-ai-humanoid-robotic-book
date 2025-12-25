# Better-Auth Reference Guide

## Core Concepts

Better-Auth is a modern authentication library that provides:
- Simple API for authentication
- Multiple authentication providers
- Database adapters
- Session management
- Security best practices

## Installation

```bash
npm install better-auth
```

## Basic Setup

### Next.js Setup

```javascript
// lib/auth.js
import { betterAuth } from "better-auth";
import { nextjs } from "better-auth/next-js";

export const auth = betterAuth({
  database: {
    provider: "sqlite",
    url: process.env.DATABASE_URL,
  },
  socialProviders: {
    google: {
      clientId: process.env.GOOGLE_CLIENT_ID,
      clientSecret: process.env.GOOGLE_CLIENT_SECRET,
    },
  },
});

export const { signIn, signOut, authHandler } = nextjs(auth);
```

### API Route (app/api/auth/[...nextauth]/route.ts)

```javascript
import { authHandler } from "@/lib/auth";

export const { GET, POST } = authHandler;
```

## Environment Variables

```
DATABASE_URL=your_database_url
GOOGLE_CLIENT_ID=your_google_client_id
GOOGLE_CLIENT_SECRET=your_google_client_secret
BETTER_AUTH_SECRET=your_secret_key
```

## Protected Routes

### Server Component

```javascript
import { auth } from "@/lib/auth";

export default async function ProtectedPage() {
  const session = await auth();

  if (!session?.user) {
    redirect("/login");
  }

  return <div>Welcome {session.user.name}</div>;
}
```

### Client Component

```javascript
"use client";
import { useSession } from "better-auth/react";

export default function ClientComponent() {
  const { data: session, status } = useSession();

  if (status === "loading") return <div>Loading...</div>;
  if (!session) return <div>Please log in</div>;

  return <div>Welcome {session.user.name}</div>;
}
```

## Common Configuration Options

### Email/Password Authentication

```javascript
const auth = betterAuth({
  emailAndPassword: {
    enabled: true,
    requireEmailVerification: false,
    async sendVerificationEmail(user, url) {
      // Send verification email
    },
  },
});
```

### Custom User Model

```javascript
const auth = betterAuth({
  user: {
    model: {
      fields: {
        role: {
          type: "string",
          default: "user",
        },
      },
    },
  },
});
```

## Troubleshooting

### Session Not Persisting
- Check if `BETTER_AUTH_URL` is set correctly
- Ensure cookies are not being blocked
- Verify domain configuration in production

### Database Connection Issues
- Verify database URL is correct
- Check if database adapter is properly installed
- Ensure database is running and accessible

### OAuth Provider Not Working
- Verify client ID and secret are correct
- Check redirect URIs are properly configured
- Ensure the provider is enabled in the auth config