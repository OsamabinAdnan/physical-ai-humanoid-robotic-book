# Database Migration Guide

This document explains how to manage database migrations for the Neon Serverless Postgres integration using Alembic.

## Overview

The project uses Alembic for database migrations with async support for Neon Serverless Postgres. All database models are defined in `backend/database/models.py` and managed through SQLAlchemy async engine.

## Prerequisites

- Ensure your Neon database connection string is properly set in your environment variables
- The database URL should be available as `NEON_DATABASE_URL` environment variable

## Migration Commands

### Creating a New Migration

To create a new migration after making changes to models:

```bash
cd backend
alembic revision --autogenerate -m "Description of the changes"
```

### Applying Migrations

To apply pending migrations to the database:

```bash
cd backend
alembic upgrade head
```

### Downgrading Migrations

To downgrade to a previous migration:

```bash
cd backend
alembic downgrade -1  # Downgrade by one migration
# or
alembic downgrade <revision_id>  # Downgrade to specific revision
```

### Checking Migration Status

To check the current migration status:

```bash
cd backend
alembic current
```

### Viewing Migration History

To view all migrations:

```bash
cd backend
alembic history
```

## Migration Configuration

The Alembic configuration is in `backend/alembic.ini` and `backend/migrations/env.py`. Key features:

- Async database support for Neon Serverless Postgres
- Automatic model detection from `backend/database/models.py`
- Proper connection handling for async operations

## Best Practices

1. Always review auto-generated migrations before applying them
2. Test migrations on a development database first
3. Create backup copies of important data before running migrations in production
4. Use descriptive migration messages to explain the changes

## Troubleshooting

### Connection Issues
If you encounter connection issues, ensure:
- The `NEON_DATABASE_URL` environment variable is properly set
- Your Neon database is accessible
- SSL settings are correctly configured

### Migration Conflicts
If you encounter migration conflicts:
1. Check the migration history with `alembic history`
2. Identify the conflicting revisions
3. Use `alembic merge` to resolve conflicts if needed

## Models Overview

The current database schema includes:

- **users**: User accounts with email, name, and timestamps
- **chat_sessions**: Chat sessions associated with users
- **chat_messages**: Messages within chat sessions with user and session relationships
- **documents**: Document metadata for the RAG system