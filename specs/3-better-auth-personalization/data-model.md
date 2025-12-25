# Data Model: Better Auth Integration and Content Personalization

## User Profile Entity

### Fields
- `id`: UUID (Primary Key, Required)
  - Description: Unique identifier for the user
  - Validation: Auto-generated UUID, unique constraint

- `email`: String (Required)
  - Description: User's email address for authentication
  - Validation: Must be valid email format, unique constraint
  - Length: Maximum 255 characters

- `password_hash`: String (Required)
  - Description: Bcrypt-hashed password for secure storage
  - Validation: Minimum 60 characters (bcrypt format)
  - Length: Maximum 255 characters

- `name`: String (Optional)
  - Description: User's display name
  - Validation: Optional field
  - Length: Maximum 255 characters

- `created_at`: DateTime (Required)
  - Description: Timestamp when user account was created
  - Validation: Auto-populated on creation

- `updated_at`: DateTime (Required)
  - Description: Timestamp when user account was last updated
  - Validation: Auto-populated on update

- `software_background`: String (Required)
  - Description: User's expertise level in software
  - Validation: Must be one of: 'beginner', 'intermediate', 'expert'
  - Length: Maximum 20 characters

- `hardware_background`: String (Required)
  - Description: User's expertise level in hardware
  - Validation: Must be one of: 'beginner', 'intermediate', 'expert'
  - Length: Maximum 20 characters

- `email_verified`: Boolean (Optional)
  - Description: Whether the user's email has been verified
  - Validation: Defaults to false
  - Default: false

### Relationships
- One-to-many: User has many ChatSessions
- One-to-many: User has many ChatMessages
- One-to-many: User has many PersonalizedContents

## Personalized Content Entity

### Fields
- `id`: UUID (Primary Key, Required)
  - Description: Unique identifier for personalized content
  - Validation: Auto-generated UUID, unique constraint

- `user_id`: UUID (Foreign Key, Required)
  - Description: Reference to the user who requested personalization
  - Validation: Must reference an existing user
  - Relationship: Many-to-one with User

- `chapter_id`: String (Required)
  - Description: Identifier for the chapter that was personalized
  - Validation: Required field
  - Length: Maximum 500 characters

- `chapter_url`: String (Required)
  - Description: Full URL of the chapter that was personalized
  - Validation: Must be a valid URL format
  - Length: Maximum 1000 characters

- `original_content_hash`: String (Required)
  - Description: Hash of the original chapter content to detect changes
  - Validation: Required field
  - Length: Maximum 255 characters

- `personalized_summary`: Text (Required)
  - Description: The AI-generated personalized summary/roadmap
  - Validation: Required field
  - Length: Unlimited text

- `personalization_level`: String (Required)
  - Description: Expertise level for which content was personalized
  - Validation: Must be one of: 'beginner', 'intermediate', 'expert'
  - Length: Maximum 20 characters

- `created_at`: DateTime (Required)
  - Description: Timestamp when personalization was created
  - Validation: Auto-populated on creation

- `updated_at`: DateTime (Required)
  - Description: Timestamp when personalization was last updated
  - Validation: Auto-populated on update

### Relationships
- Many-to-one: PersonalizedContent belongs to User

## Database Indexes

### Users Table
- Index on `email` (unique): For fast authentication lookups
- Index on `created_at`: For sorting and filtering by creation date

### PersonalizedContents Table
- Index on `user_id`: For fast user-specific queries
- Index on `chapter_id`: For fast chapter-specific queries
- Composite index on (`user_id`, `chapter_id`): For user-chapter specific queries
- Index on `chapter_url`: For URL-based lookups

## Validation Rules

### User Profile
- Email must be unique across all users
- Password must be properly hashed using bcrypt before storage
- Software and hardware background must be one of the defined values
- Email format must be validated before saving

### Personalized Content
- User ID must reference an existing user
- Chapter URL must be a valid URL format
- Personalization level must match one of the user's background levels
- Original content hash should be computed using SHA-256

## State Transitions

### User Profile
- New user registration: Creates profile with provided background information
- Profile update: Allows updating background information after initial registration
- Email verification: Updates email_verified status

### Personalized Content
- Personalization request: Creates new personalized content record
- Content refresh: Updates existing record if original content has changed
- Deletion: Removes personalized content when user profile is deleted