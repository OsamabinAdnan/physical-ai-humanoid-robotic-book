# CSS Generator Editor Enhancer - Examples

## Example 1: Generate CSS for a Responsive Card Component

**User Request:** "Create a responsive card component with hover effects"

**Skill Response:**
```css
.card {
  background: white;
  border-radius: 8px;
  box-shadow: 0 2px 10px rgba(0, 0, 0, 0.1);
  overflow: hidden;
  transition: transform 0.3s ease, box-shadow 0.3s ease;
  max-width: 300px;
}

.card:hover {
  transform: translateY(-5px);
  box-shadow: 0 10px 20px rgba(0, 0, 0, 0.15);
}

.card__image {
  width: 100%;
  height: auto;
}

.card__content {
  padding: 1.5rem;
}

.card__title {
  font-size: 1.25rem;
  font-weight: 600;
  margin-bottom: 0.5rem;
}

.card__description {
  color: #666;
  line-height: 1.5;
}
```

## Example 2: Enhance Existing CSS with Variables

**User Request:** "Update this CSS to use CSS variables for theming"

**Before:**
```css
.header {
  background-color: #1a73e8;
  color: white;
  padding: 1rem;
}
```

**After (Skill Response):**
```css
:root {
  --primary-color: #1a73e8;
  --text-light: white;
  --spacing-md: 1rem;
}

.header {
  background-color: var(--primary-color);
  color: var(--text-light);
  padding: var(--spacing-md);
}
```

## Example 3: Convert Float Layout to Flexbox

**User Request:** "Convert this float-based layout to flexbox"

**Before:**
```css
.container {
  width: 100%;
}
.sidebar {
  float: left;
  width: 25%;
}
.content {
  float: right;
  width: 75%;
}
```

**After (Skill Response):**
```css
.container {
  display: flex;
  width: 100%;
}
.sidebar {
  flex: 0 0 25%;
}
.content {
  flex: 1;
}
```

## Example 4: Add Responsive Breakpoints

**User Request:** "Add responsive breakpoints for mobile and tablet"

**Skill Response:**
```css
.grid {
  display: grid;
  grid-template-columns: repeat(3, 1fr);
  gap: 1rem;
}

@media (max-width: 768px) {
  .grid {
    grid-template-columns: repeat(2, 1fr);
  }
}

@media (max-width: 480px) {
  .grid {
    grid-template-columns: 1fr;
  }
}
```

## Example 5: Optimize CSS for Performance

**User Request:** "Optimize this CSS for better performance"

**Before:**
```css
div .container ul li span a {
  color: #007bff;
  font-weight: bold;
  text-decoration: none;
}
```

**After (Skill Response):**
```css
.nav-link {
  color: #007bff;
  font-weight: bold;
  text-decoration: none;
}
```

## Example 6: Add Accessibility Features

**User Request:** "Add accessibility features to this button"

**Skill Response:**
```css
.button {
  background: #007bff;
  color: white;
  border: none;
  padding: 0.5rem 1rem;
  border-radius: 4px;
  cursor: pointer;
  transition: background-color 0.2s;

  /* Accessibility features */
  outline: none;
}

.button:focus {
  box-shadow: 0 0 0 3px rgba(0, 123, 255, 0.5);
}

.button:hover,
.button:focus {
  background: #0069d9;
}

/* Reduced motion support */
@media (prefers-reduced-motion: reduce) {
  .button {
    transition: none;
  }
}
```

## Example 7: Generate CSS Animation

**User Request:** "Create a fade-in animation for content"

**Skill Response:**
```css
@keyframes fadeIn {
  from {
    opacity: 0;
    transform: translateY(20px);
  }
  to {
    opacity: 1;
    transform: translateY(0);
  }
}

.fade-in {
  animation: fadeIn 0.6s ease-out forwards;
}

/* Reduced motion support */
@media (prefers-reduced-motion: reduce) {
  .fade-in {
    animation: none;
    opacity: 1;
    transform: none;
  }
}
```

## Example 8: Create Utility Classes

**User Request:** "Generate utility classes similar to Tailwind CSS"

**Skill Response:**
```css
/* Spacing utilities */
.m-0 { margin: 0; }
.m-1 { margin: 0.25rem; }
.m-2 { margin: 0.5rem; }
.m-3 { margin: 0.75rem; }
.m-4 { margin: 1rem; }

.mt-0 { margin-top: 0; }
.mt-1 { margin-top: 0.25rem; }
.mt-2 { margin-top: 0.5rem; }
.mt-3 { margin-top: 0.75rem; }
.mt-4 { margin-top: 1rem; }

/* Text alignment */
.text-left { text-align: left; }
.text-center { text-align: center; }
.text-right { text-align: right; }

/* Display utilities */
.d-block { display: block; }
.d-inline { display: inline; }
.d-flex { display: flex; }
.d-none { display: none; }

/* Flex utilities */
.flex-row { flex-direction: row; }
.flex-col { flex-direction: column; }
.justify-center { justify-content: center; }
.items-center { align-items: center; }
```