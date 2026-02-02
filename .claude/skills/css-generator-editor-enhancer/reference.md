# CSS Best Practices Guide

## Naming Conventions

### BEM (Block Element Modifier)
BEM provides a clear structure for naming CSS classes that makes it easy to understand relationships between classes.

```css
/* Block */
.card { }

/* Element (child of block) */
.card__title { }
.card__body { }
.card__footer { }

/* Modifier (variant of block or element) */
.card--featured { }
.card__title--large { }
```

### Functional Naming
Use names that describe what an element does, not what it looks like:

```css
/* Good - describes function */
.btn-primary { }
.text-center { }
.margin-bottom { }

/* Avoid - describes appearance */
.red-button { }
.big-title { }
.spacer { }
```

## Architecture and Organization

### ITCSS (Inverted Triangle CSS)
Organize CSS files from generic to specific:

```
Settings     # Variables, config
Tools        # Functions, mixins
Generic      # Reset, normalize
Elements     # Base HTML elements
Objects      # OOCSS objects (no cosmetics)
Components   # Designed components
Utilities    # Helpers, overrides
```

### 7-1 Pattern
Organize CSS into 7 different folders and 1 main file:

```
styles/
├── abstracts/
├── base/
├── components/
├── layout/
├── pages/
├── themes/
├── vendors/
└── main.css
```

## Performance Optimization

### Minimize Selector Complexity
```css
/* Avoid complex selectors */
div .container ul li span a { }

/* Prefer simpler, more specific selectors */
.nav-link { }
```

### Use Efficient Selectors
- ID selectors (#id) are fastest
- Class selectors (.class) are fast
- Descendant selectors (.parent .child) are slower
- Universal selectors (*) are slowest

```css
/* Efficient */
.header { }
.btn { }

/* Less efficient */
div .container .header .btn { }
```

### Minimize CSS File Size
- Remove unused CSS
- Use shorthand properties
- Minimize whitespace and comments for production
- Use CSS variables for repeated values

```css
/* Good - using shorthand */
.box {
  margin: 10px 15px 10px 15px; /* Can be simplified */
}

/* Better */
.box {
  margin: 10px 15px;
}

/* Best for repeated values */
:root {
  --spacing-md: 15px;
  --spacing-sm: 10px;
}

.box {
  margin: var(--spacing-sm) var(--spacing-md);
}
```

## Modern CSS Features

### Use CSS Variables for Theming
```css
:root {
  --color-primary: #007bff;
  --color-secondary: #6c757d;
  --font-size-base: 1rem;
  --border-radius: 0.25rem;
  --shadow: 0 2px 4px rgba(0,0,0,0.1);
}

.button {
  background-color: var(--color-primary);
  border-radius: var(--border-radius);
  box-shadow: var(--shadow);
}
```

### Prefer Logical Properties for Internationalization
```css
/* Instead of directional properties */
.text-container {
  margin-left: 1rem;
  padding-right: 1rem;
  text-align: right;
}

/* Use logical properties */
.text-container {
  margin-inline-start: 1rem;
  padding-inline-end: 1rem;
  text-align: end;
}
```

### Use Modern Units Appropriately
```css
/* Use rem for sizing relative to root font size */
.container { font-size: 1.125rem; }

/* Use em for sizing relative to current element */
.card { width: 20em; } /* 20 times the font-size of .card */

/* Use fr for grid fractional units */
.grid { grid-template-columns: 1fr 2fr 1fr; }

/* Use % for responsive sizing */
.responsive { width: 100%; max-width: 1200px; }
```

## Responsive Design Best Practices

### Mobile-First Approach
```css
/* Start with mobile styles */
.card {
  padding: 1rem;
  font-size: 1rem;
}

/* Add enhancements for larger screens */
@media (min-width: 768px) {
  .card {
    padding: 1.5rem;
    font-size: 1.125rem;
  }
}

@media (min-width: 1024px) {
  .card {
    padding: 2rem;
    font-size: 1.25rem;
  }
}
```

### Use Relative Units for Responsiveness
```css
/* Good for responsive typography */
.heading { font-size: clamp(1.5rem, 4vw, 3rem); }

/* Use appropriate units for different contexts */
.container { max-width: 1200px; } /* Fixed max-width */
.text { font-size: 1.2rem; } /* Scales with user's font size */
.spacing { margin: 2rem; } /* Scales with font size */
```

## Accessibility Considerations

### Ensure Adequate Color Contrast
```css
:root {
  --text-primary: #212529; /* Dark gray, good contrast */
  --text-secondary: #495057; /* Good for secondary text */
  --text-muted: #6c757d; /* Still accessible */
}

/* Avoid low contrast combinations */
/* .text { color: #777777; } */ /* Too light on white background */
```

### Support Reduced Motion
```css
/* Allow users to disable animations */
@media (prefers-reduced-motion: reduce) {
  .animated-element {
    animation-duration: 0.01ms !important;
    animation-iteration-count: 1 !important;
    transition-duration: 0.01ms !important;
  }
}

/* Better approach - only apply animations when appropriate */
.button {
  transition: color 0.15s ease;
}

@media (prefers-reduced-motion: no-preference) {
  .button {
    transition: all 0.15s ease;
  }

  .button:hover {
    transform: translateY(-2px);
  }
}
```

### Focus Management
```css
/* Ensure focus is visible */
.btn:focus {
  outline: 2px solid #007bff;
  outline-offset: 2px;
}

/* Or use box-shadow as an alternative */
.btn:focus {
  box-shadow: 0 0 0 2px rgba(0, 123, 255, 0.25);
}
```

## Maintainability Practices

### Comment Your CSS
```css
/* Header component
   Navigation and branding elements */
.header {
  display: flex;
  justify-content: space-between;
  align-items: center;
  padding: 1rem;
}

/* Responsive adjustments for header */
@media (max-width: 768px) {
  .header {
    flex-direction: column;
  }
}
```

### Group Related Properties
```css
.button {
  /* Positioning */
  position: relative;
  display: inline-block;

  /* Box model */
  padding: 0.5rem 1rem;
  margin: 0.25rem;
  border: 1px solid #007bff;

  /* Typography */
  font-size: 1rem;
  font-weight: 500;
  text-align: center;

  /* Visual */
  background-color: #007bff;
  color: white;
  border-radius: 0.25rem;

  /* Other */
  cursor: pointer;
  transition: all 0.2s ease;
}
```

### Use Consistent Formatting
```css
/* Consistent indentation and spacing */
.component {
  display: flex;
  flex-direction: column;
  align-items: center;
  justify-content: center;
  width: 100%;
  padding: 1rem;
  margin-bottom: 1rem;
  background-color: #f8f9fa;
  border-radius: 0.25rem;
}

.component__title {
  font-size: 1.25rem;
  font-weight: 600;
  margin-bottom: 0.5rem;
  color: #343a40;
}
```

## Common Pitfalls to Avoid

### Don't Use !important Excessively
```css
/* Avoid */
.text { color: red !important; }

/* Better approach - use specificity */
.header .text { color: red; }

/* Or use CSS custom properties */
.text { color: var(--text-color, #333); }
```

### Don't Use IDs for Styling
```css
/* Avoid */
#header { }

/* Prefer classes */
.header { }
```

### Don't Overuse Position: Absolute
```scss
/* Avoid - creates positioning context issues */
.complex-layout {
  position: relative;
}
.complex-layout .item1 {
  position: absolute;
  top: 10px;
  left: 20px;
}
/* Many more absolute positioned items... */

/* Better - use modern layout methods */
.modern-layout {
  display: grid;
  grid-template-columns: repeat(3, 1fr);
  gap: 1rem;
}
```

### Avoid Deep Nesting
```scss
/* Avoid - creates overly specific selectors */
.nav {
  .nav-list {
    .nav-item {
      .nav-link {
        /* This creates: .nav .nav-list .nav-item .nav-link */
      }
    }
  }
}

/* Better - use BEM naming */
.nav { }
.nav__list { }
.nav__item { }
.nav__link { }
```