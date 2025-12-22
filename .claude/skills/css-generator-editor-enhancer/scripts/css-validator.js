#!/usr/bin/env node

/**
 * CSS Validator - Validates CSS syntax and best practices
 *
 * This script checks CSS for syntax errors, deprecated properties,
 * and best practice violations.
 */

// Common CSS validation patterns
const validationPatterns = {
  // Check for valid hex color values
  hexColor: /^#([A-Fa-f0-9]{6}|[A-Fa-f0-9]{3})$/,

  // Check for valid RGB color values
  rgbColor: /^rgb\(\s*\d+\s*,\s*\d+\s*,\s*\d+\s*\)$/,

  // Check for valid RGBA color values
  rgbaColor: /^rgba\(\s*\d+\s*,\s*\d+\s*,\s*\d+\s*,\s*(0|1|0\.\d+)\s*\)$/,

  // Check for valid URL values
  url: /^url\(['"]?[^'"]+['"]?\)$/,

  // Check for valid CSS units
  unit: /^(-?\d*\.?\d+)(px|em|rem|%|vw|vh|vmin|vmax|ch|ex|cm|mm|in|pt|pc|fr)$/,

  // Check for valid CSS identifiers
  identifier: /^[_a-zA-Z][_a-zA-Z0-9-]*$/
};

// List of deprecated CSS properties
const deprecatedProperties = [
  'appearance',
  'user-select',
  'box-flex',
  'box-orient',
  'box-align',
  'box-pack',
  'border-radius',
  'box-shadow',
  'text-shadow',
  'transform',
  'transition',
  'animation'
];

// List of vendor prefixes that should be avoided in favor of standard properties
const vendorPrefixes = [
  '-webkit-',
  '-moz-',
  '-ms-',
  '-o-'
];

function validateCSS(cssInput) {
  const lines = cssInput.split('\n');
  const errors = [];
  const warnings = [];
  let currentSelector = '';
  let inBlock = false;

  lines.forEach((line, index) => {
    const lineNumber = index + 1;
    const trimmedLine = line.trim();

    // Check for selector start
    if (trimmedLine.includes('{') && !trimmedLine.includes('}')) {
      currentSelector = trimmedLine.replace('{', '').trim();
      inBlock = true;
    }
    // Check for selector end
    else if (trimmedLine.includes('}')) {
      inBlock = false;
    }
    // Check for CSS property inside selector
    else if (inBlock && trimmedLine.includes(':')) {
      const parts = trimmedLine.split(':');
      if (parts.length >= 2) {
        const property = parts[0].trim();
        let value = parts.slice(1).join(':').trim().replace(';', '');

        // Check for deprecated properties
        if (deprecatedProperties.includes(property)) {
          warnings.push({
            line: lineNumber,
            message: `Property '${property}' might be deprecated or have better alternatives`,
            type: 'deprecated'
          });
        }

        // Check for vendor prefixes
        for (const prefix of vendorPrefixes) {
          if (property.startsWith(prefix)) {
            warnings.push({
              line: lineNumber,
              message: `Vendor prefix '${prefix}' detected. Consider using standard property instead`,
              type: 'vendor-prefix'
            });
          }
        }

        // Validate common value types
        if (property.includes('color')) {
          if (!validationPatterns.hexColor.test(value) &&
              !validationPatterns.rgbColor.test(value) &&
              !validationPatterns.rgbaColor.test(value) &&
              !['transparent', 'inherit', 'initial'].includes(value.toLowerCase())) {
            warnings.push({
              line: lineNumber,
              message: `Invalid color value: ${value}`,
              type: 'invalid-value'
            });
          }
        }

        // Validate units
        if (!['width', 'height', 'margin', 'padding', 'top', 'right', 'bottom', 'left',
              'font-size', 'line-height', 'border-width', 'border-radius'].includes(property)) {
          // Skip unit validation for properties that don't typically need units
        } else {
          // For properties that should have units, check if it's a number without unit
          if (/^\d+$/.test(value) && !['0'].includes(value)) {
            warnings.push({
              line: lineNumber,
              message: `Missing unit for property '${property}': ${value}`,
              type: 'missing-unit'
            });
          } else if (!['0', 'auto', 'inherit', 'initial', 'unset'].includes(value) &&
                    !validationPatterns.unit.test(value) &&
                    !value.startsWith('#') &&
                    !value.startsWith('rgb') &&
                    !value.startsWith('hsl') &&
                    !value.startsWith('var') &&
                    !value.startsWith('calc')) {
            // Additional validation for non-unit values
          }
        }

        // Check for missing semicolon (not on last property in block)
        if (!trimmedLine.endsWith(';') && !trimmedLine.includes('}')) {
          warnings.push({
            line: lineNumber,
            message: `Missing semicolon after property declaration`,
            type: 'missing-semicolon'
          });
        }
      }
    }

    // Check for common syntax errors
    if (trimmedLine.includes('{') && trimmedLine.includes('}') && trimmedLine.indexOf('{') > trimmedLine.indexOf('}')) {
      errors.push({
        line: lineNumber,
        message: `Invalid CSS syntax: '{' appears after '}'`,
        type: 'syntax-error'
      });
    }

    if ((trimmedLine.match(/{/g) || []).length > (trimmedLine.match(/}/g) || []).length) {
      errors.push({
        line: lineNumber,
        message: `Unmatched '{' in CSS`,
        type: 'syntax-error'
      });
    }

    if ((trimmedLine.match(/}/g) || []).length > (trimmedLine.match(/{/g) || []).length) {
      errors.push({
        line: lineNumber,
        message: `Unmatched '}' in CSS`,
        type: 'syntax-error'
      });
    }
  });

  return {
    isValid: errors.length === 0,
    errors,
    warnings
  };
}

// Example usage
function exampleUsage() {
  const sampleCSS = `
    .my-class {
      color: #ff0000;
      background-color: rgb(255, 0, 0);
      padding: 10px;
      margin: 5px
    }

    .another-class {
      -webkit-transform: rotate(45deg);
      width: 100px;
      height: auto;
    }
  `;

  console.log('Validating CSS:');
  console.log(sampleCSS);

  const result = validateCSS(sampleCSS);
  console.log('\nValidation Results:');
  console.log(`Valid: ${result.isValid}`);
  console.log(`Errors: ${result.errors.length}`);
  console.log(`Warnings: ${result.warnings.length}`);

  if (result.errors.length > 0) {
    console.log('\nErrors:');
    result.errors.forEach(error => {
      console.log(`  Line ${error.line}: ${error.message}`);
    });
  }

  if (result.warnings.length > 0) {
    console.log('\nWarnings:');
    result.warnings.forEach(warning => {
      console.log(`  Line ${warning.line}: ${warning.message}`);
    });
  }
}

// If this script is run directly, show an example
if (require.main === module) {
  console.log('CSS Validator - Validates CSS syntax and best practices\n');
  exampleUsage();
}

module.exports = { validateCSS, validationPatterns, deprecatedProperties, vendorPrefixes };