#!/usr/bin/env node

/**
 * CSS Generator - Generates CSS based on user specifications
 *
 * This script helps generate CSS code based on design requirements,
 * component specifications, or layout needs.
 */

function generateCSS(specifications) {
  const {
    componentName,
    layoutType,
    colors,
    spacing,
    breakpoints,
    animations
  } = specifications;

  let cssOutput = '';

  // Generate component-specific CSS
  if (componentName) {
    cssOutput += `/* CSS for ${componentName} */\n`;

    // Base styles
    cssOutput += `.${componentName} {\n`;
    if (colors && colors.primary) {
      cssOutput += `  background-color: ${colors.primary};\n`;
    }
    if (spacing && spacing.padding) {
      cssOutput += `  padding: ${spacing.padding};\n`;
    }
    if (spacing && spacing.margin) {
      cssOutput += `  margin: ${spacing.margin};\n`;
    }
    cssOutput += '}\n\n';

    // Add responsive styles if breakpoints are specified
    if (breakpoints) {
      Object.entries(breakpoints).forEach(([size, value]) => {
        cssOutput += `@media (max-width: ${value}) {\n`;
        cssOutput += `  .${componentName} {\n`;
        cssOutput += `    /* Responsive styles for ${size} */\n`;
        cssOutput += `  }\n`;
        cssOutput += '}\n\n';
      });
    }

    // Add animation styles if specified
    if (animations) {
      animations.forEach(anim => {
        cssOutput += `@keyframes ${anim.name} {\n`;
        anim.steps.forEach(step => {
          cssOutput += `  ${step.percent} {\n`;
          Object.entries(step.properties).forEach(([prop, val]) => {
            cssOutput += `    ${prop}: ${val};\n`;
          });
          cssOutput += '  }\n';
        });
        cssOutput += '}\n\n';

        cssOutput += `.${componentName}.${anim.targetClass} {\n`;
        cssOutput += `  animation: ${anim.name} ${anim.duration} ${anim.timing};\n`;
        cssOutput += '}\n\n';
      });
    }
  }

  return cssOutput;
}

// Example usage and test function
function exampleUsage() {
  const spec = {
    componentName: 'button',
    layoutType: 'flex',
    colors: {
      primary: '#007bff',
      secondary: '#6c757d'
    },
    spacing: {
      padding: '0.375rem 0.75rem',
      margin: '0.25rem'
    },
    breakpoints: {
      mobile: '768px',
      tablet: '992px'
    },
    animations: [
      {
        name: 'fadeIn',
        targetClass: 'fade',
        duration: '0.3s',
        timing: 'ease-in-out',
        steps: [
          {
            percent: '0%',
            properties: {
              opacity: '0'
            }
          },
          {
            percent: '100%',
            properties: {
              opacity: '1'
            }
          }
        ]
      }
    ]
  };

  const generatedCSS = generateCSS(spec);
  console.log(generatedCSS);
}

// If this script is run directly, show an example
if (require.main === module) {
  console.log('CSS Generator - Generate CSS based on specifications\n');
  exampleUsage();
}

module.exports = { generateCSS };