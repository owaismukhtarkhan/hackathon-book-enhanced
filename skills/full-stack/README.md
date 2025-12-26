# Full-Stack Development Skills

Skills for handling both client-side and server-side development challenges.

## Key Learnings
- Distinguishing between build-time and runtime code
- Handling environment variables across different environments
- Managing server-side rendering vs. client-side code
- Static site generation considerations

## Patterns & Solutions

### Environment Variable Handling
```typescript
// Safe environment variable usage for static site generation
const config = {
  // Use empty string as fallback for build process
  apiKey: process.env.API_KEY || '',
  // Use different values based on environment
  apiUrl: process.env.NODE_ENV === 'production'
    ? 'https://api.example.com'
    : 'https://staging.api.com'
};

// Health check that doesn't fail during build
export function validateEnvironment() {
  if (!config.apiKey && process.env.NODE_ENV === 'production') {
    console.warn('API key not set in production');
  }
  return true; // Don't fail build process
}
```

### Client-Side Only Code
```javascript
// Use BrowserOnly for components that need browser APIs
import BrowserOnly from '@docusaurus/BrowserOnly';

function MyComponent() {
  return (
    <BrowserOnly>
      {() => {
        // Code that uses browser APIs
        const { SomeBrowserFeature } = require('some-browser-library');
        return <SomeBrowserFeature />;
      }}
    </BrowserOnly>
  );
}
```

### Dynamic Imports for Heavy Libraries
```javascript
// Dynamically import libraries that might cause build issues
const loadHeavyLibrary = async () => {
  if (typeof window !== 'undefined') {
    // Only load in browser environment
    const { HeavyLibrary } = await import('heavy-library');
    return HeavyLibrary;
  }
  return null;
};
```

## Common Issues & Solutions
- **Build Failures**: Avoid browser APIs during server build
- **Environment Variables**: Handle missing variables gracefully during build
- **Static Generation**: Separate client-side features from build process
- **API Integration**: Ensure services don't break the build process

## Best Practices
- Use feature detection for browser APIs
- Implement graceful degradation
- Separate build-time and runtime logic
- Test both development and production builds
- Use conditional loading for browser-dependent features