# TypeScript Development Skills

Reusable TypeScript patterns and practices.

## Key Learnings
- Interface and type definition creation
- Service architecture design
- API integration patterns (Google Gemini)
- Handling environment variables safely

## Templates & Patterns

### Service Class Template
```typescript
interface ServiceConfig {
  apiKey: string;
  baseUrl?: string;
}

interface ServiceResponse {
  success: boolean;
  data?: any;
  error?: string;
}

class BaseService {
  protected config: ServiceConfig;

  constructor(config: ServiceConfig) {
    this.config = {
      ...config,
      baseUrl: config.baseUrl || 'https://api.example.com'
    };
  }

  protected async makeRequest(endpoint: string, options: RequestInit = {}): Promise<ServiceResponse> {
    try {
      const response = await fetch(`${this.config.baseUrl}${endpoint}`, {
        ...options,
        headers: {
          'Content-Type': 'application/json',
          ...options.headers,
        }
      });

      if (!response.ok) {
        return { success: false, error: `HTTP ${response.status}` };
      }

      const data = await response.json();
      return { success: true, data };
    } catch (error: any) {
      return { success: false, error: error.message };
    }
  }
}
```

### API Integration Pattern
```typescript
// For API keys that should only be used at runtime, not build time
const apiConfig = {
  apiKey: process.env.API_KEY || '' // Empty string for builds
};

// Health check that doesn't fail during build
if (!apiConfig.apiKey && process.env.NODE_ENV !== 'production') {
  console.warn('API key not set, functionality may be limited');
}
```

### Interface Definition Best Practices
```typescript
// Use descriptive names
export interface VLARequest {
  image?: Uint8Array;
  text: string;
  context?: Record<string, any>;
}

export interface VLEResponse {
  action_sequence: string[];
  confidence_score: number;
  reasoning: string;
  parameters?: Record<string, any>;
  scene_understanding?: string;
  target_object?: string;
}
```

## Common Issues & Solutions
- **Build-time vs Runtime**: Separate environment variable usage
- **Type Safety**: Always define clear interfaces
- **API Integration**: Handle missing keys gracefully during build
- **Module Imports**: Avoid importing browser APIs during server build

## Best Practices
- Export interfaces for reusability
- Use optional properties where appropriate
- Implement proper error handling
- Structure services for testability
- Separate concerns in class design