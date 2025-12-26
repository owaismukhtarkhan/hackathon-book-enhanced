# API Integration Skills

Best practices for integrating external APIs into applications.

## Key Learnings
- Google Gemini API integration for educational applications
- Handling API keys and environment variables securely
- Building AI-powered educational tools
- Managing API rate limits and errors

## API Integration Patterns

### Generic API Service Template
```typescript
interface ApiConfig {
  apiKey: string;
  baseUrl: string;
  timeout?: number;
}

interface ApiResponse<T> {
  success: boolean;
  data?: T;
  error?: string;
  rateLimit?: {
    remaining: number;
    resetTime: number;
  };
}

class ApiService {
  private config: ApiConfig;

  constructor(config: ApiConfig) {
    this.config = {
      ...config,
      timeout: config.timeout || 10000
    };
  }

  async makeRequest<T>(endpoint: string, options: RequestInit = {}): Promise<ApiResponse<T>> {
    try {
      const response = await fetch(`${this.config.baseUrl}${endpoint}`, {
        ...options,
        headers: {
          'Authorization': `Bearer ${this.config.apiKey}`,
          'Content-Type': 'application/json',
          ...options.headers
        },
        signal: AbortSignal.timeout(this.config.timeout)
      });

      if (!response.ok) {
        return {
          success: false,
          error: `API Error: ${response.status} ${response.statusText}`
        };
      }

      const data = await response.json();
      return { success: true, data };
    } catch (error: any) {
      if (error.name === 'AbortError') {
        return { success: false, error: 'Request timeout' };
      }
      return { success: false, error: error.message };
    }
  }
}
```

### Google Gemini Integration Pattern
```typescript
class GeminiIntegration {
  private config: GeminiConfig;

  constructor(config: GeminiConfig) {
    this.config = {
      ...config,
      model: config.model || 'gemini-pro',
      baseUrl: config.baseUrl || 'https://generativelanguage.googleapis.com/v1beta'
    };
  }

  async generateContent(request: GeminiRequest): Promise<GeminiResponse> {
    // Handle missing API key gracefully during build
    if (!this.config.apiKey) {
      return {
        success: false,
        content: '',
        error: 'API key not configured'
      };
    }

    // Prepare request payload
    const payload = {
      contents: [{
        parts: [{
          text: request.context ? `${request.context}\n\n${request.prompt}` : request.prompt
        }]
      }],
      generationConfig: {
        maxOutputTokens: request.maxOutputTokens || 1024,
        temperature: request.temperature || 0.7
      }
    };

    // Make API call
    const response = await fetch(
      `${this.config.baseUrl}/models/${this.config.model}:generateContent?key=${this.config.apiKey}`,
      {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify(payload)
      }
    );

    if (!response.ok) {
      const errorData = await response.json().catch(() => ({}));
      return {
        success: false,
        content: '',
        error: `API Error: ${errorData.error?.message || response.statusText}`
      };
    }

    const data = await response.json();
    const text = data.candidates?.[0]?.content?.parts?.[0]?.text || '';

    return {
      success: true,
      content: text
    };
  }
}
```

## Security Best Practices

### Environment Variable Handling
```javascript
// For build-time safety
const config = {
  apiKey: process.env.GOOGLE_API_KEY || '', // Empty string for builds
};

// Validate at runtime
function validateApiKey() {
  if (!config.apiKey) {
    console.warn('API key not set, functionality will be limited');
    return false;
  }
  return true;
}
```

### Rate Limiting and Error Handling
- Implement exponential backoff for retries
- Handle rate limit responses appropriately
- Cache responses when appropriate
- Implement circuit breaker patterns for reliability

## Best Practices
- Always handle missing API keys gracefully
- Implement proper error handling and fallbacks
- Use environment variables for sensitive data
- Test API integration in different environments
- Monitor API usage and costs
- Implement proper request validation
- Design for offline capability when possible