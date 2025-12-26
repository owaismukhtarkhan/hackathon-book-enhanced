/**
 * Google Gemini API integration for Physical AI & Humanoid Robotics Book
 * Provides AI capabilities for content generation and student assistance
 * while adhering to the constitutional requirement of using Google Gemini
 * instead of other AI providers like OpenAI.
 */

interface GeminiConfig {
  apiKey: string;
  model?: string; // Default to 'gemini-pro' or 'gemini-1.5-pro'
  baseUrl?: string; // Default to 'https://generativelanguage.googleapis.com/v1beta'
}

interface GeminiRequest {
  prompt: string;
  context?: string;
  maxOutputTokens?: number;
  temperature?: number;
}

interface GeminiResponse {
  success: boolean;
  content: string;
  usage?: {
    promptTokens: number;
    completionTokens: number;
    totalTokens: number;
  };
  error?: string;
}

class GeminiIntegration {
  private config: GeminiConfig;
  private baseUrl: string;

  constructor(config: GeminiConfig) {
    this.config = {
      ...config,
      model: config.model || 'gemini-1.5-pro-latest',
      baseUrl: config.baseUrl || 'https://generativelanguage.googleapis.com/v1beta'
    };
    this.baseUrl = this.config.baseUrl;
  }

  /**
   * Generate content using Google Gemini API
   */
  public async generateContent(request: GeminiRequest): Promise<GeminiResponse> {
    if (!this.config.apiKey) {
      return {
        success: false,
        content: '',
        error: 'Gemini API key is not configured'
      };
    }

    try {
      // Prepare the request payload for Gemini API
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

      // Make the API call to Gemini
      const response = await fetch(
        `${this.baseUrl}/models/${this.config.model}:generateContent?key=${this.config.apiKey}`,
        {
          method: 'POST',
          headers: {
            'Content-Type': 'application/json'
          },
          body: JSON.stringify(payload)
        }
      );

      if (!response.ok) {
        const errorData = await response.json().catch(() => ({}));
        return {
          success: false,
          content: '',
          error: `Gemini API error: ${response.status} ${response.statusText}. ${errorData.error?.message || ''}`
        };
      }

      const data = await response.json();
      const text = data.candidates?.[0]?.content?.parts?.[0]?.text || '';

      // Extract token usage if available
      const usage = data.usageMetadata ? {
        promptTokens: data.usageMetadata.promptTokenCount || 0,
        completionTokens: data.usageMetadata.candidatesTokenCount || 0,
        totalTokens: data.usageMetadata.totalTokenCount || 0
      } : undefined;

      return {
        success: true,
        content: text,
        usage
      };
    } catch (error: any) {
      return {
        success: false,
        content: '',
        error: `Gemini API request failed: ${error.message || 'Unknown error'}`
      };
    }
  }

  /**
   * Generate educational content specifically tailored for the Physical AI curriculum
   */
  public async generateEducationalContent(topic: string, level: 'beginner' | 'intermediate' | 'advanced' = 'beginner', context?: string): Promise<GeminiResponse> {
    const prompt = this.createEducationalPrompt(topic, level);
    const fullContext = context ? `Additional context: ${context}` : '';

    return await this.generateContent({
      prompt,
      context: fullContext,
      maxOutputTokens: 1024,
      temperature: 0.5 // Lower temperature for more consistent educational content
    });
  }

  /**
   * Create a prompt specifically designed for educational content generation
   */
  private createEducationalPrompt(topic: string, level: 'beginner' | 'intermediate' | 'advanced'): string {
    const levelDescription = level === 'beginner'
      ? 'Explain in simple terms with practical examples, assuming no prior knowledge'
      : level === 'intermediate'
        ? 'Provide detailed explanation with technical depth, assuming basic understanding'
        : 'Provide advanced technical explanation with implementation details, assuming expert knowledge';

    return `Generate educational content about "${topic}" for the Physical AI & Humanoid Robotics curriculum. ${levelDescription}. Focus on practical implementation and real-world applications. Format the response with clear sections and examples where applicable. The content should be suitable for absolute beginners if level is beginner.`;
  }

  /**
   * Evaluate student code or responses using Gemini
   */
  public async evaluateStudentSubmission(submission: string, requirements: string[]): Promise<GeminiResponse> {
    const prompt = `Evaluate the following student submission for the Physical AI & Humanoid Robotics course:\n\n${submission}\n\nRequirements: ${requirements.join(', ')}\n\nProvide feedback on correctness, adherence to requirements, and suggestions for improvement.`;

    return await this.generateContent({
      prompt,
      maxOutputTokens: 512,
      temperature: 0.3 // Lower temperature for more consistent evaluation
    });
  }

  /**
   * Generate ROS 2 specific code examples
   */
  public async generateRos2CodeExample(concept: string): Promise<GeminiResponse> {
    const prompt = `Generate a TypeScript/Python code example for ROS 2 demonstrating "${concept}". Include proper imports, class structure, and comments explaining key concepts. Make it suitable for beginners learning ROS 2 fundamentals.`;

    return await this.generateContent({
      prompt,
      maxOutputTokens: 1024,
      temperature: 0.2 // Very low temperature for consistent code generation
    });
  }

  /**
   * Generate Gazebo simulation examples
   */
  public async generateGazeboExample(concept: string): Promise<GeminiResponse> {
    const prompt = `Generate a Gazebo simulation example for "${concept}". Include world file structure, model definitions, and explanation of key parameters. Make it suitable for beginners learning robot simulation.`;

    return await this.generateContent({
      prompt,
      maxOutputTokens: 1024,
      temperature: 0.2
    });
  }
}

// Example usage configuration
// This would typically be loaded from environment variables or configuration
const geminiConfig: GeminiConfig = {
  apiKey: process.env.GOOGLE_API_KEY || '' // In a real implementation, this would come from secure storage
};

// Singleton instance
const geminiIntegration = new GeminiIntegration(geminiConfig);

export { GeminiIntegration, GeminiConfig, GeminiRequest, GeminiResponse };
export default geminiIntegration;