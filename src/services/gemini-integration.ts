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
    // Check if we're in a browser environment (not during server-side rendering)
    if (typeof window === 'undefined') {
      // During server-side rendering, return a placeholder response
      // This prevents build errors while still allowing the component to render
      return {
        success: false,
        content: '',
        error: 'Gemini API not available during server-side rendering'
      };
    }

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

// Lazy initialization of the singleton instance - avoid initializing during server-side rendering
let geminiIntegration: GeminiIntegration | null = null;

// Function to get or create the singleton instance
function getGeminiIntegration(): GeminiIntegration {
  // Only initialize in browser environments, not during server-side rendering
  if (typeof window !== 'undefined' && !geminiIntegration) {
    // Only access process.env when the function is actually called in browser, not at module load time
    const config: GeminiConfig = {
      apiKey: typeof process !== 'undefined' && process.env ? process.env.GOOGLE_API_KEY || '' : ''
    };
    geminiIntegration = new GeminiIntegration(config);
  }

  // If we're in SSR or the instance wasn't created, return a no-op instance
  if (!geminiIntegration) {
    geminiIntegration = new GeminiIntegration({ apiKey: '' });
  }

  return geminiIntegration;
}

// Import the VLA interfaces
interface VLARequest {
  image?: Buffer;      // Image data for vision tasks
  text: string;        // Natural language command
  context?: Record<string, any>; // Additional context (robot state, environment, etc.)
}

interface VLEResponse {
  action_sequence: string[];
  confidence_score: number;
  reasoning: string;
  parameters?: Record<string, any>;
  scene_understanding?: string;
  target_object?: string;
}

// Extended GeminiIntegrationService class for VLA (Vision-Language-Action) pipeline
class GeminiIntegrationService {
  private geminiIntegration: GeminiIntegration;

  constructor(config: GeminiConfig) {
    this.geminiIntegration = new GeminiIntegration(config);
  }

  /**
   * Process a Vision-Language-Action request using Google Gemini
   */
  async processVLARequest(request: VLARequest): Promise<VLEResponse> {
    // Check if we're in a browser environment (not during server-side rendering)
    if (typeof window === 'undefined') {
      // During server-side rendering, return a placeholder response
      // This prevents build errors while still allowing the component to render
      return {
        action_sequence: [],
        confidence_score: 0,
        reasoning: 'VLA processing not available during server-side rendering',
        parameters: {},
        scene_understanding: 'Not available during build',
        target_object: null
      };
    }

    try {
      let prompt: string;

      if (request.image) {
        // For now, we'll handle image requests by describing the approach
        // In a real implementation, we would need to convert the image to base64
        // and use the Gemini Pro Vision model
        prompt = this.buildVLAPrompt(request);
      } else {
        prompt = this.buildTextPrompt(request);
      }

      const response = await this.geminiIntegration.generateContent({
        prompt: prompt,
        maxOutputTokens: 1024,
        temperature: 0.3
      });

      if (!response.success) {
        throw new Error(response.error || 'Failed to generate content');
      }

      return this.parseVLAResponse(response.content);
    } catch (error) {
      console.error('Error processing VLA request:', error);
      return {
        action_sequence: ['error'],
        confidence_score: 0,
        reasoning: `Error processing request: ${error instanceof Error ? error.message : 'Unknown error'}`,
        parameters: {},
      };
    }
  }

  /**
   * Build a prompt for vision-language processing
   */
  private buildVLAPrompt(request: VLARequest): string {
    const context = request.context ? JSON.stringify(request.context) : "No additional context provided";

    return `
You are an advanced robot assistant with vision and language capabilities.
Analyze the provided image and interpret the user's command.

User Command: "${request.text}"

Additional Context: ${context}

Provide a detailed JSON response with the following structure:
{
  "action_sequence": ["list", "of", "actions", "to", "execute"],
  "confidence_score": 0.85,
  "reasoning": "Brief explanation of your interpretation",
  "parameters": {
    "object_name": "name of target object if relevant",
    "position": {"x": 0.0, "y": 0.0, "z": 0.0},
    "orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0}
  },
  "scene_understanding": "Description of what you see in the image",
  "target_object": "Name of the specific object related to the command"
}

Respond only with valid JSON, no additional text.
    `;
  }

  /**
   * Build a prompt for text-only processing
   */
  private buildTextPrompt(request: VLARequest): string {
    const context = request.context ? JSON.stringify(request.context) : "No additional context provided";

    return `
You are an advanced robot command interpreter.
Interpret the user's command in the context of robotics.

User Command: "${request.text}"

Additional Context: ${context}

Provide a detailed JSON response with the following structure:
{
  "action_sequence": ["list", "of", "actions", "to", "execute"],
  "confidence_score": 0.85,
  "reasoning": "Brief explanation of your interpretation",
  "parameters": {
    "object_name": "name of target object if relevant",
    "position": {"x": 0.0, "y": 0.0, "z": 0.0},
    "orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0}
  }
}

Respond only with valid JSON, no additional text.
    `;
  }

  /**
   * Parse the Gemini response into structured VLA data
   */
  private parseVLAResponse(responseText: string): VLEResponse {
    try {
      // Clean the response text
      let cleanText = responseText.trim();
      if (cleanText.startsWith('```json')) {
        cleanText = cleanText.substring(7); // Remove ```json
      }
      if (cleanText.endsWith('```')) {
        cleanText = cleanText.substring(0, cleanText.length - 3); // Remove ```
      }
      cleanText = cleanText.trim();

      // Parse JSON
      const parsed = JSON.parse(cleanText);

      return {
        action_sequence: parsed.action_sequence || [],
        confidence_score: parsed.confidence_score || 0,
        reasoning: parsed.reasoning || 'No reasoning provided',
        parameters: parsed.parameters || {},
        scene_understanding: parsed.scene_understanding,
        target_object: parsed.target_object
      };
    } catch (e) {
      console.error('Error parsing Gemini response:', e);
      // Return a default response in case of parsing failure
      return {
        action_sequence: ['error_parsing_response'],
        confidence_score: 0,
        reasoning: `Failed to parse Gemini response: ${responseText.substring(0, 200)}...`,
        parameters: {}
      };
    }
  }

  /**
   * Validate the Gemini API configuration
   */
  async validateConfig(): Promise<boolean> {
    try {
      // Try a simple test request to validate the API
      const response = await this.geminiIntegration.generateContent({
        prompt: "Hello, are you working?",
        temperature: 0.1
      });
      return response.success && response.content.length > 0;
    } catch (e) {
      console.error('Gemini API validation failed:', e);
      return false;
    }
  }

  /**
   * Process a natural language command to robot action sequence
   */
  async processNaturalLanguageCommand(command: string, context?: Record<string, any>): Promise<VLEResponse> {
    const request: VLARequest = { text: command, context };
    return this.processVLARequest(request);
  }

  /**
   * Process visual scene understanding with language
   */
  async processVisualLanguageCommand(image: Buffer, command: string, context?: Record<string, any>): Promise<VLEResponse> {
    const request: VLARequest = { image, text: command, context };
    return this.processVLARequest(request);
  }
}

export { GeminiIntegration, GeminiConfig, GeminiRequest, GeminiResponse, getGeminiIntegration, GeminiIntegrationService, VLARequest, VLEResponse };
export default getGeminiIntegration;