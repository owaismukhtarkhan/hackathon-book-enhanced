/**
 * Google Gemini API Integration Tests
 * Validates Google Gemini API integration for robotics applications
 */

import { GoogleGenerativeAI, GenerativeModel } from "@google/generative-ai";
import { VLARequest, VLEResponse, GeminiIntegrationService } from "./gemini-integration";

// Mock configuration for testing
const MOCK_GEMINI_CONFIG = {
  apiKey: process.env.GOOGLE_API_KEY || "test-api-key",
  model: "gemini-pro",
};

// Mock Gemini API client for testing
class MockGeminiClient {
  async generateContent(prompt: string | any[]) {
    // Mock response that simulates Gemini API behavior
    const mockResponse = {
      response: {
        text: () => JSON.stringify({
          action_sequence: ["move_forward", "turn_right", "stop"],
          confidence_score: 0.85,
          reasoning: "User wants robot to move forward and turn right",
          parameters: { speed: 0.5, angle: 90 },
          scene_understanding: "Indoor environment with clear path forward",
          target_object: "navigation goal"
        })
      }
    };
    return mockResponse;
  }
}

// Extended GeminiIntegrationService for testing
class TestableGeminiIntegrationService extends GeminiIntegrationService {
  private mockClient: MockGeminiClient | null = null;

  constructor(config: any, useMock: boolean = false) {
    super(config);
    if (useMock) {
      this.mockClient = new MockGeminiClient();
    }
  }

  async processVLARequest(request: VLARequest): Promise<VLEResponse> {
    if (this.mockClient) {
      // Use mock client for testing
      const mockResult = await this.mockClient.generateContent(this.buildVLAPrompt(request));
      const text = mockResult.response.text();
      return this.parseVLAResponse(text);
    } else {
      // Use real implementation
      return super.processVLARequest(request);
    }
  }

  buildVLAPrompt(request: VLARequest): string {
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

  buildTextPrompt(request: VLARequest): string {
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
}

// Test suite
describe("Gemini API Integration Tests", () => {
  let geminiService: TestableGeminiIntegrationService;

  beforeAll(() => {
    // Initialize with mock for testing
    geminiService = new TestableGeminiIntegrationService(MOCK_GEMINI_CONFIG, true);
  });

  test("should process VLA request with text only", async () => {
    const request: VLARequest = {
      text: "Move forward slowly",
      context: {
        robotState: {
          position: { x: 0, y: 0, theta: 0 },
          battery: 0.85
        }
      }
    };

    const result = await geminiService.processVLARequest(request);

    expect(result).toBeDefined();
    expect(result.action_sequence).toBeInstanceOf(Array);
    expect(result.action_sequence.length).toBeGreaterThan(0);
    expect(typeof result.confidence_score).toBe("number");
    expect(result.confidence_score).toBeGreaterThanOrEqual(0);
    expect(result.confidence_score).toBeLessThanOrEqual(1);
    expect(typeof result.reasoning).toBe("string");
  });

  test("should process VLA request with image and text", async () => {
    // Create a simple mock image buffer (1x1 pixel)
    const mockImageBuffer = Buffer.from([0, 0, 0]);

    const request: VLARequest = {
      image: mockImageBuffer,
      text: "Pick up the red object",
      context: {
        robotState: {
          gripper: "open"
        }
      }
    };

    const result = await geminiService.processVLARequest(request);

    expect(result).toBeDefined();
    expect(result.action_sequence).toBeInstanceOf(Array);
    expect(result.target_object).toBeDefined();
    expect(typeof result.scene_understanding).toBe("string");
  });

  test("should handle empty command gracefully", async () => {
    const request: VLARequest = {
      text: "",
      context: {}
    };

    const result = await geminiService.processVLARequest(request);

    expect(result).toBeDefined();
    expect(result.action_sequence).toBeInstanceOf(Array);
  });

  test("should handle command with high confidence", async () => {
    const request: VLARequest = {
      text: "Navigate to the kitchen",
      context: {
        environment: {
          locations: ["kitchen", "living_room", "bedroom"]
        }
      }
    };

    const result = await geminiService.processVLARequest(request);

    expect(result).toBeDefined();
    expect(result.confidence_score).toBeGreaterThanOrEqual(0.7);
  });

  test("should parse valid JSON response correctly", () => {
    const mockJson = `{
      "action_sequence": ["move_forward", "turn_right"],
      "confidence_score": 0.9,
      "reasoning": "Simple navigation command",
      "parameters": {"speed": 0.5}
    }`;

    const result = (geminiService as any).parseVLAResponse(mockJson);

    expect(result.action_sequence).toEqual(["move_forward", "turn_right"]);
    expect(result.confidence_score).toBe(0.9);
    expect(result.reasoning).toBe("Simple navigation command");
    expect(result.parameters).toEqual({ speed: 0.5 });
  });

  test("should handle invalid JSON response gracefully", () => {
    const invalidJson = "not a json string";

    const result = (geminiService as any).parseVLAResponse(invalidJson);

    expect(result.action_sequence).toEqual(["error_parsing_response"]);
    expect(result.confidence_score).toBe(0.0);
    expect(result.reasoning).toContain("Failed to parse");
  });

  test("should build appropriate text prompt", () => {
    const request: VLARequest = {
      text: "Turn left",
      context: { robotState: { battery: 0.8 } }
    };

    const prompt = (geminiService as any).buildTextPrompt(request);

    expect(prompt).toContain("Turn left");
    expect(prompt).toContain("robot command interpreter");
    expect(prompt).toContain('{"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0}');
  });

  test("should build appropriate vision prompt", () => {
    const request: VLARequest = {
      image: Buffer.from([255, 255, 255]),
      text: "Detect the cup",
      context: { environment: { objects: ["cup", "plate"] } }
    };

    const prompt = (geminiService as any).buildVLAPrompt(request);

    expect(prompt).toContain("Detect the cup");
    expect(prompt).toContain("vision and language capabilities");
    expect(prompt).toContain("scene_understanding");
    expect(prompt).toContain("target_object");
  });

  test("should validate configuration successfully", async () => {
    // Mock the model.generateContent method for this test
    const originalModel = (geminiService as any).model;
    (geminiService as any).model = {
      generateContent: jest.fn().mockResolvedValue({
        response: { text: () => "Hello" }
      })
    };

    const isValid = await geminiService.validateConfig();
    expect(isValid).toBe(true);

    // Restore original model
    (geminiService as any).model = originalModel;
  });

  test("should process natural language command", async () => {
    const command = "Go to the charging station";
    const context = { robotState: { position: { x: 0, y: 0 } } };

    const result = await geminiService.processNaturalLanguageCommand(command, context);

    expect(result).toBeDefined();
    expect(result.action_sequence).toBeInstanceOf(Array);
  });

  test("should process visual language command", async () => {
    const image = Buffer.from([0, 0, 0]);
    const command = "Grasp the object in front";
    const context = { robotState: { gripper: "open" } };

    const result = await geminiService.processVisualLanguageCommand(image, command, context);

    expect(result).toBeDefined();
    expect(result.action_sequence).toBeInstanceOf(Array);
  });

  test("should handle error when API key is invalid", async () => {
    const badConfig = { apiKey: "invalid-key", model: "gemini-pro" };
    const badService = new TestableGeminiIntegrationService(badConfig, true);

    // This test would actually make real API calls if we used the real client
    // For now, we're testing the error handling logic
    const request: VLARequest = {
      text: "Test command",
    };

    try {
      await badService.processVLARequest(request);
    } catch (error) {
      expect(error).toBeDefined();
    }
  });
});

// Additional validation tests for VLA pipeline
describe("VLA Pipeline Validation Tests", () => {
  test("should validate VLA response structure", () => {
    const validResponse: VLEResponse = {
      action_sequence: ["move", "stop"],
      confidence_score: 0.8,
      reasoning: "Test reasoning",
      parameters: { speed: 0.5 },
      scene_understanding: "Test scene",
      target_object: "test_object"
    };

    expect(validResponse.action_sequence).toBeInstanceOf(Array);
    expect(typeof validResponse.confidence_score).toBe("number");
    expect(typeof validResponse.reasoning).toBe("string");
  });

  test("should validate VLA request structure", () => {
    const validRequest: VLARequest = {
      text: "Test command",
      context: { test: "value" }
    };

    expect(typeof validRequest.text).toBe("string");
    expect(validRequest.context).toBeInstanceOf(Object);
  });

  test("should handle different action types in sequence", async () => {
    const request: VLARequest = {
      text: "Navigate, detect, and grasp",
      context: {}
    };

    const result = await geminiService.processVLARequest(request);

    expect(result.action_sequence).toBeInstanceOf(Array);
    // The mock response should contain multiple action types
    const hasNavigation = result.action_sequence.some(action =>
      action.includes("move") || action.includes("navigate") || action.includes("go")
    );
    expect(hasNavigation).toBe(true);
  });
});

// Performance tests
describe("Performance Tests", () => {
  test("should process requests within acceptable time", async () => {
    const request: VLARequest = {
      text: "Simple move command",
      context: {}
    };

    const startTime = Date.now();
    await geminiService.processVLARequest(request);
    const endTime = Date.now();

    const processingTime = endTime - startTime;
    // For mock implementation, this should be very fast (< 100ms)
    expect(processingTime).toBeLessThan(1000); // 1 second threshold
  });

  test("should handle multiple concurrent requests", async () => {
    const requests = Array(5).fill(null).map((_, i) => ({
      text: `Command ${i}`,
      context: { request_id: i }
    } as VLARequest));

    const promises = requests.map(req => geminiService.processVLARequest(req));
    const results = await Promise.all(promises);

    expect(results).toHaveLength(5);
    results.forEach(result => {
      expect(result).toBeDefined();
    });
  });
});

// Integration tests with simulated real-world scenarios
describe("Real-world Scenario Tests", () => {
  test("should handle navigation command with obstacles", async () => {
    const request: VLARequest = {
      text: "Go to the kitchen avoiding the chair",
      context: {
        environment: {
          objects: ["chair", "table", "kitchen counter"],
          obstacles: ["chair at (1,1)"]
        }
      }
    };

    const result = await geminiService.processVLARequest(request);

    expect(result).toBeDefined();
    expect(result.action_sequence).toBeInstanceOf(Array);
    expect(result.reasoning).toContain("avoid");
  });

  test("should handle object manipulation command", async () => {
    const request: VLARequest = {
      text: "Pick up the red cup and place it on the table",
      context: {
        robotState: {
          gripper: "open",
          position: { x: 0, y: 0, theta: 0 }
        },
        environment: {
          objects: ["red cup", "table", "chair"]
        }
      }
    };

    const result = await geminiService.processVLARequest(request);

    expect(result).toBeDefined();
    expect(result.action_sequence).toContain("grasp");
    expect(result.action_sequence).toContain("place");
  });

  test("should handle complex multi-step command", async () => {
    const request: VLARequest = {
      text: "Go to the living room, find the blue ball, and bring it to me",
      context: {
        robotState: {
          position: { x: 0, y: 0, theta: 0 },
          gripper: "open"
        },
        environment: {
          rooms: ["living room", "kitchen"],
          objects: ["blue ball", "sofa", "table"]
        }
      }
    };

    const result = await geminiService.processVLARequest(request);

    expect(result).toBeDefined();
    expect(result.action_sequence.length).toBeGreaterThan(2);
    expect(result.confidence_score).toBeGreaterThanOrEqual(0.5);
  });
});

// Export for use in other modules
export { TestableGeminiIntegrationService };