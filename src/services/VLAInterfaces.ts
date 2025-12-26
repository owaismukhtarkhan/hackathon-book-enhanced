/**
 * Vision-Language-Action (VLA) Interfaces for Robotics
 * Defines the data structures and interfaces for VLA pipeline
 */

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

export { VLARequest, VLEResponse };