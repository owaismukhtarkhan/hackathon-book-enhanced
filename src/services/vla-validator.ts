/**
 * Vision-Language-Action (VLA) Pipeline Validation Framework
 * Validates the complete VLA pipeline for robotics applications
 */

// Define the VLEResponse interface locally to avoid build issues
export interface VLEResponse {
  action_sequence: string[];
  confidence_score: number;
  reasoning: string;
  parameters?: { [key: string]: any };
  scene_understanding?: string;
  target_object?: string;
}

export interface VLAValidationResult {
  isValid: boolean;
  confidence: number;
  errors: string[];
  warnings: string[];
  metrics: VLAValidationMetrics;
  timestamp: Date;
}

export interface VLAValidationMetrics {
  actionSequenceValid: boolean;
  confidenceThresholdMet: boolean;
  safetyConstraintsMet: boolean;
  completenessScore: number; // 0.0 to 1.0
  semanticCoherence: number; // 0.0 to 1.0
  executionFeasibility: number; // 0.0 to 1.0
  responseTime: number; // in milliseconds
}

export interface VLAValidationConfig {
  minConfidenceThreshold: number; // e.g., 0.7 for 70%
  maxResponseTime: number; // in milliseconds
  allowedActions: string[]; // List of allowed robot actions
  safetyConstraints: SafetyConstraint[];
  completenessWeights: {
    actionSequence: number;
    reasoning: number;
    parameters: number;
  };
}

export interface SafetyConstraint {
  type: 'kinematic' | 'dynamic' | 'environmental' | 'operational';
  condition: (response: VLEResponse) => boolean;
  errorMessage: string;
}

export class VLAValidator {
  private config: VLAValidationConfig;

  constructor(config?: Partial<VLAValidationConfig>) {
    this.config = {
      minConfidenceThreshold: config?.minConfidenceThreshold ?? 0.7,
      maxResponseTime: config?.maxResponseTime ?? 5000, // 5 seconds
      allowedActions: config?.allowedActions ?? [
        'move_forward', 'move_backward', 'turn_left', 'turn_right',
        'stop', 'navigate_to', 'detect_object', 'grasp_object',
        'release_object', 'go_to', 'wait', 'speak', 'listen'
      ],
      safetyConstraints: config?.safetyConstraints ?? [
        // Default safety constraints
        {
          type: 'operational',
          condition: (response) => response.confidence_score >= 0.5,
          errorMessage: 'Action confidence too low for execution'
        }
      ],
      completenessWeights: config?.completenessWeights ?? {
        actionSequence: 0.5,
        reasoning: 0.3,
        parameters: 0.2
      }
    };
  }

  /**
   * Validates a complete VLA response
   */
  validateVLA(response: VLEResponse, inputCommand: string = ""): VLAValidationResult {
    const startTime = Date.now();

    const errors: string[] = [];
    const warnings: string[] = [];
    let isValid = true;
    let overallConfidence = response.confidence_score;

    // Validate action sequence
    const actionValidation = this.validateActionSequence(response.action_sequence);
    if (!actionValidation.isValid) {
      errors.push(...actionValidation.errors);
      isValid = false;
      overallConfidence *= 0.8; // Reduce confidence due to action errors
    }

    // Check confidence threshold
    const confidenceValid = this.validateConfidenceThreshold(response.confidence_score);
    if (!confidenceValid) {
      errors.push(`Confidence score ${response.confidence_score} below threshold ${this.config.minConfidenceThreshold}`);
      isValid = false;
    }

    // Validate safety constraints
    const safetyValidation = this.validateSafetyConstraints(response);
    if (!safetyValidation.isValid) {
      errors.push(...safetyValidation.errors);
      isValid = false;
      overallConfidence *= 0.7; // Reduce confidence due to safety issues
    }

    // Validate completeness
    const completenessScore = this.calculateCompletenessScore(response);

    // Validate semantic coherence
    const semanticCoherence = this.calculateSemanticCoherence(response, inputCommand);

    // Validate execution feasibility
    const executionFeasibility = this.calculateExecutionFeasibility(response);

    const responseTime = Date.now() - startTime;

    // Calculate final metrics
    const metrics: VLAValidationMetrics = {
      actionSequenceValid: actionValidation.isValid,
      confidenceThresholdMet: confidenceValid,
      safetyConstraintsMet: safetyValidation.isValid,
      completenessScore,
      semanticCoherence,
      executionFeasibility,
      responseTime
    };

    // Add warnings for low scores
    if (completenessScore < 0.7) {
      warnings.push(`Completeness score is low: ${completenessScore.toFixed(2)}`);
    }
    if (semanticCoherence < 0.7) {
      warnings.push(`Semantic coherence is low: ${semanticCoherence.toFixed(2)}`);
    }
    if (executionFeasibility < 0.7) {
      warnings.push(`Execution feasibility is low: ${executionFeasibility.toFixed(2)}`);
    }

    return {
      isValid,
      confidence: Math.min(overallConfidence, 1.0),
      errors,
      warnings,
      metrics,
      timestamp: new Date()
    };
  }

  /**
   * Validates the action sequence for correctness and safety
   */
  private validateActionSequence(actions: string[]): { isValid: boolean; errors: string[] } {
    const errors: string[] = [];
    let isValid = true;

    if (!actions || actions.length === 0) {
      errors.push("Action sequence is empty");
      isValid = false;
    }

    for (let i = 0; i < actions.length; i++) {
      const action = actions[i];
      let actionName: string;
      let actionParams: any;

      // Handle both string actions and object actions
      if (typeof action === 'string') {
        actionName = action;
        actionParams = {};
      } else if (typeof action === 'object' && action !== null) {
        actionName = action.action || action.name || '';
        actionParams = action.parameters || {};
      } else {
        errors.push(`Invalid action format at index ${i}: ${JSON.stringify(action)}`);
        isValid = false;
        continue;
      }

      // Check if action is in allowed list
      if (!this.config.allowedActions.some(allowed =>
        allowed.toLowerCase().includes(actionName.toLowerCase()) ||
        actionName.toLowerCase().includes(allowed.toLowerCase())
      )) {
        errors.push(`Unknown or disallowed action: ${actionName}`);
        isValid = false;
      }

      // Check for potentially dangerous action sequences
      if (i > 0) {
        const prevAction = actions[i-1];
        if (typeof prevAction === 'string' &&
            prevAction.toLowerCase().includes('grasp') &&
            actionName.toLowerCase().includes('move') &&
            !actionName.toLowerCase().includes('slowly')) {
          warnings.push(`Potentially unsafe: moving quickly after grasping object at index ${i}`);
        }
      }
    }

    return { isValid, errors };
  }

  /**
   * Validates that confidence meets the minimum threshold
   */
  private validateConfidenceThreshold(confidence: number): boolean {
    return confidence >= this.config.minConfidenceThreshold;
  }

  /**
   * Validates safety constraints
   */
  private validateSafetyConstraints(response: VLEResponse): { isValid: boolean; errors: string[] } {
    const errors: string[] = [];
    let isValid = true;

    for (const constraint of this.config.safetyConstraints) {
      if (!constraint.condition(response)) {
        errors.push(constraint.errorMessage);
        isValid = false;
      }
    }

    return { isValid, errors };
  }

  /**
   * Calculates completeness score based on response components
   */
  private calculateCompletenessScore(response: VLEResponse): number {
    let score = 0;
    const weights = this.config.completenessWeights;

    // Score action sequence (required)
    if (response.action_sequence && response.action_sequence.length > 0) {
      score += weights.actionSequence;
    }

    // Score reasoning (highly valued)
    if (response.reasoning && response.reasoning.trim().length > 10) { // Meaningful reasoning
      score += weights.reasoning;
    } else if (response.reasoning && response.reasoning.trim().length > 0) {
      score += weights.reasoning * 0.5; // Partial credit for minimal reasoning
    }

    // Score parameters (if applicable)
    if (response.parameters && Object.keys(response.parameters).length > 0) {
      score += weights.parameters;
    }

    return Math.min(1.0, score);
  }

  /**
   * Calculates semantic coherence between input and response
   */
  private calculateSemanticCoherence(response: VLEResponse, inputCommand: string): number {
    if (!inputCommand || !response.reasoning) {
      return response.action_sequence.length > 0 ? 0.5 : 0.0; // Partial credit if actions exist
    }

    // Simple keyword matching for semantic coherence
    const inputLower = inputCommand.toLowerCase();
    const reasoningLower = response.reasoning.toLowerCase();

    // Count how many input keywords appear in reasoning or action sequence
    const inputWords = inputLower.split(/\s+/).filter(word => word.length > 2);
    const reasoningWords = reasoningLower.split(/\s+/).filter(word => word.length > 2);

    let matchingWords = 0;
    for (const word of inputWords) {
      if (reasoningLower.includes(word) ||
          response.action_sequence.some(action =>
            typeof action === 'string' && action.toLowerCase().includes(word))) {
        matchingWords++;
      }
    }

    const coherenceRatio = inputWords.length > 0 ? matchingWords / inputWords.length : 0;
    return Math.min(1.0, coherenceRatio * 2); // Boost the ratio as it's an important metric
  }

  /**
   * Calculates execution feasibility based on action sequence
   */
  private calculateExecutionFeasibility(response: VLEResponse): number {
    if (!response.action_sequence || response.action_sequence.length === 0) {
      return 0.0;
    }

    let feasibleActions = 0;
    let totalActions = response.action_sequence.length;

    for (const action of response.action_sequence) {
      let actionName: string;

      if (typeof action === 'string') {
        actionName = action;
      } else if (typeof action === 'object' && action !== null) {
        actionName = action.action || action.name || '';
      } else {
        continue; // Skip invalid actions for feasibility calculation
      }

      // Determine if action is generally feasible
      const isFeasible = this.isActionFeasible(actionName);
      if (isFeasible) {
        feasibleActions++;
      }
    }

    return totalActions > 0 ? feasibleActions / totalActions : 0.0;
  }

  /**
   * Determines if an action is generally feasible
   */
  private isActionFeasible(actionName: string): boolean {
    // List of actions that are generally not feasible or problematic
    const infeasibleActions = [
      'destroy', 'break', 'damage', 'self_destruct', 'explode',
      'teleport', 'fly', 'phase_through_objects'
    ];

    const actionLower = actionName.toLowerCase();

    // Check against infeasible actions
    for (const infeasible of infeasibleActions) {
      if (actionLower.includes(infeasible)) {
        return false;
      }
    }

    // Check for movement actions that seem reasonable
    const movementActions = [
      'move', 'go', 'navigate', 'turn', 'rotate', 'forward', 'backward', 'left', 'right'
    ];

    for (const movement of movementActions) {
      if (actionLower.includes(movement)) {
        return true;
      }
    }

    // Check for manipulation actions that seem reasonable
    const manipulationActions = [
      'grasp', 'pick', 'place', 'release', 'hold', 'carry'
    ];

    for (const manipulation of manipulationActions) {
      if (actionLower.includes(manipulation)) {
        return true;
      }
    }

    // Check for sensing/perception actions that seem reasonable
    const sensingActions = [
      'detect', 'find', 'locate', 'see', 'observe', 'scan'
    ];

    for (const sensing of sensingActions) {
      if (actionLower.includes(sensing)) {
        return true;
      }
    }

    // Default to true for unknown actions that aren't explicitly infeasible
    return true;
  }

  /**
   * Validates multiple VLA responses together
   */
  validateMultipleVLAs(responses: { response: VLEResponse; input: string }[]): VLAValidationResult[] {
    return responses.map(item => this.validateVLA(item.response, item.input));
  }

  /**
   * Validates a VLA response sequence (for multi-step tasks)
   */
  validateVLASequence(responses: VLEResponse[], inputCommand: string = ""): VLAValidationResult {
    if (responses.length === 0) {
      return {
        isValid: false,
        confidence: 0.0,
        errors: ["VLA sequence is empty"],
        warnings: [],
        metrics: {
          actionSequenceValid: false,
          confidenceThresholdMet: false,
          safetyConstraintsMet: false,
          completenessScore: 0.0,
          semanticCoherence: 0.0,
          executionFeasibility: 0.0,
          responseTime: 0
        },
        timestamp: new Date()
      };
    }

    // Aggregate individual validations
    const individualResults = responses.map(response =>
      this.validateVLA(response, inputCommand)
    );

    // Calculate overall metrics
    const overallConfidence = individualResults.reduce((sum, r) => sum + r.confidence, 0) / individualResults.length;
    const avgCompleteness = individualResults.reduce((sum, r) => sum + r.metrics.completenessScore, 0) / individualResults.length;
    const avgCoherence = individualResults.reduce((sum, r) => sum + r.metrics.semanticCoherence, 0) / individualResults.length;
    const avgFeasibility = individualResults.reduce((sum, r) => sum + r.metrics.executionFeasibility, 0) / individualResults.length;

    // Check for consistency across sequence
    const errors: string[] = [];
    const warnings: string[] = [];
    let isValid = true;

    for (const result of individualResults) {
      if (!result.isValid) {
        isValid = false;
      }
      errors.push(...result.errors);
      warnings.push(...result.warnings);
    }

    // Check for potential sequence issues
    if (responses.length > 1) {
      for (let i = 1; i < responses.length; i++) {
        // Check for contradictory actions
        const prevActions = new Set(responses[i-1].action_sequence);
        const currActions = new Set(responses[i].action_sequence);

        if (prevActions.has('stop') && !currActions.has('start') && currActions.size > 0) {
          warnings.push(`Potential issue: Actions follow a 'stop' command at sequence position ${i}`);
        }
      }
    }

    return {
      isValid,
      confidence: overallConfidence,
      errors,
      warnings,
      metrics: {
        actionSequenceValid: individualResults.every(r => r.metrics.actionSequenceValid),
        confidenceThresholdMet: individualResults.every(r => r.metrics.confidenceThresholdMet),
        safetyConstraintsMet: individualResults.every(r => r.metrics.safetyConstraintsMet),
        completenessScore: avgCompleteness,
        semanticCoherence: avgCoherence,
        executionFeasibility: avgFeasibility,
        responseTime: individualResults.reduce((sum, r) => sum + r.metrics.responseTime, 0) / individualResults.length
      },
      timestamp: new Date()
    };
  }

  /**
   * Updates the validation configuration
   */
  updateConfig(newConfig: Partial<VLAValidationConfig>): void {
    this.config = { ...this.config, ...newConfig };
  }

  /**
   * Gets the current validation configuration
   */
  getConfig(): VLAValidationConfig {
    return { ...this.config };
  }

  /**
   * Creates a safety constraint for a specific scenario
   */
  createSafetyConstraint(
    type: SafetyConstraint['type'],
    condition: SafetyConstraint['condition'],
    errorMessage: SafetyConstraint['errorMessage']
  ): SafetyConstraint {
    return { type, condition, errorMessage };
  }

  /**
   * Adds a safety constraint to the validator
   */
  addSafetyConstraint(constraint: SafetyConstraint): void {
    this.config.safetyConstraints.push(constraint);
  }

  /**
   * Removes a safety constraint by error message
   */
  removeSafetyConstraint(errorMessage: string): void {
    this.config.safetyConstraints = this.config.safetyConstraints.filter(
      constraint => constraint.errorMessage !== errorMessage
    );
  }
}

// Default validator instance
export const defaultVLAValidator = new VLAValidator();

// Example usage and testing
if (require.main === module) {
  // Example validation
  const sampleResponse: VLEResponse = {
    action_sequence: ["navigate_to", "detect_object", "grasp_object"],
    confidence_score: 0.85,
    reasoning: "User wants to pick up an object, so I will navigate to it, detect it, then grasp it",
    parameters: { target: "red cup", location: { x: 1.0, y: 2.0, z: 0.0 } },
    scene_understanding: "Room with table and objects",
    target_object: "red cup"
  };

  const validator = new VLAValidator();
  const result = validator.validateVLA(sampleResponse, "Pick up the red cup");

  console.log("VLA Validation Result:");
  console.log(`Valid: ${result.isValid}`);
  console.log(`Confidence: ${result.confidence}`);
  console.log(`Errors: ${result.errors.join(', ')}`);
  console.log(`Warnings: ${result.warnings.join(', ')}`);
  console.log(`Metrics:`, result.metrics);
}

// Export for use in other modules
export default VLAValidator;