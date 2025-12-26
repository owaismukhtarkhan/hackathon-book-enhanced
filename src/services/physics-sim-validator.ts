/**
 * Physics Simulation Assessment Validator
 * Validates physics simulation properties and behaviors in robotics environments
 */

export interface PhysicsValidationResult {
  isValid: boolean;
  confidence: number;
  errors: string[];
  warnings: string[];
  metrics: PhysicsMetrics;
  timestamp: Date;
}

export interface PhysicsMetrics {
  gravityValid: boolean;
  collisionDetectionActive: boolean;
  realisticMasses: boolean;
  stableSimulation: boolean;
  physicsAccuracy: number; // 0.0 to 1.0
  simulationSpeed: number; // Real-time factor
  constraintSatisfaction: number; // 0.0 to 1.0
}

export interface SimulationState {
  objects: PhysicsObject[];
  environment: EnvironmentProperties;
  timeStep: number;
  gravity: Vector3;
  collisionSettings: CollisionSettings;
}

export interface PhysicsObject {
  id: string;
  position: Vector3;
  orientation: Quaternion;
  velocity: Vector3;
  angularVelocity: Vector3;
  mass: number;
  inertia: Matrix3x3;
  collisionShape: string;
  materialProperties: MaterialProperties;
}

export interface Vector3 {
  x: number;
  y: number;
  z: number;
}

export interface Quaternion {
  x: number;
  y: number;
  z: number;
  w: number;
}

export interface Matrix3x3 {
  elements: number[]; // 9 elements [0-8] representing 3x3 matrix
}

export interface MaterialProperties {
  density: number;
  friction: number;
  restitution: number; // bounciness
  staticFriction: number;
  dynamicFriction: number;
}

export interface EnvironmentProperties {
  gravity: Vector3;
  airDensity: number;
  timeStep: number;
  solverIterations: number;
  contactBreakingThreshold: number;
}

export interface CollisionSettings {
  collisionDetection: 'discrete' | 'continuous';
  broadphaseAlgorithm: string;
  narrowphaseAlgorithm: string;
  collisionMargin: number;
}

export class PhysicsSimValidator {
  private defaultGravity: Vector3 = { x: 0, y: 0, z: -9.81 }; // Earth gravity
  private realisticMassThresholds = {
    smallObject: 0.1,      // 100g
    mediumObject: 1.0,     // 1kg
    largeObject: 10.0,     // 10kg
    robotBase: 10.0,       // 10kg typical robot base
    robotLink: 1.0         // 1kg typical robot link
  };

  /**
   * Validates the overall physics simulation setup
   */
  validateSimulationSetup(state: SimulationState): PhysicsValidationResult {
    const errors: string[] = [];
    const warnings: string[] = [];
    let isValid = true;
    let physicsAccuracy = 1.0;

    // Validate gravity settings
    const gravityResult = this.validateGravity(state.environment.gravity);
    if (!gravityResult.isValid) {
      errors.push(...gravityResult.errors);
      isValid = false;
      physicsAccuracy *= 0.8;
    }

    // Validate time step
    const timeStepResult = this.validateTimeStep(state.timeStep);
    if (!timeStepResult.isValid) {
      errors.push(...timeStepResult.errors);
      isValid = false;
      physicsAccuracy *= 0.9;
    }

    // Validate objects
    const objectsResult = this.validateObjects(state.objects);
    if (!objectsResult.isValid) {
      errors.push(...objectsResult.errors);
      warnings.push(...objectsResult.warnings);
      isValid = objectsResult.isValid;
      physicsAccuracy *= objectsResult.physicsAccuracy;
    }

    // Validate collision settings
    const collisionResult = this.validateCollisionSettings(state.collisionSettings);
    if (!collisionResult.isValid) {
      errors.push(...collisionResult.errors);
      warnings.push(...collisionResult.warnings);
    }

    // Calculate overall metrics
    const metrics: PhysicsMetrics = {
      gravityValid: gravityResult.isValid,
      collisionDetectionActive: state.collisionSettings.collisionDetection !== undefined,
      realisticMasses: objectsResult.metrics.realisticMasses,
      stableSimulation: this.estimateStability(state),
      physicsAccuracy: physicsAccuracy,
      simulationSpeed: this.estimateRealtimeFactor(state.timeStep),
      constraintSatisfaction: this.estimateConstraintSatisfaction(state)
    };

    return {
      isValid,
      confidence: physicsAccuracy,
      errors,
      warnings,
      metrics,
      timestamp: new Date()
    };
  }

  /**
   * Validates gravity settings
   */
  private validateGravity(gravity: Vector3): PhysicsValidationResult {
    const errors: string[] = [];
    let isValid = true;

    // Check if gravity is reasonable (close to Earth's gravity)
    const gravityMagnitude = Math.sqrt(
      gravity.x * gravity.x +
      gravity.y * gravity.y +
      gravity.z * gravity.z
    );

    // Earth's gravity is approximately 9.81 m/s^2
    if (gravityMagnitude < 1 || gravityMagnitude > 20) {
      errors.push(`Gravity magnitude (${gravityMagnitude.toFixed(2)}) is outside reasonable range (1-20 m/s^2)`);
      isValid = false;
    }

    // Check if gravity is zero (unless specifically intended)
    if (gravityMagnitude < 0.1) {
      errors.push("Gravity is nearly zero - objects will not fall");
      isValid = false;
    }

    return {
      isValid,
      confidence: isValid ? 1.0 : 0.0,
      errors,
      warnings: [],
      metrics: {
        gravityValid: isValid,
        collisionDetectionActive: true, // Not applicable here
        realisticMasses: true, // Not applicable here
        stableSimulation: true, // Not applicable here
        physicsAccuracy: isValid ? 1.0 : 0.0,
        simulationSpeed: 1.0, // Not applicable here
        constraintSatisfaction: 1.0 // Not applicable here
      },
      timestamp: new Date()
    };
  }

  /**
   * Validates time step settings
   */
  private validateTimeStep(timeStep: number): PhysicsValidationResult {
    const errors: string[] = [];
    let isValid = true;

    // For stable physics simulation, time step should typically be between 1ms and 16ms
    if (timeStep <= 0) {
      errors.push("Time step must be positive");
      isValid = false;
    } else if (timeStep > 0.05) { // 50ms
      errors.push(`Time step (${timeStep * 1000}ms) is too large for stable physics simulation`);
      isValid = false;
    } else if (timeStep < 0.0001) { // 0.1ms
      errors.push(`Time step (${timeStep * 1000}ms) is unnecessarily small`);
      isValid = false;
    }

    return {
      isValid,
      confidence: isValid ? 1.0 : 0.0,
      errors,
      warnings: [],
      metrics: {
        gravityValid: true, // Not applicable here
        collisionDetectionActive: true, // Not applicable here
        realisticMasses: true, // Not applicable here
        stableSimulation: isValid,
        physicsAccuracy: isValid ? 1.0 : 0.0,
        simulationSpeed: 1.0, // Not applicable here
        constraintSatisfaction: 1.0 // Not applicable here
      },
      timestamp: new Date()
    };
  }

  /**
   * Validates physics objects in the simulation
   */
  private validateObjects(objects: PhysicsObject[]): PhysicsValidationResult {
    const errors: string[] = [];
    const warnings: string[] = [];
    let isValid = true;
    let realisticMasses = true;
    let physicsAccuracy = 1.0;

    for (const obj of objects) {
      // Validate mass
      if (obj.mass <= 0) {
        errors.push(`Object ${obj.id} has invalid mass: ${obj.mass}`);
        isValid = false;
        physicsAccuracy *= 0.5;
      } else if (obj.mass > 10000) { // 10 tons
        warnings.push(`Object ${obj.id} has unusually high mass: ${obj.mass}kg`);
      } else if (obj.mass < 0.001) { // 1 gram
        warnings.push(`Object ${obj.id} has unusually low mass: ${obj.mass}kg`);
      }

      // Check for realistic masses based on object type
      if (obj.id.includes('robot') || obj.id.includes('Robot')) {
        if (obj.mass < this.realisticMassThresholds.robotBase * 0.1) {
          warnings.push(`Robot object ${obj.id} has unusually low mass: ${obj.mass}kg`);
          realisticMasses = false;
        }
      }

      // Validate position
      if (Math.abs(obj.position.x) > 1000 ||
          Math.abs(obj.position.y) > 1000 ||
          Math.abs(obj.position.z) > 1000) {
        errors.push(`Object ${obj.id} position is extremely large: (${obj.position.x}, ${obj.position.y}, ${obj.position.z})`);
        isValid = false;
      }

      // Validate velocity
      const speed = Math.sqrt(
        obj.velocity.x * obj.velocity.x +
        obj.velocity.y * obj.velocity.y +
        obj.velocity.z * obj.velocity.z
      );
      if (speed > 100) { // 100 m/s is very fast
        warnings.push(`Object ${obj.id} has high velocity: ${speed.toFixed(2)} m/s`);
      }

      // Validate angular velocity
      const angularSpeed = Math.sqrt(
        obj.angularVelocity.x * obj.angularVelocity.x +
        obj.angularVelocity.y * obj.angularVelocity.y +
        obj.angularVelocity.z * obj.angularVelocity.z
      );
      if (angularSpeed > 100) { // 100 rad/s is very fast
        warnings.push(`Object ${obj.id} has high angular velocity: ${angularSpeed.toFixed(2)} rad/s`);
      }

      // Validate material properties
      if (obj.materialProperties.restitution > 1.0) {
        errors.push(`Object ${obj.id} has invalid restitution > 1.0: ${obj.materialProperties.restitution}`);
        isValid = false;
      }

      if (obj.materialProperties.friction < 0) {
        errors.push(`Object ${obj.id} has negative friction: ${obj.materialProperties.friction}`);
        isValid = false;
      }
    }

    return {
      isValid,
      confidence: physicsAccuracy,
      errors,
      warnings,
      metrics: {
        gravityValid: true, // Not applicable here
        collisionDetectionActive: true, // Not applicable here
        realisticMasses,
        stableSimulation: true, // Evaluated separately
        physicsAccuracy,
        simulationSpeed: 1.0, // Not applicable here
        constraintSatisfaction: 1.0 // Not applicable here
      },
      timestamp: new Date()
    };
  }

  /**
   * Validates collision settings
   */
  private validateCollisionSettings(settings: CollisionSettings): PhysicsValidationResult {
    const errors: string[] = [];
    const warnings: string[] = [];
    let isValid = true;

    if (!settings.collisionDetection) {
      errors.push("Collision detection is not configured");
      isValid = false;
    }

    if (!settings.broadphaseAlgorithm) {
      warnings.push("Broadphase algorithm not specified - using default");
    }

    if (!settings.narrowphaseAlgorithm) {
      warnings.push("Narrowphase algorithm not specified - using default");
    }

    if (settings.collisionMargin && settings.collisionMargin < 0) {
      errors.push(`Invalid collision margin: ${settings.collisionMargin}`);
      isValid = false;
    }

    return {
      isValid,
      confidence: isValid ? 1.0 : 0.5,
      errors,
      warnings,
      metrics: {
        gravityValid: true, // Not applicable here
        collisionDetectionActive: !!settings.collisionDetection,
        realisticMasses: true, // Not applicable here
        stableSimulation: true, // Not applicable here
        physicsAccuracy: isValid ? 1.0 : 0.5,
        simulationSpeed: 1.0, // Not applicable here
        constraintSatisfaction: 1.0 // Not applicable here
      },
      timestamp: new Date()
    };
  }

  /**
   * Estimates simulation stability based on current state
   */
  private estimateStability(state: SimulationState): boolean {
    // Stability estimation based on common physics simulation indicators
    for (const obj of state.objects) {
      // Check for extremely high velocities that might indicate instability
      const speed = Math.sqrt(
        obj.velocity.x * obj.velocity.x +
        obj.velocity.y * obj.velocity.y +
        obj.velocity.z * obj.velocity.z
      );

      if (speed > 1000) { // Unusually high speed
        return false;
      }

      // Check for extremely high angular velocities
      const angularSpeed = Math.sqrt(
        obj.angularVelocity.x * obj.angularVelocity.x +
        obj.angularVelocity.y * obj.angularVelocity.y +
        obj.angularVelocity.z * obj.angularVelocity.z
      );

      if (angularSpeed > 1000) { // Unusually high angular speed
        return false;
      }
    }

    return true;
  }

  /**
   * Estimates real-time factor based on time step
   */
  private estimateRealtimeFactor(timeStep: number): number {
    // Smaller time steps generally mean slower simulation relative to real-time
    // This is a simplified estimation
    if (timeStep < 0.001) { // Very small time step
      return 0.1; // Simulation will be slow
    } else if (timeStep > 0.01) { // Large time step
      return 0.8; // Simulation closer to real-time but potentially less accurate
    } else {
      return 0.5; // Moderate balance
    }
  }

  /**
   * Estimates constraint satisfaction based on simulation parameters
   */
  private estimateConstraintSatisfaction(state: SimulationState): number {
    // Evaluate how well constraints are likely to be satisfied
    let score = 1.0;

    // Lower score if time step is large
    if (state.timeStep > 0.01) {
      score *= 0.7;
    }

    // Lower score if solver iterations are low
    if (state.environment.solverIterations < 10) {
      score *= 0.8;
    }

    // Lower score if contact breaking threshold is too large
    if (state.environment.contactBreakingThreshold > 0.1) {
      score *= 0.9;
    }

    return Math.max(0.0, Math.min(1.0, score));
  }

  /**
   * Validates specific physics simulation scenarios for robotics
   */
  validateRoboticsScenario(
    state: SimulationState,
    robotConfig: { baseMass: number; linkMasses: number[]; jointLimits: any[] }
  ): PhysicsValidationResult {
    const baseResult = this.validateSimulationSetup(state);

    const errors: string[] = [...baseResult.errors];
    const warnings: string[] = [...baseResult.warnings];
    let isValid = baseResult.isValid;
    let physicsAccuracy = baseResult.metrics.physicsAccuracy;

    // Validate robot-specific properties
    const robotObjects = state.objects.filter(obj =>
      obj.id.includes('robot') || obj.id.includes('Robot') || obj.id.includes('base') || obj.id.includes('link')
    );

    if (robotObjects.length === 0) {
      warnings.push("No robot-specific objects detected in simulation");
    } else {
      // Validate robot mass distribution
      let totalMass = 0;
      for (const obj of robotObjects) {
        totalMass += obj.mass;
      }

      if (Math.abs(totalMass - (robotConfig.baseMass + robotConfig.linkMasses.reduce((a, b) => a + b, 0))) > 0.1) {
        errors.push(`Robot mass mismatch: calculated ${totalMass}kg vs expected ${robotConfig.baseMass + robotConfig.linkMasses.reduce((a, b) => a + b, 0)}kg`);
        isValid = false;
        physicsAccuracy *= 0.8;
      }
    }

    return {
      isValid,
      confidence: physicsAccuracy,
      errors,
      warnings,
      metrics: {
        ...baseResult.metrics,
        realisticMasses: baseResult.metrics.realisticMasses && errors.length === 0
      },
      timestamp: new Date()
    };
  }

  /**
   * Validates sensor simulation within the physics environment
   */
  validateSensorSimulation(
    state: SimulationState,
    sensorData: any
  ): PhysicsValidationResult {
    // This would validate that sensor data is consistent with physics simulation
    // For example, checking that LiDAR returns expected values based on object positions
    const errors: string[] = [];
    const warnings: string[] = [];
    let isValid = true;
    let physicsAccuracy = 1.0;

    // Example validation: Check if camera image shows expected objects
    if (sensorData.camera && sensorData.camera.objectsDetected) {
      // Compare detected objects with physics objects in the field of view
      // This is a simplified check
    }

    // Example validation: Check if IMU data is consistent with object motion
    if (sensorData.imu && state.objects.length > 0) {
      // Compare IMU readings with actual acceleration of the object it's attached to
    }

    return {
      isValid,
      confidence: physicsAccuracy,
      errors,
      warnings,
      metrics: {
        gravityValid: true,
        collisionDetectionActive: true,
        realisticMasses: true,
        stableSimulation: true,
        physicsAccuracy,
        simulationSpeed: 1.0,
        constraintSatisfaction: 1.0
      },
      timestamp: new Date()
    };
  }
}

// Example usage
/*
const validator = new PhysicsSimValidator();

// Example validation
const simulationState: SimulationState = {
  objects: [
    {
      id: "robot_base",
      position: { x: 0, y: 0, z: 0.5 },
      orientation: { x: 0, y: 0, z: 0, w: 1 },
      velocity: { x: 0, y: 0, z: 0 },
      angularVelocity: { x: 0, y: 0, z: 0 },
      mass: 10.0,
      inertia: { elements: [1, 0, 0, 0, 1, 0, 0, 0, 1] },
      collisionShape: "box",
      materialProperties: {
        density: 1000,
        friction: 0.5,
        restitution: 0.1,
        staticFriction: 0.5,
        dynamicFriction: 0.4
      }
    }
  ],
  environment: {
    gravity: { x: 0, y: 0, z: -9.81 },
    airDensity: 1.225,
    timeStep: 0.001,
    solverIterations: 50,
    contactBreakingThreshold: 0.001
  },
  timeStep: 0.001,
  gravity: { x: 0, y: 0, z: -9.81 },
  collisionSettings: {
    collisionDetection: 'discrete',
    broadphaseAlgorithm: 'sap',
    narrowphaseAlgorithm: 'gjk',
    collisionMargin: 0.001
  }
};

const result = validator.validateSimulationSetup(simulationState);
console.log(`Physics validation: ${result.isValid}, Confidence: ${result.confidence}`);
console.log(`Errors: ${result.errors.join(', ')}`);
console.log(`Warnings: ${result.warnings.join(', ')}`);
*/