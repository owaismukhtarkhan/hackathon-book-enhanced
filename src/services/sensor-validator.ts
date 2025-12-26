/**
 * Sensor Data Validation Framework for Robotics Simulation
 * Validates sensor data streams in simulation environments
 */

export interface SensorValidationResult {
  isValid: boolean;
  confidence: number;
  errors: string[];
  sensorType: string;
  timestamp: Date;
}

export interface SensorData {
  ranges?: number[];      // For LiDAR/Range sensors
  image?: any;            // For camera sensors
  imu?: any;              // For IMU sensors
  gps?: any;              // For GPS sensors
  [key: string]: any;     // Additional sensor data types
}

export class SensorValidator {
  /**
   * Validates LiDAR/range sensor data
   */
  validateRangeData(data: SensorData): SensorValidationResult {
    const errors: string[] = [];
    let isValid = true;
    let confidence = 1.0;

    if (!data.ranges) {
      errors.push("No range data provided");
      isValid = false;
      confidence = 0.0;
    } else {
      const ranges = data.ranges;

      // Check for valid range values
      const validRanges = ranges.filter(r =>
        typeof r === 'number' &&
        !isNaN(r) &&
        r >= 0 &&
        r <= 100  // Assuming max range of 100m
      );

      const validRatio = validRanges.length / ranges.length;

      if (validRatio < 0.8) {  // At least 80% valid ranges
        errors.push(`Only ${validRatio * 100}% of ranges are valid`);
        isValid = false;
        confidence = validRatio;
      }

      // Check for realistic values
      const extremeValues = ranges.filter(r => r > 50); // Unusually long ranges
      if (extremeValues.length / ranges.length > 0.5) {
        errors.push("Unusually high number of long range values");
      }
    }

    return {
      isValid,
      confidence,
      errors,
      sensorType: "range",
      timestamp: new Date()
    };
  }

  /**
   * Validates camera/visual sensor data
   */
  validateImageData(data: SensorData): SensorValidationResult {
    const errors: string[] = [];
    let isValid = true;
    let confidence = 1.0;

    if (!data.image) {
      errors.push("No image data provided");
      isValid = false;
      confidence = 0.0;
    } else {
      // Basic validation of image properties
      if (!data.image.width || !data.image.height) {
        errors.push("Invalid image dimensions");
        isValid = false;
        confidence = 0.5;
      }

      if (data.image.width <= 0 || data.image.height <= 0) {
        errors.push("Image dimensions must be positive");
        isValid = false;
        confidence = 0.3;
      }

      // Check for reasonable resolution
      if (data.image.width > 8192 || data.image.height > 8192) {
        errors.push("Image resolution is unusually high");
      }

      if (data.image.width < 16 || data.image.height < 16) {
        errors.push("Image resolution is unusually low");
      }
    }

    return {
      isValid,
      confidence,
      errors,
      sensorType: "camera",
      timestamp: new Date()
    };
  }

  /**
   * Validates IMU sensor data
   */
  validateIMUData(data: SensorData): SensorValidationResult {
    const errors: string[] = [];
    let isValid = true;
    let confidence = 1.0;

    if (!data.imu) {
      errors.push("No IMU data provided");
      isValid = false;
      confidence = 0.0;
    } else {
      // Validate orientation quaternion
      if (data.imu.orientation) {
        const { x, y, z, w } = data.imu.orientation;
        if (typeof x !== 'number' || typeof y !== 'number' ||
            typeof z !== 'number' || typeof w !== 'number') {
          errors.push("Invalid quaternion components");
          isValid = false;
          confidence = 0.5;
        } else {
          // Check if quaternion is normalized (approximately)
          const norm = Math.sqrt(x*x + y*y + z*z + w*w);
          if (Math.abs(norm - 1.0) > 0.1) {
            errors.push(`Quaternion not normalized (norm = ${norm})`);
          }
        }
      }

      // Validate angular velocity
      if (data.imu.angular_velocity) {
        const { x, y, z } = data.imu.angular_velocity;
        if (typeof x !== 'number' || typeof y !== 'number' || typeof z !== 'number') {
          errors.push("Invalid angular velocity components");
          isValid = false;
          confidence = 0.7;
        }
      }

      // Validate linear acceleration
      if (data.imu.linear_acceleration) {
        const { x, y, z } = data.imu.linear_acceleration;
        if (typeof x !== 'number' || typeof y !== 'number' || typeof z !== 'number') {
          errors.push("Invalid linear acceleration components");
          isValid = false;
          confidence = 0.7;
        }

        // Check for reasonable acceleration values (in m/s^2)
        const gravity = 9.81;
        if (Math.abs(x) > 100 || Math.abs(y) > 100 || Math.abs(z) > 100) {
          errors.push("Unusually high acceleration values detected");
        }
      }
    }

    return {
      isValid,
      confidence,
      errors,
      sensorType: "imu",
      timestamp: new Date()
    };
  }

  /**
   * Validates GPS sensor data
   */
  validateGPSData(data: SensorData): SensorValidationResult {
    const errors: string[] = [];
    let isValid = true;
    let confidence = 1.0;

    if (!data.gps) {
      errors.push("No GPS data provided");
      isValid = false;
      confidence = 0.0;
    } else {
      // Validate latitude
      if (typeof data.gps.latitude !== 'number' ||
          data.gps.latitude < -90 || data.gps.latitude > 90) {
        errors.push("Invalid latitude value");
        isValid = false;
        confidence = 0.5;
      }

      // Validate longitude
      if (typeof data.gps.longitude !== 'number' ||
          data.gps.longitude < -180 || data.gps.longitude > 180) {
        errors.push("Invalid longitude value");
        isValid = false;
        confidence = 0.5;
      }

      // Validate altitude if provided
      if (typeof data.gps.altitude === 'number' &&
          (data.gps.altitude < -1000 || data.gps.altitude > 20000)) {
        errors.push("Unusual altitude value detected");
      }
    }

    return {
      isValid,
      confidence,
      errors,
      sensorType: "gps",
      timestamp: new Date()
    };
  }

  /**
   * Main validation method that determines sensor type and validates accordingly
   */
  validateSensorData(data: SensorData): SensorValidationResult {
    // Determine sensor type based on available data
    if (data.ranges !== undefined) {
      return this.validateRangeData(data);
    } else if (data.image !== undefined) {
      return this.validateImageData(data);
    } else if (data.imu !== undefined) {
      return this.validateIMUData(data);
    } else if (data.gps !== undefined) {
      return this.validateGPSData(data);
    } else {
      return {
        isValid: false,
        confidence: 0.0,
        errors: ["Unknown sensor type - no recognized sensor data fields"],
        sensorType: "unknown",
        timestamp: new Date()
      };
    }
  }

  /**
   * Validates multiple sensor streams together
   */
  validateMultipleSensors(sensors: SensorData[]): SensorValidationResult[] {
    return sensors.map(sensor => this.validateSensorData(sensor));
  }

  /**
   * Validates simulation-specific constraints
   */
  validateSimulationConstraints(
    data: SensorData,
    simulationParams: { maxRange?: number; minAccuracy?: number }
  ): SensorValidationResult {
    const baseResult = this.validateSensorData(data);

    // Apply simulation-specific validation
    if (simulationParams.maxRange && data.ranges) {
      const exceedingRanges = data.ranges.filter(r => r > simulationParams.maxRange);
      if (exceedingRanges.length > 0) {
        baseResult.errors.push(`Some ranges exceed simulation max range of ${simulationParams.maxRange}m`);
        baseResult.isValid = false;
        baseResult.confidence *= 0.8; // Reduce confidence
      }
    }

    return baseResult;
  }
}

// Example usage
/*
const validator = new SensorValidator();

// Example validation
const sampleRangeData: SensorData = {
  ranges: [1.2, 1.5, 1.8, 2.1, 2.4, 2.7, 3.0, 3.3, 3.6, 3.9]
};

const result = validator.validateRangeData(sampleRangeData);
console.log(`Validation result: ${result.isValid}, Confidence: ${result.confidence}`);
*/