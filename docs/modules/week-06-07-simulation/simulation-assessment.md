---
title: Simulation Module Assessment
sidebar_position: 4
---

# Simulation Module Assessment

## Overview

This assessment evaluates your understanding of robot simulation environments, specifically Gazebo and Unity, as well as sensor simulation concepts. You must achieve an 85% accuracy threshold to pass this module.

## Assessment Structure

The assessment consists of both theoretical questions and practical implementation tasks that must be completed in simulation.

## Learning Objectives Covered

- Gazebo simulation environment setup and configuration
- Unity visualization for robotics applications
- Sensor simulation including LiDAR, camera, and IMU
- Validation of sensor data streams
- Integration of simulation environments with ROS 2

## Assessment Tasks

### Task 1: Gazebo World Creation (25 points)

Create a Gazebo world file that includes:
- A ground plane and lighting
- At least 3 obstacles with different shapes and sizes
- Physics properties correctly configured
- A designated spawn area for robots

**Validation Requirements:**
- The world must load without errors in Gazebo
- All obstacles must have appropriate collision and visual properties
- Physics simulation must behave realistically

### Task 2: Robot Model with Sensors (25 points)

Create a robot model that includes:
- A base link with appropriate physical properties
- At least 2 different sensor types (LiDAR and camera)
- Proper joint configurations for any moving parts
- Correct URDF/SDF formatting

**Validation Requirements:**
- The robot must spawn correctly in the simulation
- All sensors must publish data to appropriate ROS topics
- Sensor data must be validated using the provided validation framework

### Task 3: Sensor Data Validation (25 points)

Implement and run the sensor validation framework that:
- Subscribes to all sensor topics published by your robot
- Validates the data format and ranges for each sensor
- Reports validation statistics with at least 85% accuracy threshold
- Handles invalid data appropriately

**Validation Requirements:**
- Validation framework must run continuously during simulation
- At least 85% of sensor readings must pass validation checks
- Proper logging of validation results must be demonstrated

### Task 4: Unity Visualization (25 points)

Create a Unity scene that:
- Visualizes the robot model from Task 2
- Shows the simulated environment from Task 1
- Displays sensor data in a meaningful way
- Integrates with ROS 2 using the TCP connector

**Validation Requirements:**
- Unity scene must correctly visualize the robot in the environment
- Sensor data must be appropriately represented in the visualization
- ROS connection must be established and maintained

## Submission Requirements

1. **Gazebo World File**: `simulation_world.sdf`
2. **Robot Model File**: `robot_model.urdf` or `robot_model.sdf`
3. **Sensor Validation Code**: `sensor_validator.py`
4. **Unity Scene Files**: Exported scene and relevant scripts
5. **Validation Report**: Document showing validation statistics
6. **Implementation Notes**: Brief documentation of your implementation approach

## Grading Criteria

### Pass Requirements (85% threshold)
- All tasks completed with functional code
- Sensor validation achieves 85% or higher accuracy
- All components integrate correctly with ROS 2
- Proper documentation and code comments provided

### Scoring Breakdown
- Task 1: 25 points
- Task 2: 25 points
- Task 3: 25 points
- Task 4: 25 points
- **Total**: 100 points

### Deductions
- Runtime errors or crashes: -10 points each
- Missing validation requirements: -5 points each
- Inadequate documentation: -5 points
- Code that doesn't follow ROS 2 best practices: -5 points

## Resources

- Gazebo documentation
- Unity Robotics documentation
- Sensor message type definitions
- Provided code examples from modules

## Time Limit

You have 4 hours to complete this assessment. Plan your time accordingly to ensure all components are completed.

## Submission Process

1. Package all required files in a zip archive named `simulation_assessment_<student_name>.zip`
2. Run your validation framework and capture output
3. Submit through the course management system
4. Include your validation report with accuracy metrics

## Success Metrics

To pass this assessment, you must achieve:
- Overall score of 85/100 (85%)
- Individual task scores of at least 70% each
- Sensor validation accuracy of 85% or higher
- All components working together in integrated simulation