---
title: Gazebo Simulation Environment Setup
sidebar_position: 1
---

# Gazebo Simulation Environment Setup

## Overview

Gazebo is a 3D simulation environment that provides realistic physics simulation, high-quality graphics, and convenient programmatic interfaces. It's essential for testing robotic algorithms in a safe, cost-effective environment before deploying on real hardware.

## Learning Objectives

By the end of this module, students will be able to:
- Install and configure Gazebo simulation environment
- Create basic simulation worlds with physics properties
- Spawn and control simulated robots
- Integrate Gazebo with ROS 2 for robot simulation

## Prerequisites

- Completed ROS 2 fundamentals module (Weeks 3-5)
- Ubuntu 22.04 LTS or equivalent environment
- Basic understanding of robotics concepts

## Installing Gazebo

### System Requirements
- Ubuntu 22.04 LTS
- Minimum 8GB RAM
- Dedicated GPU recommended for rendering

### Installation Steps

```bash
# Add ROS 2 Gazebo repository
sudo apt update
sudo apt install ros-humble-gazebo-ros-pkgs ros-humble-gazebo-ros2-control ros-humble-gazebo-ros2-control-demos

# Install Gazebo Garden (or latest stable version)
sudo apt install gazebo libgazebo-dev
```

### Verification

```bash
# Launch Gazebo GUI
gazebo

# If successful, a window with an empty world should appear
```

## Creating Your First Simulation World

### Basic World Structure

Gazebo worlds are defined using SDF (Simulation Description Format) files. Create a basic world file:

```xml
<?xml version="1.0" ?>
<sdf version="1.7">
  <world name="basic_world">
    <!-- Include a default ground plane -->
    <include>
      <uri>model://ground_plane</uri>
    </include>

    <!-- Include a default light source -->
    <include>
      <uri>model://sun</uri>
    </include>

    <!-- Add a simple box obstacle -->
    <model name="box">
      <pose>2 2 0.5 0 0 0</pose>
      <link name="link">
        <collision name="collision">
          <geometry>
            <box>
              <size>1 1 1</size>
            </box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box>
              <size>1 1 1</size>
            </box>
          </geometry>
          <material>
            <ambient>1 0 0 1</ambient>
            <diffuse>1 0 0 1</diffuse>
          </material>
        </visual>
      </link>
    </model>
  </world>
</sdf>
```

### Running Your World

Save the above XML as `basic_world.sdf` and run:

```bash
gazebo basic_world.sdf
```

## Physics Properties in Gazebo

### Gravity
- Default: Earth's gravity (9.8 m/s²)
- Configurable per world: `<gravity>0 0 -9.8</gravity>`

### Collision Detection
- ODE (Open Dynamics Engine) by default
- Can be configured for different precision levels

### Friction Properties
- Static and dynamic friction coefficients
- Surface contact properties

## Integrating with ROS 2

### Gazebo ROS 2 Control

The Gazebo ROS 2 Control package provides interfaces between Gazebo and ROS 2:

```bash
# Launch Gazebo with ROS 2 bridge
ros2 launch gazebo_ros gazebo.launch.py
```

### Spawning Robots

Use the `spawn_entity` service to add robots to your simulation:

```bash
# Example command to spawn a robot
ros2 run gazebo_ros spawn_entity.py -entity my_robot -file /path/to/robot/model.sdf -x 0 -y 0 -z 1
```

## Practical Exercise: Robot Navigation in Simulation

### Objective
Create a simple world with obstacles and navigate a differential drive robot through it.

### Steps
1. Create a world with multiple obstacles
2. Spawn a differential drive robot
3. Implement basic navigation to avoid obstacles
4. Validate sensor data streams

### World File Example

```xml
<?xml version="1.0" ?>
<sdf version="1.7">
  <world name="navigation_world">
    <!-- Ground plane -->
    <include>
      <uri>model://ground_plane</uri>
    </include>

    <!-- Sun light -->
    <include>
      <uri>model://sun</uri>
    </include>

    <!-- Walls -->
    <model name="wall_1">
      <pose>0 -5 1 0 0 0</pose>
      <link name="link">
        <collision name="collision">
          <geometry>
            <box>
              <size>10 1 2</size>
            </box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box>
              <size>10 1 2</size>
            </box>
          </geometry>
          <material>
            <ambient>0.5 0.5 0.5 1</ambient>
            <diffuse>0.5 0.5 0.5 1</diffuse>
          </material>
        </visual>
      </link>
    </model>

    <!-- Add more obstacles as needed -->
  </world>
</sdf>
```

## Assessment

Students must demonstrate the following:
1. Successfully create a simulation world with physics properties
2. Spawn a robot model in the simulation
3. Validate that the robot responds to physics (gravity, collisions)
4. Implement basic movement in the simulation environment

**Success Threshold: 85% accuracy in simulation validation**

## Next Steps

After completing this module, proceed to Unity visualization for alternative simulation approaches and sensor simulation for realistic sensor data generation.