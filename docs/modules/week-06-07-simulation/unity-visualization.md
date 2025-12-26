---
title: Unity Visualization for Robotics
sidebar_position: 2
---

# Unity Visualization for Robotics

## Overview

Unity is a powerful 3D development platform that can be used for robotics simulation and visualization. While Gazebo excels at physics simulation, Unity provides high-quality graphics rendering and more flexible visualization capabilities that are particularly useful for creating immersive robot training environments and user interfaces.

## Learning Objectives

By the end of this module, students will be able to:
- Set up Unity for robotics visualization projects
- Create 3D environments for robot simulation
- Implement realistic lighting and materials
- Export Unity scenes for integration with ROS 2

## Prerequisites

- Basic understanding of 3D graphics concepts
- Completed Gazebo setup module
- Unity Hub and Unity 2022.3 LTS or newer installed

## Installing Unity for Robotics

### System Requirements
- Windows 10/11, macOS 10.14+, or Ubuntu 20.04+
- 8GB+ RAM, dedicated GPU recommended
- 20GB+ free disk space

### Installation Steps

1. Download and install Unity Hub from [Unity's official website](https://unity.com/download)
2. Use Unity Hub to install Unity 2022.3 LTS
3. Install the Robotics package from the Unity Package Manager

### Unity Robotics Hub

Unity provides the Robotics Hub which includes:
- URDF Importer: Import ROS URDF robot models into Unity
- Perception: Tools for generating synthetic sensor data
- Robotics Simulation Framework: Integration with ROS 2

## Setting Up Your First Robotics Scene

### Creating a New Project

1. Open Unity Hub and create a new 3D project
2. Name it "RoboticsSimulation"
3. Import the Robotics packages via Window → Package Manager

### Basic Scene Setup

```csharp
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;

public class RobotController : MonoBehaviour
{
    // Reference to ROS connection
    private RosConnection ros;

    void Start()
    {
        // Connect to ROS
        ros = GetComponent<RosConnection>();
        ros.RegisterPublisher<Unity.Robotics.ROSTCPConnector.MessageTypes.Std_msgs.Float32>("robot_velocity");
    }

    void Update()
    {
        // Send robot velocity data to ROS
        var velocityMsg = new Unity.Robotics.ROSTCPConnector.MessageTypes.Std_msgs.Float32();
        velocityMsg.data = 1.0f; // Example velocity value
        ros.Publish("robot_velocity", velocityMsg);
    }
}
```

## Creating Robot Models in Unity

### Using URDF Importer

Unity's URDF Importer allows you to import ROS URDF files directly:

1. Place your URDF file in the Assets folder
2. Select the URDF file in Unity
3. The importer will automatically create the robot hierarchy
4. Configure joint limits and physical properties

### Example Robot Model

```xml
<!-- Example URDF for a simple differential drive robot -->
<?xml version="1.0"?>
<robot name="diff_robot">
  <link name="base_link">
    <visual>
      <geometry>
        <cylinder radius="0.2" length="0.1"/>
      </geometry>
      <material name="blue">
        <color rgba="0 0 1 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <cylinder radius="0.2" length="0.1"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="1"/>
      <inertia ixx="0.1" ixy="0" ixz="0" iyy="0.1" iyz="0" izz="0.1"/>
    </inertial>
  </link>

  <joint name="left_wheel_joint" type="continuous">
    <parent link="base_link"/>
    <child link="left_wheel"/>
    <origin xyz="-0.1 0.2 0" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
  </joint>

  <link name="left_wheel">
    <visual>
      <geometry>
        <cylinder radius="0.05" length="0.05"/>
      </geometry>
      <material name="black">
        <color rgba="0 0 0 1"/>
      </material>
    </visual>
  </link>
</robot>
```

## Physics Simulation in Unity

### Unity Physics vs Gazebo

While Unity has built-in physics, for robotics applications you may want to:
- Use Unity primarily for visualization
- Keep physics simulation in Gazebo
- Synchronize state between both engines

### Configuring Physics Materials

```csharp
using UnityEngine;

public class PhysicsMaterialSetup : MonoBehaviour
{
    [Header("Physics Materials")]
    public PhysicMaterial lowFrictionMaterial;
    public PhysicMaterial highFrictionMaterial;

    void Start()
    {
        // Apply physics materials to robot components
        foreach (Collider col in GetComponentsInChildren<Collider>())
        {
            col.material = lowFrictionMaterial;
        }
    }
}
```

## Sensor Simulation in Unity

### Camera Sensors

```csharp
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using Unity.Robotics.ROSTCPConnector.MessageTypes.Sensor_msgs;

public class CameraSensor : MonoBehaviour
{
    public Camera camera;
    private RosConnection ros;
    private string topicName = "camera/image_raw";

    void Start()
    {
        ros = GetComponent<RosConnection>();
        ros.RegisterPublisher<Image>(topicName);
    }

    void Update()
    {
        // Capture and publish camera image
        // Implementation depends on Unity's Perception package
    }
}
```

### LiDAR Simulation

Unity's Perception package provides tools for generating synthetic LiDAR data:

1. Add the Synthetic Data Capture component to your robot
2. Configure ray count, range, and field of view
3. Export data in ROS-compatible formats

## Practical Exercise: Unity Robot Visualization

### Objective
Create a Unity scene with a robot model that can be controlled via ROS 2.

### Steps
1. Import a robot URDF into Unity
2. Set up ROS connection for communication
3. Create a simple environment with obstacles
4. Implement basic robot movement controls

### Scene Configuration

Create a Unity scene with:
- A robot model imported from URDF
- A floor plane with realistic materials
- Obstacle objects with appropriate physics
- Camera for visualization
- Light sources for realistic rendering

## Integration with ROS 2

### ROS-TCP-Connector

The Unity ROS TCP Connector enables communication between Unity and ROS 2:

1. Add the ROS TCP Connector component to your scene
2. Configure the IP address and port for ROS connection
3. Register publishers and subscribers for robot data

### Example Integration Script

```csharp
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using Unity.Robotics.ROSTCPConnector.MessageTypes.Std_msgs;

public class ROSIntegration : MonoBehaviour
{
    [SerializeField]
    private string rosIPAddress = "127.0.0.1";
    [SerializeField]
    private int rosPort = 10000;

    private RosConnection ros;

    void Start()
    {
        ros = RosConnection.GetOrCreateInstance();
        ros.Initialize(rosIPAddress, rosPort);

        // Subscribe to velocity commands
        ros.Subscribe<Float32>("cmd_vel", OnVelocityCommand);
    }

    void OnVelocityCommand(Float32 velocityMsg)
    {
        // Process velocity command
        float velocity = velocityMsg.data;
        // Apply to robot movement
    }
}
```

## Assessment

Students must demonstrate the following:
1. Successfully create a Unity scene with a robot model
2. Configure Unity for robotics visualization
3. Implement basic robot controls in Unity
4. Validate sensor data generation in Unity environment

**Success Threshold: 85% accuracy in visualization validation**

## Next Steps

After completing this module, proceed to sensor simulation to learn about realistic sensor modeling in both Gazebo and Unity environments.