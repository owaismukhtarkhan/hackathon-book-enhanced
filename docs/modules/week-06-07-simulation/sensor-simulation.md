---
title: Sensor Simulation in Robotics
sidebar_position: 3
---

# Sensor Simulation in Robotics

## Overview

Sensor simulation is critical for developing and testing robotic systems in virtual environments. This module covers simulating various types of sensors including LiDAR, cameras, IMUs, and other perception sensors that provide the data needed for robot navigation, mapping, and decision-making.

## Learning Objectives

By the end of this module, students will be able to:
- Configure and validate LiDAR sensor simulation in Gazebo and Unity
- Set up camera and depth sensor simulation
- Implement IMU and other inertial sensor simulation
- Validate sensor data streams for accuracy and reliability

## Prerequisites

- Completed Gazebo setup and Unity visualization modules
- Understanding of basic sensor types and their applications
- Basic knowledge of ROS 2 sensor message types

## LiDAR Sensor Simulation

### Gazebo LiDAR Configuration

LiDAR sensors in Gazebo are configured using the `<sensor>` tag in SDF files:

```xml
<sensor name="lidar" type="ray">
  <pose>0 0 0.2 0 0 0</pose>
  <ray>
    <scan>
      <horizontal>
        <samples>360</samples>
        <resolution>1</resolution>
        <min_angle>-3.14159</min_angle>
        <max_angle>3.14159</max_angle>
      </horizontal>
    </scan>
    <range>
      <min>0.1</min>
      <max>30.0</max>
      <resolution>0.01</resolution>
    </range>
  </ray>
  <plugin name="lidar_controller" filename="libgazebo_ros_ray_sensor.so">
    <ros>
      <namespace>/lidar</namespace>
      <remapping>~/out:=scan</remapping>
    </ros>
    <output_type>sensor_msgs/LaserScan</output_type>
  </plugin>
</sensor>
```

### Unity LiDAR Simulation

Using Unity's Perception package for LiDAR simulation:

```csharp
using UnityEngine;
using Unity.Perception.GroundTruth;

public class LiDARSetup : MonoBehaviour
{
    public int rayCount = 360;
    public float range = 30.0f;
    public float fieldOfView = 360.0f;

    void Start()
    {
        // Configure LiDAR sensor using Perception package
        var lidarSensor = GetComponent<LiDARSensor>();
        lidarSensor.rayCount = rayCount;
        lidarSensor.range = range;
        lidarSensor.fieldOfView = fieldOfView;
    }
}
```

## Camera and Depth Sensor Simulation

### RGB Camera Configuration

```xml
<!-- Gazebo camera sensor -->
<sensor name="camera" type="camera">
  <camera>
    <horizontal_fov>1.047</horizontal_fov>
    <image>
      <width>640</width>
      <height>480</height>
      <format>R8G8B8</format>
    </image>
    <clip>
      <near>0.1</near>
      <far>100</far>
    </clip>
  </camera>
  <plugin name="camera_controller" filename="libgazebo_ros_camera.so">
    <ros>
      <namespace>/camera</namespace>
      <remapping>~/image_raw:=image_raw</remapping>
      <remapping>~/camera_info:=camera_info</remapping>
    </ros>
  </plugin>
</sensor>
```

### Depth Camera Configuration

```xml
<!-- Gazebo depth camera -->
<sensor name="depth_camera" type="depth">
  <camera>
    <horizontal_fov>1.047</horizontal_fov>
    <image>
      <width>640</width>
      <height>480</height>
    </image>
    <clip>
      <near>0.1</near>
      <far>10</far>
    </clip>
  </camera>
  <plugin name="depth_camera_controller" filename="libgazebo_ros_openni_kinect.so">
    <ros>
      <namespace>/depth_camera</namespace>
      <remapping>~/image_raw:=image_raw</remapping>
      <remapping>~/depth/image_raw:=depth/image_raw</remapping>
    </ros>
  </plugin>
</sensor>
```

## IMU Sensor Simulation

### IMU Configuration in Gazebo

```xml
<sensor name="imu_sensor" type="imu">
  <always_on>1</always_on>
  <update_rate>100</update_rate>
  <imu>
    <angular_velocity>
      <x>
        <noise type="gaussian">
          <mean>0.0</mean>
          <stddev>0.001</stddev>
        </noise>
      </x>
      <y>
        <noise type="gaussian">
          <mean>0.0</mean>
          <stddev>0.001</stddev>
        </noise>
      </y>
      <z>
        <noise type="gaussian">
          <mean>0.0</mean>
          <stddev>0.001</stddev>
        </noise>
      </z>
    </angular_velocity>
    <linear_acceleration>
      <x>
        <noise type="gaussian">
          <mean>0.0</mean>
          <stddev>1.7e-2</stddev>
        </noise>
      </x>
      <y>
        <noise type="gaussian">
          <mean>0.0</mean>
          <stddev>1.7e-2</stddev>
        </noise>
      </y>
      <z>
        <noise type="gaussian">
          <mean>0.0</mean>
          <stddev>1.7e-2</stddev>
        </noise>
      </z>
    </linear_acceleration>
  </imu>
  <plugin name="imu_plugin" filename="libgazebo_ros_imu.so">
    <ros>
      <namespace>/imu</namespace>
      <remapping>~/out:=data</remapping>
    </ros>
    <initial_orientation_as_reference>false</initial_orientation_as_reference>
  </plugin>
</sensor>
```

## Sensor Data Validation

### Validation Framework

Create a validation framework to ensure sensor data is accurate and reliable:

```python
#!/usr/bin/env python3
# sensor_validator.py

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, Image, Imu
import numpy as np

class SensorValidator(Node):
    def __init__(self):
        super().__init__('sensor_validator')

        # Subscribe to sensor topics
        self.lidar_subscription = self.create_subscription(
            LaserScan,
            '/lidar/scan',
            self.lidar_callback,
            10)

        self.camera_subscription = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.camera_callback,
            10)

        self.imu_subscription = self.create_subscription(
            Imu,
            '/imu/data',
            self.imu_callback,
            10)

        self.get_logger().info('Sensor validator initialized')

    def lidar_callback(self, msg):
        # Validate LiDAR data
        ranges = np.array(msg.ranges)

        # Check for valid range values
        valid_ranges = ranges[(ranges >= msg.range_min) & (ranges <= msg.range_max)]

        # Calculate validation metrics
        if len(valid_ranges) / len(ranges) < 0.95:
            self.get_logger().warn('LiDAR data has too many invalid ranges')
        else:
            self.get_logger().info('LiDAR data validation passed')

    def camera_callback(self, msg):
        # Validate camera data
        height = msg.height
        width = msg.width
        data_size = len(msg.data)

        # Check image dimensions and data consistency
        expected_size = height * width * 3  # Assuming RGB
        if data_size != expected_size:
            self.get_logger().warn('Camera data size mismatch')
        else:
            self.get_logger().info('Camera data validation passed')

    def imu_callback(self, msg):
        # Validate IMU data
        orientation = msg.orientation
        angular_velocity = msg.angular_velocity
        linear_acceleration = msg.linear_acceleration

        # Check for valid quaternion (normalized)
        norm = np.sqrt(orientation.x**2 + orientation.y**2 +
                      orientation.z**2 + orientation.w**2)

        if abs(norm - 1.0) > 0.01:
            self.get_logger().warn('IMU orientation quaternion not normalized')
        else:
            self.get_logger().info('IMU data validation passed')

def main(args=None):
    rclpy.init(args=args)
    validator = SensorValidator()
    rclpy.spin(validator)
    validator.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Practical Exercise: Multi-Sensor Robot Simulation

### Objective
Create a robot model with multiple sensors (LiDAR, camera, IMU) and validate all sensor data streams.

### Steps
1. Create a robot model with LiDAR, camera, and IMU sensors
2. Configure each sensor with appropriate parameters
3. Launch the simulation and verify sensor topics
4. Run validation checks on all sensor data streams

### Example Robot with Multiple Sensors

```xml
<?xml version="1.0"?>
<robot name="multi_sensor_robot">
  <link name="base_link">
    <visual>
      <geometry>
        <cylinder radius="0.2" length="0.1"/>
      </geometry>
      <material name="blue">
        <color rgba="0 0 1 1"/>
      </material>
    </visual>
  </link>

  <!-- LiDAR sensor -->
  <joint name="lidar_joint" type="fixed">
    <parent link="base_link"/>
    <child link="lidar_link"/>
    <origin xyz="0 0 0.3" rpy="0 0 0"/>
  </joint>

  <link name="lidar_link">
    <!-- LiDAR sensor definition -->
    <sensor name="lidar" type="ray">
      <pose>0 0 0 0 0 0</pose>
      <ray>
        <scan>
          <horizontal>
            <samples>360</samples>
            <resolution>1</resolution>
            <min_angle>-3.14159</min_angle>
            <max_angle>3.14159</max_angle>
          </horizontal>
        </scan>
        <range>
          <min>0.1</min>
          <max>30.0</max>
          <resolution>0.01</resolution>
        </range>
      </ray>
    </sensor>
  </link>

  <!-- Camera sensor -->
  <joint name="camera_joint" type="fixed">
    <parent link="base_link"/>
    <child link="camera_link"/>
    <origin xyz="0.1 0 0.2" rpy="0 0 0"/>
  </joint>

  <link name="camera_link">
    <!-- Camera sensor definition -->
    <sensor name="camera" type="camera">
      <pose>0 0 0 0 0 0</pose>
      <camera>
        <horizontal_fov>1.047</horizontal_fov>
        <image>
          <width>640</width>
          <height>480</height>
          <format>R8G8B8</format>
        </image>
        <clip>
          <near>0.1</near>
          <far>100</far>
        </clip>
      </camera>
    </sensor>
  </link>

  <!-- IMU sensor -->
  <joint name="imu_joint" type="fixed">
    <parent link="base_link"/>
    <child link="imu_link"/>
    <origin xyz="0 0 0.1" rpy="0 0 0"/>
  </joint>

  <link name="imu_link">
    <!-- IMU sensor definition -->
    <sensor name="imu" type="imu">
      <pose>0 0 0 0 0 0</pose>
      <imu>
        <angular_velocity>
          <x>
            <noise type="gaussian">
              <mean>0.0</mean>
              <stddev>0.001</stddev>
            </noise>
          </x>
        </angular_velocity>
      </imu>
    </sensor>
  </link>
</robot>
```

## Assessment

Students must demonstrate the following:
1. Successfully configure multiple sensor types on a robot model
2. Validate sensor data streams for accuracy and reliability
3. Implement sensor fusion concepts in simulation
4. Achieve 85% accuracy threshold in sensor validation tests

**Success Threshold: 85% accuracy in sensor simulation validation**

## Next Steps

After completing this module, students should have a solid understanding of simulation environments and sensor modeling, preparing them for more advanced topics in AI integration with robotics systems.