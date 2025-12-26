#!/usr/bin/env python3
# sensor_simulation.py

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, Image, Imu, PointCloud2
from geometry_msgs.msg import PointStamped
import numpy as np
import math

class SensorSimulator(Node):
    def __init__(self):
        super().__init__('sensor_simulator')

        # Create publishers for different sensor types
        self.lidar_publisher = self.create_publisher(LaserScan, '/simulated_lidar/scan', 10)
        self.camera_publisher = self.create_publisher(Image, '/simulated_camera/image_raw', 10)
        self.imu_publisher = self.create_publisher(Imu, '/simulated_imu/data', 10)
        self.pointcloud_publisher = self.create_publisher(PointCloud2, '/simulated_pointcloud', 10)

        # Timer for sensor data publishing
        self.timer = self.create_timer(0.1, self.publish_sensor_data)  # 10Hz

        self.get_logger().info('Sensor simulator initialized')

    def publish_sensor_data(self):
        # Publish simulated LiDAR data
        self.publish_lidar_data()

        # Publish simulated camera data (placeholder)
        self.publish_camera_data()

        # Publish simulated IMU data
        self.publish_imu_data()

        # Publish simulated point cloud data
        self.publish_pointcloud_data()

    def publish_lidar_data(self):
        msg = LaserScan()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'lidar_link'

        # LiDAR parameters
        msg.angle_min = -math.pi
        msg.angle_max = math.pi
        msg.angle_increment = 2 * math.pi / 360  # 360 samples
        msg.time_increment = 0.0
        msg.scan_time = 0.1
        msg.range_min = 0.1
        msg.range_max = 30.0

        # Generate simulated ranges (with some obstacles)
        ranges = []
        for i in range(360):
            angle = msg.angle_min + i * msg.angle_increment

            # Simulate a circular obstacle at (2, 0) with radius 1
            obstacle_distance = math.sqrt((2 - 1.5 * math.cos(angle))**2 + (0 - 1.5 * math.sin(angle))**2)

            # Add some noise
            range_val = min(obstacle_distance + np.random.normal(0, 0.05), msg.range_max)
            ranges.append(max(range_val, msg.range_min))

        msg.ranges = ranges
        msg.intensities = [100.0] * 360  # Constant intensity

        self.lidar_publisher.publish(msg)

    def publish_camera_data(self):
        # Placeholder for camera data
        # In a real implementation, this would generate simulated camera images
        pass

    def publish_imu_data(self):
        msg = Imu()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'imu_link'

        # Simulate orientation (pointing forward)
        msg.orientation.x = 0.0
        msg.orientation.y = 0.0
        msg.orientation.z = 0.0
        msg.orientation.w = 1.0

        # Simulate angular velocity (small random movements)
        msg.angular_velocity.x = np.random.normal(0, 0.01)
        msg.angular_velocity.y = np.random.normal(0, 0.01)
        msg.angular_velocity.z = np.random.normal(0, 0.01)

        # Simulate linear acceleration (gravity + movement)
        msg.linear_acceleration.x = np.random.normal(0, 0.1)
        msg.linear_acceleration.y = np.random.normal(0, 0.1)
        msg.linear_acceleration.z = 9.81 + np.random.normal(0, 0.1)

        # Add covariance matrices (diagonal values only)
        for i in range(9):
            if i % 4 == 0:  # Diagonal elements
                msg.orientation_covariance[i] = 0.01
                msg.angular_velocity_covariance[i] = 0.01
                msg.linear_acceleration_covariance[i] = 0.01

        self.imu_publisher.publish(msg)

    def publish_pointcloud_data(self):
        # Placeholder for point cloud data
        # In a real implementation, this would generate simulated point cloud
        pass

def main(args=None):
    rclpy.init(args=args)
    sensor_simulator = SensorSimulator()

    try:
        rclpy.spin(sensor_simulator)
    except KeyboardInterrupt:
        pass
    finally:
        sensor_simulator.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()