#!/usr/bin/env python3
# sensor_validator.py

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, Image, Imu
import numpy as np
from std_msgs.msg import Bool

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

        # Publisher for validation results
        self.validation_publisher = self.create_publisher(Bool, '/sensor_validation/result', 10)

        # Statistics for validation
        self.lidar_valid_count = 0
        self.lidar_total_count = 0
        self.camera_valid_count = 0
        self.camera_total_count = 0
        self.imu_valid_count = 0
        self.imu_total_count = 0

        # Validation thresholds
        self.validation_threshold = 0.85  # 85% success rate

        self.get_logger().info('Sensor validator initialized')

    def lidar_callback(self, msg):
        self.lidar_total_count += 1

        # Validate LiDAR data
        ranges = np.array(msg.ranges)
        valid_ranges = ranges[(ranges >= msg.range_min) & (ranges <= msg.range_max) & ~np.isnan(ranges) & ~np.isinf(ranges)]

        # Check if sufficient valid ranges
        if len(valid_ranges) / len(ranges) >= 0.95:
            self.lidar_valid_count += 1
            self.get_logger().debug('LiDAR data validation passed')
        else:
            self.get_logger().warn(f'LiDAR data validation failed: {len(valid_ranges)}/{len(ranges)} valid ranges')

        # Check validation rate periodically
        if self.lidar_total_count % 100 == 0:
            validation_rate = self.lidar_valid_count / self.lidar_total_count
            if validation_rate >= self.validation_threshold:
                self.get_logger().info(f'LiDAR validation rate: {validation_rate:.2f} - PASSED')
            else:
                self.get_logger().error(f'LiDAR validation rate: {validation_rate:.2f} - FAILED')

    def camera_callback(self, msg):
        self.camera_total_count += 1

        # Validate camera data
        height = msg.height
        width = msg.width
        encoding = msg.encoding
        step = msg.step
        data_size = len(msg.data)

        # Check basic image properties
        expected_size = height * step if encoding.startswith('mono') else height * step
        is_valid = (height > 0 and width > 0 and step > 0 and
                   data_size == height * step and
                   encoding in ['rgb8', 'bgr8', 'mono8', 'rgba8', 'bgra8'])

        if is_valid:
            self.camera_valid_count += 1
            self.get_logger().debug('Camera data validation passed')
        else:
            self.get_logger().warn(f'Camera data validation failed: H={height}, W={width}, encoding={encoding}')

        # Check validation rate periodically
        if self.camera_total_count % 100 == 0:
            validation_rate = self.camera_valid_count / self.camera_total_count
            if validation_rate >= self.validation_threshold:
                self.get_logger().info(f'Camera validation rate: {validation_rate:.2f} - PASSED')
            else:
                self.get_logger().error(f'Camera validation rate: {validation_rate:.2f} - FAILED')

    def imu_callback(self, msg):
        self.imu_total_count += 1

        # Validate IMU data
        orientation = msg.orientation
        angular_velocity = msg.angular_velocity
        linear_acceleration = msg.linear_acceleration

        # Check orientation quaternion normalization
        norm = np.sqrt(orientation.x**2 + orientation.y**2 + orientation.z**2 + orientation.w**2)
        orientation_valid = abs(norm - 1.0) <= 0.01

        # Check for valid values (not NaN or infinity)
        imu_values = [
            orientation.x, orientation.y, orientation.z, orientation.w,
            angular_velocity.x, angular_velocity.y, angular_velocity.z,
            linear_acceleration.x, linear_acceleration.y, linear_acceleration.z
        ]
        values_valid = all(not (np.isnan(v) or np.isinf(v)) for v in imu_values)

        if orientation_valid and values_valid:
            self.imu_valid_count += 1
            self.get_logger().debug('IMU data validation passed')
        else:
            self.get_logger().warn(f'IMU data validation failed: orientation_valid={orientation_valid}, values_valid={values_valid}')

        # Check validation rate periodically
        if self.imu_total_count % 100 == 0:
            validation_rate = self.imu_valid_count / self.imu_total_count
            if validation_rate >= self.validation_threshold:
                self.get_logger().info(f'IMU validation rate: {validation_rate:.2f} - PASSED')
            else:
                self.get_logger().error(f'IMU validation rate: {validation_rate:.2f} - FAILED')

    def get_overall_validation_status(self):
        """Return overall validation status based on all sensors"""
        lidar_rate = self.lidar_valid_count / max(self.lidar_total_count, 1)
        camera_rate = self.camera_valid_count / max(self.camera_total_count, 1)
        imu_rate = self.imu_valid_count / max(self.imu_total_count, 1)

        overall_valid = (lidar_rate >= self.validation_threshold and
                        camera_rate >= self.validation_threshold and
                        imu_rate >= self.validation_threshold)

        return overall_valid

def main(args=None):
    rclpy.init(args=args)
    validator = SensorValidator()

    # Print initial validation status periodically
    timer = validator.create_timer(5.0, lambda: validator.get_logger().info(
        f'Overall validation status: {validator.get_overall_validation_status()}'
    ))

    try:
        rclpy.spin(validator)
    except KeyboardInterrupt:
        # Print final statistics
        lidar_rate = validator.lidar_valid_count / max(validator.lidar_total_count, 1)
        camera_rate = validator.camera_valid_count / max(validator.camera_total_count, 1)
        imu_rate = validator.imu_valid_count / max(validator.imu_total_count, 1)

        validator.get_logger().info('Final Validation Statistics:')
        validator.get_logger().info(f'  LiDAR: {validator.lidar_valid_count}/{validator.lidar_total_count} ({lidar_rate:.2f})')
        validator.get_logger().info(f'  Camera: {validator.camera_valid_count}/{validator.camera_total_count} ({camera_rate:.2f})')
        validator.get_logger().info(f'  IMU: {validator.imu_valid_count}/{validator.imu_total_count} ({imu_rate:.2f})')
        validator.get_logger().info(f'  Overall Status: {validator.get_overall_validation_status()}')
    finally:
        validator.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()