#!/usr/bin/env python3
# vla_pipeline.py

"""
Vision-Language-Action (VLA) Pipeline Implementation
This module implements a complete VLA pipeline using Google Gemini for
vision-language processing and ROS 2 for action execution.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CompressedImage
from std_msgs.msg import String
from geometry_msgs.msg import Twist, PoseStamped
from cv_bridge import CvBridge
import cv2
import google.generativeai as genai
import json
import os
from dotenv import load_dotenv
import numpy as np
from typing import Dict, List, Optional, Tuple
import time
import base64
from io import BytesIO


class VLAPipeline(Node):
    """
    Vision-Language-Action Pipeline Node
    Integrates visual perception, language understanding, and action execution
    """

    def __init__(self):
        super().__init__('vla_pipeline')

        # Load environment variables
        load_dotenv()

        # Configure Google Gemini API
        api_key = os.getenv("GOOGLE_API_KEY")
        if not api_key:
            self.get_logger().error("GOOGLE_API_KEY environment variable not set")
            raise ValueError("GOOGLE_API_KEY environment variable not set")

        genai.configure(api_key=api_key)

        # Initialize components
        self.bridge = CvBridge()
        self.vision_model = genai.GenerativeModel('gemini-pro-vision')
        self.language_model = genai.GenerativeModel('gemini-pro')

        # Robot state
        self.latest_image = None
        self.latest_command = None
        self.robot_state = {
            'position': {'x': 0.0, 'y': 0.0, 'theta': 0.0},
            'battery': 1.0,
            'gripper': 'open',
            'current_task': 'idle'
        }

        # Subscribers
        self.image_subscription = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10
        )

        self.command_subscription = self.create_subscription(
            String,
            '/natural_language_command',
            self.command_callback,
            10
        )

        # Publishers
        self.action_publisher = self.create_publisher(
            String,
            '/robot_action_sequence',
            10
        )

        self.cmd_vel_publisher = self.create_publisher(
            Twist,
            '/cmd_vel',
            10
        )

        self.feedback_publisher = self.create_publisher(
            String,
            '/vla_feedback',
            10
        )

        # Timer for processing pipeline
        self.pipeline_timer = self.create_timer(0.1, self.pipeline_step)

        # Action execution tracking
        self.current_action_sequence = []
        self.action_index = 0
        self.action_start_time = None
        self.action_timeout = 10.0  # seconds

        self.get_logger().info('VLA Pipeline initialized successfully')

    def image_callback(self, msg: Image) -> None:
        """Handle incoming camera images"""
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            self.latest_image = cv_image
        except Exception as e:
            self.get_logger().error(f'Error processing image: {e}')

    def command_callback(self, msg: String) -> None:
        """Handle incoming natural language commands"""
        self.latest_command = msg.data
        self.get_logger().info(f'Received command: {msg.data}')

    def pipeline_step(self) -> None:
        """Main pipeline execution step"""
        # Process new command if available
        if self.latest_command and self.latest_image is not None:
            self.process_command_with_vision()
            self.latest_command = None

        # Execute current action if available
        if self.current_action_sequence and self.action_index < len(self.current_action_sequence):
            self.execute_current_action()
        else:
            # Clear action sequence when complete
            if self.current_action_sequence:
                self.current_action_sequence = []
                self.action_index = 0
                self.action_start_time = None

    def process_command_with_vision(self) -> None:
        """Process command using visual and language understanding"""
        try:
            # Prepare the multi-modal prompt
            prompt = f"""
            You are an advanced robot assistant with vision capabilities.
            Analyze the current scene and interpret the user command.

            User Command: "{self.latest_command}"

            Current Robot State: {json.dumps(self.robot_state)}

            Provide a detailed JSON response with:
            1. scene_analysis: Description of what the robot sees
            2. command_interpretation: How to interpret the command in this context
            3. action_sequence: List of specific actions to execute
            4. object_targets: Relevant objects for the task (if any)
            5. spatial_instructions: Navigation or manipulation details
            6. confidence_score: Confidence level (0.0 to 1.0)
            7. safety_considerations: Any safety issues to be aware of
            """

            # Convert image for Gemini
            pil_image = self.cv2_to_pil(self.latest_image)

            # Process with Gemini Vision
            response = self.vision_model.generate_content([prompt, pil_image])

            # Parse and validate response
            parsed_response = self.parse_vla_response(response.text)

            if parsed_response and parsed_response.get('confidence_score', 0) >= 0.7:
                # Store action sequence for execution
                self.current_action_sequence = parsed_response.get('action_sequence', [])
                self.action_index = 0

                # Publish feedback
                feedback_msg = String()
                feedback_msg.data = json.dumps({
                    'status': 'command_processed',
                    'actions_count': len(self.current_action_sequence),
                    'confidence': parsed_response.get('confidence_score', 0)
                })
                self.feedback_publisher.publish(feedback_msg)

                self.get_logger().info(f'VLA pipeline processed command with {len(self.current_action_sequence)} actions')
            else:
                self.get_logger().warn('Command processing failed or low confidence')

        except Exception as e:
            self.get_logger().error(f'Error in VLA processing: {e}')

    def parse_vla_response(self, response_text: str) -> Optional[Dict]:
        """Parse Gemini response into structured VLA data"""
        try:
            # Clean the response text
            clean_text = response_text.strip()
            if clean_text.startswith('```json'):
                clean_text = clean_text[7:]  # Remove ```json
            if clean_text.endswith('```'):
                clean_text = clean_text[:-3]  # Remove ```
            clean_text = clean_text.strip()

            # Parse JSON
            response_data = json.loads(clean_text)
            return response_data
        except json.JSONDecodeError as e:
            self.get_logger().error(f'Error parsing VLA response: {e}')
            # Try to extract information from non-JSON response
            return {
                'scene_analysis': 'Failed to parse scene',
                'command_interpretation': 'Failed to interpret command',
                'action_sequence': ['unknown'],
                'object_targets': [],
                'spatial_instructions': '',
                'confidence_score': 0.0,
                'safety_considerations': []
            }

    def execute_current_action(self) -> None:
        """Execute the current action in the sequence"""
        if not self.current_action_sequence or self.action_index >= len(self.current_action_sequence):
            return

        current_action = self.current_action_sequence[self.action_index]

        # Check for action timeout
        if self.action_start_time and time.time() - self.action_start_time > self.action_timeout:
            self.get_logger().warn(f'Action timeout: {current_action}')
            self.action_index += 1
            self.action_start_time = time.time()
            return

        # Execute action based on type
        if isinstance(current_action, str):
            action_name = current_action
            action_params = {}
        elif isinstance(current_action, dict):
            action_name = current_action.get('action', 'unknown')
            action_params = current_action.get('parameters', {})
        else:
            action_name = 'unknown'
            action_params = {}

        # Execute specific actions
        if action_name.lower() in ['move_forward', 'forward', 'go_forward']:
            self.execute_move_forward(action_params)
        elif action_name.lower() in ['turn_left', 'left']:
            self.execute_turn_left(action_params)
        elif action_name.lower() in ['turn_right', 'right']:
            self.execute_turn_right(action_params)
        elif action_name.lower() in ['move_to', 'navigate_to', 'go_to']:
            self.execute_navigate_to(action_params)
        elif action_name.lower() in ['detect_object', 'find_object', 'locate']:
            self.execute_detect_object(action_params)
        elif action_name.lower() in ['grasp', 'pick_up', 'manipulate']:
            self.execute_manipulate(action_params)
        elif action_name.lower() in ['stop', 'halt', 'pause']:
            self.execute_stop()
        else:
            self.get_logger().info(f'Executing unknown action: {action_name}')
            self.action_index += 1  # Move to next action

        # Initialize action start time if not set
        if not self.action_start_time:
            self.action_start_time = time.time()

    def execute_move_forward(self, params: Dict) -> None:
        """Execute forward movement action"""
        speed = params.get('speed', 0.2)  # Default 0.2 m/s
        duration = params.get('duration', 1.0)  # Default 1 second

        twist = Twist()
        twist.linear.x = float(speed)
        twist.angular.z = 0.0

        self.cmd_vel_publisher.publish(twist)

        # Check if duration-based execution is complete
        if self.action_start_time and time.time() - self.action_start_time >= duration:
            self.execute_stop()
            self.action_index += 1
            self.action_start_time = None

    def execute_turn_left(self, params: Dict) -> None:
        """Execute left turn action"""
        angular_speed = params.get('angular_speed', 0.5)  # Default 0.5 rad/s
        angle = params.get('angle', 1.57)  # Default 90 degrees in radians

        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = float(angular_speed)

        self.cmd_vel_publisher.publish(twist)

        # Check if turn is complete
        if self.action_start_time and time.time() - self.action_start_time >= abs(angle / angular_speed):
            self.execute_stop()
            self.action_index += 1
            self.action_start_time = None

    def execute_turn_right(self, params: Dict) -> None:
        """Execute right turn action"""
        angular_speed = params.get('angular_speed', 0.5)  # Default 0.5 rad/s
        angle = params.get('angle', 1.57)  # Default 90 degrees in radians

        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = -float(angular_speed)  # Negative for right turn

        self.cmd_vel_publisher.publish(twist)

        # Check if turn is complete
        if self.action_start_time and time.time() - self.action_start_time >= abs(angle / angular_speed):
            self.execute_stop()
            self.action_index += 1
            self.action_start_time = None

    def execute_navigate_to(self, params: Dict) -> None:
        """Execute navigation to target location"""
        target = params.get('target', 'unknown')
        x = params.get('x', 0.0)
        y = params.get('y', 0.0)

        # For simplicity, we'll use basic navigation
        # In a real system, this would interface with navigation stack
        self.get_logger().info(f'Navigating to {target} at ({x}, {y})')

        # Publish navigation goal (simplified)
        nav_msg = String()
        nav_msg.data = json.dumps({
            'target': target,
            'position': {'x': x, 'y': y}
        })

        # For now, just move to next action after a delay
        if self.action_start_time and time.time() - self.action_start_time >= 2.0:
            self.action_index += 1
            self.action_start_time = None

    def execute_detect_object(self, params: Dict) -> None:
        """Execute object detection action"""
        target_object = params.get('object', 'unknown')

        self.get_logger().info(f'Detecting object: {target_object}')

        # In a real system, this would interface with object detection
        # For now, we'll simulate detection with the camera image
        if self.latest_image is not None:
            # Process image to detect objects (simplified)
            height, width = self.latest_image.shape[:2]

            # Simulate detection result
            detection_result = {
                'object': target_object,
                'found': True,
                'position': {'x': width/2, 'y': height/2},  # Center of image
                'confidence': 0.9
            }

            # Publish detection result
            detection_msg = String()
            detection_msg.data = json.dumps(detection_result)

        # Move to next action after delay
        if self.action_start_time and time.time() - self.action_start_time >= 1.0:
            self.action_index += 1
            self.action_start_time = None

    def execute_manipulate(self, params: Dict) -> None:
        """Execute manipulation action"""
        action_type = params.get('type', 'grasp')
        object_name = params.get('object', 'unknown')

        self.get_logger().info(f'Performing manipulation: {action_type} on {object_name}')

        # In a real system, this would control robot manipulator
        # For now, just move to next action after delay
        if self.action_start_time and time.time() - self.action_start_time >= 2.0:
            self.action_index += 1
            self.action_start_time = None

    def execute_stop(self) -> None:
        """Stop all robot motion"""
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        self.cmd_vel_publisher.publish(twist)

    def cv2_to_pil(self, cv_image):
        """Convert OpenCV image to PIL Image"""
        import PIL.Image
        rgb_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)
        pil_image = PIL.Image.fromarray(rgb_image)
        return pil_image


def main(args=None):
    """Main function to run the VLA pipeline"""
    rclpy.init(args=args)

    try:
        vla_pipeline = VLAPipeline()
        rclpy.spin(vla_pipeline)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"Error running VLA pipeline: {e}")
    finally:
        if 'vla_pipeline' in locals():
            vla_pipeline.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()