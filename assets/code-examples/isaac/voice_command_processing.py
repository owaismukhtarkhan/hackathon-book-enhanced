#!/usr/bin/env python3
# voice_command_processing.py

"""
Voice Command Processing System for Robotics
Integrates speech recognition with Google Gemini for natural language processing
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
import speech_recognition as sr
import google.generativeai as genai
import json
import os
import threading
import queue
import time
from typing import Dict, List, Optional
import numpy as np
from PIL import Image as PILImage
import io


class VoiceCommandProcessor(Node):
    """
    Voice Command Processing Node
    Integrates speech recognition, language processing with Google Gemini,
    and action execution for robotics applications
    """

    def __init__(self):
        super().__init__('voice_command_processor')

        # Initialize components
        self.recognizer = sr.Recognizer()
        self.microphone = sr.Microphone()
        self.bridge = CvBridge()

        # Load Google API key
        api_key = os.getenv("GOOGLE_API_KEY")
        if not api_key:
            self.get_logger().error("GOOGLE_API_KEY environment variable not set")
            raise ValueError("GOOGLE_API_KEY environment variable not set")

        genai.configure(api_key=api_key)
        self.gemini_model = genai.GenerativeModel('gemini-pro')
        self.vision_model = genai.GenerativeModel('gemini-pro-vision')

        # Robot state and context
        self.robot_state = {
            'position': {'x': 0.0, 'y': 0.0, 'theta': 0.0},
            'battery': 1.0,
            'gripper': 'open',
            'current_task': 'idle'
        }

        # Latest sensor data
        self.latest_image = None
        self.latest_objects = []

        # Voice recognition components
        self.audio_queue = queue.Queue()
        self.listening = True

        # Publishers
        self.action_publisher = self.create_publisher(
            String,
            '/robot_actions',
            10
        )

        self.cmd_vel_publisher = self.create_publisher(
            Twist,
            '/cmd_vel',
            10
        )

        self.feedback_publisher = self.create_publisher(
            String,
            '/voice_feedback',
            10
        )

        # Subscribers
        self.image_subscription = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10
        )

        # Initialize speech recognition
        self._setup_speech_recognition()

        # Action execution tracking
        self.current_action_sequence = []
        self.action_index = 0
        self.action_start_time = None
        self.action_timeout = 15.0  # seconds

        # Start voice processing in background
        self.voice_thread = threading.Thread(target=self._voice_processing_loop)
        self.voice_thread.daemon = True
        self.voice_thread.start()

        # Timer for action execution
        self.action_timer = self.create_timer(0.1, self._execute_action_step)

        self.get_logger().info('Voice command processor initialized')

    def _setup_speech_recognition(self):
        """Setup speech recognition parameters"""
        # Adjust for ambient noise
        with self.microphone as source:
            self.recognizer.adjust_for_ambient_noise(source, duration=1.0)

        # Set energy threshold for voice activity detection
        self.recognizer.energy_threshold = 3000  # Adjust based on environment
        self.recognizer.dynamic_energy_threshold = True

        self.get_logger().info('Speech recognition setup complete')

    def image_callback(self, msg):
        """Handle incoming camera images"""
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            self.latest_image = cv_image
        except Exception as e:
            self.get_logger().error(f'Error processing image: {e}')

    def _voice_processing_loop(self):
        """Background loop for continuous voice processing"""
        with self.microphone as source:
            self.get_logger().info('Starting voice recognition...')

        while self.listening:
            try:
                # Listen for audio with timeout
                with self.microphone as source:
                    audio = self.recognizer.listen(source, timeout=1.0, phrase_time_limit=5.0)

                # Add audio to queue for processing
                self.audio_queue.put(audio)

            except sr.WaitTimeoutError:
                # No speech detected, continue loop
                continue
            except Exception as e:
                self.get_logger().error(f'Error in voice recognition: {e}')
                time.sleep(0.1)

    def _process_audio_from_queue(self):
        """Process audio from the queue"""
        try:
            audio = self.audio_queue.get_nowait()
            self._process_audio(audio)
        except queue.Empty:
            return

    def _process_audio(self, audio):
        """Process audio and convert to text"""
        try:
            # Use Google Web Speech API for recognition
            text = self.recognizer.recognize_google(audio)
            self.get_logger().info(f'Recognized: {text}')

            # Process the recognized text
            self._process_recognized_text(text)

        except sr.UnknownValueError:
            self.get_logger().info('Speech not understood')
        except sr.RequestError as e:
            self.get_logger().error(f'Speech recognition error: {e}')
        except Exception as e:
            self.get_logger().error(f'Error processing audio: {e}')

    def _process_recognized_text(self, text: str):
        """Process recognized text using Google Gemini"""
        try:
            # Prepare context for Gemini
            context = {
                'robot_state': self.robot_state,
                'environment': {
                    'objects': self.latest_objects,
                    'image_available': self.latest_image is not None
                }
            }

            # Build prompt for Gemini
            prompt = f"""
            You are a robot command interpreter. Interpret the following voice command
            in the context of the robot's current state and environment.

            Voice Command: "{text}"

            Robot State: {json.dumps(context['robot_state'])}
            Environment Context: {json.dumps(context['environment'])}

            Provide a detailed JSON response with:
            1. action_sequence: List of specific actions to execute
            2. confidence_score: Confidence in interpretation (0.0 to 1.0)
            3. reasoning: Brief explanation of interpretation
            4. parameters: Any parameters needed for actions
            5. requires_vision: Boolean if visual processing is needed
            6. safety_considerations: Any safety issues to consider
            """

            # Process with Gemini
            response = self.gemini_model.generate_content(prompt)
            parsed_response = self._parse_gemini_response(response.text)

            if parsed_response and parsed_response.get('confidence_score', 0) >= 0.6:
                # If vision is required and we have an image, process with vision model
                if parsed_response.get('requires_vision') and self.latest_image:
                    vision_response = self._process_with_vision(text, self.latest_image, context)
                    if vision_response:
                        parsed_response = vision_response

                # Store action sequence for execution
                self.current_action_sequence = parsed_response.get('action_sequence', [])
                self.action_index = 0
                self.action_start_time = time.time()

                # Publish action sequence
                action_msg = String()
                action_msg.data = json.dumps(parsed_response)
                self.action_publisher.publish(action_msg)

                # Publish feedback
                feedback_msg = String()
                feedback_msg.data = json.dumps({
                    'status': 'command_processed',
                    'command': text,
                    'actions_count': len(self.current_action_sequence),
                    'confidence': parsed_response.get('confidence_score', 0)
                })
                self.feedback_publisher.publish(feedback_msg)

                self.get_logger().info(f'Processed voice command: {text} with {len(self.current_action_sequence)} actions')

            else:
                self.get_logger().warn(f'Low confidence processing command: {text}')

        except Exception as e:
            self.get_logger().error(f'Error processing recognized text: {e}')

    def _process_with_vision(self, text: str, image, context: Dict) -> Optional[Dict]:
        """Process command with visual context using Gemini Vision"""
        try:
            # Convert OpenCV image to PIL
            rgb_image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
            pil_image = PILImage.fromarray(rgb_image)

            # Build vision prompt
            prompt = f"""
            You are a robot with vision and language capabilities.
            Analyze the image and interpret the following command: "{text}"

            Robot State: {json.dumps(context['robot_state'])}

            Provide a detailed JSON response with:
            1. action_sequence: List of specific actions to execute
            2. confidence_score: Confidence in interpretation (0.0 to 1.0)
            3. reasoning: Brief explanation of interpretation
            4. target_object: Object in the image related to the command
            5. object_location: Location of the target object in the image (x, y)
            6. parameters: Any parameters needed for actions
            """

            # Process with vision model
            response = self.vision_model.generate_content([prompt, pil_image])
            return self._parse_gemini_response(response.text)

        except Exception as e:
            self.get_logger().error(f'Error in vision processing: {e}')
            return None

    def _parse_gemini_response(self, response_text: str) -> Optional[Dict]:
        """Parse Gemini response into structured data"""
        try:
            # Clean the response text
            clean_text = response_text.strip()
            if clean_text.startswith('```json'):
                clean_text = clean_text[7:]  # Remove ```json
            if clean_text.endswith('```'):
                clean_text = clean_text[:-3]  # Remove ```
            clean_text = clean_text.strip()

            # Parse JSON
            parsed = json.loads(clean_text)
            return parsed

        except json.JSONDecodeError as e:
            self.get_logger().error(f'Error parsing Gemini response: {e}')
            # Try to extract basic information from non-JSON response
            return {
                'action_sequence': ['unknown_command'],
                'confidence_score': 0.0,
                'reasoning': f'Failed to parse response: {response_text[:200]}...',
                'parameters': {},
                'requires_vision': False
            }

    def _execute_action_step(self):
        """Execute actions in the sequence"""
        # Process any audio in the queue
        try:
            self._process_audio_from_queue()
        except:
            pass  # Continue even if audio processing fails

        # Execute current action if available
        if self.current_action_sequence and self.action_index < len(self.current_action_sequence):
            self._execute_current_action()
        else:
            # Clear action sequence when complete
            if self.current_action_sequence:
                self.current_action_sequence = []
                self.action_index = 0
                self.action_start_time = None

                # Publish completion feedback
                feedback_msg = String()
                feedback_msg.data = json.dumps({'status': 'actions_complete'})
                self.feedback_publisher.publish(feedback_msg)

    def _execute_current_action(self):
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

        # Parse action (could be string or dict)
        if isinstance(current_action, str):
            action_name = current_action.lower()
            action_params = {}
        elif isinstance(current_action, dict):
            action_name = current_action.get('action', 'unknown').lower()
            action_params = current_action.get('parameters', {})
        else:
            action_name = 'unknown'
            action_params = {}

        # Execute specific actions
        if 'move' in action_name or 'forward' in action_name:
            self._execute_move_action(action_params)
        elif 'backward' in action_name or 'back' in action_name:
            self._execute_move_backward_action(action_params)
        elif 'turn' in action_name or 'rotate' in action_name:
            self._execute_turn_action(action_name, action_params)
        elif 'navigate' in action_name or 'go_to' in action_name:
            self._execute_navigate_action(action_params)
        elif 'detect' in action_name or 'find' in action_name:
            self._execute_detect_action(action_params)
        elif 'grasp' in action_name or 'pick' in action_name:
            self._execute_grasp_action(action_params)
        elif 'stop' in action_name or 'halt' in action_name:
            self._execute_stop_action()
        elif 'wait' in action_name or 'pause' in action_name:
            self._execute_wait_action(action_params)
        else:
            self.get_logger().info(f'Executing unknown action: {action_name}')
            self.action_index += 1  # Move to next action immediately

        # Initialize action start time if not set
        if not self.action_start_time:
            self.action_start_time = time.time()

    def _execute_move_action(self, params: Dict):
        """Execute forward movement action"""
        speed = params.get('speed', 0.2)  # Default 0.2 m/s
        duration = params.get('duration', 2.0)  # Default 2 seconds

        twist = Twist()
        twist.linear.x = float(speed)
        twist.angular.z = 0.0

        self.cmd_vel_publisher.publish(twist)

        # Check if duration-based execution is complete
        if self.action_start_time and time.time() - self.action_start_time >= duration:
            self._execute_stop_action()
            self.action_index += 1
            self.action_start_time = None

    def _execute_move_backward_action(self, params: Dict):
        """Execute backward movement action"""
        speed = params.get('speed', 0.2)  # Default 0.2 m/s
        duration = params.get('duration', 1.0)  # Default 1 second

        twist = Twist()
        twist.linear.x = -float(speed)  # Negative for backward
        twist.angular.z = 0.0

        self.cmd_vel_publisher.publish(twist)

        # Check if duration-based execution is complete
        if self.action_start_time and time.time() - self.action_start_time >= duration:
            self._execute_stop_action()
            self.action_index += 1
            self.action_start_time = None

    def _execute_turn_action(self, action_name: str, params: Dict):
        """Execute turn action (left or right)"""
        angular_speed = params.get('angular_speed', 0.5)  # Default 0.5 rad/s
        angle = params.get('angle', 1.57)  # Default 90 degrees in radians

        twist = Twist()
        twist.linear.x = 0.0

        # Determine turn direction based on action name
        if 'left' in action_name:
            twist.angular.z = float(angular_speed)
        elif 'right' in action_name:
            twist.angular.z = -float(angular_speed)  # Negative for right turn
        else:
            # Default to left if direction not specified
            twist.angular.z = float(angular_speed)

        self.cmd_vel_publisher.publish(twist)

        # Check if turn is complete
        if self.action_start_time and time.time() - self.action_start_time >= abs(angle / angular_speed):
            self._execute_stop_action()
            self.action_index += 1
            self.action_start_time = None

    def _execute_navigate_action(self, params: Dict):
        """Execute navigation to target location"""
        target = params.get('target', 'unknown')
        x = params.get('x', 0.0)
        y = params.get('y', 0.0)

        self.get_logger().info(f'Navigating to {target} at ({x}, {y})')

        # For now, just move to next action after a delay
        # In a real system, this would interface with navigation stack
        if self.action_start_time and time.time() - self.action_start_time >= 3.0:
            self.action_index += 1
            self.action_start_time = None

    def _execute_detect_action(self, params: Dict):
        """Execute object detection action"""
        target_object = params.get('object', 'unknown')

        self.get_logger().info(f'Detecting object: {target_object}')

        # In a real system, this would interface with object detection
        # For now, simulate detection with available image
        if self.latest_image is not None:
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
        if self.action_start_time and time.time() - self.action_start_time >= 2.0:
            self.action_index += 1
            self.action_start_time = None

    def _execute_grasp_action(self, params: Dict):
        """Execute grasping/manipulation action"""
        object_name = params.get('object', 'unknown')

        self.get_logger().info(f'Attempting to grasp: {object_name}')

        # In a real system, this would control robot manipulator
        # For now, just move to next action after delay
        if self.action_start_time and time.time() - self.action_start_time >= 3.0:
            self.action_index += 1
            self.action_start_time = None

    def _execute_stop_action(self):
        """Stop all robot motion"""
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        self.cmd_vel_publisher.publish(twist)

    def _execute_wait_action(self, params: Dict):
        """Execute wait/pause action"""
        duration = params.get('duration', 1.0)  # Default 1 second

        # Wait for specified duration
        if self.action_start_time and time.time() - self.action_start_time >= duration:
            self.action_index += 1
            self.action_start_time = None

    def destroy_node(self):
        """Clean up before node destruction"""
        self.listening = False
        super().destroy_node()


def main(args=None):
    """Main function to run the voice command processor"""
    rclpy.init(args=args)

    try:
        processor = VoiceCommandProcessor()
        rclpy.spin(processor)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"Error running voice command processor: {e}")
    finally:
        if 'processor' in locals():
            processor.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()