---
title: Speech Recognition for Robotics
sidebar_position: 2
---

# Speech Recognition for Robotics

## Overview

This module covers speech recognition systems for robotics applications, focusing on converting spoken language to text that can be processed by language models like Google Gemini. We'll explore both local and cloud-based speech recognition solutions and their integration with robotic systems.

## Learning Objectives

By the end of this module, students will be able to:
- Set up speech recognition systems for robotic applications
- Integrate speech recognition with language models
- Handle speech recognition errors and uncertainties
- Implement robust voice command processing for robots

## Prerequisites

- Completed language models in robotics module
- Basic understanding of audio processing
- Completed ROS 2 fundamentals

## Speech Recognition in Robotics Context

### Why Speech Recognition for Robots?

Speech recognition enables natural human-robot interaction:
- **Hands-free operation**: Users can control robots without physical interfaces
- **Natural communication**: More intuitive than button-based interfaces
- **Accessibility**: Enables interaction for users with mobility limitations
- **Multimodal interaction**: Combines with vision and other sensors

### Challenges in Robotic Speech Recognition

- **Noisy environments**: Robots operate in various acoustic conditions
- **Real-time processing**: Need for low-latency response
- **Vocabulary constraints**: Limited domain-specific commands
- **Robustness**: Must handle various accents and speaking styles

## Speech Recognition Approaches

### Cloud-Based Speech Recognition

Cloud-based solutions offer high accuracy but require network connectivity:

#### Google Cloud Speech-to-Text

```python
# Install required package
pip install google-cloud-speech

# Example implementation
from google.cloud import speech
import pyaudio
import io

def transcribe_audio_cloud(audio_data):
    """Transcribe audio using Google Cloud Speech-to-Text"""
    client = speech.SpeechClient()

    audio = speech.RecognitionAudio(content=audio_data)
    config = speech.RecognitionConfig(
        encoding=speech.RecognitionConfig.AudioEncoding.LINEAR16,
        sample_rate_hertz=16000,
        language_code="en-US",
    )

    response = client.recognize(config=config, audio=audio)

    for result in response.results:
        return result.alternatives[0].transcript

    return ""
```

### Local Speech Recognition

Local solutions work without network connectivity but may have lower accuracy:

#### Using SpeechRecognition Library

```python
import speech_recognition as sr

class LocalSpeechRecognizer:
    def __init__(self):
        self.recognizer = sr.Recognizer()
        self.microphone = sr.Microphone()

        # Adjust for ambient noise
        with self.microphone as source:
            self.recognizer.adjust_for_ambient_noise(source)

    def listen_for_command(self):
        """Listen for and transcribe a voice command"""
        with self.microphone as source:
            print("Listening for command...")
            audio = self.recognizer.listen(source, timeout=5, phrase_time_limit=5)

        try:
            # Use Google Web Speech API (free, limited)
            command = self.recognizer.recognize_google(audio)
            print(f"Recognized: {command}")
            return command
        except sr.UnknownValueError:
            print("Could not understand audio")
            return None
        except sr.RequestError as e:
            print(f"Error with speech recognition service: {e}")
            return None
```

## Integration with Robot Systems

### ROS 2 Speech Recognition Node

```python
#!/usr/bin/env python3
# speech_recognition_node.py

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from audio_common_msgs.msg import AudioData
import speech_recognition as sr
import threading
import queue

class SpeechRecognitionNode(Node):
    def __init__(self):
        super().__init__('speech_recognition_node')

        # Publisher for recognized text
        self.text_publisher = self.create_publisher(
            String,
            'recognized_speech',
            10
        )

        # Publisher for voice activity
        self.vad_publisher = self.create_publisher(
            String,
            'voice_activity',
            10
        )

        # Initialize speech recognizer
        self.recognizer = sr.Recognizer()
        self.microphone = sr.Microphone()

        # Set energy threshold for voice activity detection
        self.recognizer.energy_threshold = 4000  # Adjust based on environment

        # Create a queue to share audio data between threads
        self.audio_queue = queue.Queue()

        # Start speech recognition in a separate thread
        self.recognition_thread = threading.Thread(target=self.recognition_worker)
        self.recognition_thread.daemon = True
        self.recognition_thread.start()

        self.get_logger().info('Speech recognition node initialized')

    def recognition_worker(self):
        """Worker function for speech recognition in separate thread"""
        with self.microphone as source:
            self.recognizer.adjust_for_ambient_noise(source, duration=1)

        # Use a callback for continuous listening
        stop_listening = self.recognizer.listen_in_background(
            self.microphone,
            self.audio_callback
        )

        # Keep the thread alive
        try:
            while rclpy.ok():
                # Process any queued audio
                try:
                    audio = self.audio_queue.get(timeout=1)
                    self.process_audio(audio)
                except queue.Empty:
                    continue
        except KeyboardInterrupt:
            stop_listening(wait_for_stop=False)

    def audio_callback(self, recognizer, audio):
        """Callback for when audio is captured"""
        # Put audio in queue for processing
        self.audio_queue.put(audio)

        # Publish voice activity detection
        vad_msg = String()
        vad_msg.data = "speech_detected"
        self.vad_publisher.publish(vad_msg)

    def process_audio(self, audio):
        """Process audio and publish recognized text"""
        try:
            # Recognize speech using Google Web Speech API
            text = recognizer.recognize_google(audio)

            # Publish recognized text
            text_msg = String()
            text_msg.data = text
            self.text_publisher.publish(text_msg)

            self.get_logger().info(f'Recognized: {text}')

        except sr.UnknownValueError:
            self.get_logger().info('Speech not understood')
        except sr.RequestError as e:
            self.get_logger().error(f'Speech recognition error: {e}')

def main(args=None):
    rclpy.init(args=args)
    speech_node = SpeechRecognitionNode()

    try:
        rclpy.spin(speech_node)
    except KeyboardInterrupt:
        pass
    finally:
        speech_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Voice Command Processing Pipeline

### Combining Speech Recognition with Language Models

```python
#!/usr/bin/env python3
# voice_command_processor.py

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import google.generativeai as genai
import json
import speech_recognition as sr

class VoiceCommandProcessor(Node):
    def __init__(self):
        super().__init__('voice_command_processor')

        # Subscribe to recognized speech
        self.speech_subscription = self.create_subscription(
            String,
            'recognized_speech',
            self.speech_callback,
            10
        )

        # Publisher for processed commands
        self.command_publisher = self.create_publisher(
            String,
            'processed_commands',
            10
        )

        # Publisher for robot actions
        self.action_publisher = self.create_publisher(
            String,
            'robot_actions',
            10
        )

        # Initialize Gemini model
        self.model = genai.GenerativeModel('gemini-pro')

        # Command confidence threshold
        self.confidence_threshold = 0.7

        self.get_logger().info('Voice command processor initialized')

    def speech_callback(self, msg):
        """Process recognized speech command"""
        try:
            # Get robot state and context
            robot_state = self.get_robot_state()
            environment_context = self.get_environment_context()

            # Process command with Gemini
            processed_command = self.process_command_with_gemini(
                msg.data,
                robot_state,
                environment_context
            )

            # Check confidence threshold
            if processed_command.get('confidence_score', 0) >= self.confidence_threshold:
                # Publish processed command
                command_msg = String()
                command_msg.data = json.dumps(processed_command)
                self.command_publisher.publish(command_msg)

                # Publish robot actions
                action_msg = String()
                action_msg.data = json.dumps(processed_command.get('action_sequence', []))
                self.action_publisher.publish(action_msg)

                self.get_logger().info(f'Executed command: {msg.data}')
            else:
                self.get_logger().warn(
                    f'Command confidence too low: {processed_command.get("confidence_score", 0)}'
                )

        except Exception as e:
            self.get_logger().error(f'Error processing speech: {e}')

    def process_command_with_gemini(self, command, robot_state, environment_context):
        """Process command using Gemini with context"""
        prompt = f"""
        You are a robot command interpreter. Interpret the following voice command
        in the context of the robot's current state and environment.

        Robot State: {json.dumps(robot_state)}
        Environment Context: {json.dumps(environment_context)}
        Voice Command: {command}

        Provide a JSON response with:
        1. action_sequence: List of actions to execute
        2. confidence_score: Confidence in interpretation (0.0 to 1.0)
        3. reasoning: Brief explanation of interpretation
        4. parameters: Any needed parameters for actions
        5. clarification_needed: Boolean if command is ambiguous
        """

        try:
            response = self.model.generate_content(prompt)
            result = json.loads(response.text.strip().strip('```json').strip('`'))
            return result
        except (json.JSONDecodeError, Exception) as e:
            self.get_logger().error(f'Error processing with Gemini: {e}')
            return {
                "action_sequence": ["unknown"],
                "confidence_score": 0.0,
                "reasoning": "Failed to process command",
                "parameters": {},
                "clarification_needed": True
            }

    def get_robot_state(self):
        """Get current robot state"""
        return {
            "position": {"x": 0.0, "y": 0.0, "theta": 0.0},
            "battery": 0.85,
            "gripper": "open",
            "current_task": "idle"
        }

    def get_environment_context(self):
        """Get environment context"""
        return {
            "objects": ["table", "chair", "cup"],
            "rooms": ["kitchen", "living_room"],
            "navigation_goals": ["kitchen", "bedroom"]
        }

def main(args=None):
    rclpy.init(args=args)
    processor = VoiceCommandProcessor()

    try:
        rclpy.spin(processor)
    except KeyboardInterrupt:
        pass
    finally:
        processor.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Handling Recognition Errors and Uncertainties

### Confidence Scoring and Error Handling

```python
class RobustSpeechProcessor:
    def __init__(self):
        self.min_confidence = 0.7
        self.retry_count = 3

    def process_with_confidence(self, audio_data):
        """Process audio with confidence scoring"""
        # This is a simplified example
        # In practice, you'd use the recognition service's confidence scores
        recognized_text = self.recognize_audio(audio_data)

        # Estimate confidence based on various factors
        confidence = self.estimate_confidence(recognized_text, audio_data)

        if confidence >= self.min_confidence:
            return recognized_text, confidence
        else:
            # Request clarification or retry
            return self.handle_low_confidence(recognized_text, confidence)

    def estimate_confidence(self, text, audio_data):
        """Estimate confidence in recognition result"""
        # Implement confidence estimation logic
        # Consider factors like audio quality, word probabilities, etc.
        return 0.8  # Placeholder

    def handle_low_confidence(self, text, confidence):
        """Handle cases where confidence is too low"""
        # Ask for clarification, repeat command, or use fallback
        return text, confidence
```

## Practical Exercise: Voice Command Robot Interface

### Objective
Create a complete voice command system that converts speech to robot actions.

### Steps
1. Set up speech recognition node
2. Integrate with language model processing
3. Implement command execution pipeline
4. Test with various voice commands

### Complete System Architecture

```
Microphone → Audio Capture → Speech Recognition → Text → Language Model → Robot Actions
```

## Assessment

Students must demonstrate the following:
1. Successfully set up speech recognition for robot commands
2. Integrate speech recognition with language model processing
3. Handle recognition errors and uncertainties appropriately
4. Achieve 80% success rate in voice command processing

**Success Threshold: 80% success rate in voice command processing**

## Next Steps

After completing this module, proceed to multi-modal interaction to create a complete conversational robotics system that combines vision, language, and speech recognition.