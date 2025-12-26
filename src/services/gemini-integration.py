#!/usr/bin/env python3
# gemini_integration.py

"""
Google Gemini API Integration for Robotics
Implements the required Google Gemini integration as per constitution
"""

import google.generativeai as genai
import json
import base64
import os
from typing import Dict, List, Optional, Tuple, Any
from dataclasses import dataclass
from PIL import Image
import io


@dataclass
class VLARequest:
    """Vision-Language-Action request data structure"""
    image: Optional[bytes] = None  # Image data for vision tasks
    text: str = ""                 # Natural language command
    context: Optional[Dict] = None # Additional context (robot state, environment, etc.)


@dataclass
class VLEResponse:
    """Vision-Language-Action response data structure"""
    action_sequence: List[str]
    confidence_score: float
    reasoning: str
    parameters: Optional[Dict[str, Any]] = None
    scene_understanding: Optional[str] = None
    target_object: Optional[str] = None


class GeminiIntegrationService:
    """
    Google Gemini API Integration Service for Robotics
    Provides Vision-Language-Action capabilities using Google's Gemini models
    """

    def __init__(self, api_key: str, model_name: str = "gemini-pro"):
        """
        Initialize the Gemini integration service

        Args:
            api_key: Google API key for Gemini
            model_name: Name of the Gemini model to use
        """
        genai.configure(api_key=api_key)
        self.model_name = model_name
        self.text_model = genai.GenerativeModel(model_name)
        self.vision_model = genai.GenerativeModel('gemini-pro-vision')

    def process_vla_request(self, request: VLARequest) -> VLEResponse:
        """
        Process a Vision-Language-Action request using Google Gemini

        Args:
            request: VLA request containing image, text, and context

        Returns:
            VLA response with action sequence and metadata
        """
        try:
            if request.image:
                # Process with vision model (image + text)
                return self._process_vision_request(request)
            else:
                # Process with text-only model
                return self._process_text_request(request)
        except Exception as e:
            print(f"Error processing VLA request: {e}")
            return VLEResponse(
                action_sequence=["error"],
                confidence_score=0.0,
                reasoning=f"Error processing request: {str(e)}"
            )

    def _process_vision_request(self, request: VLARequest) -> VLEResponse:
        """Process request with image and text using vision model"""
        # Convert bytes to PIL Image
        image = Image.open(io.BytesIO(request.image))

        # Build prompt
        prompt = self._build_vision_prompt(request)

        # Generate content
        response = self.vision_model.generate_content([prompt, image])

        return self._parse_vla_response(response.text)

    def _process_text_request(self, request: VLARequest) -> VLEResponse:
        """Process text-only request using language model"""
        # Build prompt
        prompt = self._build_text_prompt(request)

        # Generate content
        response = self.text_model.generate_content(prompt)

        return self._parse_vla_response(response.text)

    def _build_vision_prompt(self, request: VLARequest) -> str:
        """Build prompt for vision-language processing"""
        context_str = json.dumps(request.context) if request.context else "No additional context provided"

        return f"""
        You are an advanced robot assistant with vision and language capabilities.
        Analyze the provided image and interpret the user's command.

        User Command: "{request.text}"

        Additional Context: {context_str}

        Provide a detailed JSON response with the following structure:
        {{
          "action_sequence": ["list", "of", "actions", "to", "execute"],
          "confidence_score": 0.85,
          "reasoning": "Brief explanation of your interpretation",
          "parameters": {{
            "object_name": "name of target object if relevant",
            "position": {{"x": 0.0, "y": 0.0, "z": 0.0}},
            "orientation": {{"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0}}
          }},
          "scene_understanding": "Description of what you see in the image",
          "target_object": "Name of the specific object related to the command"
        }}

        Respond only with valid JSON, no additional text.
        """

    def _build_text_prompt(self, request: VLARequest) -> str:
        """Build prompt for text-only processing"""
        context_str = json.dumps(request.context) if request.context else "No additional context provided"

        return f"""
        You are an advanced robot command interpreter.
        Interpret the user's command in the context of robotics.

        User Command: "{request.text}"

        Additional Context: {context_str}

        Provide a detailed JSON response with the following structure:
        {{
          "action_sequence": ["list", "of", "actions", "to", "execute"],
          "confidence_score": 0.85,
          "reasoning": "Brief explanation of your interpretation",
          "parameters": {{
            "object_name": "name of target object if relevant",
            "position": {{"x": 0.0, "y": 0.0, "z": 0.0}},
            "orientation": {{"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0}}
          }}
        }}

        Respond only with valid JSON, no additional text.
        """

    def _parse_vla_response(self, response_text: str) -> VLEResponse:
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
            parsed = json.loads(clean_text)

            return VLEResponse(
                action_sequence=parsed.get("action_sequence", []),
                confidence_score=parsed.get("confidence_score", 0.0),
                reasoning=parsed.get("reasoning", "No reasoning provided"),
                parameters=parsed.get("parameters", {}),
                scene_understanding=parsed.get("scene_understanding"),
                target_object=parsed.get("target_object")
            )
        except json.JSONDecodeError as e:
            print(f"Error parsing Gemini response: {e}")
            # Return a default response in case of parsing failure
            return VLEResponse(
                action_sequence=["error_parsing_response"],
                confidence_score=0.0,
                reasoning=f"Failed to parse Gemini response: {response_text[:200]}...",
                parameters={}
            )

    def validate_config(self) -> bool:
        """Validate the Gemini API configuration"""
        try:
            # Try a simple test request to validate the API key
            response = self.text_model.generate_content("Hello")
            return len(response.text) > 0
        except Exception as e:
            print(f"Gemini API validation failed: {e}")
            return False

    def process_natural_language_command(self, command: str, context: Optional[Dict] = None) -> VLEResponse:
        """
        Process a natural language command to robot action sequence

        Args:
            command: Natural language command from user
            context: Additional context about robot state and environment

        Returns:
            VLA response with action sequence
        """
        request = VLARequest(text=command, context=context)
        return self.process_vla_request(request)

    def process_visual_language_command(self, image: bytes, command: str, context: Optional[Dict] = None) -> VLEResponse:
        """
        Process visual scene understanding with language

        Args:
            image: Image data as bytes
            command: Natural language command
            context: Additional context about robot state and environment

        Returns:
            VLA response with action sequence
        """
        request = VLARequest(image=image, text=command, context=context)
        return self.process_vla_request(request)


# Example usage and testing
def example_usage():
    """Example usage of the Gemini Integration Service"""

    # Get API key from environment
    api_key = os.getenv("GOOGLE_API_KEY")
    if not api_key:
        print("GOOGLE_API_KEY environment variable not set")
        return

    # Initialize the service
    gemini_service = GeminiIntegrationService(api_key, "gemini-pro")

    # Validate configuration
    if not gemini_service.validate_config():
        print("Gemini API configuration is invalid")
        return

    # Example natural language command
    command = "Pick up the red cup on the table"
    context = {
        "robot_state": {
            "position": {"x": 0, "y": 0, "theta": 0},
            "gripper": "open",
            "battery": 0.85
        },
        "environment": {
            "objects": ["table", "red cup", "chair"],
            "locations": ["kitchen", "living room"]
        }
    }

    try:
        response = gemini_service.process_natural_language_command(command, context)
        print("VLA Response:", response)
    except Exception as e:
        print(f"Error processing command: {e}")


if __name__ == "__main__":
    example_usage()