---
sidebar_position: 18
title: Chapter 14 | Voice to Action using OpenAI Whisper for voice commands
---

# Chapter 14: Voice to Action using OpenAI Whisper for Voice Commands

In this chapter, we'll explore how to implement voice-to-action systems using OpenAI Whisper for speech recognition. We'll learn how to process spoken commands and convert them into robotic actions, enabling natural human-robot interaction through speech.

## Learning Objectives

By the end of this chapter, you will be able to:
- Set up and configure OpenAI Whisper for robotic voice command processing
- Implement real-time speech recognition for robot control
- Integrate voice commands with robotic action execution
- Design voice command grammars for robotic tasks
- Handle speech recognition errors and implement robust command processing

## Introduction to Voice to Action Systems

Voice-to-action systems enable robots to understand and respond to spoken commands, providing a natural and intuitive interface for human-robot interaction. These systems convert audio input to text, interpret the meaning, and execute corresponding robotic actions.

### The Voice Command Pipeline

#### Audio Capture and Preprocessing
- Microphone array configuration
- Audio signal conditioning
- Noise reduction and filtering
- Voice activity detection

#### Speech Recognition
- Automatic Speech Recognition (ASR)
- Language model integration
- Context-aware recognition
- Real-time processing capabilities

#### Natural Language Understanding
- Command interpretation
- Intent classification
- Entity extraction
- Context resolution

#### Action Execution
- Command-to-action mapping
- Safety validation
- Execution monitoring
- Feedback generation

### Benefits of Voice Command Systems

#### Natural Interaction
- Intuitive communication method
- Hands-free operation
- Accessible to diverse users
- Familiar interaction pattern

#### Flexibility
- No need for physical interfaces
- Multiple command formats
- Context-sensitive commands
- Adaptable to user preferences

## OpenAI Whisper for Speech Recognition

### Whisper Architecture Overview

OpenAI Whisper is a state-of-the-art speech recognition model that provides exceptional accuracy across multiple languages and audio conditions. It uses a transformer-based architecture with both encoder and decoder components.

#### Key Features
- Multilingual support (99+ languages)
- Robust performance in noisy environments
- Speaker diarization capabilities
- Timestamp alignment for speech segments
- Open-source implementation available

### Whisper Models

#### Model Variants
- **tiny**: Fastest, smallest model (~39M parameters)
- **base**: Small model (~74M parameters)
- **small**: Medium model (~244M parameters)
- **medium**: Large model (~769M parameters)
- **large**: Largest model (~1550M parameters)
- **large-v2**: Improved large model

#### Model Selection Considerations
- **Latency requirements**: Tiny/base for real-time applications
- **Accuracy needs**: Medium/large for high accuracy
- **Hardware constraints**: Consider GPU/CPU capabilities
- **Language requirements**: Verify multilingual support

### Whisper Implementation for Robotics

#### Installation and Setup
```bash
# Install Whisper and dependencies
pip install openai-whisper
pip install torch torchvision torchaudio
pip install sounddevice numpy

# For GPU acceleration (optional)
pip install openai-whisper[cuda]
```

#### Basic Whisper Usage
```python
import whisper
import torch
import numpy as np
import sounddevice as sd
import queue
import threading
import time

class WhisperVoiceRecognizer:
    def __init__(self, model_size="base"):
        """
        Initialize Whisper voice recognizer
        """
        # Check for GPU availability
        device = "cuda" if torch.cuda.is_available() else "cpu"

        # Load Whisper model
        self.model = whisper.load_model(model_size).to(device)

        # Audio parameters
        self.sample_rate = 16000
        self.chunk_duration = 1.0  # seconds
        self.chunk_size = int(self.sample_rate * self.chunk_duration)

        # Data buffers
        self.audio_buffer = np.array([])

        # Voice activity detection parameters
        self.vad_threshold = 0.01
        self.silence_duration = 1.0  # seconds of silence to stop

        # Processing queues
        self.transcription_queue = queue.Queue()
        self.command_queue = queue.Queue()

        # Processing threads
        self.processing_thread = None
        self.listening_active = False

    def start_listening(self):
        """Start listening for voice commands"""
        self.listening_active = True

        # Start audio input callback
        def audio_callback(indata, frames, time, status):
            if status:
                print(f"Audio status: {status}")

            # Add to buffer
            audio_data = indata[:, 0].copy()
            self.audio_buffer = np.concatenate([self.audio_buffer, audio_data])

            # Check for voice activity
            energy = np.mean(audio_data ** 2)
            if energy > self.vad_threshold:
                # Voice detected, keep buffer
                # Limit buffer size to prevent excessive memory usage
                max_buffer_size = self.sample_rate * 10  # 10 seconds max
                if len(self.audio_buffer) > max_buffer_size:
                    self.audio_buffer = self.audio_buffer[-max_buffer_size:]

        # Start audio stream
        self.stream = sd.InputStream(callback=audio_callback, channels=1, samplerate=self.sample_rate)
        self.stream.start()

        # Start processing thread
        self.processing_thread = threading.Thread(target=self.processing_loop, daemon=True)
        self.processing_thread.start()

        print("Started listening for voice commands...")

    def stop_listening(self):
        """Stop listening for voice commands"""
        self.listening_active = False
        if hasattr(self, 'stream'):
            self.stream.stop()
            self.stream.close()
        if self.processing_thread:
            self.processing_thread.join()

    def processing_loop(self):
        """Main processing loop for voice recognition"""
        silence_counter = 0
        silence_threshold = int(self.silence_duration * self.sample_rate / self.chunk_size)

        while self.listening_active:
            time.sleep(0.1)

            # Check for voice activity
            if len(self.audio_buffer) > self.sample_rate * 0.5:  # At least 0.5 seconds
                recent_audio = self.audio_buffer[-self.sample_rate:]  # Last 1 second
                energy = np.mean(recent_audio ** 2)

                if energy < self.vad_threshold:
                    silence_counter += len(recent_audio)

                    if silence_counter >= silence_threshold and len(self.audio_buffer) > self.sample_rate:
                        # Process accumulated audio
                        audio_to_process = self.audio_buffer.copy()
                        self.audio_buffer = np.array([])  # Clear buffer
                        silence_counter = 0

                        # Transcribe audio
                        transcription = self.transcribe_audio(audio_to_process)

                        if transcription.strip():
                            self.transcription_queue.put(transcription)
                            print(f'Transcribed: {transcription}')
                else:
                    silence_counter = 0

    def transcribe_audio(self, audio_data):
        """Transcribe audio using Whisper"""
        try:
            # Normalize audio
            audio_normalized = audio_data / np.max(np.abs(audio_data))

            # Transcribe
            result = self.model.transcribe(
                audio_normalized,
                language="en",
                fp16=torch.cuda.is_available(),
                temperature=0.0
            )

            return result["text"].strip()
        except Exception as e:
            print(f'Whisper transcription error: {e}')
            return ""

    def get_transcription(self, timeout=None):
        """Get next transcription from queue"""
        try:
            return self.transcription_queue.get(timeout=timeout)
        except queue.Empty:
            return None

    def process_voice_command(self, command_text):
        """Process voice command and convert to robot action"""
        # This would connect to the LLM and ROS system from previous chapters
        print(f"Processing command: {command_text}")

        # In a real implementation, this would:
        # 1. Send command to LLM for interpretation
        # 2. Generate action sequence
        # 3. Execute via ROS
        return {"command": command_text, "actions": []}
```

## Voice Command Processing for Robotics

### Command Grammar Design

#### Robot Command Vocabulary
For effective voice-to-action systems, we need to design a command grammar that robots can reliably recognize and interpret:

```python
import re
import json

class RobotCommandGrammar:
    def __init__(self):
        # Define command patterns
        self.command_patterns = {
            # Navigation commands
            "move_forward": [
                r"go forward (?P<distance>\d+(?:\.\d+)?) meters?",
                r"move forward (?P<distance>\d+(?:\.\d+)?) meters?",
                r"take (?P<distance>\d+(?:\.\d+)?) steps forward",
                r"move (?P<distance>\d+(?:\.\d+)?) meters forward"
            ],

            "move_backward": [
                r"go backward (?P<distance>\d+(?:\.\d+)?) meters?",
                r"move backward (?P<distance>\d+(?:\.\d+)?) meters?",
                r"take (?P<distance>\d+(?:\.\d+)?) steps backward",
                r"move (?P<distance>\d+(?:\.\d+)?) meters backward"
            ],

            "turn_left": [
                r"turn left (?P<angle>\d+(?:\.\d+)?) degrees?",
                r"rotate left (?P<angle>\d+(?:\.\d+)?) degrees?",
                r"pivot left (?P<angle>\d+(?:\.\d+)?) degrees?"
            ],

            "turn_right": [
                r"turn right (?P<angle>\d+(?:\.\d+)?) degrees?",
                r"rotate right (?P<angle>\d+(?:\.\d+)?) degrees?",
                r"pivot right (?P<angle>\d+(?:\.\d+)?) degrees?"
            ],

            "go_to_location": [
                r"go to the (?P<location>\w+)",
                r"move to the (?P<location>\w+)",
                r"navigate to the (?P<location>\w+)",
                r"walk to the (?P<location>\w+)"
            ],

            "pick_object": [
                r"pick up the (?P<object>\w+)",
                r"grab the (?P<object>\w+)",
                r"take the (?P<object>\w+)",
                r"collect the (?P<object>\w+)"
            ],

            "place_object": [
                r"place the (?P<object>\w+) (?P<location>on|in|at) the (?P<target>\w+)",
                r"put the (?P<object>\w+) (?P<location>on|in|at) the (?P<target>\w+)",
                r"set the (?P<object>\w+) (?P<location>on|in|at) the (?P<target>\w+)"
            ],

            "speak": [
                r"say '(?P<text>[^']+)'",
                r"speak '(?P<text>[^']+)'",
                r"tell me '(?P<text>[^']+)'",
                r"repeat '(?P<text>[^']+)'"
            ]
        }

        # Location keywords
        self.locations = {
            "kitchen", "bedroom", "living room", "office",
            "bathroom", "dining room", "hallway", "garage"
        }

        # Object keywords
        self.objects = {
            "cup", "bottle", "book", "phone", "keys",
            "apple", "banana", "water", "snack", "toy"
        }

    def parse_command(self, text):
        """Parse recognized text into robot command"""
        text_lower = text.lower().strip()

        for command_type, patterns in self.command_patterns.items():
            for pattern in patterns:
                import re
                match = re.search(pattern, text_lower)
                if match:
                    params = match.groupdict()

                    # Validate parameters
                    if command_type in ["move_forward", "move_backward"]:
                        if "distance" in params:
                            params["distance"] = float(params["distance"])

                    elif command_type in ["turn_left", "turn_right"]:
                        if "angle" in params:
                            params["angle"] = float(params["angle"])

                    return {
                        "command": command_type,
                        "parameters": params,
                        "original_text": text
                    }

        # If no specific command matched, return as general command
        return {
            "command": "unknown",
            "parameters": {"text": text},
            "original_text": text
        }

    def validate_command(self, parsed_command):
        """Validate command against robot capabilities"""
        command_type = parsed_command["command"]

        # Check if command is supported
        if command_type == "unknown":
            return False, "Command not recognized"

        # Validate parameters
        if command_type in ["move_forward", "move_backward"]:
            distance = parsed_command["parameters"].get("distance", 0)
            if distance <= 0 or distance > 10:  # Reasonable limits
                return False, f"Invalid distance: {distance}. Must be 0-10 meters."

        elif command_type in ["turn_left", "turn_right"]:
            angle = parsed_command["parameters"].get("angle", 0)
            if angle <= 0 or angle > 360:  # Reasonable limits
                return False, f"Invalid angle: {angle}. Must be 0-360 degrees."

        return True, "Command is valid"
```

### Voice Command Processor

```python
import threading
import time
from enum import Enum

class CommandStatus(Enum):
    PENDING = "pending"
    EXECUTING = "executing"
    SUCCESS = "success"
    FAILED = "failed"
    CANCELLED = "cancelled"

class VoiceCommandProcessor:
    def __init__(self, whisper_recognizer, robot_controller):
        self.whisper_recognizer = whisper_recognizer
        self.robot_controller = robot_controller
        self.command_grammar = RobotCommandGrammar()

        # Command processing queues
        self.command_queue = queue.Queue()
        self.result_queue = queue.Queue()

        # Command execution state
        self.current_command = None
        self.command_status = CommandStatus.PENDING

        # Processing thread
        self.processing_thread = None
        self.processing_active = False

    def start_processing(self):
        """Start command processing loop"""
        self.processing_active = True
        self.processing_thread = threading.Thread(target=self.processing_loop, daemon=True)
        self.processing_thread.start()

    def stop_processing(self):
        """Stop command processing"""
        self.processing_active = False
        if self.processing_thread:
            self.processing_thread.join()

    def processing_loop(self):
        """Main command processing loop"""
        while self.processing_active:
            try:
                # Get transcription from Whisper
                transcription = self.whisper_recognizer.get_transcription(timeout=0.1)

                if transcription:
                    # Parse command
                    parsed_command = self.command_grammar.parse_command(transcription)

                    if parsed_command["command"] != "unknown":
                        # Validate command
                        is_valid, validation_msg = self.command_grammar.validate_command(parsed_command)

                        if is_valid:
                            # Execute command
                            self.execute_command(parsed_command)
                        else:
                            print(f"Invalid command: {validation_msg}")
                            self.speak_response(f"Sorry, {validation_msg}")

            except Exception as e:
                print(f"Command processing error: {e}")
                time.sleep(0.1)

    def execute_command(self, parsed_command):
        """Execute parsed command on robot"""
        command_type = parsed_command["command"]
        parameters = parsed_command["parameters"]

        print(f"Executing command: {command_type} with parameters: {parameters}")

        try:
            if command_type == "move_forward":
                distance = parameters.get("distance", 1.0)
                success = self.robot_controller.move_forward(distance)
            elif command_type == "move_backward":
                distance = parameters.get("distance", 1.0)
                success = self.robot_controller.move_backward(distance)
            elif command_type == "turn_left":
                angle = parameters.get("angle", 90.0)
                success = self.robot_controller.turn_left(angle)
            elif command_type == "turn_right":
                angle = parameters.get("angle", 90.0)
                success = self.robot_controller.turn_right(angle)
            elif command_type == "go_to_location":
                location = parameters.get("location", "unknown")
                success = self.robot_controller.go_to_location(location)
            elif command_type == "pick_object":
                obj = parameters.get("object", "unknown")
                success = self.robot_controller.pick_object(obj)
            elif command_type == "place_object":
                obj = parameters.get("object", "unknown")
                target = parameters.get("target", "unknown")
                success = self.robot_controller.place_object(obj, target)
            elif command_type == "speak":
                text = parameters.get("text", "")
                success = self.robot_controller.speak(text)
            else:
                print(f"Unknown command type: {command_type}")
                success = False

            if success:
                print(f"Command {command_type} executed successfully")
                self.speak_response("Command completed successfully")
            else:
                print(f"Command {command_type} failed")
                self.speak_response("Sorry, I couldn't complete that command")

        except Exception as e:
            print(f"Command execution error: {e}")
            self.speak_response("Sorry, there was an error executing your command")

    def speak_response(self, text):
        """Make robot speak response (placeholder implementation)"""
        print(f"Robot says: {text}")
        # In real implementation, this would use text-to-speech system

class RobotController:
    """Placeholder robot controller - would interface with actual robot"""

    def move_forward(self, distance):
        """Move robot forward by specified distance"""
        print(f"Moving forward {distance} meters")
        # In real implementation, this would send command to navigation system
        return True

    def move_backward(self, distance):
        """Move robot backward by specified distance"""
        print(f"Moving backward {distance} meters")
        return True

    def turn_left(self, angle):
        """Turn robot left by specified angle"""
        print(f"Turning left {angle} degrees")
        return True

    def turn_right(self, angle):
        """Turn robot right by specified angle"""
        print(f"Turning right {angle} degrees")
        return True

    def go_to_location(self, location):
        """Navigate to specified location"""
        print(f"Going to {location}")
        return True

    def pick_object(self, obj):
        """Pick up specified object"""
        print(f"Picking up {obj}")
        return True

    def place_object(self, obj, target):
        """Place object at target location"""
        print(f"Placing {obj} at {target}")
        return True

    def speak(self, text):
        """Make robot speak text"""
        print(f"Speaking: {text}")
        return True
```

## Advanced Voice Processing Features

### Real-Time Processing

```python
import pyaudio
import numpy as np
import threading
import queue
import webrtcvad

class RealTimeVoiceProcessor:
    def __init__(self, whisper_model, robot_controller):
        self.whisper_model = whisper_model
        self.robot_controller = robot_controller

        # Audio configuration
        self.rate = 16000
        self.chunk = 1024
        self.format = pyaudio.paInt16
        self.channels = 1

        # Voice activity detection
        self.vad = webrtcvad.Vad()
        self.vad.set_mode(1)  # Aggressiveness mode (0-3)

        # Audio buffers
        self.audio_queue = queue.Queue()
        self.voice_buffer = np.array([], dtype=np.int16)

        # Processing state
        self.listening = False
        self.processing_thread = None
        self.audio_thread = None

    def start_listening(self):
        """Start real-time voice processing"""
        self.listening = True

        # Start audio input thread
        self.audio_thread = threading.Thread(target=self.audio_input_loop, daemon=True)
        self.audio_thread.start()

        # Start processing thread
        self.processing_thread = threading.Thread(target=self.processing_loop, daemon=True)
        self.processing_thread.start()

    def stop_listening(self):
        """Stop real-time voice processing"""
        self.listening = False
        if self.audio_thread:
            self.audio_thread.join()
        if self.processing_thread:
            self.processing_thread.join()

    def audio_input_loop(self):
        """Continuously capture audio from microphone"""
        p = pyaudio.PyAudio()

        stream = p.open(
            format=self.format,
            channels=self.channels,
            rate=self.rate,
            input=True,
            frames_per_buffer=self.chunk
        )

        try:
            while self.listening:
                data = stream.read(self.chunk, exception_on_overflow=False)
                audio_data = np.frombuffer(data, dtype=np.int16)

                # Add to processing queue
                self.audio_queue.put(audio_data)

        except Exception as e:
            print(f"Audio input error: {e}")
        finally:
            stream.stop_stream()
            stream.close()
            p.terminate()

    def processing_loop(self):
        """Process audio chunks in real-time"""
        voice_active = False
        silence_count = 0
        min_voice_duration = 0.5  # seconds
        max_silence_duration = 1.0  # seconds

        while self.listening:
            try:
                # Get audio chunk
                audio_chunk = self.audio_queue.get(timeout=0.1)

                # Check for voice activity
                is_speech = self.is_voice_active(audio_chunk)

                if is_speech:
                    # Voice detected
                    self.voice_buffer = np.concatenate([self.voice_buffer, audio_chunk])
                    voice_active = True
                    silence_count = 0
                else:
                    # Silence detected
                    silence_count += len(audio_chunk) / self.rate  # Convert samples to seconds

                    if voice_active and silence_count >= max_silence_duration:
                        # Voice ended, process accumulated audio
                        if len(self.voice_buffer) >= self.rate * min_voice_duration:  # At least min duration
                            self.process_voice_segment(self.voice_buffer.copy())

                        # Reset for next segment
                        self.voice_buffer = np.array([], dtype=np.int16)
                        voice_active = False
                    elif voice_active:
                        # Add silence to buffer to maintain continuity
                        self.voice_buffer = np.concatenate([self.voice_buffer, audio_chunk])

            except queue.Empty:
                continue
            except Exception as e:
                print(f"Audio processing error: {e}")

    def is_voice_active(self, audio_chunk):
        """Check if voice is active in audio chunk using WebRTC VAD"""
        # WebRTC VAD expects 16-bit PCM audio
        # It works with 10, 20, or 30 ms chunks
        chunk_size_ms = (len(audio_chunk) / self.rate) * 1000

        if chunk_size_ms >= 30:  # Process in 30ms chunks
            for i in range(0, len(audio_chunk), int(0.03 * self.rate)):
                chunk_30ms = audio_chunk[i:i + int(0.03 * self.rate)]
                if len(chunk_30ms) == int(0.03 * self.rate):
                    # Convert to bytes for WebRTC VAD
                    chunk_bytes = chunk_30ms.tobytes()
                    try:
                        if self.vad.is_speech(chunk_bytes, self.rate):
                            return True
                    except:
                        continue
        else:
            # For smaller chunks, just check energy
            energy = np.mean(audio_chunk.astype(np.float32) ** 2)
            return energy > 1000  # Threshold for voice activity

        return False

    def process_voice_segment(self, audio_segment):
        """Process a complete voice segment"""
        try:
            # Convert to float32 for Whisper
            audio_float = audio_segment.astype(np.float32) / 32768.0

            # Transcribe using Whisper
            result = self.whisper_model.transcribe(
                audio_float,
                language="en",
                fp16=torch.cuda.is_available(),
                temperature=0.0
            )

            transcription = result["text"].strip()

            if transcription:
                print(f"Transcribed: {transcription}")

                # Process the command
                self.process_command(transcription)

        except Exception as e:
            print(f"Error processing voice segment: {e}")

    def process_command(self, command_text):
        """Process transcribed command"""
        # Use the command processor from earlier
        grammar = RobotCommandGrammar()
        parsed_command = grammar.parse_command(command_text)

        if parsed_command["command"] != "unknown":
            is_valid, msg = grammar.validate_command(parsed_command)
            if is_valid:
                # Execute command via robot controller
                self.execute_parsed_command(parsed_command)
            else:
                print(f"Invalid command: {msg}")
        else:
            print(f"Could not parse command: {command_text}")

    def execute_parsed_command(self, parsed_command):
        """Execute parsed command on robot"""
        # This would interface with the robot controller
        print(f"Executing: {parsed_command}")
```

## Integration with ROS 2

### ROS 2 Voice Command Node

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from humanoid_robot_msgs.msg import VoiceCommand, VoiceResponse
from builtin_interfaces.msg import Time

class VoiceCommandNode(Node):
    def __init__(self):
        super().__init__('voice_command_node')

        # Initialize Whisper and command processor
        self.whisper_model = whisper.load_model("base")
        self.voice_processor = RealTimeVoiceProcessor(self.whisper_model, None)

        # Publishers and subscribers
        self.voice_cmd_pub = self.create_publisher(VoiceCommand, 'voice_commands', 10)
        self.voice_resp_pub = self.create_publisher(VoiceResponse, 'voice_responses', 10)
        self.speech_pub = self.create_publisher(String, 'speech_commands', 10)

        # Command processing
        self.command_processor = VoiceCommandProcessor(None, self)  # Will be updated

        # Start voice processing
        self.voice_processor.start_listening()

        # Timer for processing commands
        self.process_timer = self.create_timer(0.1, self.process_voice_commands)

        self.get_logger().info('Voice Command Node initialized')

    def process_voice_commands(self):
        """Process voice commands in ROS context"""
        # This would integrate with the voice processing system
        pass

    def publish_voice_command(self, command_text):
        """Publish voice command as ROS message"""
        cmd_msg = VoiceCommand()
        cmd_msg.command_text = command_text
        cmd_msg.timestamp = self.get_clock().now().to_msg()
        cmd_msg.source = "voice_recognition"

        self.voice_cmd_pub.publish(cmd_msg)

    def publish_voice_response(self, response_text, command_id=""):
        """Publish voice response as ROS message"""
        resp_msg = VoiceResponse()
        resp_msg.response_text = response_text
        resp_msg.command_id = command_id
        resp_msg.timestamp = self.get_clock().now().to_msg()
        resp_msg.status = "success"  # or "error", "timeout", etc.

        self.voice_resp_pub.publish(resp_msg)

    def speak_text(self, text):
        """Publish text for robot to speak"""
        speech_msg = String()
        speech_msg.data = text
        self.speech_pub.publish(speech_msg)

    def move_forward(self, distance):
        """Move robot forward (placeholder - would use navigation stack)"""
        self.get_logger().info(f'Moving forward {distance} meters')
        # In real implementation, this would send navigation command
        return True

    def turn_left(self, angle):
        """Turn robot left (placeholder)"""
        self.get_logger().info(f'Turning left {angle} degrees')
        return True

    def turn_right(self, angle):
        """Turn robot right (placeholder)"""
        self.get_logger().info(f'Turning right {angle} degrees')
        return True

    def go_to_location(self, location):
        """Go to named location (placeholder)"""
        self.get_logger().info(f'Going to {location}')
        return True

    def pick_object(self, obj):
        """Pick object (placeholder)"""
        self.get_logger().info(f'Picking {obj}')
        return True

    def place_object(self, obj, target):
        """Place object (placeholder)"""
        self.get_logger().info(f'Placing {obj} at {target}')
        return True

    def speak(self, text):
        """Make robot speak"""
        self.get_logger().info(f'Speaking: {text}')
        self.speak_text(text)
        return True

def main(args=None):
    rclpy.init(args=args)

    voice_node = VoiceCommandNode()

    try:
        rclpy.spin(voice_node)
    except KeyboardInterrupt:
        pass
    finally:
        voice_node.voice_processor.stop_listening()
        voice_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Voice Command Grammar for Robotics

### Comprehensive Command Set

```python
class RoboticsVoiceGrammar:
    def __init__(self):
        # Navigation commands
        self.navigation_patterns = [
            # Go to locations
            {
                "pattern": r"(?:go to|move to|navigate to|walk to) (?:the )?(?P<location>\w+)(?: room| area)?",
                "action": "go_to_location",
                "params": ["location"]
            },
            {
                "pattern": r"(?:go|move|travel) (?P<direction>forward|backward|ahead|back) (?P<distance>\d+(?:\.\d+)?) meters?",
                "action": "move_linear",
                "params": ["direction", "distance"]
            },
            {
                "pattern": r"(?:turn|rotate|pivot) (?P<direction>left|right) (?P<angle>\d+(?:\.\d+)?) degrees?",
                "action": "turn",
                "params": ["direction", "angle"]
            },
            {
                "pattern": r"(?:move|go) (?P<direction>forward|backward|left|right) (?P<distance>\d+(?:\.\d+)?) meters?",
                "action": "move_direction",
                "params": ["direction", "distance"]
            }
        ]

        # Manipulation commands
        self.manipulation_patterns = [
            {
                "pattern": r"(?:pick up|grab|take|get) (?:the )?(?P<object>\w+)(?: from (?P<location>\w+))?",
                "action": "pick_object",
                "params": ["object", "location"]
            },
            {
                "pattern": r"(?:place|put|set) (?:the )?(?P<object>\w+) (?P<preposition>on|in|at|under) (?:the )?(?P<target>\w+)",
                "action": "place_object",
                "params": ["object", "preposition", "target"]
            },
            {
                "pattern": r"(?:grasp|hold|catch) (?:the )?(?P<object>\w+)",
                "action": "grasp_object",
                "params": ["object"]
            },
            {
                "pattern": r"(?:release|drop|let go of) (?:the )?(?P<object>\w+)",
                "action": "release_object",
                "params": ["object"]
            }
        ]

        # Interaction commands
        self.interaction_patterns = [
            {
                "pattern": r"(?:say|speak|tell me|repeat) ['\"](?P<text>[^'\"]+)['\"]",
                "action": "speak_text",
                "params": ["text"]
            },
            {
                "pattern": r"(?:listen|hear|pay attention) (?P<duration>\d+(?:\.\d+)?) seconds?",
                "action": "listen_for_duration",
                "params": ["duration"]
            },
            {
                "pattern": r"(?:follow me|come with me|accompany me)",
                "action": "follow_user",
                "params": []
            },
            {
                "pattern": r"(?:stop|halt|pause|freeze)",
                "action": "stop_robot",
                "params": []
            }
        ]

        # Inspection commands
        self.inspection_patterns = [
            {
                "pattern": r"(?:look at|examine|inspect|check) (?:the )?(?P<object>\w+)",
                "action": "look_at_object",
                "params": ["object"]
            },
            {
                "pattern": r"(?:find|locate|search for) (?:the )?(?P<object>\w+)",
                "action": "find_object",
                "params": ["object"]
            },
            {
                "pattern": r"(?:describe|tell me about) (?:the )?(?P<target>\w+)",
                "action": "describe_target",
                "params": ["target"]
            }
        ]

        # All patterns combined
        self.all_patterns = (
            self.navigation_patterns +
            self.manipulation_patterns +
            self.interaction_patterns +
            self.inspection_patterns
        )

    def parse_command(self, command_text):
        """Parse command text using all available patterns"""
        command_text_lower = command_text.lower().strip()

        for pattern_config in self.all_patterns:
            import re
            match = re.search(pattern_config["pattern"], command_text_lower)
            if match:
                params = match.groupdict()

                # Convert numeric parameters
                for param_name, value in params.items():
                    if value and value.replace('.', '').isdigit():
                        params[param_name] = float(value)

                return {
                    "action": pattern_config["action"],
                    "parameters": params,
                    "original_command": command_text,
                    "matched_pattern": pattern_config["pattern"]
                }

        # If no pattern matched, return unknown
        return {
            "action": "unknown",
            "parameters": {"text": command_text},
            "original_command": command_text,
            "matched_pattern": None
        }

    def validate_command(self, parsed_command):
        """Validate parsed command"""
        action = parsed_command["action"]
        params = parsed_command["parameters"]

        # Validate based on action type
        if action == "move_linear":
            distance = params.get("distance", 0)
            if distance <= 0 or distance > 10:  # Reasonable limits
                return False, f"Invalid distance: {distance}. Must be 0-10 meters."
        elif action == "turn":
            angle = params.get("angle", 0)
            if angle <= 0 or angle > 360:  # Reasonable limits
                return False, f"Invalid angle: {angle}. Must be 0-360 degrees."
        elif action == "speak_text":
            text = params.get("text", "")
            if not text.strip():
                return False, "No text to speak"
            if len(text) > 200:  # Reasonable limit
                return False, "Text too long for speech synthesis"

        return True, "Command is valid"

    def suggest_corrections(self, command_text):
        """Suggest possible corrections for unrecognized commands"""
        # This would implement fuzzy matching to suggest corrections
        suggestions = []

        # For now, return empty list
        # In practice, this would use string similarity algorithms
        return suggestions
```

## Error Handling and Robustness

### Voice Recognition Error Handling

```python
class RobustVoiceCommandProcessor:
    def __init__(self, whisper_model, robot_controller):
        self.whisper_model = whisper_model
        self.robot_controller = robot_controller
        self.grammar = RoboticsVoiceGrammar()

        # Error handling parameters
        self.confidence_threshold = 0.7  # Minimum confidence for command acceptance
        self.max_retry_attempts = 3
        self.confirmation_timeout = 5.0  # seconds to wait for confirmation

        # Error statistics
        self.error_stats = {
            "recognition_errors": 0,
            "parsing_errors": 0,
            "execution_errors": 0,
            "total_commands": 0
        }

    def process_command_with_error_handling(self, audio_data):
        """Process command with comprehensive error handling"""
        self.error_stats["total_commands"] += 1

        # Step 1: Transcribe audio
        try:
            transcription = self.transcribe_audio(audio_data)
        except Exception as e:
            self.error_stats["recognition_errors"] += 1
            self.handle_recognition_error(str(e))
            return None

        # Step 2: Parse command
        try:
            parsed_command = self.grammar.parse_command(transcription)
        except Exception as e:
            self.error_stats["parsing_errors"] += 1
            self.handle_parsing_error(transcription, str(e))
            return None

        # Step 3: Validate command
        is_valid, validation_msg = self.grammar.validate_command(parsed_command)
        if not is_valid:
            self.handle_validation_error(parsed_command, validation_msg)
            return None

        # Step 4: Execute command
        try:
            success = self.execute_command_with_retry(parsed_command)
            if not success:
                self.error_stats["execution_errors"] += 1
                self.handle_execution_error(parsed_command)
                return None
        except Exception as e:
            self.error_stats["execution_errors"] += 1
            self.handle_execution_error(parsed_command, str(e))
            return None

        return parsed_command

    def transcribe_audio(self, audio_data):
        """Transcribe audio with error handling"""
        try:
            # Normalize audio
            audio_float = audio_data.astype(np.float32) / np.max(np.abs(audio_data))

            # Transcribe with multiple attempts if needed
            for attempt in range(3):
                try:
                    result = self.whisper_model.transcribe(
                        audio_float,
                        language="en",
                        fp16=torch.cuda.is_available(),
                        temperature=0.0 if attempt == 0 else 0.2  # Increase randomness on retries
                    )
                    return result["text"].strip()
                except Exception as e:
                    if attempt == 2:  # Last attempt
                        raise e
                    time.sleep(0.1)  # Brief pause before retry
        except Exception as e:
            raise Exception(f"Audio transcription failed: {e}")

    def execute_command_with_retry(self, parsed_command):
        """Execute command with retry logic"""
        for attempt in range(self.max_retry_attempts):
            try:
                success = self.execute_single_command(parsed_command)
                if success:
                    return True
            except Exception as e:
                self.get_logger().warning(f"Command execution attempt {attempt + 1} failed: {e}")
                if attempt < self.max_retry_attempts - 1:
                    time.sleep(0.5)  # Wait before retry

        return False

    def execute_single_command(self, parsed_command):
        """Execute a single parsed command"""
        action = parsed_command["action"]
        params = parsed_command["parameters"]

        # Execute based on action type
        if action == "go_to_location":
            location = params.get("location", "unknown")
            return self.robot_controller.go_to_location(location)
        elif action == "move_linear":
            distance = params.get("distance", 1.0)
            direction = params.get("direction", "forward")
            if direction in ["forward", "ahead"]:
                return self.robot_controller.move_forward(distance)
            elif direction == "backward":
                return self.robot_controller.move_backward(distance)
            else:
                # For left/right movement, we'd need strafe capability
                return False
        elif action == "turn":
            angle = params.get("angle", 90.0)
            direction = params.get("direction", "left")
            if direction == "left":
                return self.robot_controller.turn_left(angle)
            else:
                return self.robot_controller.turn_right(angle)
        elif action == "pick_object":
            obj = params.get("object", "unknown")
            location = params.get("location", None)
            return self.robot_controller.pick_object(obj, location)
        elif action == "place_object":
            obj = params.get("object", "unknown")
            target = params.get("target", "unknown")
            return self.robot_controller.place_object(obj, target)
        elif action == "speak_text":
            text = params.get("text", "")
            return self.robot_controller.speak(text)
        else:
            # Unknown action - this shouldn't happen if validation passed
            return False

    def handle_recognition_error(self, error_msg):
        """Handle speech recognition errors"""
        print(f"Recognition error: {error_msg}")
        self.robot_controller.speak("Sorry, I couldn't understand that. Could you please repeat your command?")

    def handle_parsing_error(self, command_text, error_msg):
        """Handle command parsing errors"""
        print(f"Parsing error for command '{command_text}': {error_msg}")
        self.robot_controller.speak(f"Sorry, I didn't understand your command: {command_text}")

    def handle_validation_error(self, parsed_command, validation_msg):
        """Handle command validation errors"""
        print(f"Validation error: {validation_msg}")
        self.robot_controller.speak(f"Sorry, {validation_msg}")

    def handle_execution_error(self, parsed_command, error_msg=None):
        """Handle command execution errors"""
        print(f"Execution error for command {parsed_command}: {error_msg}")
        self.robot_controller.speak("Sorry, I couldn't execute that command. Please try again.")

    def get_error_statistics(self):
        """Get error statistics"""
        return self.error_stats.copy()
```

## Performance Optimization

### Efficient Processing Strategies

```python
import asyncio
import concurrent.futures
from threading import Lock

class OptimizedVoiceProcessor:
    def __init__(self, whisper_model, robot_controller):
        self.whisper_model = whisper_model
        self.robot_controller = robot_controller
        self.grammar = RoboticsVoiceGrammar()

        # Use thread pool for Whisper processing
        self.executor = concurrent.futures.ThreadPoolExecutor(max_workers=2)

        # Audio processing queue
        self.audio_queue = asyncio.Queue()
        self.command_queue = asyncio.Queue()

        # Processing locks
        self.transcription_lock = Lock()
        self.command_lock = Lock()

        # Performance metrics
        self.metrics = {
            "transcription_time_avg": 0.0,
            "processing_time_avg": 0.0,
            "throughput": 0.0
        }

    async def process_audio_stream(self, audio_stream):
        """Process continuous audio stream efficiently"""
        async for audio_chunk in audio_stream:
            await self.audio_queue.put(audio_chunk)

        # Process audio chunks
        while True:
            audio_chunk = await self.audio_queue.get()
            await self.process_audio_chunk(audio_chunk)

    async def process_audio_chunk(self, audio_chunk):
        """Process audio chunk with optimization"""
        start_time = time.time()

        # Submit transcription to thread pool (non-blocking)
        future = self.executor.submit(self.transcribe_audio_chunk, audio_chunk)

        # Do other processing while waiting
        transcription = await asyncio.wrap_future(future)

        if transcription.strip():
            # Process command
            await self.process_command_async(transcription)

        # Update metrics
        processing_time = time.time() - start_time
        self.update_metrics(processing_time)

    def transcribe_audio_chunk(self, audio_chunk):
        """Transcribe audio chunk (runs in thread pool)"""
        with self.transcription_lock:
            try:
                # Normalize audio
                audio_float = audio_chunk.astype(np.float32)
                audio_normalized = audio_float / np.max(np.abs(audio_float))

                # Transcribe
                result = self.whisper_model.transcribe(
                    audio_normalized,
                    language="en",
                    fp16=torch.cuda.is_available(),
                    temperature=0.0
                )

                return result["text"].strip()
            except Exception as e:
                print(f"Transcription error: {e}")
                return ""

    async def process_command_async(self, command_text):
        """Process command asynchronously"""
        try:
            # Parse command
            parsed_command = self.grammar.parse_command(command_text)

            if parsed_command["action"] != "unknown":
                # Validate command
                is_valid, validation_msg = self.grammar.validate_command(parsed_command)

                if is_valid:
                    # Execute command in executor to avoid blocking
                    future = self.executor.submit(
                        self.robot_controller.execute_command,
                        parsed_command
                    )
                    success = await asyncio.wrap_future(future)

                    if success:
                        await self.respond_to_user("Command executed successfully")
                    else:
                        await self.respond_to_user("Command execution failed")
                else:
                    await self.respond_to_user(f"Invalid command: {validation_msg}")
            else:
                await self.respond_to_user("I didn't understand that command")
        except Exception as e:
            await self.respond_to_user(f"Error processing command: {e}")

    async def respond_to_user(self, response_text):
        """Respond to user (async)"""
        # Schedule speech in executor to avoid blocking
        future = self.executor.submit(self.robot_controller.speak, response_text)
        await asyncio.wrap_future(future)

    def update_metrics(self, processing_time):
        """Update performance metrics"""
        # Simple moving average
        alpha = 0.1  # Smoothing factor
        self.metrics["processing_time_avg"] = (
            alpha * processing_time +
            (1 - alpha) * self.metrics["processing_time_avg"]
        )

    def get_performance_metrics(self):
        """Get current performance metrics"""
        return self.metrics.copy()
```

## Chapter Summary

Voice-to-action systems using OpenAI Whisper enable natural and intuitive human-robot interaction through speech commands. These systems bridge the gap between natural language understanding and robotic action execution, providing a flexible and accessible interface for controlling robots.

Key components of effective voice-to-action systems include:
- Real-time audio processing and voice activity detection
- Accurate speech recognition using models like Whisper
- Natural language understanding for command interpretation
- Robust command execution with safety validation
- Error handling and recovery mechanisms

The integration of voice recognition with robotic control systems enables robots to operate in human-friendly environments where natural language interaction is preferred over traditional interfaces. Proper implementation requires attention to real-time performance, accuracy, and safety considerations.

As we continue through this textbook, you'll learn how to integrate these voice command systems with cognitive planning capabilities that can translate natural language to complex robotic action sequences.

## Check Your Understanding

1. **Conceptual**: Explain the key differences between traditional button-based robot control and voice command systems. What advantages does voice control provide?

2. **Application**: Design a voice command system that can handle a command like "Robot, please go to the kitchen and bring me a red cup from the counter." What components would you need and how would they work together?

3. **Analysis**: What are the main challenges in implementing real-time voice recognition for robotics, and how might you address issues related to accuracy, latency, and environmental noise?

## Next Steps

In the next chapter, we'll explore cognitive planning using LLMs to translate natural language commands into sequences of ROS 2 actions, building upon the voice recognition foundation we've established here.

---

**Reflection Question**: Consider a scenario where a humanoid robot needs to operate in a noisy environment like a factory floor or busy restaurant. How would you modify the voice recognition system to maintain accuracy and reliability in such challenging acoustic conditions? What alternative interaction methods might supplement voice commands in these environments?