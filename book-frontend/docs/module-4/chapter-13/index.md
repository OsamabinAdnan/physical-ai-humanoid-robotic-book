---
sidebar_position: 17
title: Chapter 13 | Focus on The convergence of LLMs and Robotics
---

# Chapter 13: Focus: The Convergence of LLMs and Robotics

In this chapter, we'll explore the revolutionary convergence of Large Language Models (LLMs) with robotics, focusing on how modern AI systems are transforming the way robots understand, plan, and execute tasks. This convergence enables robots to interpret natural language commands, reason about complex tasks, and interact with humans in unprecedented ways.

## Learning Objectives

By the end of this chapter, you will be able to:
- Understand the fundamental concepts of Vision-Language-Action (VLA) in robotics
- Implement LLM-based task planning for robotic systems
- Integrate natural language understanding with robot control
- Design cognitive architectures that connect language to action
- Evaluate the capabilities and limitations of LLM-powered robotics

## Introduction to Vision-Language-Action (VLA) in Robotics

The convergence of vision, language, and action in robotics represents a paradigm shift from traditional programmed behavior to AI-powered understanding and execution. This integration enables robots to process natural language commands, perceive their environment visually, and execute complex tasks with minimal human intervention.

### The VLA Framework

Vision-Language-Action (VLA) systems combine three essential components:

#### Vision (Perception)
- Environmental understanding through cameras and sensors
- Object detection and recognition
- Scene analysis and spatial reasoning
- Multi-modal sensory input processing

#### Language (Understanding)
- Natural language processing and comprehension
- Command interpretation and intent recognition
- Contextual reasoning and dialogue management
- Task decomposition and planning

#### Action (Execution)
- Motion planning and control
- Manipulation and navigation
- Safety validation and execution
- Feedback and adaptation mechanisms

### Historical Context

#### Traditional Robotics Approach
- Hard-coded behaviors and state machines
- Limited natural interaction capabilities
- Extensive programming for each new task
- Rigid, inflexible operation patterns

#### Modern AI-Powered Approach
- Natural language command interpretation
- Generalization to novel tasks and environments
- Learning from demonstration and interaction
- Flexible, adaptive behavior patterns

## Large Language Models in Robotics

### LLM Capabilities for Robotics

#### Natural Language Understanding
- Command interpretation
- Question answering
- Context awareness
- Dialogue management

#### Reasoning and Planning
- Task decomposition
- Logical inference
- Commonsense reasoning
- Problem-solving strategies

#### Knowledge Integration
- World knowledge incorporation
- Physical reasoning
- Safety considerations
- Cultural and social norms

### Popular LLM Architectures for Robotics

#### Transformer-Based Models
- GPT series for generative tasks
- BERT-based models for understanding
- T5 for text-to-text transformations
- Vision-Language Transformers (ViLT) for multi-modal tasks

#### Specialized Robotics Models
- PaLM-E: Scaling embodied reasoning with vision-language models
- RT-2: Converting natural language to robot actions
- VIMA: Generalist robot learning with multimodal affordances
- SayCan: Do as I can, not as I say approach

### Example: Integrating LLM with Robot Control

```python
import openai
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Pose
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import json

class LLMRobotController:
    def __init__(self, api_key):
        # Initialize LLM client
        openai.api_key = api_key

        # Initialize ROS components
        self.bridge = CvBridge()
        self.cmd_pub = rospy.Publisher('/robot/command', String, queue_size=10)

        # Subscribe to camera feed for visual input
        self.image_sub = rospy.Subscriber('/camera/rgb/image_raw', Image, self.image_callback)

        # Store latest image for VLA processing
        self.latest_image = None

        # Robot capabilities description
        self.robot_capabilities = {
            "navigation": {
                "move_forward": "Move robot forward by specified distance",
                "turn_left": "Turn robot left by specified angle",
                "turn_right": "Turn robot right by specified angle",
                "go_to_location": "Navigate to specified location"
            },
            "manipulation": {
                "pick_object": "Pick up object at specified location",
                "place_object": "Place object at specified location",
                "grasp": "Grasp object with specified parameters"
            },
            "interaction": {
                "speak": "Speak specified text",
                "listen": "Listen for user input",
                "show_emotion": "Display specified emotion"
            }
        }

    def image_callback(self, msg):
        """Store latest image for processing"""
        self.latest_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")

    def process_command(self, command_text):
        """
        Process natural language command and convert to robot actions
        """
        # Prepare context for LLM
        context = self.build_context(command_text)

        # Generate robot action sequence using LLM
        action_sequence = self.generate_action_sequence(context)

        # Execute the action sequence
        self.execute_action_sequence(action_sequence)

        return action_sequence

    def build_context(self, command):
        """
        Build context for LLM including command, robot state, and visual input
        """
        context_prompt = f"""
        You are controlling a humanoid robot. The user has given the following command:

        Command: "{command}"

        The robot has the following capabilities:
        {json.dumps(self.robot_capabilities, indent=2)}

        The current environment is captured in the attached image. Based on the command, environment, and robot capabilities, generate a sequence of actions for the robot to execute.

        Respond with a JSON list of actions in the following format:
        [
            {{
                "action": "action_name",
                "parameters": {{
                    "param1": "value1",
                    "param2": "value2"
                }},
                "description": "Brief description of what this action does"
            }}
        ]

        Keep the actions simple and executable. Each action should correspond to a capability in the robot's capabilities list.
        """

        return context_prompt

    def generate_action_sequence(self, context):
        """
        Use LLM to generate action sequence from context
        """
        try:
            response = openai.ChatCompletion.create(
                model="gpt-4-vision-preview",  # Using vision-capable model
                messages=[
                    {"role": "system", "content": "You are a helpful assistant that converts natural language commands into robot action sequences."},
                    {"role": "user", "content": [
                        {"type": "text", "text": context}
                    ]}
                ],
                temperature=0.1,  # Low temperature for more deterministic output
                max_tokens=1000
            )

            # Extract and parse the action sequence
            response_text = response.choices[0].message.content

            # Clean up the response to extract JSON
            json_start = response_text.find('[')
            json_end = response_text.rfind(']') + 1

            if json_start != -1 and json_end != 0:
                json_str = response_text[json_start:json_end]
                action_sequence = json.loads(json_str)

                return action_sequence
            else:
                # If JSON parsing fails, return a simple default action
                return [{"action": "speak", "parameters": {"text": "I don't understand the command"}, "description": "Report inability to understand command"}]

        except Exception as e:
            rospy.logerr(f"Error generating action sequence: {e}")
            return [{"action": "speak", "parameters": {"text": "Error processing command"}, "description": "Report processing error"}]

    def execute_action_sequence(self, action_sequence):
        """
        Execute the sequence of actions on the robot
        """
        for action in action_sequence:
            self.execute_single_action(action)

    def execute_single_action(self, action):
        """
        Execute a single action on the robot
        """
        action_name = action["action"]
        parameters = action.get("parameters", {})

        rospy.loginfo(f"Executing action: {action_name} with parameters: {parameters}")

        if action_name == "move_forward":
            self.move_forward(parameters.get("distance", 1.0))
        elif action_name == "turn_left":
            self.turn_left(parameters.get("angle", 90.0))
        elif action_name == "turn_right":
            self.turn_right(parameters.get("angle", 90.0))
        elif action_name == "go_to_location":
            self.go_to_location(parameters.get("location", "unknown"))
        elif action_name == "pick_object":
            self.pick_object(parameters.get("object", "unknown"),
                           parameters.get("location", None))
        elif action_name == "place_object":
            self.place_object(parameters.get("location", "unknown"))
        elif action_name == "speak":
            self.speak(parameters.get("text", ""))
        else:
            rospy.logwarn(f"Unknown action: {action_name}")

    def move_forward(self, distance):
        """Move robot forward by specified distance"""
        cmd = {
            "type": "motion",
            "command": "move_forward",
            "distance": distance
        }
        self.cmd_pub.publish(json.dumps(cmd))

    def turn_left(self, angle):
        """Turn robot left by specified angle"""
        cmd = {
            "type": "motion",
            "command": "turn",
            "direction": "left",
            "angle": angle
        }
        self.cmd_pub.publish(json.dumps(cmd))

    def turn_right(self, angle):
        """Turn robot right by specified angle"""
        cmd = {
            "type": "motion",
            "command": "turn",
            "direction": "right",
            "angle": angle
        }
        self.cmd_pub.publish(json.dumps(cmd))

    def go_to_location(self, location):
        """Navigate to specified location"""
        cmd = {
            "type": "navigation",
            "command": "go_to",
            "location": location
        }
        self.cmd_pub.publish(json.dumps(cmd))

    def pick_object(self, obj, location=None):
        """Pick up specified object"""
        cmd = {
            "type": "manipulation",
            "command": "pick",
            "object": obj,
            "location": location
        }
        self.cmd_pub.publish(json.dumps(cmd))

    def place_object(self, location):
        """Place held object at location"""
        cmd = {
            "type": "manipulation",
            "command": "place",
            "location": location
        }
        self.cmd_pub.publish(json.dumps(cmd))

    def speak(self, text):
        """Make robot speak text"""
        cmd = {
            "type": "interaction",
            "command": "speak",
            "text": text
        }
        self.cmd_pub.publish(json.dumps(cmd))

# Example usage
if __name__ == "__main__":
    rospy.init_node('llm_robot_controller')

    # Initialize controller with API key
    controller = LLMRobotController(api_key="YOUR_API_KEY_HERE")

    # Example command
    command = "Please go to the kitchen and bring me a red apple from the counter"
    actions = controller.process_command(command)

    rospy.spin()
```

## Cognitive Architectures for VLA

### Hierarchical Task Planning

#### High-Level Planning
- Natural language command interpretation
- Task decomposition into sub-tasks
- Resource allocation and scheduling
- Long-term goal management

#### Mid-Level Planning
- Action sequencing and coordination
- Constraint satisfaction
- Failure recovery planning
- Multi-modal integration

#### Low-Level Execution
- Motor control and motion planning
- Real-time feedback processing
- Safety validation
- Execution monitoring

### Memory and Context Management

#### Working Memory
- Short-term context for current tasks
- Object and location tracking
- Interaction history
- Temporal relationships

#### Long-Term Memory
- Learned skills and procedures
- Environmental knowledge
- User preferences and habits
- General world knowledge

#### Episodic Memory
- Past interaction experiences
- Successful/unsuccessful strategies
- Context-specific learning
- Adaptation and improvement

### Example: Hierarchical VLA Architecture

```python
class HierarchicalVLAArchitecture:
    def __init__(self):
        # Initialize LLM for high-level planning
        self.llm_planner = LLMPlanner()

        # Initialize perception system
        self.perception_system = PerceptionSystem()

        # Initialize action executor
        self.action_executor = ActionExecutor()

        # Initialize memory systems
        self.memory_system = MemorySystem()

        # Initialize safety monitor
        self.safety_monitor = SafetyMonitor()

    def process_command(self, command, visual_input=None):
        """
        Process command through hierarchical architecture
        """
        # High-level planning
        high_level_plan = self.llm_planner.create_plan(command)

        # Update context with visual information
        context = self.perception_system.process_visual_input(visual_input)
        context.update({"command": command})

        # Mid-level planning with context
        mid_level_plan = self.refine_plan(high_level_plan, context)

        # Execute plan with safety monitoring
        success = self.execute_plan_with_monitoring(mid_level_plan, context)

        # Update memory with experience
        self.memory_system.store_experience(command, mid_level_plan, success)

        return success

    def refine_plan(self, high_level_plan, context):
        """
        Refine high-level plan based on current context
        """
        refined_plan = []

        for high_level_action in high_level_plan:
            # Get detailed action sequence for each high-level action
            detailed_actions = self.get_detailed_actions(high_level_action, context)
            refined_plan.extend(detailed_actions)

        return refined_plan

    def get_detailed_actions(self, high_level_action, context):
        """
        Convert high-level action to detailed executable actions
        """
        # This would use the perception system to get specific details
        # like exact locations, object properties, etc.
        return [high_level_action]  # Simplified for example

    def execute_plan_with_monitoring(self, plan, context):
        """
        Execute plan while monitoring safety and progress
        """
        for action in plan:
            # Check safety constraints
            if not self.safety_monitor.check_action_feasible(action, context):
                rospy.logerr(f"Safety check failed for action: {action}")
                return False

            # Execute action
            success = self.action_executor.execute(action)

            if not success:
                rospy.logwarn(f"Action failed: {action}")
                # Try recovery or continue based on strategy
                continue

            # Update context with action outcome
            context = self.update_context_after_action(action, context)

        return True

    def update_context_after_action(self, action, context):
        """
        Update context based on action execution
        """
        # Update object locations, robot state, etc.
        return context
```

## Vision-Language Integration

### Multi-Modal Understanding

#### Visual Question Answering
- Answering questions about visual scenes
- Object attribute recognition
- Spatial relationship understanding
- Activity recognition

#### Grounded Language Understanding
- Connecting language concepts to visual entities
- Referring expression comprehension
- Spatial language understanding
- Action localization

### Example: Vision-Language Model Integration

```python
import torch
import clip
from PIL import Image
import numpy as np

class VisionLanguageIntegrator:
    def __init__(self):
        # Load pre-trained CLIP model
        self.clip_model, self.clip_preprocess = clip.load("ViT-B/32")
        self.clip_model.eval()

        # Device configuration
        self.device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
        self.clip_model = self.clip_model.to(self.device)

    def analyze_scene(self, image_path_or_array):
        """
        Analyze scene using vision-language model
        """
        if isinstance(image_path_or_array, str):
            image = self.clip_preprocess(Image.open(image_path_or_array)).unsqueeze(0)
        else:
            # Convert numpy array to PIL Image then preprocess
            image = Image.fromarray(image_path_or_array)
            image = self.clip_preprocess(image).unsqueeze(0)

        image = image.to(self.device)

        return image

    def find_best_match(self, image, text_options):
        """
        Find best matching text option for given image
        """
        image_features = self.clip_model.encode_image(image)

        # Encode text options
        text_inputs = torch.cat([clip.tokenize(text) for text in text_options]).to(self.device)
        text_features = self.clip_model.encode_text(text_inputs)

        # Normalize features
        image_features /= image_features.norm(dim=-1, keepdim=True)
        text_features /= text_features.norm(dim=-1, keepdim=True)

        # Calculate similarity
        similarity = (100.0 * image_features @ text_features.T).softmax(dim=-1)
        probabilities = similarity[0].cpu().numpy()

        best_match_idx = np.argmax(probabilities)
        best_match = text_options[best_match_idx]
        confidence = probabilities[best_match_idx]

        return best_match, confidence, dict(zip(text_options, probabilities))

    def describe_scene(self, image_path):
        """
        Generate natural language description of scene
        """
        # This would typically use a specialized image captioning model
        # For now, we'll use CLIP for classification-style tasks
        common_objects = [
            "a person", "a robot", "a table", "a chair", "a cup",
            "a book", "a plant", "a door", "a window", "a couch"
        ]

        image = self.analyze_scene(image_path)
        best_object, confidence, all_scores = self.find_best_match(image, common_objects)

        description = f"The scene contains {best_object} with {confidence*100:.1f}% confidence."
        return description, all_scores
```

## Challenges and Limitations

### Current Limitations

#### Hallucination Issues
- LLMs may generate plausible-sounding but incorrect information
- Actions that seem reasonable but are physically impossible
- Misinterpretation of visual scenes
- Confabulation of details

#### Real-Time Performance
- LLM inference latency
- Visual processing delays
- Action execution timing
- Synchronization challenges

#### Safety and Reliability
- Ensuring safe action execution
- Handling ambiguous commands
- Validation of LLM outputs
- Fallback mechanisms

### Mitigation Strategies

#### Verification and Validation
- Multiple validation layers
- Safety constraints enforcement
- Human oversight capabilities
- Uncertainty quantification

#### Performance Optimization
- Model quantization and pruning
- Edge computing deployment
- Caching and pre-computation
- Asynchronous processing

## Integration with Robotic Platforms

### ROS/ROS2 Integration

#### Action Servers for LLM Commands
```python
import actionlib
from actionlib_msgs.msg import GoalStatus
from humanoid_robot_msgs.msg import ExecuteCommandAction, ExecuteCommandResult

class LLMCommandServer:
    def __init__(self, name):
        self._action_name = name
        self._server = actionlib.SimpleActionServer(
            self._action_name,
            ExecuteCommandAction,
            execute_cb=self.execute_command,
            auto_start=False
        )
        self._server.start()

        # Initialize LLM controller
        self.llm_controller = LLMRobotController(api_key="YOUR_API_KEY")

    def execute_command(self, goal):
        """Execute natural language command"""
        result = ExecuteCommandResult()

        try:
            # Process the command
            action_sequence = self.llm_controller.process_command(goal.command)

            # Execute the sequence
            success = self.llm_controller.execute_action_sequence(action_sequence)

            if success:
                result.success = True
                result.message = "Command executed successfully"
                self._server.set_succeeded(result)
            else:
                result.success = False
                result.message = "Command execution failed"
                self._server.set_aborted(result)

        except Exception as e:
            result.success = False
            result.message = f"Error processing command: {str(e)}"
            self._server.set_aborted(result)
```

### Hardware Considerations

#### Compute Requirements
- GPU acceleration for LLM inference
- Real-time processing capabilities
- Memory requirements for context
- Power consumption for mobile robots

#### Sensor Integration
- Camera systems for visual input
- Microphone arrays for speech
- IMU and other sensors for context
- Actuator feedback systems

## Evaluation Metrics

### Performance Metrics

#### Task Completion Rate
- Percentage of tasks successfully completed
- Time to completion
- Efficiency of execution
- Resource utilization

#### Language Understanding Accuracy
- Command interpretation accuracy
- Context awareness
- Error recovery capability
- Naturalness of interaction

#### Safety Metrics
- Safety violation incidents
- Emergency stop activations
- Collision avoidance success
- Human intervention requirements

### Benchmarking

#### Standard Benchmarks
- ALFRED benchmark for household tasks
- RoboTurk for manipulation tasks
- SayCan evaluation framework
- VIMA benchmark suite

#### Custom Evaluation
- Task-specific metrics
- Domain-specific benchmarks
- User satisfaction measures
- Long-term deployment metrics

## Future Directions

### Emerging Technologies

#### Foundation Models
- Large multimodal models
- Embodied intelligence models
- Continual learning systems
- Transfer learning capabilities

#### Advanced Integration
- Neuromorphic computing
- Spiking neural networks
- Quantum-enhanced processing
- Edge-cloud collaboration

### Research Challenges

#### Common Sense Reasoning
- Physical commonsense
- Social commonsense
- Temporal reasoning
- Causal reasoning

#### Human-Robot Collaboration
- Shared autonomy
- Trust and transparency
- Social intelligence
- Cultural adaptation

## Chapter Summary

The convergence of Large Language Models with robotics represents a transformative shift in how robots understand and interact with the world. Vision-Language-Action systems enable robots to process natural language commands, perceive their environment visually, and execute complex tasks with unprecedented flexibility and intelligence.

The integration of LLMs with robotic systems provides several key advantages:
- Natural human-robot interaction through language
- Generalization to novel tasks and environments
- Flexible, adaptive behavior patterns
- Rich contextual understanding

However, this integration also presents significant challenges including hallucination issues, real-time performance requirements, and safety considerations that must be carefully addressed.

As we continue through this textbook, you'll learn how to implement these VLA systems with practical examples, explore voice-to-action systems, and develop cognitive planning capabilities that connect natural language to robotic action.

## Check Your Understanding

1. **Conceptual**: Explain the key differences between traditional programmed robotics and LLM-powered Vision-Language-Action systems.

2. **Application**: Design a VLA system that can interpret a command like "Bring me the red cup from the table near the window" and execute it. What components would you need and how would they interact?

3. **Analysis**: What are the main challenges in deploying LLM-powered robotics systems in real-world environments, and how might you address them?

## Code Example: Vision-Language-Action (VLA) System Implementation

Here's a complete implementation of a Vision-Language-Action system that integrates natural language understanding, visual perception, and robotic action execution:

```python
import os
import json
import numpy as np
import torch
import openai
import rospy
import cv2
from cv_bridge import CvBridge
from std_msgs.msg import String
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped
from humanoid_robot_msgs.msg import ExecuteCommandAction, ExecuteCommandResult
import actionlib
from PIL import Image as PILImage
import clip
import time

class VLASystem:
    """
    A Vision-Language-Action (VLA) system that integrates:
    - Vision (perception of environment)
    - Language (natural language understanding)
    - Action (robotic control and execution)
    """

    def __init__(self, api_key=None, model_name="gpt-4"):
        # Initialize OpenAI API if key provided
        if api_key:
            openai.api_key = api_key
        self.model_name = model_name

        # Initialize ROS components
        self.bridge = CvBridge()

        # Initialize CLIP for vision-language integration
        self.clip_model, self.clip_preprocess = clip.load("ViT-B/32")
        self.clip_model.eval()
        self.device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
        self.clip_model = self.clip_model.to(self.device)

        # Robot capabilities - define what actions the robot can perform
        self.robot_capabilities = {
            "navigation": {
                "move_forward": {"params": ["distance"], "description": "Move robot forward by specified distance (meters)"},
                "turn_left": {"params": ["angle"], "description": "Turn robot left by specified angle (degrees)"},
                "turn_right": {"params": ["angle"], "description": "Turn robot right by specified angle (degrees)"},
                "go_to_location": {"params": ["location"], "description": "Navigate to specified location"},
                "explore_area": {"params": ["area"], "description": "Explore specified area"}
            },
            "manipulation": {
                "pick_object": {"params": ["object", "location"], "description": "Pick up object at specified location"},
                "place_object": {"params": ["location"], "description": "Place held object at specified location"},
                "grasp": {"params": ["object"], "description": "Grasp specified object"},
                "release": {"params": ["location"], "description": "Release object at location"},
                "transport_object": {"params": ["object", "destination"], "description": "Transport object to destination"}
            },
            "interaction": {
                "speak": {"params": ["text"], "description": "Speak specified text"},
                "listen": {"params": [], "description": "Listen for user input"},
                "show_emotion": {"params": ["emotion"], "description": "Display specified emotion"},
                "wait_for_user": {"params": ["duration"], "description": "Wait for user for specified duration"}
            },
            "perception": {
                "detect_objects": {"params": [], "description": "Detect and identify objects in current view"},
                "analyze_scene": {"params": [], "description": "Analyze the current scene"},
                "find_person": {"params": [], "description": "Locate a person in the environment"}
            }
        }

        # Store latest image for processing
        self.latest_image = None

        # Subscribe to camera feed
        self.image_sub = rospy.Subscriber('/camera/rgb/image_raw', Image, self.image_callback)

        # Publisher for robot commands
        self.cmd_pub = rospy.Publisher('/robot/command', String, queue_size=10)

        # Safety limits
        self.safety_limits = {
            "max_distance": 5.0,  # meters
            "max_speed": 1.0,     # m/s
            "max_turn_angle": 180 # degrees
        }

    def image_callback(self, msg):
        """Store latest image for processing"""
        try:
            self.latest_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            rospy.logerr(f"Error processing image: {e}")

    def process_natural_language_command(self, command_text):
        """
        Process a natural language command and convert it to a sequence of actions.

        Args:
            command_text (str): Natural language command from user

        Returns:
            list: Sequence of actions to execute
        """
        # Build context for LLM
        context = self._build_context(command_text)

        # Generate action sequence using LLM
        action_sequence = self._generate_action_sequence_with_llm(context)

        # Validate and refine the action sequence
        validated_sequence = self._validate_and_refine_actions(action_sequence)

        return validated_sequence

    def _build_context(self, command):
        """Build context for LLM including command, robot capabilities, and visual input."""
        context = {
            "command": command,
            "robot_capabilities": self.robot_capabilities,
            "environment_description": self._describe_current_environment() if self.latest_image is not None else "No visual input available",
            "safety_constraints": self.safety_limits
        }

        return json.dumps(context, indent=2)

    def _describe_current_environment(self):
        """Describe the current environment using vision-language model."""
        if self.latest_image is None:
            return "No image available"

        try:
            # Convert OpenCV image to PIL
            pil_image = PILImage.fromarray(cv2.cvtColor(self.latest_image, cv2.COLOR_BGR2RGB))

            # Preprocess for CLIP
            image_input = self.clip_preprocess(pil_image).unsqueeze(0).to(self.device)

            # Common objects to check for in the scene
            common_objects = [
                "person", "chair", "table", "couch", "cup", "bottle",
                "plant", "door", "window", "book", "computer", "phone",
                "kitchen", "living room", "bedroom", "office"
            ]

            # Tokenize text options
            text_inputs = torch.cat([clip.tokenize(f"a photo of {obj}") for obj in common_objects]).to(self.device)

            # Get similarities
            with torch.no_grad():
                image_features = self.clip_model.encode_image(image_input)
                text_features = self.clip_model.encode_text(text_inputs)

                # Normalize
                image_features /= image_features.norm(dim=-1, keepdim=True)
                text_features /= text_features.norm(dim=-1, keepdim=True)

                # Calculate similarity
                similarity = (100.0 * image_features @ text_features.T).softmax(dim=-1)
                probabilities = similarity[0].cpu().numpy()

            # Get top matches
            top_indices = np.argsort(probabilities)[-5:][::-1]  # Top 5 matches
            top_objects = [(common_objects[i], probabilities[i]) for i in top_indices if probabilities[i] > 0.01]

            description = f"Environment contains: {', '.join([f'{obj} ({prob*100:.1f}%)' for obj, prob in top_objects[:3]])}"
            return description

        except Exception as e:
            rospy.logerr(f"Error describing environment: {e}")
            return "Unable to analyze environment due to processing error"

    def _generate_action_sequence_with_llm(self, context):
        """Generate action sequence using LLM."""
        try:
            prompt = f"""
            You are controlling a humanoid robot with the following capabilities:

            {context}

            Convert the user's command into a sequence of executable actions for the robot.

            Respond with a JSON list of actions in the following format:
            [
                {{
                    "action": "action_name",
                    "parameters": {{
                        "param1": "value1",
                        "param2": "value2"
                    }},
                    "description": "Brief description of what this action does"
                }}
            ]

            Make sure each action corresponds to a capability in the robot's capabilities list.
            Be specific with parameters and ensure the actions are logically sequenced.
            """

            response = openai.ChatCompletion.create(
                model=self.model_name,
                messages=[
                    {"role": "system", "content": "You are a helpful assistant that converts natural language commands into robot action sequences. Respond only with valid JSON."},
                    {"role": "user", "content": prompt}
                ],
                temperature=0.1,
                max_tokens=1000
            )

            response_text = response.choices[0].message.content

            # Extract JSON from response
            start_idx = response_text.find('[')
            end_idx = response_text.rfind(']') + 1

            if start_idx != -1 and end_idx != 0:
                json_str = response_text[start_idx:end_idx]
                action_sequence = json.loads(json_str)
                return action_sequence
            else:
                # If JSON parsing fails, return a simple error action
                return [{
                    "action": "speak",
                    "parameters": {"text": f"I'm sorry, I couldn't understand the command: {context[:100]}..."},
                    "description": "Report inability to understand command"
                }]

        except Exception as e:
            rospy.logerr(f"Error generating action sequence: {e}")
            return [{
                "action": "speak",
                "parameters": {"text": "Sorry, I'm having trouble processing that command right now."},
                "description": "Report processing error"
            }]

    def _validate_and_refine_actions(self, action_sequence):
        """Validate and refine the action sequence for safety and feasibility."""
        refined_sequence = []

        for action in action_sequence:
            # Validate action name exists in capabilities
            action_valid = False
            for category, actions in self.robot_capabilities.items():
                if action["action"] in actions:
                    action_valid = True
                    break

            if not action_valid:
                rospy.logwarn(f"Invalid action '{action['action']}' skipped")
                continue

            # Validate parameters
            validated_action = self._validate_action_parameters(action)
            if validated_action:
                refined_sequence.append(validated_action)

        return refined_sequence

    def _validate_action_parameters(self, action):
        """Validate action parameters for safety and feasibility."""
        action_name = action["action"]
        params = action.get("parameters", {})

        # Validate navigation actions
        if action_name in ["move_forward"]:
            if "distance" in params:
                distance = float(params["distance"])
                if distance > self.safety_limits["max_distance"]:
                    rospy.logwarn(f"Distance {distance} exceeds maximum of {self.safety_limits['max_distance']}")
                    params["distance"] = min(distance, self.safety_limits["max_distance"])

        elif action_name in ["turn_left", "turn_right"]:
            if "angle" in params:
                angle = float(params["angle"])
                if abs(angle) > self.safety_limits["max_turn_angle"]:
                    rospy.logwarn(f"Turn angle {angle} exceeds maximum of {self.safety_limits['max_turn_angle']}")
                    params["angle"] = min(abs(angle), self.safety_limits["max_turn_angle"]) * (1 if angle >= 0 else -1)

        # Update action with validated parameters
        action["parameters"] = params
        return action

    def execute_action_sequence(self, action_sequence):
        """
        Execute a sequence of actions on the robot.

        Args:
            action_sequence (list): List of actions to execute

        Returns:
            bool: True if all actions completed successfully, False otherwise
        """
        for i, action in enumerate(action_sequence):
            rospy.loginfo(f"Executing action {i+1}/{len(action_sequence)}: {action['action']}")

            success = self._execute_single_action(action)
            if not success:
                rospy.logerr(f"Action failed: {action}")
                return False

            # Small delay between actions
            time.sleep(0.1)

        return True

    def _execute_single_action(self, action):
        """
        Execute a single action on the robot.

        Args:
            action (dict): Action dictionary with action name and parameters

        Returns:
            bool: True if action executed successfully, False otherwise
        """
        action_name = action["action"]
        parameters = action.get("parameters", {})

        try:
            if action_name == "move_forward":
                return self._move_forward(parameters.get("distance", 1.0))
            elif action_name == "turn_left":
                return self._turn_left(parameters.get("angle", 90.0))
            elif action_name == "turn_right":
                return self._turn_right(parameters.get("angle", 90.0))
            elif action_name == "go_to_location":
                return self._go_to_location(parameters.get("location", "unknown"))
            elif action_name == "pick_object":
                return self._pick_object(
                    parameters.get("object", "unknown"),
                    parameters.get("location", None)
                )
            elif action_name == "place_object":
                return self._place_object(parameters.get("location", "unknown"))
            elif action_name == "speak":
                return self._speak(parameters.get("text", ""))
            elif action_name == "detect_objects":
                return self._detect_objects()
            elif action_name == "analyze_scene":
                return self._analyze_scene()
            else:
                rospy.logwarn(f"Unknown action: {action_name}")
                return False

        except Exception as e:
            rospy.logerr(f"Error executing action {action_name}: {e}")
            return False

    def _move_forward(self, distance):
        """Move robot forward by specified distance."""
        cmd = {
            "type": "motion",
            "command": "move_forward",
            "distance": distance,
            "speed": min(distance, self.safety_limits["max_speed"])  # Adjust speed based on distance
        }
        self.cmd_pub.publish(json.dumps(cmd))
        rospy.loginfo(f"Moving forward {distance} meters")
        time.sleep(max(1.0, distance))  # Approximate sleep time
        return True

    def _turn_left(self, angle):
        """Turn robot left by specified angle."""
        cmd = {
            "type": "motion",
            "command": "turn",
            "direction": "left",
            "angle": angle
        }
        self.cmd_pub.publish(json.dumps(cmd))
        rospy.loginfo(f"Turning left {angle} degrees")
        time.sleep(max(0.5, abs(angle)/180.0))  # Approximate sleep time
        return True

    def _turn_right(self, angle):
        """Turn robot right by specified angle."""
        cmd = {
            "type": "motion",
            "command": "turn",
            "direction": "right",
            "angle": angle
        }
        self.cmd_pub.publish(json.dumps(cmd))
        rospy.loginfo(f"Turning right {angle} degrees")
        time.sleep(max(0.5, abs(angle)/180.0))  # Approximate sleep time
        return True

    def _go_to_location(self, location):
        """Navigate to specified location."""
        cmd = {
            "type": "navigation",
            "command": "go_to",
            "location": location
        }
        self.cmd_pub.publish(json.dumps(cmd))
        rospy.loginfo(f"Navigating to {location}")
        time.sleep(5.0)  # Approximate navigation time
        return True

    def _pick_object(self, obj, location=None):
        """Pick up specified object."""
        cmd = {
            "type": "manipulation",
            "command": "pick",
            "object": obj,
            "location": location
        }
        self.cmd_pub.publish(json.dumps(cmd))
        rospy.loginfo(f"Picking up {obj}")
        time.sleep(3.0)  # Approximate manipulation time
        return True

    def _place_object(self, location):
        """Place held object at location."""
        cmd = {
            "type": "manipulation",
            "command": "place",
            "location": location
        }
        self.cmd_pub.publish(json.dumps(cmd))
        rospy.loginfo(f"Placing object at {location}")
        time.sleep(3.0)  # Approximate manipulation time
        return True

    def _speak(self, text):
        """Make robot speak text."""
        cmd = {
            "type": "interaction",
            "command": "speak",
            "text": text
        }
        self.cmd_pub.publish(json.dumps(cmd))
        rospy.loginfo(f"Speaking: {text}")
        time.sleep(len(text.split()) * 0.3)  # Approximate speaking time
        return True

    def _detect_objects(self):
        """Detect and identify objects in current view."""
        if self.latest_image is not None:
            description = self._describe_current_environment()
            rospy.loginfo(f"Detected objects: {description}")
            self._speak(f"I see {description.split('contains:')[1] if 'contains:' in description else 'objects in the environment'}")
        else:
            rospy.logwarn("No image available for object detection")
            self._speak("I don't have visual input right now.")
        return True

    def _analyze_scene(self):
        """Analyze the current scene."""
        if self.latest_image is not None:
            description = self._describe_current_environment()
            rospy.loginfo(f"Scene analysis: {description}")
            self._speak(f"The environment currently {description.lower()}")
        else:
            rospy.logwarn("No image available for scene analysis")
            self._speak("I can't see the environment right now.")
        return True


def main():
    """
    Example usage of the VLA system.
    """
    rospy.init_node('vla_system_example')

    # Initialize VLA system (use your OpenAI API key)
    vla_system = VLASystem(api_key=os.getenv("OPENAI_API_KEY"))  # Set your API key as environment variable

    # Example commands to demonstrate the system
    example_commands = [
        "Please go to the kitchen and bring me a bottle of water",
        "Turn around and tell me what you see",
        "Move forward two meters then turn left",
        "Find a person and wave at them"
    ]

    for command in example_commands:
        print(f"\nProcessing command: '{command}'")
        rospy.loginfo(f"Processing command: {command}")

        # Process the command
        action_sequence = vla_system.process_natural_language_command(command)
        print(f"Generated action sequence: {json.dumps(action_sequence, indent=2)}")

        # Execute the action sequence
        success = vla_system.execute_action_sequence(action_sequence)
        print(f"Execution {'succeeded' if success else 'failed'}")

        # Wait before next command
        time.sleep(2.0)

    print("\nVLA system example completed!")
    print("This demonstrates how natural language commands can be converted to robot actions")
    print("through the integration of vision, language understanding, and action execution.")


if __name__ == "__main__":
    main()
```

This comprehensive example demonstrates a complete Vision-Language-Action system that:

1. **Integrates Vision**: Uses CLIP model to analyze visual input and understand the environment
2. **Processes Language**: Converts natural language commands to action sequences using LLMs
3. **Executes Actions**: Translates high-level commands into specific robot control actions
4. **Ensures Safety**: Validates actions against predefined safety limits
5. **Manages Context**: Maintains awareness of robot capabilities and environmental state

To use this system:

1. Set up your OpenAI API key as an environment variable:
   ```bash
   export OPENAI_API_KEY='your-api-key-here'
   ```

2. Install required dependencies:
   ```bash
   pip install torch torchvision torchaudio
   pip install clip
   pip install openai
   pip install opencv-python
   pip install pillow
   ```

3. Run the system within a ROS environment:
   ```bash
   rosrun your_package vla_system.py
   ```

4. Send natural language commands to the system, and it will:
   - Analyze the current visual environment
   - Interpret your command using the LLM
   - Generate a sequence of appropriate actions
   - Execute the actions safely on the robot

This example shows the complete pipeline from natural language understanding to robotic action execution, demonstrating the power of VLA systems in enabling more intuitive human-robot interaction.

## Next Steps

In the next chapter, we'll explore voice-to-action systems using OpenAI Whisper for speech recognition, building upon the VLA foundation we've established here.

---

**Reflection Question**: Consider a humanoid robot that needs to assist an elderly person with daily activities. How would a VLA system enable more natural and effective interaction compared to traditional button-based or gesture-based interfaces? What specific challenges would need to be addressed for safe and reliable operation?