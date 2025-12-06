---
sidebar_position: 19
title: Chapter 15 | Cognitive Planning | Using LLMs to translate natural language to ROS 2 actions
---

# Chapter 15: Cognitive Planning: Using LLMs to Translate Natural Language to ROS 2 Actions

In this chapter, we'll explore cognitive planning systems that use Large Language Models (LLMs) to translate natural language commands into sequences of ROS 2 actions. We'll learn how to create intelligent systems that can decompose complex commands into executable robotic tasks while considering context, constraints, and safety.

## Learning Objectives

By the end of this chapter, you will be able to:
- Design cognitive planning architectures that connect language to action
- Implement LLM-based task decomposition for robotic systems
- Create context-aware planning systems that consider environmental constraints
- Validate and execute sequences of ROS 2 actions from natural language
- Implement safety and error handling in cognitive planning systems

## Introduction to Cognitive Planning in Robotics

Cognitive planning represents the bridge between high-level natural language commands and low-level robotic actions. It involves understanding the user's intent, decomposing complex tasks into manageable steps, and generating executable action sequences that consider the robot's capabilities and environmental constraints.

### The Cognitive Planning Pipeline

#### Natural Language Understanding
- Command interpretation and intent recognition
- Entity extraction and spatial reasoning
- Context awareness and dialogue management
- Task decomposition and planning

#### Action Sequence Generation
- Mapping high-level goals to low-level actions
- Considering robot kinematic and dynamic constraints
- Generating safe and feasible action sequences
- Incorporating environmental and safety constraints

#### Execution and Monitoring
- Action execution with feedback integration
- Plan monitoring and adaptation
- Failure detection and recovery
- Human-robot interaction during execution

### Cognitive Planning vs. Traditional Planning

#### Traditional Planning Approaches
- Pre-programmed behavior sequences
- Limited adaptability to new tasks
- Manual task decomposition
- Rigid, inflexible execution patterns

#### Cognitive Planning Approaches
- Natural language command interpretation
- Automatic task decomposition
- Context-aware planning
- Flexible, adaptive execution patterns

## LLM-Based Task Decomposition

### Hierarchical Task Planning

#### High-Level Task Structure
Cognitive planning systems decompose complex tasks hierarchically:

```python
class HierarchicalTaskPlanner:
    def __init__(self, llm_client):
        self.llm_client = llm_client
        self.robot_capabilities = self.load_robot_capabilities()
        self.environment_model = EnvironmentModel()

    def decompose_task(self, natural_language_command):
        """
        Decompose natural language command into hierarchical task structure
        """
        # Analyze command and context
        analysis = self.analyze_command(natural_language_command)

        # Decompose into high-level tasks
        high_level_tasks = self.generate_high_level_tasks(analysis)

        # Decompose high-level tasks into subtasks
        detailed_plan = self.decompose_to_subtasks(high_level_tasks)

        # Validate and optimize plan
        validated_plan = self.validate_plan(detailed_plan)

        return validated_plan

    def analyze_command(self, command):
        """
        Analyze natural language command to extract intent and context
        """
        analysis_prompt = f"""
        Analyze the following natural language command:
        "{command}"

        Extract the following information:
        1. Primary goal/intent
        2. Secondary objectives
        3. Spatial references (locations, directions, distances)
        4. Object references (what to manipulate, move, etc.)
        5. Temporal constraints (when, how long, etc.)
        6. Conditional elements (if-then, when, etc.)
        7. Safety considerations

        Return the analysis in JSON format.
        """

        response = self.llm_client.generate(analysis_prompt)
        return json.loads(response)

    def generate_high_level_tasks(self, analysis):
        """
        Generate high-level tasks from command analysis
        """
        planning_prompt = f"""
        Based on the following command analysis:
        {json.dumps(analysis, indent=2)}

        Generate a sequence of high-level tasks that would accomplish the goal.
        Each task should be a single, cohesive action that can be further decomposed.

        Consider the robot's capabilities:
        {json.dumps(self.robot_capabilities, indent=2)}

        Return the tasks in JSON format with the following structure:
        {{
            "tasks": [
                {{
                    "id": "unique_task_id",
                    "description": "Brief description of the task",
                    "type": "navigation|manipulation|interaction|perception",
                    "dependencies": ["task_id_1", "task_id_2"],
                    "subtasks": []
                }}
            ]
        }}
        """

        response = self.llm_client.generate(planning_prompt)
        return json.loads(response)

    def decompose_to_subtasks(self, high_level_tasks):
        """
        Decompose high-level tasks into executable subtasks
        """
        detailed_plan = {"tasks": []}

        for task in high_level_tasks["tasks"]:
            subtasks = self.decompose_single_task(task)
            task["subtasks"] = subtasks
            detailed_plan["tasks"].append(task)

        return detailed_plan

    def decompose_single_task(self, task):
        """
        Decompose a single task into executable subtasks
        """
        decomposition_prompt = f"""
        Decompose the following task into executable subtasks:
        Task: {task['description']}
        Type: {task['type']}

        Consider the robot's capabilities:
        {json.dumps(self.robot_capabilities, indent=2)}

        Environmental constraints:
        {json.dumps(self.environment_model.get_current_state(), indent=2)}

        Decompose this task into specific, executable subtasks that can be converted to ROS 2 actions.
        Each subtask should be simple and unambiguous.

        Return in JSON format:
        {{
            "subtasks": [
                {{
                    "id": "subtask_id",
                    "description": "Detailed description of subtask",
                    "action_type": "move_to|pick_up|place|speak|navigate|perceive",
                    "parameters": {{"param1": "value1"}},
                    "success_criteria": "How to verify this subtask is complete",
                    "failure_modes": ["possible failure reasons"]
                }}
            ]
        }}
        """

        response = self.llm_client.generate(decomposition_prompt)
        result = json.loads(response)
        return result["subtasks"]

    def validate_plan(self, plan):
        """
        Validate plan for feasibility and safety
        """
        validation_prompt = f"""
        Validate the following plan for feasibility and safety:
        {json.dumps(plan, indent=2)}

        Check for:
        1. Feasibility given robot capabilities
        2. Safety considerations
        3. Logical sequence of actions
        4. Environmental constraints
        5. Potential conflicts between tasks

        Return validation results in JSON format:
        {{
            "is_valid": true/false,
            "issues": ["list of issues if any"],
            "recommendations": ["suggestions for improvement"],
            "modified_plan": {{...}}  // Return modified plan if needed
        }}
        """

        response = self.llm_client.generate(validation_prompt)
        validation_result = json.loads(response)

        if validation_result["is_valid"]:
            return plan
        else:
            return validation_result.get("modified_plan", plan)

    def load_robot_capabilities(self):
        """
        Load robot capabilities from configuration
        """
        # This would load from robot description or service
        return {
            "navigation": {
                "max_speed": 0.5,
                "min_turn_radius": 0.3,
                "max_gradient": 15.0  # degrees
            },
            "manipulation": {
                "max_reach": 1.2,
                "max_load": 5.0,
                "precision": "centimeter"
            },
            "perception": {
                "camera_range": 10.0,
                "lidar_range": 30.0,
                "object_detection": ["person", "chair", "table", "cup", "bottle"]
            },
            "interaction": {
                "speaking": True,
                "gesture": True,
                "display": True
            }
        }
```

### Context-Aware Planning

#### Environmental Context Integration
```python
class ContextAwarePlanner:
    def __init__(self, environment_model, perception_system):
        self.environment_model = environment_model
        self.perception_system = perception_system
        self.context_history = []

    def plan_with_context(self, command, user_context=None):
        """
        Generate plan considering environmental and user context
        """
        # Get current environmental context
        env_context = self.perception_system.get_environmental_context()

        # Combine with user context
        full_context = {
            "environment": env_context,
            "user": user_context or {},
            "robot": self.get_robot_state(),
            "history": self.context_history[-10:]  # Last 10 interactions
        }

        # Generate context-aware plan
        plan = self.generate_contextual_plan(command, full_context)

        # Update context history
        self.context_history.append({
            "command": command,
            "context": full_context,
            "plan": plan,
            "timestamp": time.time()
        })

        return plan

    def generate_contextual_plan(self, command, context):
        """
        Generate plan considering full context
        """
        contextual_prompt = f"""
        Generate a plan for: "{command}"

        Consider the current context:
        Environment: {json.dumps(context['environment'], indent=2)}
        User: {json.dumps(context['user'], indent=2)}
        Robot State: {json.dumps(context['robot'], indent=2)}
        Recent History: {json.dumps(context['history'][-3:], indent=2)}

        Generate a plan that:
        1. Accounts for environmental constraints
        2. Considers user preferences and history
        3. Adapts to current robot state
        4. Maintains consistency with previous interactions

        Return plan in standard format.
        """

        # This would call the LLM to generate the contextual plan
        # For now, returning a placeholder
        return {"tasks": [], "context_considerations": []}

    def get_robot_state(self):
        """
        Get current robot state
        """
        # This would query robot state from ROS topics
        return {
            "location": [0.0, 0.0, 0.0],
            "battery_level": 85.0,
            "current_task": None,
            "manipulator_status": "free",
            "navigation_status": "ready"
        }
```

## ROS 2 Action Generation

### Action Space Definition

#### Robot Action Space
```python
class RobotActionSpace:
    def __init__(self):
        # Define action space for the robot
        self.actions = {
            "navigation": {
                "move_to": {
                    "parameters": ["x", "y", "theta", "frame"],
                    "description": "Move robot to specified pose"
                },
                "go_to_named_location": {
                    "parameters": ["location_name"],
                    "description": "Navigate to predefined location"
                },
                "follow_path": {
                    "parameters": ["waypoints"],
                    "description": "Follow a sequence of waypoints"
                }
            },
            "manipulation": {
                "pick_object": {
                    "parameters": ["object_name", "object_pose", "approach_direction"],
                    "description": "Pick up specified object"
                },
                "place_object": {
                    "parameters": ["object_name", "target_pose", "placement_strategy"],
                    "description": "Place object at target location"
                },
                "grasp": {
                    "parameters": ["object_name", "grasp_type", "grasp_pose"],
                    "description": "Grasp object with specified grasp"
                }
            },
            "interaction": {
                "speak": {
                    "parameters": ["text", "language", "voice_style"],
                    "description": "Speak specified text"
                },
                "listen": {
                    "parameters": ["duration", "keywords"],
                    "description": "Listen for user input"
                },
                "display_message": {
                    "parameters": ["message", "display_duration"],
                    "description": "Display message on robot screen"
                }
            },
            "perception": {
                "look_at": {
                    "parameters": ["target_pose", "look_duration"],
                    "description": "Direct camera toward target"
                },
                "detect_objects": {
                    "parameters": ["region_of_interest", "object_classes"],
                    "description": "Detect objects in specified region"
                },
                "measure_distance": {
                    "parameters": ["target_direction", "max_range"],
                    "description": "Measure distance in specified direction"
                }
            }
        }

    def get_action_schema(self, action_type, action_name):
        """
        Get schema for specific action
        """
        if action_type in self.actions and action_name in self.actions[action_type]:
            return self.actions[action_type][action_name]
        return None

    def validate_action_parameters(self, action_type, action_name, parameters):
        """
        Validate action parameters against schema
        """
        schema = self.get_action_schema(action_type, action_name)
        if not schema:
            return False, f"Action {action_name} not found in {action_type}"

        # Basic validation - check required parameters exist
        # In practice, this would be more sophisticated
        required_params = schema.get("parameters", [])

        for param in required_params:
            if param not in parameters:
                return False, f"Missing required parameter: {param}"

        return True, "Valid"
```

### Natural Language to Action Mapping

```python
class LanguageToActionMapper:
    def __init__(self, llm_client, robot_action_space):
        self.llm_client = llm_client
        self.action_space = robot_action_space
        self.action_mapping_cache = {}

    def map_command_to_actions(self, command_text, context=None):
        """
        Map natural language command to sequence of ROS 2 actions
        """
        # Check cache first
        cache_key = f"{command_text}_{hash(str(context))}"
        if cache_key in self.action_mapping_cache:
            return self.action_mapping_cache[cache_key]

        # Generate action mapping
        action_sequence = self.generate_action_sequence(command_text, context)

        # Cache result
        self.action_mapping_cache[cache_key] = action_sequence

        return action_sequence

    def generate_action_sequence(self, command_text, context=None):
        """
        Generate sequence of ROS 2 actions from natural language command
        """
        mapping_prompt = f"""
        Convert the following natural language command to a sequence of ROS 2 actions:
        "{command_text}"

        Available robot actions:
        {json.dumps(self.action_space.actions, indent=2)}

        Current context:
        {json.dumps(context, indent=2) if context else "None"}

        Convert this command into a sequence of specific actions that can be executed by the robot.
        Each action should have:
        1. action_type: navigation, manipulation, interaction, or perception
        2. action_name: specific action from the action space
        3. parameters: specific values for the action
        4. success_criteria: how to know when action is complete
        5. failure_indicators: signs of failure

        Return the action sequence in JSON format:
        {{
            "action_sequence": [
                {{
                    "id": "action_id",
                    "action_type": "navigation|manipulation|interaction|perception",
                    "action_name": "specific_action_name",
                    "parameters": {{"param1": "value1", "param2": "value2"}},
                    "success_criteria": "condition for success",
                    "failure_indicators": ["possible failure conditions"],
                    "timeout": 30.0  // seconds
                }}
            ]
        }}
        """

        try:
            response = self.llm_client.generate(mapping_prompt)
            result = json.loads(response)
            return result["action_sequence"]
        except Exception as e:
            print(f"Error generating action sequence: {e}")
            # Return fallback action sequence
            return self.generate_fallback_sequence(command_text)

    def generate_fallback_sequence(self, command_text):
        """
        Generate fallback action sequence when LLM fails
        """
        # Simple fallback for common commands
        command_lower = command_text.lower()

        if "go to" in command_lower or "navigate to" in command_lower:
            return [{
                "id": "fallback_navigate",
                "action_type": "navigation",
                "action_name": "go_to_named_location",
                "parameters": {"location_name": "unknown"},
                "success_criteria": "reach destination",
                "failure_indicators": ["obstacle", "timeout"],
                "timeout": 60.0
            }]
        elif "pick" in command_lower or "grasp" in command_lower:
            return [{
                "id": "fallback_manipulate",
                "action_type": "manipulation",
                "action_name": "pick_object",
                "parameters": {"object_name": "unknown"},
                "success_criteria": "object grasped",
                "failure_indicators": ["object_not_found", "grasp_failed"],
                "timeout": 30.0
            }]
        elif "speak" in command_lower or "say" in command_lower:
            return [{
                "id": "fallback_speak",
                "action_type": "interaction",
                "action_name": "speak",
                "parameters": {"text": command_text},
                "success_criteria": "text spoken",
                "failure_indicators": ["tts_error"],
                "timeout": 10.0
            }]
        else:
            return []

    def refine_action_parameters(self, action_sequence, current_state):
        """
        Refine action parameters based on current state
        """
        refined_sequence = []

        for action in action_sequence:
            refined_action = action.copy()

            # Refine parameters based on current state
            if action["action_name"] == "move_to":
                # Ensure coordinates are valid
                if "x" in action["parameters"] and "y" in action["parameters"]:
                    # Check if target is reachable
                    if not self.is_target_reachable(
                        current_state["location"],
                        [action["parameters"]["x"], action["parameters"]["y"]]
                    ):
                        # Adjust or skip action
                        continue

            refined_sequence.append(refined_action)

        return refined_sequence

    def is_target_reachable(self, current_pos, target_pos):
        """
        Check if target is reachable from current position
        """
        # Simple distance check
        distance = math.sqrt(
            (target_pos[0] - current_pos[0])**2 +
            (target_pos[1] - current_pos[1])**2
        )
        return distance < 100.0  # Arbitrary maximum distance
```

## Cognitive Planning Architecture

### Planning System Components

#### Plan Generator
```python
class CognitivePlanGenerator:
    def __init__(self, llm_client, robot_interface, environment_model):
        self.llm_client = llm_client
        self.robot_interface = robot_interface
        self.environment_model = environment_model
        self.action_mapper = LanguageToActionMapper(llm_client, RobotActionSpace())
        self.context_planner = ContextAwarePlanner(environment_model, robot_interface)

    def generate_plan(self, natural_command, user_context=None):
        """
        Generate cognitive plan from natural language command
        """
        # Step 1: Analyze command
        analysis = self.analyze_command(natural_command)

        # Step 2: Plan with context
        contextual_plan = self.context_planner.plan_with_context(
            natural_command, user_context
        )

        # Step 3: Map to actions
        action_sequence = self.action_mapper.map_command_to_actions(
            natural_command, contextual_plan
        )

        # Step 4: Validate plan
        validated_plan = self.validate_plan(action_sequence)

        # Step 5: Create execution plan
        execution_plan = self.create_execution_plan(validated_plan)

        return execution_plan

    def analyze_command(self, command):
        """
        Analyze natural language command
        """
        analysis_prompt = f"""
        Analyze the command: "{command}"

        Identify:
        1. Primary objective
        2. Secondary objectives
        3. Spatial references
        4. Object references
        5. Temporal constraints
        6. Conditional elements
        7. Implicit assumptions

        Return analysis in structured format.
        """

        response = self.llm_client.generate(analysis_prompt)
        return json.loads(response)

    def validate_plan(self, action_sequence):
        """
        Validate action sequence for feasibility
        """
        # Check each action
        for action in action_sequence:
            is_valid, error = self.action_mapper.action_space.validate_action_parameters(
                action["action_type"],
                action["action_name"],
                action["parameters"]
            )

            if not is_valid:
                print(f"Invalid action: {error}")
                # Try to fix or remove action
                action_sequence.remove(action)

        return action_sequence

    def create_execution_plan(self, validated_plan):
        """
        Create execution plan with monitoring and recovery
        """
        execution_plan = {
            "id": str(uuid.uuid4()),
            "actions": validated_plan,
            "monitoring_conditions": [],
            "recovery_strategies": [],
            "execution_context": {}
        }

        # Add monitoring conditions
        for i, action in enumerate(validated_plan):
            execution_plan["monitoring_conditions"].append({
                "action_index": i,
                "monitored_variables": ["execution_status", "robot_state", "environment_changes"],
                "success_criteria": action["success_criteria"],
                "failure_indicators": action["failure_indicators"]
            })

        # Add recovery strategies
        execution_plan["recovery_strategies"] = self.generate_recovery_strategies(validated_plan)

        return execution_plan

    def generate_recovery_strategies(self, action_sequence):
        """
        Generate recovery strategies for each action
        """
        recovery_strategies = []

        for action in action_sequence:
            strategy = {
                "action_id": action.get("id", "unknown"),
                "recovery_options": [
                    {
                        "type": "retry",
                        "conditions": ["temporary_failure"],
                        "max_attempts": 3,
                        "delay": 2.0
                    },
                    {
                        "type": "alternative_approach",
                        "conditions": ["permanent_failure"],
                        "alternative_action": self.generate_alternative_action(action)
                    },
                    {
                        "type": "abort_and_report",
                        "conditions": ["critical_failure", "safety_violation"],
                        "report_message": "Critical failure during action execution"
                    }
                ]
            }
            recovery_strategies.append(strategy)

        return recovery_strategies

    def generate_alternative_action(self, original_action):
        """
        Generate alternative action when original fails
        """
        # This would generate a similar but different approach
        alternative = original_action.copy()

        # Example: if navigation fails, try alternative route
        if original_action["action_name"] == "move_to":
            alternative["parameters"]["avoid_obstacles"] = True
            alternative["parameters"]["use_alternative_route"] = True

        return alternative
```

## ROS 2 Action Execution

### Action Executor

```python
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
from humanoid_robot_msgs.action import ExecuteAction, NavigateToPose, ManipulateObject

class ActionExecutor(Node):
    def __init__(self):
        super().__init__('action_executor')

        # Action clients for different action types
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.manip_client = ActionClient(self, ManipulateObject, 'manipulate_object')
        self.execute_client = ActionClient(self, ExecuteAction, 'execute_action')

        # Publishers for monitoring
        self.status_pub = self.create_publisher(String, 'action_status', 10)
        self.feedback_pub = self.create_publisher(String, 'action_feedback', 10)

        # Action execution queue
        self.action_queue = []
        self.current_action = None
        self.action_lock = threading.Lock()

        # Execution thread
        self.execution_thread = threading.Thread(target=self.execution_loop, daemon=True)
        self.execution_thread.start()

    def execute_plan(self, execution_plan):
        """
        Execute cognitive plan
        """
        with self.action_lock:
            # Add all actions to queue
            for action in execution_plan["actions"]:
                self.action_queue.append(action)

        # Process queue
        self.process_action_queue()

    def process_action_queue(self):
        """
        Process actions in the queue
        """
        while self.action_queue:
            with self.action_lock:
                if self.action_queue:
                    action = self.action_queue.pop(0)
                else:
                    break

            self.execute_single_action(action)

    def execute_single_action(self, action):
        """
        Execute a single action
        """
        self.current_action = action

        # Update status
        status_msg = String()
        status_msg.data = f"Executing: {action['action_name']} with params: {action['parameters']}"
        self.status_pub.publish(status_msg)

        try:
            if action["action_type"] == "navigation":
                success = self.execute_navigation_action(action)
            elif action["action_type"] == "manipulation":
                success = self.execute_manipulation_action(action)
            elif action["action_type"] == "interaction":
                success = self.execute_interaction_action(action)
            elif action["action_type"] == "perception":
                success = self.execute_perception_action(action)
            else:
                success = False
                self.get_logger().error(f"Unknown action type: {action['action_type']}")

            # Report result
            result_msg = String()
            result_msg.data = f"Action {action['action_name']} {'succeeded' if success else 'failed'}"
            self.feedback_pub.publish(result_msg)

            return success

        except Exception as e:
            self.get_logger().error(f"Error executing action {action['action_name']}: {e}")
            return False

    def execute_navigation_action(self, action):
        """
        Execute navigation action
        """
        goal_msg = NavigateToPose.Goal()

        if action["action_name"] == "move_to":
            goal_msg.target_pose.header.frame_id = action["parameters"].get("frame", "map")
            goal_msg.target_pose.pose.position.x = action["parameters"]["x"]
            goal_msg.target_pose.pose.position.y = action["parameters"]["y"]
            goal_msg.target_pose.pose.position.z = action["parameters"].get("z", 0.0)

            # Set orientation
            theta = action["parameters"].get("theta", 0.0)
            goal_msg.target_pose.pose.orientation = self.euler_to_quaternion(0, 0, theta)

        elif action["action_name"] == "go_to_named_location":
            # This would require a location mapping service
            location_name = action["parameters"]["location_name"]
            # Lookup location from service
            goal_msg = self.lookup_location_pose(location_name)

        # Send goal
        self.nav_client.wait_for_server()
        future = self.nav_client.send_goal_async(goal_msg)

        # Wait for result with timeout
        timeout = action.get("timeout", 60.0)
        start_time = time.time()

        while time.time() - start_time < timeout:
            if future.done():
                result = future.result()
                return result.result.success

            time.sleep(0.1)

        # Timeout occurred
        return False

    def execute_manipulation_action(self, action):
        """
        Execute manipulation action
        """
        goal_msg = ManipulateObject.Goal()

        if action["action_name"] == "pick_object":
            goal_msg.operation = "pick"
            goal_msg.object_name = action["parameters"]["object_name"]
            goal_msg.object_pose = self.dict_to_pose(action["parameters"]["object_pose"])

        elif action["action_name"] == "place_object":
            goal_msg.operation = "place"
            goal_msg.object_name = action["parameters"]["object_name"]
            goal_msg.target_pose = self.dict_to_pose(action["parameters"]["target_pose"])

        # Send goal
        self.manip_client.wait_for_server()
        future = self.manip_client.send_goal_async(goal_msg)

        # Wait for result with timeout
        timeout = action.get("timeout", 30.0)
        start_time = time.time()

        while time.time() - start_time < timeout:
            if future.done():
                result = future.result()
                return result.result.success

            time.sleep(0.1)

        return False

    def execute_interaction_action(self, action):
        """
        Execute interaction action
        """
        if action["action_name"] == "speak":
            text = action["parameters"]["text"]
            return self.speak_text(text)

        return False

    def execute_perception_action(self, action):
        """
        Execute perception action
        """
        # Perception actions typically return information
        # rather than success/failure in the same way
        if action["action_name"] == "detect_objects":
            region = action["parameters"]["region_of_interest"]
            classes = action["parameters"]["object_classes"]
            return self.detect_objects(region, classes)

        return False

    def speak_text(self, text):
        """
        Speak text using robot's speech system
        """
        # This would publish to text-to-speech system
        # For now, just print
        print(f"Robot says: {text}")
        return True

    def detect_objects(self, region, classes):
        """
        Detect objects in specified region
        """
        # This would call perception system
        # For now, return dummy result
        return True

    def dict_to_pose(self, pose_dict):
        """
        Convert dictionary to Pose message
        """
        from geometry_msgs.msg import Pose
        pose = Pose()
        pose.position.x = pose_dict.get("x", 0.0)
        pose.position.y = pose_dict.get("y", 0.0)
        pose.position.z = pose_dict.get("z", 0.0)
        pose.orientation.x = pose_dict.get("qx", 0.0)
        pose.orientation.y = pose_dict.get("qy", 0.0)
        pose.orientation.z = pose_dict.get("qz", 0.0)
        pose.orientation.w = pose_dict.get("qw", 1.0)
        return pose

    def euler_to_quaternion(self, roll, pitch, yaw):
        """
        Convert Euler angles to quaternion
        """
        from geometry_msgs.msg import Quaternion
        import math

        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)

        q = Quaternion()
        q.w = cr * cp * cy + sr * sp * sy
        q.x = sr * cp * cy - cr * sp * sy
        q.y = cr * sp * cy + sr * cp * sy
        q.z = cr * cp * sy - sr * sp * cy

        return q

    def lookup_location_pose(self, location_name):
        """
        Look up pose for named location
        """
        # This would call a location service
        # For now, return dummy pose
        pose_stamped = PoseStamped()
        pose_stamped.header.frame_id = "map"
        pose_stamped.pose.position.x = 1.0
        pose_stamped.pose.position.y = 1.0
        return NavigateToPose.Goal(target_pose=pose_stamped)

    def execution_loop(self):
        """
        Main execution loop
        """
        while rclpy.ok():
            time.sleep(0.1)  # Prevent busy waiting
```

## Safety and Validation

### Plan Validation System

```python
class PlanValidator:
    def __init__(self, robot_model, environment_model):
        self.robot_model = robot_model
        self.environment_model = environment_model

    def validate_plan(self, plan, context):
        """
        Validate plan for safety and feasibility
        """
        validation_results = {
            "is_valid": True,
            "warnings": [],
            "errors": [],
            "safety_issues": [],
            "feasibility_issues": []
        }

        # Check each action in the plan
        for i, action in enumerate(plan["actions"]):
            action_validation = self.validate_single_action(action, context)

            if not action_validation["is_valid"]:
                validation_results["is_valid"] = False
                validation_results["errors"].extend(action_validation["errors"])

            validation_results["warnings"].extend(action_validation["warnings"])
            validation_results["safety_issues"].extend(action_validation["safety_issues"])
            validation_results["feasibility_issues"].extend(action_validation["feasibility_issues"])

        # Check plan-level constraints
        plan_level_validation = self.validate_plan_level_constraints(plan, context)
        validation_results["errors"].extend(plan_level_validation["errors"])
        validation_results["safety_issues"].extend(plan_level_validation["safety_issues"])

        if validation_results["errors"]:
            validation_results["is_valid"] = False

        return validation_results

    def validate_single_action(self, action, context):
        """
        Validate single action
        """
        validation = {
            "is_valid": True,
            "warnings": [],
            "errors": [],
            "safety_issues": [],
            "feasibility_issues": []
        }

        # Check action parameters
        action_params = action.get("parameters", {})
        action_name = action.get("action_name", "")
        action_type = action.get("action_type", "")

        # Validate action exists in robot model
        if not self.robot_model.action_exists(action_type, action_name):
            validation["errors"].append(f"Action {action_name} not supported by robot")
            validation["is_valid"] = False

        # Validate parameters
        required_params = self.robot_model.get_required_parameters(action_type, action_name)
        for param in required_params:
            if param not in action_params:
                validation["errors"].append(f"Missing required parameter: {param}")
                validation["is_valid"] = False

        # Check safety constraints
        if action_name == "move_to":
            target = [action_params.get("x", 0), action_params.get("y", 0)]
            if self.environment_model.is_hazardous_location(target):
                validation["safety_issues"].append(f"Target location {target} is hazardous")
                validation["is_valid"] = False

        elif action_name == "pick_object":
            obj_pose = action_params.get("object_pose", {})
            if self.environment_model.is_unreachable_object(obj_pose):
                validation["feasibility_issues"].append("Object is unreachable")

        # Check kinematic constraints
        if action_type == "manipulation":
            if not self.robot_model.can_perform_manipulation(action_params):
                validation["feasibility_issues"].append("Manipulation not kinematically feasible")

        return validation

    def validate_plan_level_constraints(self, plan, context):
        """
        Validate plan-level constraints
        """
        validation = {
            "errors": [],
            "safety_issues": []
        }

        # Check battery constraints
        estimated_battery_usage = self.estimate_battery_usage(plan)
        current_battery = context.get("robot_state", {}).get("battery_level", 100.0)

        if current_battery - estimated_battery_usage < 10.0:  # Less than 10% remaining
            validation["safety_issues"].append("Plan may drain battery below safe level")

        # Check time constraints
        estimated_duration = self.estimate_plan_duration(plan)
        max_allowed_time = context.get("time_constraints", {}).get("max_duration", float('inf'))

        if estimated_duration > max_allowed_time:
            validation["errors"].append(f"Plan duration ({estimated_duration}s) exceeds maximum allowed ({max_allowed_time}s)")

        return validation

    def estimate_battery_usage(self, plan):
        """
        Estimate battery usage for plan
        """
        total_usage = 0.0

        for action in plan["actions"]:
            action_name = action.get("action_name", "")
            duration = action.get("timeout", 10.0)  # Default 10 seconds

            if action_name in ["move_to", "go_to_named_location"]:
                total_usage += duration * 0.1  # 0.1% per second for navigation
            elif action_name in ["pick_object", "place_object"]:
                total_usage += 2.0  # 2% for manipulation
            elif action_name == "speak":
                total_usage += duration * 0.05  # 0.05% per second for speaking

        return total_usage

    def estimate_plan_duration(self, plan):
        """
        Estimate total duration of plan
        """
        total_duration = 0.0

        for action in plan["actions"]:
            total_duration += action.get("timeout", 10.0)

        return total_duration
```

## Error Handling and Recovery

### Plan Execution Monitoring

```python
class PlanExecutionMonitor:
    def __init__(self, action_executor, plan_validator):
        self.action_executor = action_executor
        self.plan_validator = plan_validator
        self.current_plan = None
        self.current_action_index = 0
        self.execution_history = []

    def monitor_execution(self, plan):
        """
        Monitor plan execution and handle failures
        """
        self.current_plan = plan
        self.current_action_index = 0
        self.execution_history = []

        while self.current_action_index < len(plan["actions"]):
            action = plan["actions"][self.current_action_index]

            # Execute action
            success = self.action_executor.execute_single_action(action)

            # Record execution result
            execution_record = {
                "action_index": self.current_action_index,
                "action": action,
                "success": success,
                "timestamp": time.time()
            }
            self.execution_history.append(execution_record)

            if success:
                # Move to next action
                self.current_action_index += 1
            else:
                # Handle failure
                recovery_success = self.handle_action_failure(action, execution_record)

                if not recovery_success:
                    # Recovery failed, abort plan
                    return False

        # All actions completed successfully
        return True

    def handle_action_failure(self, action, execution_record):
        """
        Handle action failure with recovery strategies
        """
        print(f"Action failed: {action['action_name']}")

        # Find recovery strategy for this action
        recovery_strategy = self.find_recovery_strategy(action)

        if not recovery_strategy:
            print("No recovery strategy available")
            return False

        # Try each recovery option
        for recovery_option in recovery_strategy["recovery_options"]:
            success = self.attempt_recovery(recovery_option, action, execution_record)

            if success:
                return True

        # All recovery options failed
        return False

    def find_recovery_strategy(self, action):
        """
        Find recovery strategy for action
        """
        plan = self.current_plan

        for strategy in plan.get("recovery_strategies", []):
            if strategy["action_id"] == action.get("id"):
                return strategy

        return None

    def attempt_recovery(self, recovery_option, original_action, execution_record):
        """
        Attempt a specific recovery option
        """
        recovery_type = recovery_option["type"]

        if recovery_type == "retry":
            return self.retry_action(original_action, recovery_option)
        elif recovery_type == "alternative_approach":
            return self.use_alternative_approach(original_action, recovery_option)
        elif recovery_type == "abort_and_report":
            self.abort_and_report(original_action, execution_record, recovery_option)
            return False

        return False

    def retry_action(self, action, recovery_option):
        """
        Retry the failed action
        """
        max_attempts = recovery_option.get("max_attempts", 3)
        delay = recovery_option.get("delay", 2.0)

        for attempt in range(max_attempts):
            print(f"Retrying action {action['action_name']}, attempt {attempt + 1}")

            time.sleep(delay)

            success = self.action_executor.execute_single_action(action)

            if success:
                return True

        return False

    def use_alternative_approach(self, original_action, recovery_option):
        """
        Use alternative approach for action
        """
        alternative_action = recovery_option.get("alternative_action")

        if not alternative_action:
            return False

        print(f"Using alternative action: {alternative_action['action_name']}")
        success = self.action_executor.execute_single_action(alternative_action)

        return success

    def abort_and_report(self, action, execution_record, recovery_option):
        """
        Abort plan and report failure
        """
        report_message = recovery_option.get("report_message", "Action failed")
        print(f"Aborting plan: {report_message}")

        # Publish failure notification
        # This would publish to failure reporting system
        failure_msg = String()
        failure_msg.data = f"Plan aborted due to failure in action {action['action_name']}: {report_message}"
        # self.failure_pub.publish(failure_msg)  # Uncomment in ROS environment
```

## Integration with Previous Modules

### Combining Voice Recognition with Cognitive Planning

```python
class IntegratedVoicePlanner:
    def __init__(self, llm_client, robot_interface, environment_model):
        # Voice recognition component (from Chapter 14)
        self.voice_recognizer = WhisperVoiceRecognizer(model_size="base")

        # Cognitive planning component
        self.cognitive_planner = CognitivePlanGenerator(
            llm_client, robot_interface, environment_model
        )

        # Action execution
        self.action_executor = ActionExecutor()

        # Execution monitoring
        self.monitor = PlanExecutionMonitor(
            self.action_executor,
            PlanValidator(robot_interface, environment_model)
        )

    def process_voice_command(self, audio_input):
        """
        Process voice command through full pipeline
        """
        # Step 1: Recognize speech
        recognized_text = self.voice_recognizer.transcribe_audio(audio_input)

        if not recognized_text:
            return {"status": "error", "message": "Could not recognize speech"}

        # Step 2: Generate cognitive plan
        try:
            execution_plan = self.cognitive_planner.generate_plan(recognized_text)
        except Exception as e:
            return {"status": "error", "message": f"Could not generate plan: {e}"}

        # Step 3: Validate plan
        validator = PlanValidator(self.action_executor, self.cognitive_planner.environment_model)
        validation_results = validator.validate_plan(execution_plan, {})

        if not validation_results["is_valid"]:
            return {
                "status": "error",
                "message": "Plan validation failed",
                "issues": validation_results["errors"]
            }

        # Step 4: Execute plan
        success = self.monitor.monitor_execution(execution_plan)

        return {
            "status": "success" if success else "partial_success",
            "recognized_text": recognized_text,
            "execution_plan": execution_plan,
            "success": success
        }

    def continuous_voice_processing(self):
        """
        Continuously listen for and process voice commands
        """
        self.voice_recognizer.start_listening()

        while True:
            try:
                # Get recognized command
                command = self.voice_recognizer.process_voice_command()

                if command:
                    # Process the command
                    result = self.process_voice_command(command)

                    # Provide feedback
                    if result["status"] == "success":
                        self.action_executor.speak_text("Command completed successfully")
                    else:
                        self.action_executor.speak_text(f"Command failed: {result.get('message', 'Unknown error')}")

            except KeyboardInterrupt:
                break
            except Exception as e:
                print(f"Error in continuous processing: {e}")
                continue
```

## Chapter Summary

Cognitive planning systems that translate natural language to ROS 2 actions represent a significant advancement in human-robot interaction. These systems enable robots to understand complex commands, decompose them into executable actions, and execute them safely while considering environmental constraints and safety requirements.

The integration of LLMs with robotic action systems provides several key benefits:
- Natural and intuitive human-robot interaction
- Automatic task decomposition and planning
- Context-aware execution
- Flexible and adaptable behavior

However, implementing these systems requires careful consideration of safety, validation, and error handling to ensure reliable and safe operation.

The cognitive planning architecture we've explored provides a framework for building intelligent robotic systems that can interpret natural language commands and execute them as sequences of ROS 2 actions. This foundation enables the development of sophisticated robotic applications that can operate effectively in complex, dynamic environments.

As we continue through this textbook, you'll learn how to integrate these cognitive planning capabilities into a complete autonomous humanoid robot system in the capstone project.

## Check Your Understanding

1. **Conceptual**: Explain the difference between traditional programmed robotic behavior and cognitive planning systems that use LLMs for action generation.

2. **Application**: Design a cognitive planning system that can handle a command like "Go to the kitchen, pick up the red cup from the counter, and bring it to me" - break down the planning process and execution steps.

3. **Analysis**: What are the main challenges in ensuring safety and reliability when using LLMs to generate robotic actions, and how might you address these challenges?

## Next Steps

In the next chapter, we'll explore the capstone project where we'll integrate all the concepts we've learned into a complete autonomous humanoid robot system that can understand natural language commands, navigate its environment, and perform complex tasks.

---

**Reflection Question**: Consider a humanoid robot operating in a hospital environment where it needs to respond to various staff members' requests. How would you design a cognitive planning system that can prioritize tasks, handle interruptions, and ensure patient safety while executing natural language commands?