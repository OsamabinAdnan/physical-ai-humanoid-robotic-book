---
sidebar_position: 4
title: Chapter 3 | Bridging Python Agents to ROS controllers using rclpy
---

# Chapter 3: Bridging Python Agents to ROS Controllers using rclpy

In this chapter, we'll explore how to connect Python-based AI agents with ROS controllers using rclpy, the Python client library for ROS 2. This bridging is essential for modern robotics, where AI agents need to interact with low-level robot controllers to execute complex tasks.

## Learning Objectives

By the end of this chapter, you will be able to:
- Understand the architecture of AI agent to ROS controller integration
- Implement Python agents that communicate with ROS systems
- Use rclpy to create nodes that bridge AI and control systems
- Design effective communication patterns between AI agents and controllers
- Debug and optimize the bridge between high-level AI and low-level control

## Understanding the AI-Agent-to-Controller Bridge

Think of the bridge between AI agents and ROS controllers as a translator between two different languages. The AI agent speaks in high-level concepts ("navigate to the kitchen," "pick up the red object"), while the controller speaks in low-level commands ("move joint 1 to 45 degrees," "apply 2.5 Nm torque to motor 3").

### The Need for Bridging

Modern robotics requires coordination between:
- **High-level AI agents**: Making strategic decisions based on perception and planning
- **Low-level controllers**: Executing precise physical movements and maintaining stability
- **Middleware systems**: Facilitating communication between different system layers

### The Bridge Architecture

The bridge typically consists of three layers:

1. **AI Agent Layer**: Python-based decision-making and planning
2. **Bridge Layer**: rclpy-based communication and message translation
3. **Controller Layer**: ROS-based low-level control systems

## Introduction to rclpy

rclpy is the Python client library for ROS 2, providing Python access to all ROS 2 capabilities. It allows Python developers to create ROS 2 nodes, publish and subscribe to topics, and call services.

### Installing and Setting Up rclpy

rclpy is part of the ROS 2 installation, but you can ensure it's properly installed:

```bash
# Verify rclpy installation
python3 -c "import rclpy; print('rclpy version:', rclpy.__version__)"
```

### Basic rclpy Node Structure

```python
import rclpy
from rclpy.node import Node

class AIControllerBridge(Node):
    def __init__(self):
        super().__init__('ai_controller_bridge')
        self.get_logger().info('AI-Controller Bridge initialized')

def main(args=None):
    rclpy.init(args=args)
    bridge_node = AIControllerBridge()

    try:
        rclpy.spin(bridge_node)
    except KeyboardInterrupt:
        pass
    finally:
        bridge_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Creating an AI Agent Node

Let's start by creating a simple AI agent that makes decisions based on input data:

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import random

class SimpleAIAgent(Node):
    def __init__(self):
        super().__init__('simple_ai_agent')

        # Subscribe to sensor data
        self.subscription = self.create_subscription(
            LaserScan,
            'scan',
            self.scan_callback,
            10)

        # Publish velocity commands
        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)

        # Timer for AI decision making
        self.timer = self.create_timer(0.5, self.ai_decision_loop)

        # Internal state
        self.scan_data = None
        self.obstacle_detected = False
        self.target_reached = False

        self.get_logger().info('Simple AI Agent initialized')

    def scan_callback(self, msg):
        """Process incoming laser scan data"""
        self.scan_data = msg
        # Check for obstacles in front of the robot
        if self.scan_data:
            front_range = self.scan_data.ranges[len(self.scan_data.ranges)//2]
            self.obstacle_detected = front_range < 1.0  # 1 meter threshold

    def ai_decision_loop(self):
        """Main AI decision-making loop"""
        if not self.scan_data:
            return

        cmd_vel = Twist()

        if self.obstacle_detected:
            # Stop and turn to avoid obstacle
            cmd_vel.linear.x = 0.0
            cmd_vel.angular.z = 0.5  # Turn right
            self.get_logger().info('Obstacle detected! Turning right.')
        else:
            # Move forward
            cmd_vel.linear.x = 0.3
            cmd_vel.angular.z = 0.0
            self.get_logger().info('Path clear! Moving forward.')

        # Publish command
        self.publisher.publish(cmd_vel)

def main(args=None):
    rclpy.init(args=args)
    ai_agent = SimpleAIAgent()

    try:
        rclpy.spin(ai_agent)
    except KeyboardInterrupt:
        pass
    finally:
        ai_agent.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Advanced AI Agent Integration

For more sophisticated AI agents, we might want to integrate with machine learning models or complex planning algorithms:

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, LaserScan
from geometry_msgs.msg import Twist, Pose
from nav_msgs.msg import Odometry
from std_msgs.msg import String
import numpy as np
import cv2
from cv2 import cv2
from typing import List, Tuple

class AdvancedAIAgent(Node):
    def __init__(self):
        super().__init__('advanced_ai_agent')

        # Subscriptions
        self.image_subscription = self.create_subscription(
            Image, 'camera/image_raw', self.image_callback, 10)
        self.scan_subscription = self.create_subscription(
            LaserScan, 'scan', self.scan_callback, 10)
        self.odom_subscription = self.create_subscription(
            Odometry, 'odom', self.odom_callback, 10)

        # Publishers
        self.cmd_publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.status_publisher = self.create_publisher(String, 'ai_status', 10)

        # AI agent state
        self.latest_image = None
        self.latest_scan = None
        self.current_pose = None
        self.ai_state = 'IDLE'  # IDLE, NAVIGATING, MANIPULATING, etc.

        # AI decision timer
        self.ai_timer = self.create_timer(0.1, self.ai_decision_callback)

        self.get_logger().info('Advanced AI Agent initialized')

    def image_callback(self, msg):
        """Process camera images for object detection"""
        # Convert ROS Image to OpenCV format
        try:
            # Assuming the image is in RGB8 format
            height = msg.height
            width = msg.width
            data = np.frombuffer(msg.data, dtype=np.uint8)
            image = data.reshape((height, width, 3))

            self.latest_image = image
            self.process_image(image)
        except Exception as e:
            self.get_logger().error(f'Error processing image: {e}')

    def scan_callback(self, msg):
        """Process laser scan data"""
        self.latest_scan = msg

    def odom_callback(self, msg):
        """Process odometry data"""
        self.current_pose = msg.pose.pose

    def process_image(self, image):
        """AI-based image processing"""
        # Example: Simple color-based object detection
        # Convert BGR to HSV for better color detection
        hsv = cv2.cvtColor(image, cv2.COLOR_RGB2HSV)

        # Define range for red color
        lower_red = np.array([0, 50, 50])
        upper_red = np.array([10, 255, 255])

        # Create mask for red objects
        mask = cv2.inRange(hsv, lower_red, upper_red)

        # Find contours
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        if contours:
            # Find the largest contour (assuming it's the target object)
            largest_contour = max(contours, key=cv2.contourArea)
            if cv2.contourArea(largest_contour) > 100:  # Minimum area threshold
                # Calculate center of the object
                M = cv2.moments(largest_contour)
                if M["m00"] != 0:
                    cx = int(M["m10"] / M["m00"])
                    cy = int(M["m01"] / M["m00"])

                    # Determine if object is in the center third of the image
                    img_width = image.shape[1]
                    center_region = (img_width // 3, 2 * img_width // 3)

                    if center_region[0] < cx < center_region[1]:
                        self.get_logger().info('Target object centered!')
                        self.ai_state = 'TARGET_CENTERED'
                    else:
                        self.get_logger().info('Target object detected but not centered')
                        self.ai_state = 'TARGET_DETECTED'
                else:
                    self.ai_state = 'SEARCHING'
            else:
                self.ai_state = 'SEARCHING'
        else:
            self.ai_state = 'SEARCHING'

    def ai_decision_callback(self):
        """Main AI decision-making function"""
        if self.ai_state == 'SEARCHING':
            self.search_behavior()
        elif self.ai_state == 'TARGET_DETECTED':
            self.approach_behavior()
        elif self.ai_state == 'TARGET_CENTERED':
            self.approach_behavior()  # Continue approaching when centered

        # Publish AI status
        status_msg = String()
        status_msg.data = f"State: {self.ai_state}, Position: ({self.current_pose.position.x:.2f}, {self.current_pose.position.y:.2f})" if self.current_pose else f"State: {self.ai_state}"
        self.status_publisher.publish(status_msg)

    def search_behavior(self):
        """Behavior when searching for objects"""
        cmd_vel = Twist()
        cmd_vel.linear.x = 0.1  # Slow forward movement
        cmd_vel.angular.z = 0.3  # Gentle rotation to scan
        self.cmd_publisher.publish(cmd_vel)

    def approach_behavior(self):
        """Behavior when approaching detected object"""
        cmd_vel = Twist()

        # Check scan data to avoid obstacles
        if self.latest_scan and min(self.latest_scan.ranges) < 0.5:  # 0.5m safety distance
            cmd_vel.linear.x = 0.0  # Stop if too close to obstacle
            cmd_vel.angular.z = 0.1  # Gentle turn to avoid
            self.get_logger().warn('Approach stopped - obstacle detected!')
        else:
            cmd_vel.linear.x = 0.2  # Approach slowly
            cmd_vel.angular.z = 0.0  # Go straight

        self.cmd_publisher.publish(cmd_vel)

def main(args=None):
    rclpy.init(args=args)
    ai_agent = AdvancedAIAgent()

    try:
        rclpy.spin(ai_agent)
    except KeyboardInterrupt:
        pass
    finally:
        ai_agent.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Designing Effective Communication Patterns

### Publisher-Subscriber for Continuous Data

For continuous sensor data and robot state, use the publisher-subscriber pattern:

```python
class SensorDataProcessor(Node):
    def __init__(self):
        super().__init__('sensor_data_processor')

        # Subscribe to multiple sensor streams
        self.camera_sub = self.create_subscription(
            Image, 'camera/image_raw', self.camera_callback, 10)
        self.scan_sub = self.create_subscription(
            LaserScan, 'scan', self.scan_callback, 10)
        self.imu_sub = self.create_subscription(
            Imu, 'imu/data', self.imu_callback, 10)

        # Publish processed data
        self.ai_input_pub = self.create_publisher(
            String, 'ai_input_data', 10)

    def camera_callback(self, msg):
        # Process camera data and publish for AI
        pass

    def scan_callback(self, msg):
        # Process scan data and publish for AI
        pass

    def imu_callback(self, msg):
        # Process IMU data and publish for AI
        pass
```

### Service-Based Communication for Discrete Actions

For discrete actions that require confirmation, use services:

```python
from example_interfaces.srv import Trigger

class ActionPlanner(Node):
    def __init__(self):
        super().__init__('action_planner')

        # Service for requesting specific actions
        self.action_service = self.create_service(
            Trigger, 'request_action', self.handle_action_request)

        # Client for calling controller services
        self.move_client = self.create_client(Trigger, 'move_to_pose')

    def handle_action_request(self, request, response):
        """Handle high-level action requests"""
        # Plan and execute action
        success = self.execute_planned_action()

        response.success = success
        response.message = "Action completed" if success else "Action failed"

        return response

    def execute_planned_action(self):
        """Execute the planned action sequence"""
        # Implementation details
        return True
```

## Bridge Node Implementation

Let's create a comprehensive bridge node that connects AI agents to controllers:

```python
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import String, Bool
from geometry_msgs.msg import Twist, PoseStamped
from sensor_msgs.msg import LaserScan, Image
from nav_msgs.msg import Odometry
from move_base_msgs.action import MoveBase
from actionlib_msgs.msg import GoalStatus

class AIBridgeNode(Node):
    def __init__(self):
        super().__init__('ai_bridge_node')

        # Callback group for handling multiple callbacks
        self.callback_group = ReentrantCallbackGroup()

        # AI agent interface
        self.ai_command_sub = self.create_subscription(
            String, 'ai_commands', self.ai_command_callback, 10)
        self.ai_status_pub = self.create_publisher(
            String, 'ai_status', 10)

        # Robot state interface
        self.odom_sub = self.create_subscription(
            Odometry, 'odom', self.odom_callback, 10)
        self.scan_sub = self.create_subscription(
            LaserScan, 'scan', self.scan_callback, 10)

        # Control interface
        self.cmd_vel_pub = self.create_publisher(
            Twist, 'cmd_vel', 10)

        # Action interface for navigation
        self.nav_client = ActionClient(
            self, MoveBase, 'move_base')

        # State variables
        self.current_pose = None
        self.scan_data = None
        self.ai_active = True

        self.get_logger().info('AI Bridge Node initialized')

    def ai_command_callback(self, msg):
        """Process commands from AI agent"""
        command = msg.data.lower()

        if command == 'stop':
            self.stop_robot()
        elif command.startswith('move_to:'):
            # Extract coordinates: move_to:x,y
            try:
                coords_str = command.split(':')[1]
                x, y = map(float, coords_str.split(','))
                self.navigate_to(x, y)
            except ValueError:
                self.get_logger().error(f'Invalid move command format: {command}')
        elif command.startswith('velocity:'):
            # Extract velocity: velocity:linear,angular
            try:
                vel_str = command.split(':')[1]
                linear, angular = map(float, vel_str.split(','))
                self.set_velocity(linear, angular)
            except ValueError:
                self.get_logger().error(f'Invalid velocity command format: {command}')
        else:
            self.get_logger().warn(f'Unknown AI command: {command}')

    def odom_callback(self, msg):
        """Update current robot pose"""
        self.current_pose = msg.pose.pose

    def scan_callback(self, msg):
        """Update scan data"""
        self.scan_data = msg

    def stop_robot(self):
        """Stop the robot"""
        cmd_vel = Twist()
        cmd_vel.linear.x = 0.0
        cmd_vel.angular.z = 0.0
        self.cmd_vel_pub.publish(cmd_vel)
        self.get_logger().info('Robot stopped')

    def set_velocity(self, linear, angular):
        """Set robot velocity"""
        cmd_vel = Twist()
        cmd_vel.linear.x = linear
        cmd_vel.angular.z = angular
        self.cmd_vel_pub.publish(cmd_vel)
        self.get_logger().info(f'Set velocity: linear={linear}, angular={angular}')

    def navigate_to(self, x, y):
        """Navigate to specified coordinates"""
        if not self.nav_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().error('Navigation server not available')
            return False

        goal_msg = MoveBase.Goal()
        goal_msg.target_pose.header.frame_id = 'map'
        goal_msg.target_pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.target_pose.pose.position.x = x
        goal_msg.target_pose.pose.position.y = y
        goal_msg.target_pose.pose.orientation.w = 1.0  # No rotation

        # Send navigation goal
        self.nav_client.send_goal_async(
            goal_msg,
            feedback_callback=self.nav_feedback_callback
        )

        self.get_logger().info(f'Navigating to ({x}, {y})')
        return True

    def nav_feedback_callback(self, feedback_msg):
        """Handle navigation feedback"""
        current_pose = feedback_msg.feedback.base_position
        self.get_logger().info(f'Navigating... Current position: ({current_pose.position.x:.2f}, {current_pose.position.y:.2f})')

    def publish_status(self, status):
        """Publish AI status"""
        status_msg = String()
        status_msg.data = status
        self.ai_status_pub.publish(status_msg)

def main(args=None):
    rclpy.init(args=args)

    bridge_node = AIBridgeNode()

    # Use multi-threaded executor to handle multiple callbacks
    executor = MultiThreadedExecutor()
    executor.add_node(bridge_node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        bridge_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Integrating with Popular AI Libraries

### TensorFlow/Keras Integration

```python
import tensorflow as tf
import numpy as np
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist

class TensorFlowAIBridge(Node):
    def __init__(self):
        super().__init__('tensorflow_ai_bridge')

        # Load pre-trained model
        self.model = tf.keras.models.load_model('path/to/your/model.h5')

        # Subscribe to camera data
        self.image_sub = self.create_subscription(
            Image, 'camera/image_raw', self.image_callback, 10)

        # Publish commands
        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)

        self.get_logger().info('TensorFlow AI Bridge initialized')

    def image_callback(self, msg):
        """Process image and make AI decision"""
        # Convert ROS Image to numpy array
        image = self.ros_image_to_numpy(msg)

        # Preprocess image for the model
        processed_image = self.preprocess_image(image)

        # Make prediction
        prediction = self.model.predict(np.expand_dims(processed_image, axis=0))

        # Convert prediction to robot command
        cmd_vel = self.prediction_to_command(prediction)

        # Publish command
        self.cmd_pub.publish(cmd_vel)

    def ros_image_to_numpy(self, msg):
        """Convert ROS Image message to numpy array"""
        # Implementation depends on image encoding
        pass

    def preprocess_image(self, image):
        """Preprocess image for the model"""
        # Resize, normalize, etc.
        pass

    def prediction_to_command(self, prediction):
        """Convert model prediction to Twist command"""
        cmd_vel = Twist()
        # Interpret prediction and set appropriate velocities
        pass
```

### PyTorch Integration

```python
import torch
import torchvision.transforms as transforms
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist

class PyTorchAIBridge(Node):
    def __init__(self):
        super().__init__('pytorch_ai_bridge')

        # Load pre-trained model
        self.model = torch.load('path/to/your/model.pth')
        self.model.eval()  # Set to evaluation mode

        # Image preprocessing
        self.transform = transforms.Compose([
            transforms.ToPILImage(),
            transforms.Resize((224, 224)),
            transforms.ToTensor(),
            transforms.Normalize(mean=[0.485, 0.456, 0.406],
                               std=[0.229, 0.224, 0.225])
        ])

        # ROS interfaces
        self.image_sub = self.create_subscription(
            Image, 'camera/image_raw', self.image_callback, 10)
        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)

        self.get_logger().info('PyTorch AI Bridge initialized')

    def image_callback(self, msg):
        """Process image and make AI decision"""
        # Convert ROS Image to PIL Image
        pil_image = self.ros_image_to_pil(msg)

        # Preprocess image
        input_tensor = self.transform(pil_image).unsqueeze(0)

        # Make prediction
        with torch.no_grad():
            prediction = self.model(input_tensor)

        # Convert to command
        cmd_vel = self.prediction_to_command(prediction)

        # Publish command
        self.cmd_pub.publish(cmd_vel)

    def ros_image_to_pil(self, msg):
        """Convert ROS Image to PIL Image"""
        # Implementation depends on image encoding
        pass

    def prediction_to_command(self, prediction):
        """Convert model output to Twist command"""
        cmd_vel = Twist()
        # Interpret prediction and set appropriate velocities
        return cmd_vel
```

## Performance Optimization

### Threading Considerations

For high-performance AI-ROS integration, consider using separate threads for AI processing:

```python
import threading
import queue
from rclpy.node import Node

class ThreadingAIBridge(Node):
    def __init__(self):
        super().__init__('threading_ai_bridge')

        # Queues for thread communication
        self.sensor_queue = queue.Queue(maxsize=10)
        self.command_queue = queue.Queue(maxsize=10)

        # ROS interfaces
        self.sensor_sub = self.create_subscription(
            LaserScan, 'scan', self.sensor_callback, 10)
        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)

        # Start AI processing thread
        self.ai_thread = threading.Thread(target=self.ai_processing_loop)
        self.ai_thread.daemon = True
        self.ai_thread.start()

        self.get_logger().info('Threading AI Bridge initialized')

    def sensor_callback(self, msg):
        """Add sensor data to processing queue"""
        try:
            self.sensor_queue.put_nowait(msg)
        except queue.Full:
            # Queue is full, drop oldest data
            try:
                self.sensor_queue.get_nowait()
                self.sensor_queue.put_nowait(msg)
            except queue.Empty:
                pass  # Shouldn't happen

    def ai_processing_loop(self):
        """AI processing in separate thread"""
        while rclpy.ok():
            try:
                # Get sensor data from queue
                sensor_data = self.sensor_queue.get(timeout=0.1)

                # Process with AI
                command = self.process_with_ai(sensor_data)

                # Put command in output queue
                try:
                    self.command_queue.put_nowait(command)
                except queue.Full:
                    # Command queue full, drop this command
                    pass

            except queue.Empty:
                continue  # No data to process

    def process_with_ai(self, sensor_data):
        """AI processing logic"""
        # Implementation of AI decision making
        cmd_vel = Twist()
        # ... AI logic here ...
        return cmd_vel

    def timer_callback(self):
        """Main thread callback to publish commands"""
        try:
            command = self.command_queue.get_nowait()
            self.cmd_pub.publish(command)
        except queue.Empty:
            pass  # No commands to publish
```

## Error Handling and Robustness

### Graceful Degradation

Implement error handling to ensure the system continues to operate safely:

```python
class RobustAIBridge(Node):
    def __init__(self):
        super().__init__('robust_ai_bridge')

        # ROS interfaces
        self.scan_sub = self.create_subscription(
            LaserScan, 'scan', self.scan_callback, 10)
        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)

        # Fallback timer for safety
        self.safety_timer = self.create_timer(0.1, self.safety_callback)

        # State variables
        self.last_scan_time = self.get_clock().now()
        self.ai_operational = True
        self.fallback_active = False

        self.get_logger().info('Robust AI Bridge initialized')

    def scan_callback(self, msg):
        """Process scan data with error handling"""
        try:
            self.last_scan_time = self.get_clock().now()
            self.process_scan_safely(msg)
        except Exception as e:
            self.get_logger().error(f'Scan processing error: {e}')
            self.activate_fallback()

    def process_scan_safely(self, msg):
        """AI processing with error handling"""
        try:
            # AI processing logic
            cmd_vel = self.ai_decision_function(msg)
            self.cmd_pub.publish(cmd_vel)
            self.ai_operational = True
            self.fallback_active = False
        except Exception as e:
            self.get_logger().error(f'AI decision error: {e}')
            self.activate_fallback()

    def safety_callback(self):
        """Safety check timer"""
        current_time = self.get_clock().now()
        time_since_last_scan = (current_time - self.last_scan_time).nanoseconds / 1e9

        if time_since_last_scan > 1.0:  # No scan for 1 second
            self.get_logger().warn('No scan data received, activating safety mode')
            self.stop_robot()
        elif self.fallback_active:
            self.execute_fallback_behavior()

    def activate_fallback(self):
        """Activate safe fallback behavior"""
        self.fallback_active = True
        self.get_logger().warn('Activated fallback behavior')

        # Stop robot and wait for normal operation to resume
        self.stop_robot()

    def execute_fallback_behavior(self):
        """Execute safe fallback behavior"""
        cmd_vel = Twist()
        # Implement safe behavior (e.g., stop, or very slow movement)
        cmd_vel.linear.x = 0.0
        cmd_vel.angular.z = 0.0
        self.cmd_pub.publish(cmd_vel)

    def stop_robot(self):
        """Emergency stop"""
        cmd_vel = Twist()
        cmd_vel.linear.x = 0.0
        cmd_vel.angular.z = 0.0
        self.cmd_pub.publish(cmd_vel)
```

## Debugging and Monitoring

### Logging and Diagnostics

```python
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus

class DiagnosticAIBridge(Node):
    def __init__(self):
        super().__init__('diagnostic_ai_bridge')

        # Standard interfaces
        self.scan_sub = self.create_subscription(
            LaserScan, 'scan', self.scan_callback, 10)
        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)

        # Diagnostic publisher
        qos_profile = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE)
        self.diag_pub = self.create_publisher(DiagnosticArray, '/diagnostics', qos_profile)

        # Diagnostic timer
        self.diag_timer = self.create_timer(1.0, self.publish_diagnostics)

        # Statistics
        self.scan_count = 0
        self.cmd_count = 0
        self.ai_error_count = 0
        self.last_scan_time = self.get_clock().now()

    def scan_callback(self, msg):
        """Process scan with statistics"""
        self.scan_count += 1
        self.last_scan_time = self.get_clock().now()

        try:
            cmd_vel = self.ai_decision_function(msg)
            self.cmd_pub.publish(cmd_vel)
            self.cmd_count += 1
        except Exception as e:
            self.ai_error_count += 1
            self.get_logger().error(f'AI error: {e}')

    def publish_diagnostics(self):
        """Publish diagnostic information"""
        diag_array = DiagnosticArray()
        diag_array.header.stamp = self.get_clock().now().to_msg()

        # System status
        status = DiagnosticStatus()
        status.name = 'AI Bridge Node'
        status.level = DiagnosticStatus.OK
        status.message = 'Running normally'

        # Add key-value pairs for statistics
        status.values.extend([
            {'key': 'Scan Messages', 'value': str(self.scan_count)},
            {'key': 'Command Messages', 'value': str(self.cmd_count)},
            {'key': 'AI Errors', 'value': str(self.ai_error_count)},
            {'key': 'Last Scan Age (s)', 'value': f'{(self.get_clock().now() - self.last_scan_time).nanoseconds / 1e9:.2f}'}
        ])

        diag_array.status.append(status)
        self.diag_pub.publish(diag_array)
```

## Chapter Summary

Bridging Python AI agents to ROS controllers using rclpy is a crucial skill for modern robotics development. This integration enables high-level AI decision-making to control low-level robot behaviors effectively.

**Key Takeaways:**
1. **rclpy** provides Python access to all ROS 2 capabilities
2. **Communication patterns** should match the type of interaction (topics for continuous data, services for discrete actions)
3. **Threading** can improve performance for computationally intensive AI processing
4. **Error handling** is essential for safe robot operation
5. **Diagnostics** help monitor and debug AI-ROS integration

## Check Your Understanding

1. **Conceptual**: Why is it important to have a bridge between AI agents and ROS controllers rather than implementing everything in one system?

2. **Application**: Design a communication pattern for a robot that needs to:
   - Navigate to locations specified by a high-level planner
   - Detect and avoid obstacles in real-time
   - Manipulate objects based on visual input

3. **Analysis**: What are the potential failure modes when bridging AI agents to controllers, and how would you handle them?

## Next Steps

In the next chapter, we'll explore URDF (Unified Robot Description Format), which is essential for describing robot structures and is often used in conjunction with AI-controlled robots.

---

**Reflection Question**: Consider a robot that needs to navigate through a building to deliver packages. How would the AI agent, bridge node, and controller work together to accomplish this task? What information would flow between each component?