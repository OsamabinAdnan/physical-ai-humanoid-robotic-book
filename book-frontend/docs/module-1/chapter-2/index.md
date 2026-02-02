---
sidebar_position: 3
title: Chapter 2 | ROS 2 Nodes, Topics, and Services
---

# Chapter 2: ROS 2 Nodes, Topics, and Services

In this chapter, we'll dive deeper into the three fundamental communication patterns that make up the ROS 2 ecosystem: nodes, topics, and services. These concepts are the building blocks of any ROS 2 system and understanding them is crucial for developing effective robot applications.

## Learning Objectives

By the end of this chapter, you will be able to:
- Create and manage ROS 2 nodes with proper lifecycle management
- Implement publisher and subscriber patterns using topics
- Design and use service-based communication between nodes
- Understand when to use topics vs. services for different scenarios
- Debug communication issues between ROS 2 components

## Understanding ROS 2 Nodes

Think of a node as an individual process that performs a specific task within your robot system. If we continue with our nervous system analogy, nodes are like specialized organs - each with its own function but working together as part of a larger system.

### What is a Node?

A node is the fundamental unit of computation in ROS 2. It can:
- Publish messages to topics
- Subscribe to messages from topics
- Provide services
- Call services provided by other nodes
- Execute specific robot functions

### Creating Your First Node

Let's start with a simple node that performs a basic function:

```python
# minimal_publisher.py
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class MinimalPublisher(Node):
    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher = self.create_publisher(String, 'topic', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = String()
        msg.data = 'Hello World: %d' % self.i
        self.publisher.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)
        self.i += 1

def main(args=None):
    rclpy.init(args=args)
    minimal_publisher = MinimalPublisher()
    rclpy.spin(minimal_publisher)
    minimal_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Node Lifecycle

Understanding the lifecycle of a node is important for proper resource management:

1. **Initialization**: The node is created and configured
2. **Activation**: The node becomes active and can communicate
3. **Execution**: The node performs its assigned tasks
4. **Shutdown**: The node cleans up resources and terminates

### Node Parameters

Nodes can accept parameters to customize their behavior:

```python
class ParameterNode(Node):
    def __init__(self):
        super().__init__('parameter_node')

        # Declare parameters with default values
        self.declare_parameter('robot_name', 'turtlebot')
        self.declare_parameter('max_velocity', 1.0)

        # Get parameter values
        robot_name = self.get_parameter('robot_name').value
        max_velocity = self.get_parameter('max_velocity').value

        self.get_logger().info(f'Robot: {robot_name}, Max velocity: {max_velocity}')
```

## Topics: The Publish-Subscribe Pattern

Topics implement a publish-subscribe communication pattern, which is ideal for continuous data streams like sensor readings or robot state information.

### How Topics Work

Think of topics like a radio station broadcast. The publisher sends out information, and any number of subscribers can "tune in" to receive that information.

**Key Characteristics:**
- **Asynchronous**: Publishers and subscribers don't need to be synchronized
- **Many-to-many**: Multiple publishers can publish to the same topic; multiple subscribers can subscribe to the same topic
- **Data-driven**: Communication is triggered by data availability

### Publisher Example

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

class SensorPublisher(Node):
    def __init__(self):
        super().__init__('sensor_publisher')

        # Create publisher for laser scan data
        self.scan_publisher = self.create_publisher(LaserScan, 'scan', 10)

        # Timer to simulate sensor data
        self.timer = self.create_timer(0.1, self.publish_scan_data)

    def publish_scan_data(self):
        msg = LaserScan()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'laser_frame'

        # Fill in laser scan parameters
        msg.angle_min = -1.57  # -90 degrees
        msg.angle_max = 1.57   # 90 degrees
        msg.angle_increment = 0.0174  # 1 degree
        msg.range_min = 0.1
        msg.range_max = 10.0

        # Simulate range data
        msg.ranges = [2.0] * 181  # 181 readings

        self.scan_publisher.publish(msg)
```

### Subscriber Example

```python
class SensorSubscriber(Node):
    def __init__(self):
        super().__init__('sensor_subscriber')

        # Create subscriber for laser scan data
        self.subscription = self.create_subscription(
            LaserScan,
            'scan',
            self.scan_callback,
            10)
        self.subscription  # prevent unused variable warning

    def scan_callback(self, msg):
        # Process the laser scan data
        min_range = min(msg.ranges) if msg.ranges else float('inf')

        if min_range < 1.0:  # Obstacle within 1 meter
            self.get_logger().warn('Obstacle detected! Distance: %.2f' % min_range)
        else:
            self.get_logger().info('Clear path, min distance: %.2f' % min_range)
```

### Topic Best Practices

1. **Use Descriptive Names**: `/robot1/sensors/laser_scan` is better than `/scan`
2. **Follow Naming Conventions**: Use lowercase with underscores
3. **Consider Message Frequency**: Balance between responsiveness and bandwidth
4. **Handle Message Queues**: Configure queue sizes appropriately

## Services: The Request-Response Pattern

Services implement a request-response communication pattern, which is ideal for actions that require a specific result or confirmation.

### How Services Work

Think of services like asking a question and waiting for a specific answer. You make a request, the service processes it, and returns a response.

**Key Characteristics:**
- **Synchronous**: The client waits for the response
- **One-to-one**: One client requests from one server (though multiple clients can request)
- **Action-oriented**: Communication is triggered by specific actions

### Service Definition

Services require a service definition file (`.srv`). Here's an example:

```
# GetDistance.srv
float64 x
float64 y
---
float64 distance
```

This defines a service that takes two coordinates (x, y) and returns the distance from the origin.

### Service Server Example

```python
from example_interfaces.srv import AddTwoInts
import rclpy
from rclpy.node import Node

class MinimalService(Node):
    def __init__(self):
        super().__init__('minimal_service')
        self.srv = self.create_service(
            AddTwoInts,
            'add_two_ints',
            self.add_two_ints_callback)

    def add_two_ints_callback(self, request, response):
        response.sum = request.a + request.b
        self.get_logger().info('Incoming request\na: %d b: %d' % (request.a, request.b))
        return response

def main(args=None):
    rclpy.init(args=args)
    minimal_service = MinimalService()
    rclpy.spin(minimal_service)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Service Client Example

```python
from example_interfaces.srv import AddTwoInts
import rclpy
from rclpy.node import Node

class MinimalClient(Node):
    def __init__(self):
        super().__init__('minimal_client')
        self.cli = self.create_client(AddTwoInts, 'add_two_ints')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting again...')
        self.req = AddTwoInts.Request()

    def send_request(self, a, b):
        self.req.a = a
        self.req.b = b
        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()

def main(args=None):
    rclpy.init(args=args)
    minimal_client = MinimalClient()
    response = minimal_client.send_request(2, 3)
    minimal_client.get_logger().info('Result of add_two_ints: %d' % response.sum)
    minimal_client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Service Best Practices

1. **Use for Actionable Requests**: When you need a specific result
2. **Consider Timeouts**: Always implement timeout handling
3. **Design Idempotent Services**: When possible, the same request should produce the same result
4. **Handle Errors Gracefully**: Return appropriate error codes

## When to Use Topics vs. Services

Understanding when to use each communication pattern is crucial for effective ROS 2 design:

### Use Topics When:
- **Continuous Data Flow**: Sensor data, robot state, camera feeds
- **One-to-Many Communication**: Broadcasting to multiple subscribers
- **Real-time Requirements**: When latency matters more than confirmation
- **Event Notifications**: Broadcasting that something happened

### Use Services When:
- **Action Confirmation**: Need to confirm that an action completed
- **Query Operations**: Requesting specific information
- **One-to-One Communication**: Direct request-response pattern
- **Critical Operations**: When you must know if the operation succeeded

### Practical Example: Robot Navigation

Let's consider a navigation scenario to see both patterns in action:

**Topics in Navigation:**
- `/cmd_vel`: Continuous velocity commands to the robot
- `/scan`: Continuous laser scan data for obstacle detection
- `/tf`: Continuous transformation data for robot pose
- `/odom`: Continuous odometry data

**Services in Navigation:**
- `/navigate_to_pose`: Request to navigate to a specific location
- `/get_path`: Request to plan a path to a goal
- `/cancel_goal`: Request to cancel current navigation

## Advanced Communication Patterns

### Actions

For long-running tasks with feedback, ROS 2 provides Actions, which combine the best of both topics and services:

```python
from rclpy.action import ActionClient
from rclpy.node import Node
from nav2_msgs.action import NavigateToPose

class NavigateActionClient(Node):
    def __init__(self):
        super().__init__('navigate_action_client')
        self._action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

    def send_goal(self, pose):
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = pose

        self._action_client.wait_for_server()
        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback)

        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info(f'Remaining distance: {feedback.distance_remaining}')
```

### Quality of Service (QoS) Settings

QoS settings allow you to customize communication behavior:

```python
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

# For reliable, persistent communication
qos_profile = QoSProfile(
    depth=10,
    reliability=ReliabilityPolicy.RELIABLE,
    history=HistoryPolicy.KEEP_LAST
)

# For best-effort, high-frequency data
fast_qos_profile = QoSProfile(
    depth=1,
    reliability=ReliabilityPolicy.BEST_EFFORT,
    history=HistoryPolicy.KEEP_LAST
)
```

## Debugging Communication Issues

### Common Problems and Solutions

1. **Nodes Not Communicating**
   - Check if nodes are on the same ROS domain
   - Verify topic/service names match exactly
   - Ensure nodes are running and connected to ROS graph

2. **High Latency**
   - Check network connectivity for distributed systems
   - Review QoS settings for appropriate reliability
   - Monitor system resources (CPU, memory, bandwidth)

3. **Message Loss**
   - Increase queue sizes for high-frequency topics
   - Check for processing bottlenecks
   - Consider using reliable QoS policies

### Debugging Tools

```bash
# List all active topics
ros2 topic list

# Echo messages from a specific topic
ros2 topic echo /topic_name

# List all active services
ros2 service list

# Call a service
ros2 service call /service_name service_type "request_data"

# Show ROS graph
rqt_graph
```

## Practical Exercise: Building a Simple Robot System

Let's apply what we've learned by building a simple robot system with three nodes:

1. **Sensor Node**: Publishes simulated sensor data
2. **Controller Node**: Subscribes to sensor data and makes decisions
3. **Motor Node**: Provides a service to execute motor commands

### Step 1: Sensor Node

```python
# sensor_node.py
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Range
import random

class SensorNode(Node):
    def __init__(self):
        super().__init__('sensor_node')
        self.publisher = self.create_publisher(Range, 'distance_sensor', 10)
        self.timer = self.create_timer(0.5, self.publish_sensor_data)

    def publish_sensor_data(self):
        msg = Range()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'sensor_frame'
        msg.radiation_type = Range.ULTRASOUND
        msg.field_of_view = 0.1
        msg.min_range = 0.0
        msg.max_range = 5.0
        msg.range = random.uniform(0.5, 4.0)  # Random distance

        self.publisher.publish(msg)
        self.get_logger().info(f'Distance: {msg.range:.2f}m')
```

### Step 2: Controller Node

```python
# controller_node.py
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Range
from example_interfaces.srv import SetBool

class ControllerNode(Node):
    def __init__(self):
        super().__init__('controller_node')
        self.subscription = self.create_subscription(
            Range, 'distance_sensor', self.sensor_callback, 10)
        self.client = self.create_client(SetBool, 'motor_service')

    def sensor_callback(self, msg):
        if msg.range < 1.0:  # Too close to obstacle
            self.call_motor_service(False)  # Stop motor
            self.get_logger().warn('Obstacle detected! Stopping motor.')
        else:
            self.call_motor_service(True)   # Start motor
            self.get_logger().info('Path clear! Starting motor.')

    def call_motor_service(self, enable):
        if self.client.wait_for_service(timeout_sec=1.0):
            request = SetBool.Request()
            request.data = enable
            self.client.call_async(request)
        else:
            self.get_logger().error('Motor service not available')
```

### Step 3: Motor Node

```python
# motor_node.py
import rclpy
from rclpy.node import Node
from example_interfaces.srv import SetBool

class MotorNode(Node):
    def __init__(self):
        super().__init__('motor_node')
        self.srv = self.create_service(
            SetBool, 'motor_service', self.motor_callback)
        self.motor_enabled = False

    def motor_callback(self, request, response):
        self.motor_enabled = request.data
        if self.motor_enabled:
            self.get_logger().info('Motor ENABLED')
            response.success = True
            response.message = 'Motor started'
        else:
            self.get_logger().info('Motor DISABLED')
            response.success = True
            response.message = 'Motor stopped'
        return response
```

## Chapter Summary

ROS 2 nodes, topics, and services form the fundamental communication architecture that enables complex robot systems. Understanding these patterns is essential for effective robotics development.

**Key Takeaways:**
1. **Nodes** are the basic computational units that perform specific tasks
2. **Topics** enable asynchronous, publish-subscribe communication for continuous data streams
3. **Services** enable synchronous, request-response communication for specific actions
4. Choose the right communication pattern based on your application's requirements
5. Use debugging tools to verify and troubleshoot communication

## Check Your Understanding

1. **Conceptual**: Explain the difference between topics and services in your own words
2. **Application**: For each of these robot functions, decide whether to use topics or services:
   - Publishing camera images
   - Requesting a map from a mapping service
   - Sending velocity commands to a robot
   - Requesting robot battery status
3. **Analysis**: Why might you use QoS settings in a robot system?

## Next Steps

In the next chapter, we'll explore how to bridge Python-based AI agents with ROS controllers using rclpy, building on the communication patterns we've learned about here.

---

**Reflection Question**: Consider a robot vacuum cleaner. Which of its functions would use topics vs. services? How would the different components communicate with each other?