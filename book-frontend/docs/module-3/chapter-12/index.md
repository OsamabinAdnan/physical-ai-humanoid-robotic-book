---
sidebar_position: 15
title: Chapter 12 | Nav2 | Path planning for bipedal humanoid movement
---

# Chapter 12: Nav2: Path Planning for Bipedal Humanoid Movement

In this chapter, we'll explore Navigation2 (Nav2) with a specific focus on path planning for bipedal humanoid robots. We'll examine how to adapt traditional navigation approaches for the unique challenges of humanoid locomotion and balance.

## Learning Objectives

By the end of this chapter, you will be able to:
- Configure Nav2 for bipedal humanoid robot navigation
- Implement specialized path planning algorithms for bipedal movement
- Address balance and stability constraints in navigation planning
- Integrate humanoid-specific kinematic constraints into navigation
- Validate navigation performance for bipedal locomotion

## Introduction to Nav2 for Humanoid Robots

Navigation2 (Nav2) is the latest navigation stack for ROS 2, designed for autonomous mobile robots. When applied to bipedal humanoid robots, it requires special considerations due to the unique challenges of two-legged locomotion.

### Traditional vs. Humanoid Navigation

#### Standard Mobile Robot Navigation
- Wheeled or tracked platforms
- Continuous motion capabilities
- Stable base of support
- Predictable kinematics

#### Bipedal Humanoid Navigation
- Dynamic balance requirements
- Discrete step-based movement
- Shifting center of mass
- Complex kinematic constraints

### Humanoid-Specific Navigation Challenges

#### Balance and Stability
- Maintaining center of mass within support polygon
- Dynamic balance during movement
- Recovery from perturbations
- Stable gait pattern maintenance

#### Kinematic Constraints
- Limited joint ranges of motion
- Foot placement constraints
- Leg swing trajectory planning
- Upper body stabilization requirements

#### Environmental Considerations
- Step height limitations
- Surface traversability
- Slope navigation capabilities
- Obstacle clearance requirements

## Nav2 Architecture Overview

### Core Components

#### Navigation Server
- Coordinates the navigation process
- Manages navigation state machine
- Handles action requests and responses
- Integrates all navigation components

#### Global Planner
- Computes optimal path from start to goal
- Considers global map and static obstacles
- Generates high-level navigation plan
- Accounts for humanoid-specific constraints

#### Local Planner
- Executes path following in real-time
- Handles dynamic obstacle avoidance
- Maintains robot stability
- Adjusts trajectory for balance

#### Controller Server
- Translates plans into robot commands
- Implements humanoid-specific controllers
- Manages gait patterns and stepping
- Ensures balance during execution

### Nav2 Behavior Trees

Nav2 uses behavior trees for flexible navigation execution:

```xml
<root main_tree_to_execute="MainTree">
    <BehaviorTree ID="MainTree">
        <RecoveryNode number_of_retries="6" name="NavigateRecovery">
            <PipelineSequence name="NavigateWithRecovery">
                <RateController hz="1.0">
                    <ReactiveGoalChecker name="GoalChecker"/>
                </RateController>
                <ClearEntirely name="ClearGlobalCostmap-Context" service_name="global_costmap/clear_entirely"/>
                <PipelineSequence name="GlobalPlanWithRecovery">
                    <RecoveryNode number_of_retries="1" name="ComputePathToPoseRecovery">
                        <PipelineSequence name="ComputePathToPose">
                            <PoseToPose name="GoalPoseDecay"/>
                            <ComputePathToPose name="ComputePathToPose"/>
                            <PoseToPose name="PathPoseDecay"/>
                        </PipelineSequence>
                        <BackUp name="BackUp"/>
                    </RecoveryNode>
                </PipelineSequence>
                <ClearEntirely name="ClearLocalCostmap-Context" service_name="local_costmap/clear_entirely"/>
                <PipelineSequence name="FollowPathWithRecovery">
                    <RecoveryNode number_of_retries="2" name="FollowPathRecovery">
                        <PipelineSequence name="FollowPath">
                            <FollowPath name="FollowPath"/>
                        </PipelineSequence>
                        <Spin name="Spin"/>
                    </RecoveryNode>
                </PipelineSequence>
            </PipelineSequence>
            <SmoothingAndGaitController name="SmoothingAndGaitController"/>
        </RecoveryNode>
    </BehaviorTree>
</root>
```

## Humanoid-Specific Path Planning

### Bipedal Motion Constraints

#### Step-Based Navigation
Bipedal robots navigate differently than wheeled robots:

```python
import math
from geometry_msgs.msg import Point, Pose
from nav_msgs.msg import Path

class BipedalPathPlanner:
    def __init__(self, step_length=0.3, step_width=0.2):
        self.step_length = step_length  # Distance between consecutive steps
        self.step_width = step_width    # Distance between left and right feet
        self.max_step_height = 0.1      # Maximum step height capability

    def plan_bipedal_path(self, global_path):
        """
        Convert continuous path to discrete bipedal steps
        """
        bipedal_steps = []

        # Convert global path to discrete footsteps
        for i in range(0, len(global_path.poses), self.get_step_interval()):
            step_pose = global_path.poses[i].pose

            # Ensure step is feasible for bipedal locomotion
            if self.is_step_feasible(step_pose):
                bipedal_steps.append(self.create_footstep(step_pose))

        return bipedal_steps

    def get_step_interval(self):
        """
        Calculate appropriate interval between steps based on robot capabilities
        """
        # This would depend on robot's walking speed and gait pattern
        return 2  # Every 2nd point from global path

    def is_step_feasible(self, pose):
        """
        Check if a step is feasible for bipedal locomotion
        """
        # Check for step height constraints
        if pose.position.z > self.max_step_height:
            return False

        # Check for surface stability
        if not self.is_surface_stable(pose):
            return False

        # Check for obstacle clearance
        if self.has_obstacle_clearance(pose):
            return False

        return True

    def create_footstep(self, pose):
        """
        Create a footstep from a pose
        """
        footstep = {
            'position': pose.position,
            'orientation': pose.orientation,
            'support_polygon': self.calculate_support_polygon(pose),
            'step_type': self.determine_step_type(pose)
        }
        return footstep
```

### Support Polygon and Balance Planning

#### Center of Mass Management
```python
class BalanceAwarePathPlanner:
    def __init__(self):
        self.robot_com_height = 0.8  # Center of mass height
        self.support_polygon_margin = 0.05  # Safety margin for support polygon

    def validate_balance_constraints(self, path):
        """
        Validate that path maintains balance throughout execution
        """
        for i, step in enumerate(path):
            # Calculate instantaneous support polygon
            support_poly = self.calculate_instantaneous_support_polygon(step)

            # Check if center of mass projection is within support polygon
            com_proj = self.project_com_onto_ground(step)

            if not self.point_in_polygon(com_proj, support_poly):
                # Path is dynamically unstable, need to replan
                return False, f"Balance violation at step {i}"

        return True, "Path is dynamically stable"

    def calculate_instantaneous_support_polygon(self, current_step):
        """
        Calculate support polygon based on current foot placement
        """
        # For single support (one foot down): point at foot center
        # For double support (both feet down): convex hull of both feet
        if current_step['phase'] == 'single_support':
            return [current_step['support_foot']['center']]
        else:  # double support
            return [
                current_step['left_foot']['center'],
                current_step['right_foot']['center']
            ]

    def project_com_onto_ground(self, step):
        """
        Project center of mass onto ground plane
        """
        # This would integrate with robot state and kinematics
        return Point(x=step['com_x'], y=step['com_y'], z=0.0)

    def point_in_polygon(self, point, polygon):
        """
        Check if point is inside polygon using ray casting algorithm
        """
        x, y = point.x, point.y
        n = len(polygon)
        inside = False

        p1x, p1y = polygon[0].x, polygon[0].y
        for i in range(1, n + 1):
            p2x, p2y = polygon[i % n].x, polygon[i % n].y
            if y > min(p1y, p2y):
                if y <= max(p1y, p2y):
                    if x <= max(p1x, p2x):
                        if p1y != p2y:
                            xinters = (y - p1y) * (p2x - p1x) / (p2y - p1y) + p1x
                        if p1x == p2x or x <= xinters:
                            inside = not inside
            p1x, p1y = p2x, p2y

        return inside
```

### Gait Pattern Integration

#### Walking Pattern Constraints
```python
class GaitPatternPathPlanner:
    def __init__(self):
        self.gait_patterns = {
            'walk': {
                'step_length_range': (0.2, 0.4),
                'step_width': 0.2,
                'step_frequency': (0.5, 2.0),
                'swing_height': 0.05
            },
            'trot': {
                'step_length_range': (0.3, 0.6),
                'step_width': 0.25,
                'step_frequency': (1.0, 3.0),
                'swing_height': 0.08
            },
            'crawl': {
                'step_length_range': (0.1, 0.2),
                'step_width': 0.15,
                'step_frequency': (0.2, 0.8),
                'swing_height': 0.03
            }
        }
        self.current_gait = 'walk'

    def adapt_path_to_gait(self, planned_path, gait_type='walk'):
        """
        Adapt planned path to conform to specific gait pattern
        """
        gait_params = self.gait_patterns[gait_type]

        # Adjust step parameters based on gait
        adapted_path = []
        for step in planned_path:
            adjusted_step = self.adjust_step_for_gait(step, gait_params)
            adapted_path.append(adjusted_step)

        return adapted_path

    def adjust_step_for_gait(self, step, gait_params):
        """
        Adjust step parameters to match gait constraints
        """
        # Ensure step length is within gait-specific range
        step_length = self.calculate_step_length(step)
        if step_length < gait_params['step_length_range'][0]:
            # Increase step length to minimum
            step = self.scale_step_to_minimum(step, gait_params['step_length_range'][0])
        elif step_length > gait_params['step_length_range'][1]:
            # Decrease step length to maximum
            step = self.scale_step_to_maximum(step, gait_params['step_length_range'][1])

        # Adjust step width if needed
        step['width'] = gait_params['step_width']

        # Set swing height for foot trajectory
        step['swing_height'] = gait_params['swing_height']

        return step
```

## Nav2 Configuration for Humanoid Robots

### Costmap Configuration

#### Global Costmap for Humanoid Navigation
```yaml
# global_costmap_params.yaml
global_costmap:
  global_frame: map
  robot_base_frame: base_link

  # Update frequency and resolution appropriate for humanoid
  update_frequency: 1.0
  publish_frequency: 1.0
  resolution: 0.05  # Higher resolution for precise footstep planning

  # Humanoid-specific inflation
  inflation:
    cost_scaling_factor: 3.0  # Increased for safety margin
    inflation_radius: 0.5     # Larger than typical wheeled robots

  # Obstacle layer with humanoid-specific parameters
  obstacle_layer:
    enabled: true
    observation_sources: scan
    scan:
      topic: /scan
      max_obstacle_height: 2.0    # Humanoid can potentially step over some obstacles
      clearing: true
      marking: true
      data_type: LaserScan
      raytrace_range: 3.0
      obstacle_range: 2.5

  # Additional layers for humanoid-specific considerations
  step_height_layer:
    enabled: true
    max_step_height: 0.15    # Maximum step height humanoid can handle
    min_traversable_height: 0.05  # Minimum clearance needed

  slope_layer:
    enabled: true
    max_slope_angle: 25.0    # Maximum slope angle in degrees
    slope_calculation_window: 3  # Number of cells to consider for slope calculation
```

#### Local Costmap for Humanoid Navigation
```yaml
# local_costmap_params.yaml
local_costmap:
  global_frame: odom
  robot_base_frame: base_link

  # Higher update frequency for dynamic balance
  update_frequency: 10.0
  publish_frequency: 5.0
  resolution: 0.025  # Very high resolution for precise foot placement

  # Smaller footprint reflecting support polygon
  footprint: [[-0.1, -0.1], [-0.1, 0.1], [0.1, 0.1], [0.1, -0.1]]
  footprint_padding: 0.02

  # Humanoid-specific inflation
  inflation:
    cost_scaling_factor: 5.0  # Much higher for safety during dynamic movement
    inflation_radius: 0.3     # Conservative inflation for stability

  # Obstacle layer tuned for humanoid
  obstacle_layer:
    enabled: true
    observation_sources: scan front_camera depth_camera
    scan:
      topic: /scan
      sensor_frame: base_scan
      observation_persistence: 0.0
      max_obstacle_height: 2.0
      min_obstacle_height: 0.0
      obstacle_range: 2.5
      raytrace_range: 3.0
      clearing: true
      marking: true
      data_type: LaserScan
      queue_size: 10
    front_camera:
      topic: /camera/depth/image_raw
      sensor_frame: camera_link
      observation_persistence: 0.0
      max_obstacle_height: 2.0
      min_obstacle_height: 0.0
      obstacle_range: 3.0
      raytrace_range: 4.0
      clearing: true
      marking: true
      data_type: Image
      queue_size: 5
```

### Controller Configuration for Bipedal Locomotion

#### Humanoid Controller Server
```yaml
# controller_server_params.yaml
controller_server:
  ros__parameters:
    controller_frequency: 20.0  # Higher frequency for balance control
    min_x_velocity_threshold: 0.01
    min_y_velocity_threshold: 0.01
    min_theta_velocity_threshold: 0.01

    # Humanoid-specific controllers
    progress_checker:
      plugin: "nav2_controller::SimpleProgressChecker"
      required_movement_radius: 0.5  # Adjusted for step-based movement
      movement_time_allowance: 10.0

    goal_checker:
      plugin: "nav2_controller::SimpleGoalChecker"
      xy_goal_tolerance: 0.15    # Larger tolerance for step-based positioning
      yaw_goal_tolerance: 0.2    # Tolerance for orientation (about 11.5 degrees)
      stateful: true

    # Humanoid-specific controller
    humanoid_trajectory_tracker:
      plugin: "nav2_humanoid_controllers::TrajectoryTracker"
      velocity_deadband: 0.05
      min_normalized_velocity: 0.1
      simulate_execution: false

      # Bipedal-specific parameters
      max_linear_speed: 0.3       # Conservative walking speed
      max_angular_speed: 0.5      # Turning speed appropriate for bipedal
      step_size: 0.3             # Desired step length
      step_offset: 0.15          # Offset between feet
      balance_margin: 0.05       # Safety margin for balance

    # Alternative controller plugins
    plugins: ["humanoid_trajectory_tracker"]
```

## Specialized Planners for Humanoid Navigation

### Footstep Planner

#### Discrete Footstep Planning Algorithm
```python
import numpy as np
from scipy.spatial import KDTree
import heapq

class FootstepPlanner:
    def __init__(self, step_length=0.3, step_width=0.15, max_step_height=0.15):
        self.step_length = step_length
        self.step_width = step_width
        self.max_step_height = max_step_height
        self.step_types = ['left', 'right']  # Alternating foot steps

    def plan_footsteps(self, start_pose, goal_pose, costmap):
        """
        Plan sequence of footsteps from start to goal
        """
        # Convert to footstep planning problem
        start_footsteps = self.initialize_start_footsteps(start_pose)
        goal_footsteps = self.initialize_goal_footsteps(goal_pose)

        # Use A* or Dijkstra's algorithm adapted for footsteps
        path = self.footstep_astar(start_footsteps, goal_footsteps, costmap)

        return path

    def initialize_start_footsteps(self, start_pose):
        """
        Initialize starting footsteps based on robot's current stance
        """
        # Assume robot starts with feet in standard position
        left_foot = {
            'x': start_pose.position.x,
            'y': start_pose.position.y + self.step_width/2,
            'theta': start_pose.orientation,
            'type': 'left'
        }

        right_foot = {
            'x': start_pose.position.x,
            'y': start_pose.position.y - self.step_width/2,
            'theta': start_pose.orientation,
            'type': 'right'
        }

        return [left_foot, right_foot]

    def footstep_astar(self, start_footsteps, goal_footsteps, costmap):
        """
        A* algorithm adapted for footstep planning
        """
        # Priority queue for A* search
        open_set = []

        # Start with initial footsteps
        start_node = {
            'footsteps': start_footsteps,
            'cost': 0,
            'heuristic': self.calculate_heuristic(start_footsteps, goal_footsteps),
            'total_cost': 0 + self.calculate_heuristic(start_footsteps, goal_footsteps)
        }

        heapq.heappush(open_set, (start_node['total_cost'], id(start_node), start_node))

        visited = set()

        while open_set:
            _, _, current_node = heapq.heappop(open_set)

            # Check if goal reached
            if self.is_goal_reached(current_node['footsteps'], goal_footsteps):
                return current_node['footsteps']

            # Generate successor footsteps
            successors = self.generate_successor_footsteps(current_node['footsteps'], costmap)

            for successor in successors:
                # Check if already visited
                successor_tuple = self.footsteps_to_tuple(successor)
                if successor_tuple in visited:
                    continue

                # Calculate cost and heuristic
                cost = current_node['cost'] + self.calculate_transition_cost(current_node['footsteps'], successor)
                heuristic = self.calculate_heuristic(successor, goal_footsteps)

                successor_node = {
                    'footsteps': successor,
                    'cost': cost,
                    'heuristic': heuristic,
                    'total_cost': cost + heuristic
                }

                heapq.heappush(open_set, (successor_node['total_cost'], id(successor_node), successor_node))
                visited.add(successor_tuple)

        # No path found
        return None

    def generate_successor_footsteps(self, current_footsteps, costmap):
        """
        Generate possible next footsteps from current configuration
        """
        successors = []

        # Determine which foot to move next (alternating pattern)
        next_foot_type = self.get_next_foot_type(current_footsteps)

        # Generate possible step locations
        possible_positions = self.generate_possible_step_positions(
            current_footsteps, next_foot_type, costmap
        )

        for pos in possible_positions:
            new_footsteps = self.create_new_footsteps(current_footsteps, next_foot_type, pos)

            # Validate step feasibility
            if self.is_step_feasible(new_footsteps, costmap):
                successors.append(new_footsteps)

        return successors

    def calculate_heuristic(self, footsteps, goal_footsteps):
        """
        Calculate heuristic cost to goal (Euclidean distance to goal region)
        """
        # Calculate distance to goal for the supporting foot
        if len(footsteps) >= 2:
            supporting_foot = footsteps[-1]  # Last placed foot
            goal_pos = goal_footsteps[0]     # Approximate goal position

            dist = np.sqrt(
                (supporting_foot['x'] - goal_pos['x'])**2 +
                (supporting_foot['y'] - goal_pos['y'])**2
            )
            return dist

        return float('inf')

    def is_goal_reached(self, footsteps, goal_footsteps):
        """
        Check if footsteps have reached the goal region
        """
        if len(footsteps) < 2:
            return False

        # Check if any foot is within goal region
        goal_pos = goal_footsteps[0]
        goal_tolerance = 0.2  # 20 cm tolerance

        for foot in footsteps:
            dist = np.sqrt(
                (foot['x'] - goal_pos['x'])**2 +
                (foot['y'] - goal_pos['y'])**2
            )
            if dist <= goal_tolerance:
                return True

        return False

    def is_step_feasible(self, footsteps, costmap):
        """
        Check if a footstep configuration is feasible
        """
        # Check each foot placement
        for foot in footsteps:
            if not self.is_foot_placement_safe(foot, costmap):
                return False

        # Check balance constraints
        if not self.is_balance_maintained(footsteps):
            return False

        # Check step height constraints
        if not self.is_step_height_acceptable(footsteps):
            return False

        return True

    def is_foot_placement_safe(self, foot, costmap):
        """
        Check if individual foot placement is safe
        """
        # This would integrate with costmap to check for obstacles
        # and terrain traversability
        return True  # Placeholder

    def is_balance_maintained(self, footsteps):
        """
        Check if footsteps maintain robot balance
        """
        # Calculate support polygon and check COM position
        return True  # Placeholder

    def is_step_height_acceptable(self, footsteps):
        """
        Check if step heights are within robot capabilities
        """
        return True  # Placeholder
```

### Balance-Aware Local Planner

#### Dynamic Balance Maintenance
```python
import math
import numpy as np
from geometry_msgs.msg import Twist
from nav_msgs.msg import Path
from builtin_interfaces.msg import Duration

class BalanceAwareLocalPlanner:
    def __init__(self):
        self.robot_mass = 50.0  # kg
        self.com_height = 0.8  # Center of mass height
        self.balance_margin = 0.05  # Safety margin

        # ZMP (Zero Moment Point) parameters
        self.zmp_window_size = 5  # Number of steps to consider for ZMP
        self.zmp_tolerance = 0.05  # Tolerance for ZMP stability

        # Walking parameters
        self.step_frequency = 1.0  # Steps per second
        self.swing_height = 0.05   # Height of foot swing

    def compute_velocity_commands(self, current_pose, current_velocity,
                                  local_plan, goal_checker):
        """
        Compute velocity commands while maintaining balance
        """
        # Calculate desired velocity based on path following
        desired_twist = self.calculate_path_following_velocity(
            current_pose, current_velocity, local_plan
        )

        # Check balance constraints
        balanced_twist = self.apply_balance_constraints(
            current_pose, desired_twist
        )

        # Validate against dynamic constraints
        final_twist = self.apply_dynamic_constraints(balanced_twist)

        return final_twist

    def calculate_path_following_velocity(self, current_pose, current_velocity, local_plan):
        """
        Calculate velocity to follow the local plan
        """
        if len(local_plan.poses) < 2:
            return Twist()  # Stop if no path

        # Calculate look-ahead distance based on current speed
        look_ahead_dist = max(0.3, abs(current_velocity.linear.x) * 1.5)

        # Find closest point on path
        closest_idx = self.find_closest_point(current_pose, local_plan)

        # Look ahead to find target point
        target_idx = self.find_look_ahead_point(
            current_pose, local_plan, closest_idx, look_ahead_dist
        )

        if target_idx is None:
            return Twist()  # Stop if no target found

        target_pose = local_plan.poses[target_idx]

        # Calculate desired direction and distance
        dx = target_pose.pose.position.x - current_pose.position.x
        dy = target_pose.pose.position.y - current_pose.position.y
        distance = math.sqrt(dx*dx + dy*dy)

        # Calculate desired linear velocity
        desired_linear = min(
            self.calculate_speed_profile(distance),
            self.max_linear_speed
        )

        # Calculate desired angular velocity
        desired_angular = self.calculate_angular_velocity(
            current_pose, target_pose, desired_linear
        )

        twist = Twist()
        twist.linear.x = desired_linear
        twist.angular.z = desired_angular

        return twist

    def apply_balance_constraints(self, current_pose, desired_twist):
        """
        Apply balance constraints to velocity commands
        """
        # Calculate current ZMP (simplified)
        current_zmp = self.calculate_current_zmp(current_pose, desired_twist)

        # Calculate support polygon based on foot positions
        support_polygon = self.calculate_support_polygon(current_pose)

        # Check if ZMP is within support polygon
        if not self.is_zmp_stable(current_zmp, support_polygon):
            # Reduce velocity to maintain balance
            reduction_factor = self.calculate_balance_reduction(
                current_zmp, support_polygon
            )

            reduced_twist = Twist()
            reduced_twist.linear.x = desired_twist.linear.x * reduction_factor
            reduced_twist.angular.z = desired_twist.angular.z * reduction_factor

            return reduced_twist

        return desired_twist

    def calculate_current_zmp(self, current_pose, current_twist):
        """
        Calculate current Zero Moment Point (simplified model)
        """
        # Simplified ZMP calculation
        # ZMP_x = CoM_x - (CoM_z / g) * CoM_acc_x
        # ZMP_y = CoM_y - (CoM_z / g) * CoM_acc_y

        # For now, return current position as approximation
        return (current_pose.position.x, current_pose.position.y)

    def calculate_support_polygon(self, current_pose):
        """
        Calculate support polygon based on current foot positions
        """
        # This would integrate with robot kinematics to get actual foot positions
        # For now, return a simple rectangular support polygon
        foot_separation = 0.2  # 20cm between feet

        support_points = [
            (current_pose.position.x - 0.1, current_pose.position.y + foot_separation/2),  # Front-left
            (current_pose.position.x - 0.1, current_pose.position.y - foot_separation/2),  # Front-right
            (current_pose.position.x + 0.1, current_pose.position.y - foot_separation/2),  # Back-right
            (current_pose.position.x + 0.1, current_pose.position.y + foot_separation/2),  # Back-left
        ]

        return support_points

    def is_zmp_stable(self, zmp, support_polygon):
        """
        Check if ZMP is within support polygon
        """
        zmp_point = Point()
        zmp_point.x = zmp[0]
        zmp_point.y = zmp[1]

        return self.point_in_polygon(zmp_point, support_polygon)

    def calculate_balance_reduction(self, current_zmp, support_polygon):
        """
        Calculate reduction factor to bring ZMP back to stable region
        """
        # Calculate distance from ZMP to nearest support polygon edge
        min_distance = self.min_distance_to_polygon_edge(current_zmp, support_polygon)

        if min_distance < -self.balance_margin:
            # ZMP is outside stable region, reduce speed significantly
            return max(0.1, min_distance / -self.balance_margin)
        elif min_distance < self.balance_margin:
            # Close to edge, apply gradual reduction
            return 0.5 + 0.5 * (min_distance / self.balance_margin)
        else:
            # Within safe region, no reduction needed
            return 1.0

    def apply_dynamic_constraints(self, twist):
        """
        Apply dynamic constraints to velocity commands
        """
        constrained_twist = Twist()

        # Apply linear velocity limits
        constrained_twist.linear.x = max(
            -self.max_linear_speed,
            min(twist.linear.x, self.max_linear_speed)
        )

        # Apply angular velocity limits
        constrained_twist.angular.z = max(
            -self.max_angular_speed,
            min(twist.angular.z, self.max_angular_speed)
        )

        # Apply acceleration limits
        # This would integrate with actual acceleration limits

        return constrained_twist

    def find_closest_point(self, current_pose, local_plan):
        """
        Find the closest point on the local plan to current position
        """
        min_dist = float('inf')
        closest_idx = 0

        for i, pose in enumerate(local_plan.poses):
            dist = math.sqrt(
                (pose.pose.position.x - current_pose.position.x)**2 +
                (pose.pose.position.y - current_pose.position.y)**2
            )

            if dist < min_dist:
                min_dist = dist
                closest_idx = i

        return closest_idx

    def find_look_ahead_point(self, current_pose, local_plan, start_idx, look_ahead_dist):
        """
        Find point on path at look-ahead distance
        """
        cumulative_dist = 0.0

        for i in range(start_idx, len(local_plan.poses) - 1):
            pose1 = local_plan.poses[i].pose.position
            pose2 = local_plan.poses[i + 1].pose.position

            segment_dist = math.sqrt(
                (pose2.x - pose1.x)**2 + (pose2.y - pose1.y)**2
            )

            if cumulative_dist + segment_dist >= look_ahead_dist:
                # Interpolate to exact look-ahead distance
                remaining_dist = look_ahead_dist - cumulative_dist
                ratio = remaining_dist / segment_dist

                return i + 1

            cumulative_dist += segment_dist

        # Return last point if look-ahead distance exceeds path
        return len(local_plan.poses) - 1
```

## Integration with Isaac ROS

### Isaac ROS Perception for Humanoid Navigation

#### Humanoid-Specific Perception Pipeline
```yaml
# Isaac ROS perception pipeline for humanoid navigation
isaac_ros_perception_pipeline:
  ros__parameters:
    # Stereo depth for terrain analysis
    stereo_depth:
      enable: true
      input_resolution: [640, 480]
      output_resolution: [640, 480]
      depth_units: "millimeters"
      rectified_images: true

    # Humanoid-specific obstacle detection
    humanoid_obstacle_detector:
      enable: true
      min_obstacle_height: 0.1    # Minimum height to consider as obstacle
      max_obstacle_height: 1.8    # Maximum height for step-over capability
      step_cliff_detector:
        enable: true
        min_cliff_height: 0.05    # Minimum cliff height to detect
        max_passable_slope: 25.0  # Maximum slope angle in degrees

    # Traversability analysis
    traversability_analyzer:
      enable: true
      step_height_threshold: 0.15    # Maximum step height
      slope_threshold: 25.0          # Maximum slope angle
      surface_roughness_threshold: 0.05  # Maximum surface irregularity
      obstacle_clearance_threshold: 0.1  # Minimum clearance needed
```

### Isaac ROS Navigation Integration

#### Example Integration Code
```python
import rclpy
from rclpy.node import Node
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import PointCloud2
from visualization_msgs.msg import MarkerArray
from builtin_interfaces.msg import Duration

class IsaacHumanoidNavigator(Node):
    def __init__(self):
        super().__init__('isaac_humanoid_navigator')

        # Create navigation action client
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        # Subscribe to Isaac ROS perception data
        self.traversability_sub = self.create_subscription(
            PointCloud2, '/isaac_ros/traversability_map',
            self.traversability_callback, 10
        )

        # Subscribe to balance state
        self.balance_sub = self.create_subscription(
            Float64MultiArray, '/humanoid/balance_state',
            self.balance_callback, 10
        )

        # Publishers for visualization
        self.footstep_viz_pub = self.create_publisher(
            MarkerArray, '/humanoid/footsteps_visualization', 10
        )

        # Initialize humanoid-specific navigation components
        self.footstep_planner = FootstepPlanner()
        self.balance_controller = BalanceAwareLocalPlanner()

        # Navigation state
        self.current_balance_state = {'com_x': 0.0, 'com_y': 0.0, 'stability': 1.0}

    def navigate_with_humanoid_constraints(self, goal_pose):
        """
        Navigate to goal with humanoid-specific constraints
        """
        # Preprocess goal for humanoid navigation
        humanoid_goal = self.preprocess_goal_for_humanoid(goal_pose)

        # Plan footsteps to goal
        footsteps = self.footstep_planner.plan_footsteps(
            self.get_current_pose(),
            humanoid_goal,
            self.get_costmap()
        )

        if footsteps is None:
            self.get_logger().error("No valid footstep path found to goal")
            return False

        # Execute navigation with balance awareness
        success = self.execute_navigation_with_balance(footsteps)

        return success

    def preprocess_goal_for_humanoid(self, goal_pose):
        """
        Preprocess navigation goal for humanoid-specific requirements
        """
        # Adjust goal pose to account for humanoid dimensions
        # and balance requirements
        humanoid_goal = PoseStamped()
        humanoid_goal.header = goal_pose.header
        humanoid_goal.pose = goal_pose.pose

        # Add safety margins and adjust for foot placement
        humanoid_goal.pose.position.x -= 0.1  # Small offset for foot placement

        return humanoid_goal

    def execute_navigation_with_balance(self, footsteps):
        """
        Execute navigation while maintaining balance
        """
        for i, footstep in enumerate(footsteps):
            # Check current balance state
            if not self.is_balance_acceptable():
                self.get_logger().warn("Balance state not acceptable, pausing navigation")
                if not self.restore_balance():
                    return False

            # Execute individual step
            step_success = self.execute_single_step(footstep)

            if not step_success:
                self.get_logger().error(f"Failed to execute step {i}")
                return False

            # Wait for step completion and balance stabilization
            if not self.wait_for_step_completion():
                return False

        return True

    def execute_single_step(self, footstep):
        """
        Execute a single bipedal step
        """
        # Plan foot trajectory
        foot_trajectory = self.plan_foot_trajectory(footstep)

        # Execute trajectory with balance control
        success = self.execute_foot_trajectory(foot_trajectory)

        if success:
            # Update support polygon and balance control
            self.update_balance_control(footstep)

        return success

    def is_balance_acceptable(self):
        """
        Check if current balance state is acceptable for navigation
        """
        stability = self.current_balance_state['stability']
        return stability > 0.7  # Require 70% stability threshold

    def restore_balance(self):
        """
        Attempt to restore robot balance
        """
        # Implement balance restoration procedures
        # This might involve pausing, adjusting stance, etc.
        return True  # Placeholder

    def wait_for_step_completion(self):
        """
        Wait for current step to complete and balance to stabilize
        """
        # Wait for step completion with timeout
        timeout = Duration()
        timeout.sec = 5  # 5 second timeout

        # Monitor balance and step completion
        return True  # Placeholder
```

## Performance Optimization for Humanoid Navigation

### Real-Time Considerations

#### Computational Efficiency
- Optimize path planning algorithms for real-time execution
- Use hierarchical planning (coarse-to-fine)
- Implement caching for repeated computations
- Parallel processing where possible

#### Memory Management
- Efficient data structures for spatial queries
- Memory pooling for dynamic allocations
- Proper cleanup of planning data
- Optimize for embedded systems

### Multi-Modal Navigation

#### Adapting to Different Terrains
```python
class AdaptiveHumanoidNavigator:
    def __init__(self):
        self.terrain_classifiers = {
            'flat_ground': FlatGroundGaitController(),
            'stairs': StairGaitController(),
            'rough_terrain': RoughTerrainGaitController(),
            'narrow_paths': PreciseFootstepController()
        }

        self.current_terrain = 'flat_ground'
        self.adaptation_threshold = 0.8

    def adapt_navigation_strategy(self, environment_data):
        """
        Adapt navigation strategy based on environment
        """
        terrain_type = self.classify_terrain(environment_data)

        if terrain_type != self.current_terrain:
            # Change gait controller and planning parameters
            self.switch_gait_controller(terrain_type)
            self.update_planning_parameters(terrain_type)
            self.current_terrain = terrain_type

    def classify_terrain(self, environment_data):
        """
        Classify terrain type based on sensor data
        """
        # Analyze point cloud, images, and other sensor data
        # to determine terrain type
        return 'flat_ground'  # Placeholder
```

## Validation and Testing

### Humanoid Navigation Benchmarks

#### Performance Metrics
- **Path Optimality**: Ratio of actual path length to optimal path
- **Success Rate**: Percentage of successful navigation attempts
- **Balance Maintenance**: Time spent in stable vs. unstable states
- **Step Accuracy**: Deviation from planned footsteps
- **Computation Time**: Planning and control execution time

#### Testing Scenarios
- Straight-line navigation with obstacles
- Turning and maneuvering in tight spaces
- Negotiating steps and curbs
- Dynamic obstacle avoidance
- Recovery from disturbances

### Simulation Testing

#### Isaac Sim Integration
Testing humanoid navigation in Isaac Sim allows for:
- Photorealistic environment simulation
- Accurate physics modeling
- Various terrain types and obstacles
- Controlled disturbance testing
- Large-scale scenario testing

## Troubleshooting and Best Practices

### Common Issues

#### Balance-Related Problems
- Center of mass drift during navigation
- Step timing and coordination issues
- Recovery from perturbations
- Transition between different gaits

#### Planning Issues
- Infeasible path generation
- Step collision with obstacles
- Failure to maintain support polygon
- Inadequate look-ahead planning

### Best Practices

#### Safety Considerations
- Implement multiple safety layers
- Ensure graceful degradation
- Plan for emergency stops
- Monitor system health continuously

#### Performance Optimization
- Use appropriate planning frequencies
- Optimize for robot-specific constraints
- Implement adaptive control strategies
- Monitor and tune parameters continuously

## Chapter Summary

Path planning for bipedal humanoid robots requires specialized approaches that account for the unique challenges of two-legged locomotion. Nav2 provides a solid foundation, but must be adapted to handle balance constraints, discrete step-based movement, and complex kinematic requirements.

The integration of humanoid-specific constraints into traditional navigation approaches enables safe and effective locomotion for bipedal robots. This includes specialized path planning algorithms, balance-aware control systems, and terrain-adaptive navigation strategies.

Understanding how to configure and customize Nav2 for humanoid navigation is essential for developing robots that can operate effectively in human environments. The combination of accurate perception, balance-aware planning, and adaptive control enables humanoid robots to navigate complex environments while maintaining stability.

As you continue through this textbook, you'll learn how to integrate these navigation capabilities with higher-level AI systems that can interpret natural language commands and execute complex tasks. The foundation laid in this chapter provides the mobility platform that enables humanoid robots to operate in real-world environments.

## Check Your Understanding

1. **Conceptual**: Explain the key differences between path planning for wheeled robots versus bipedal humanoid robots, focusing on the impact of balance and discrete stepping.

2. **Application**: Design a navigation system for a humanoid robot that needs to navigate through a crowded environment with both static and moving obstacles. What specific challenges would you need to address?

3. **Analysis**: How would you modify the standard Nav2 costmap approach to account for the unique requirements of bipedal locomotion, such as step height limitations and balance constraints?

## Next Steps

In the next module, we'll explore the convergence of language models and robotic action, where we'll integrate the navigation capabilities we've developed with AI systems that can interpret natural language commands and execute complex tasks.

---

**Reflection Question**: Consider a humanoid robot that needs to navigate through an environment with uneven terrain, stairs, and narrow passages. How would you design a navigation system that can seamlessly transition between different locomotion modes (walking, stair climbing, crawling) while maintaining balance and safety?