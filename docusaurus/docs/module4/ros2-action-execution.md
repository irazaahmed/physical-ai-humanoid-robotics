---
doc_id: module4_m4_ch06
title: "Action Execution via ROS 2"
module: "Module 4: Vision-Language-Action (VLA)"
estimated_tokens: 1150
embedding_required: true
chunk_hint_tokens: 500
---

# Action Execution via ROS 2

## Objective

This chapter introduces students to the implementation of robot action execution using ROS 2, focusing on how to translate high-level commands from VLA systems into specific ROS 2 actions and services. Students will learn to bridge cognitive planning and perception systems with actual robot control.

## Learning Outcomes

After completing this chapter, students will be able to:
1. Understand ROS 2 architecture and its role in robot control and action execution
2. Implement action execution pipelines that connect VLA systems to ROS 2
3. Create ROS 2 action clients and servers for robot control
4. Design safe and robust action execution mechanisms with feedback and monitoring

## Theory

ROS 2 (Robot Operating System 2) is a middleware framework that provides a collection of libraries and tools to help software developers create robot applications. In the context of Vision-Language-Action (VLA) systems, ROS 2 serves as the critical bridge between high-level cognitive planning and perception systems and the low-level robot control systems.

### ROS 2 Architecture

ROS 2 is built on DDS (Data Distribution Service) and provides:

1. **Node architecture**: Individual processes that perform computation
2. **Topic communication**: Publish/subscribe model for data streams
3. **Service communication**: Request/response model for remote procedure calls
4. **Action communication**: Goal-oriented communication for long-running tasks with feedback
5. **Parameters**: Configuration values accessible to nodes
6. **Lifecycle management**: Tools for managing node states

### Action Execution in VLA Context

In VLA systems, action execution involves several key components:

1. **Command Mapping**: Translating high-level commands from cognitive planners into ROS 2 action/service calls
2. **Action Client Implementation**: Creating clients that send goals to action servers
3. **Feedback Processing**: Handling feedback from action servers to monitor execution
4. **Error Handling**: Managing failures and exceptions during execution
5. **Safety Integration**: Ensuring actions are safe before and during execution

### ROS 2 Action Concepts

ROS 2 actions are designed for long-running tasks and include:
- **Goal**: The request to start an action
- **Feedback**: Periodic updates on action progress
- **Result**: The final outcome of the action
- **State**: The current state of the action (PENDING, ACTIVE, PREEMPTED, SUCCEEDED, ABORTED, RECALLED, LOST)

### Integration with VLA Systems

The integration of ROS 2 with VLA systems involves:

1. **Message Conversion**: Converting VLA system outputs to appropriate ROS 2 message types
2. **Action Selection**: Mapping VLA commands to appropriate ROS 2 actions
3. **Parameter Handling**: Passing necessary parameters (e.g., object poses, navigation goals) to actions
4. **Response Interpretation**: Processing ROS 2 responses for higher-level VLA components

### Safety and Validation

Safety considerations in VLA-to-ROS 2 integration include:
- Pre-execution validation of action parameters
- Monitoring of action execution
- Emergency stop mechanisms
- Collision avoidance and safety checks

## Practical Examples

### Example 1: VLA-to-ROS 2 Action Bridge

Implementing a bridge between VLA system outputs and ROS 2 actions:

```python
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

# Import ROS 2 standard actions
from geometry_msgs.msg import Point, Pose, PoseStamped
from nav2_msgs.action import NavigateToPose
from std_msgs.msg import String
from sensor_msgs.msg import Image
from builtin_interfaces.msg import Duration

from typing import Dict, Any, Optional
import json

class VLAActionBridge(Node):
    def __init__(self):
        super().__init__('vla_action_bridge')
        
        # Create callback group for multi-threading
        self.callback_group = ReentrantCallbackGroup()
        
        # Create action clients for navigation
        self.nav_to_pose_client = ActionClient(
            self, 
            NavigateToPose, 
            'navigate_to_pose',
            callback_group=self.callback_group
        )
        
        # Publisher for high-level task status
        self.status_pub = self.create_publisher(String, 'vla_task_status', 10)
        
        # Subscription to VLA commands
        self.vla_command_sub = self.create_subscription(
            String,
            'vla_commands',
            self.vla_command_callback,
            10,
            callback_group=self.callback_group
        )
        
        # Store active goals
        self.active_goals = {}
        
        self.get_logger().info('VLA Action Bridge node initialized')

    def vla_command_callback(self, msg: String):
        """
        Handle VLA commands and convert to ROS 2 actions
        """
        try:
            # Parse the VLA command (should be JSON format)
            command_data = json.loads(msg.data)
            
            command_type = command_data.get('command_type')
            parameters = command_data.get('parameters', {})
            
            if command_type == 'navigate':
                self.execute_navigation_command(parameters)
            elif command_type == 'manipulate':
                self.execute_manipulation_command(parameters)
            elif command_type == 'perceive':
                self.execute_perception_command(parameters)
            else:
                self.get_logger().warn(f'Unknown command type: {command_type}')
                
        except json.JSONDecodeError:
            self.get_logger().error(f'Invalid JSON in VLA command: {msg.data}')
        except Exception as e:
            self.get_logger().error(f'Error processing VLA command: {str(e)}')

    def execute_navigation_command(self, params: Dict[str, Any]):
        """
        Execute navigation command from VLA system
        """
        try:
            # Extract navigation parameters
            target_x = params['x']
            target_y = params['y']
            target_z = params.get('z', 0.0)  # Default to 0 if not provided
            orientation_w = params.get('orientation_w', 1.0)
            
            # Wait for action server
            self.nav_to_pose_client.wait_for_server()
            
            # Create navigation goal
            goal_msg = NavigateToPose.Goal()
            goal_msg.pose.header.frame_id = 'map'
            goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
            goal_msg.pose.pose.position.x = target_x
            goal_msg.pose.pose.position.y = target_y
            goal_msg.pose.pose.position.z = target_z
            goal_msg.pose.pose.orientation.w = orientation_w  # Simplified orientation
            
            # Send navigation goal
            goal_future = self.nav_to_pose_client.send_goal_async(
                goal_msg,
                feedback_callback=self.navigation_feedback_callback
            )
            
            # Store future reference for potential cancellation
            goal_id = goal_future.goal_id
            self.active_goals[goal_id] = goal_future
            
            self.get_logger().info(
                f'Sent navigation goal to ({target_x}, {target_y}, {target_z})'
            )
            
        except Exception as e:
            self.get_logger().error(f'Error executing navigation: {str(e)}')

    def navigation_feedback_callback(self, feedback_msg):
        """
        Handle navigation feedback
        """
        self.get_logger().info(
            f'Navigation progress: {feedback_msg.current_distance}/{feedback_msg.distance_remaining}'
        )
        
        # Publish status update
        status_msg = String()
        status_msg.data = f"navigating_to_goal_progress:{feedback_msg.current_distance}"
        self.status_pub.publish(status_msg)

    def execute_manipulation_command(self, params: Dict[str, Any]):
        """
        Execute manipulation command from VLA system
        """
        # In a real implementation, this would create action calls to manipulation servers
        # For this example, we'll just log the command
        self.get_logger().warn('Manipulation commands not implemented in this example')
        self.get_logger().info(f'Received manipulation command with params: {params}')

    def execute_perception_command(self, params: Dict[str, Any]):
        """
        Execute perception command from VLA system
        """
        # In a real implementation, this would create action calls to perception servers
        self.get_logger().warn('Perception commands not implemented in this example')
        self.get_logger().info(f'Received perception command with params: {params}')

def main(args=None):
    rclpy.init(args=args)
    
    # Create VLA Action Bridge node
    vla_bridge = VLAActionBridge()
    
    # Use multi-threaded executor to handle callbacks
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(vla_bridge)
    
    try:
        executor.spin()
    except KeyboardInterrupt:
        vla_bridge.get_logger().info('Shutting down VLA Action Bridge...')
    finally:
        vla_bridge.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Example 2: Safe Action Execution Manager

Implementing a safety layer for action execution:

```python
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, DurabilityPolicy
from rclpy.duration import Duration as RCLDuration
from geometry_msgs.msg import Point, Pose, Twist
from std_msgs.msg import Bool, String
from sensor_msgs.msg import LaserScan, Image
from builtin_interfaces.msg import Duration

from typing import Dict, Any, Optional, Callable
import threading
import time

class SafeActionExecutor(Node):
    def __init__(self):
        super().__init__('safe_action_executor')
        
        # Publishers and subscribers for safety monitoring
        self.velocity_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.emergency_stop_pub = self.create_publisher(Bool, 'emergency_stop', 10)
        self.safety_status_pub = self.create_publisher(String, 'safety_status', 10)
        
        # Subscriber for sensor data
        self.laser_sub = self.create_subscription(
            LaserScan, 
            'scan', 
            self.laser_callback, 
            10
        )
        
        # Safety variables
        self.safety_enabled = True
        self.obstacle_detected = False
        self.obstacle_distance = float('inf')
        self.last_safe_check = self.get_clock().now()
        
        # Action execution locks
        self.execution_lock = threading.Lock()
        
        self.get_logger().info('Safe Action Executor initialized')

    def laser_callback(self, msg: LaserScan):
        """
        Process laser scan data for obstacle detection
        """
        # Simple obstacle detection: check if any distance is less than threshold
        min_distance = min(msg.ranges) if msg.ranges else float('inf')
        self.obstacle_distance = min_distance
        self.obstacle_detected = min_distance < 0.5  # 0.5m threshold
        
        # Log safety status
        status = "OBSTACLE_DETECTED" if self.obstacle_detected else "CLEAR"
        self.get_logger().debug(f'Obstacle status: {status}, distance: {min_distance:.2f}m')

    def safe_execute_action(self, 
                           action_function: Callable, 
                           safety_check: Callable = None,
                           timeout: float = 30.0) -> bool:
        """
        Safely execute an action with safety checks
        """
        with self.execution_lock:
            # Initial safety check
            if safety_check and not safety_check():
                self.get_logger().error('Initial safety check failed')
                return False
            
            # Check for obstacles before starting action
            if self.obstacle_detected:
                self.get_logger().error('Obstacle detected before action execution')
                return False
            
            # Record start time
            start_time = self.get_clock().now()
            
            try:
                # Execute the action
                result = action_function()
                
                # Monitor safety during execution
                while rclpy.ok():
                    current_time = self.get_clock().now()
                    elapsed = (current_time - start_time).nanoseconds / 1e9
                    
                    # Check timeout
                    if elapsed > timeout:
                        self.get_logger().error('Action timeout exceeded')
                        self._send_emergency_stop()
                        return False
                    
                    # Check safety continuously during execution
                    if self.obstacle_detected:
                        self.get_logger().warn('Obstacle detected during action - stopping')
                        self._send_emergency_stop()
                        return False
                    
                    # Small delay to allow other callbacks to run
                    time.sleep(0.1)
                
                return result
                
            except Exception as e:
                self.get_logger().error(f'Action execution failed: {str(e)}')
                self._send_emergency_stop()
                return False

    def _send_emergency_stop(self):
        """
        Send emergency stop command
        """
        stop_msg = Bool()
        stop_msg.data = True
        self.emergency_stop_pub.publish(stop_msg)
        
        # Stop current motion
        zero_twist = Twist()
        self.velocity_pub.publish(zero_twist)
        
        self.get_logger().warn('Emergency stop executed')

    def navigation_safety_check(self) -> bool:
        """
        Specific safety check for navigation actions
        """
        # Check if we can safely navigate
        if not self.safety_enabled:
            return True  # Safety disabled for testing
        
        if self.obstacle_detected:
            self.get_logger().warn('Navigation safety check failed: obstacle detected')
            return False
        
        # Check if last sensor data is too old
        current_time = self.get_clock().now()
        if (current_time - self.last_safe_check).nanoseconds / 1e9 > 1.0:
            self.get_logger().warn('Navigation safety check failed: sensor data too old')
            return False
        
        return True

    def execute_navigation_with_safety(self, target_pose: Pose) -> bool:
        """
        Execute navigation with safety checks
        """
        def navigation_action():
            # In a real implementation, this would send navigation goals
            self.get_logger().info(f'Executing navigation to {target_pose.position.x}, {target_pose.position.y}')
            # Simulate navigation action
            time.sleep(2)  # Simulate execution time
            return True
        
        return self.safe_execute_action(
            navigation_action,
            safety_check=self.navigation_safety_check,
            timeout=60.0  # 60 second timeout for navigation
        )

# Example usage
def example_safe_execution():
    """
    Example of safe action execution in a VLA context
    """
    rclpy.init()
    
    # Create safe executor
    safe_executor = SafeActionExecutor()
    
    # Create a pose to navigate to
    target_pose = Pose()
    target_pose.position.x = 5.0
    target_pose.position.y = 3.0
    target_pose.position.z = 0.0
    target_pose.orientation.w = 1.0
    
    # Execute navigation with safety
    success = safe_executor.execute_navigation_with_safety(target_pose)
    safe_executor.get_logger().info(f'Navigation execution result: {success}')
    
    # Cleanup
    safe_executor.destroy_node()
    rclpy.shutdown()
    
    return success

def vla_to_ros_integration_example():
    """
    Example of how VLA systems integrate with ROS 2 through action mapping
    """
    print("VLA to ROS 2 Integration Example:")
    print("1. VLA system generates high-level command: 'Go to kitchen'") 
    print("2. Command is parsed and mapped to navigation action")
    print("3. Target pose is determined for 'kitchen' location")
    print("4. ROS 2 navigation action is called with safety checks")
    print("5. Robot executes navigation while monitoring for obstacles")
    print("6. Success/failure is reported back to VLA system")
    
    # Simulated integration
    success = example_safe_execution()
    print(f"Integration example completed successfully: {success}")
```

## Hands-on Lab

### Prerequisites
- Basic ROS 2 knowledge and setup
- Understanding of action and service concepts in ROS 2
- Python programming skills
- Familiarity with robotics concepts from Module 1

### Step 1: Set Up ROS 2 Environment
1. Ensure ROS 2 is properly installed and sourced
2. Create a new ROS 2 package for the VLA integration
3. Set up the necessary dependencies for action execution

### Step 2: Implement Basic Action Client
1. Create a simple ROS 2 node that receives VLA commands
2. Implement conversion from VLA command format to ROS 2 action calls
3. Test with basic navigation actions using dummy action servers

### Step 3: Add Safety Layer
1. Implement the safety monitoring system
2. Add obstacle detection using simulated sensor data
3. Test emergency stop functionality

### Step 4: Integrate with Navigation
1. Connect to actual ROS 2 navigation stack
2. Test navigation commands from VLA system
3. Verify safety checks work during navigation

### Step 5: Comprehensive Testing
1. Test various VLA-to-ROS command conversions
2. Verify safety systems respond appropriately
3. Evaluate performance and response times

## Exercises

1. Implement an action execution system that can handle manipulation tasks (e.g., grasping objects). How would you extend the safety system to handle manipulation-specific risks?

2. Design a feedback system that reports execution status back to the VLA cognitive planner. How would the planner use this information?

3. Create a recovery system that handles failed actions and attempts alternative approaches. How would this integrate with the LLM-based planning from Chapter 4?

4. Research ROS 2 security best practices and implement authentication and encryption for action execution. Why is this important for VLA systems?

5. Develop a simulation environment that allows testing of VLA-to-ROS integration without physical hardware. What are the challenges in making simulation realistic enough?

## Summary

This chapter covered the critical component of action execution in VLA systems, specifically focusing on integration with ROS 2. We explored how to bridge high-level VLA commands with low-level robot control through the ROS 2 framework, with particular emphasis on safety and validation. The integration of VLA systems with ROS 2 completes the action execution loop of the VLA paradigm, enabling cognitive plans and perception outputs to result in actual robot behavior.

## Further Reading

- "ROS 2 for Beginners: A Practical Guide" - Introduction to ROS 2 Concepts
- "Action Architecture in ROS 2: Best Practices" - Technical Guide to Actions
- "Safe Robot Control in Dynamic Environments" - Safety-focused Robotics Article
- "Integrating Natural Language with ROS 2 Control Systems" - VLA-specific Integration Guide
- "Real-time Control Systems in Robotics" - Real-time Performance Considerations