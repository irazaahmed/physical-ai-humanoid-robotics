---
doc_id: project1_m1_ch07
title: "Lab 2: Robot Controller Implementation"
module: module1
estimated_tokens: 1900
embedding_required: true
chunk_hint_tokens: 500
---

<!-- original_metadata: {"doc_id": "project1_m1_ch07", "title": "Lab 2: Robot Controller Implementation", "module": "module1", "estimated_tokens": 1900, "embedding_required": true, "chunk_hint_tokens": 500} -->
<!-- embedding_placeholder: true -->

<!-- changelog: auto-fixed MDX parse error, converted frontmatter to YAML format -->

# Lab 2: Robot Controller Implementation

## Objective

Implement and simulate basic robot controllers using ROS 2, demonstrating how to control robot joints and actuators through the ROS 2 control framework and simulation environments.

## Learning Outcomes

1. Understand the ROS 2 control framework architecture
2. Implement joint trajectory controllers for robot manipulation
3. Create simple position and velocity controllers
4. Test controllers in a simulated environment

## Introduction

Robot controllers in ROS 2 are responsible for converting high-level commands into low-level signals that drive robot actuators. The ROS 2 Control framework provides a standardized way to interface with various robot hardware abstraction layers (HALs) and supports different control types including position, velocity, and effort control.

This lab focuses on creating and testing controllers in simulation before deployment to physical robots. We'll use the ros2_control framework, which provides a hardware abstraction layer and a controller manager to handle different types of controllers.

## ROS 2 Control Architecture

The ROS 2 Control framework consists of several key components:

- **Hardware Interface**: Abstraction layer that communicates with actual robot hardware or simulation
- **Controller Manager**: Manages the lifecycle of controllers (load, configure, start, stop, unload)
- **Controllers**: Implement specific control algorithms (joint trajectory controllers, position controllers, etc.)
- **Resource Manager**: Tracks and manages hardware resources

## Prerequisites

- ROS 2 Humble Hawksbill installed
- ros2_control packages installed
- Gazebo simulation environment (optional for this lab)
- Basic understanding of URDF and xacro

## Lab Exercise 1: Setting up the ros2_control Framework

### Step 1: Install required packages

```bash
sudo apt update
sudo apt install ros-humble-ros2-control ros-humble-ros2-controllers
```

### Step 2: Create a new package for the controller lab

```bash
cd ~/ros2_ws/src
ros2 pkg create --build-type ament_python robot_controller_lab --dependencies rclpy controller_manager joint_state_broadcaster joint_trajectory_controller
```

### Step 3: Create a URDF with ros2_control interface

```xml
<!-- ~/ros2_ws/src/robot_controller_lab/urdf/simple_robot.urdf.xacro -->
<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="simple_robot">
  <xacro:property name="mass" value="0.1" />
  <xacro:property name="length" value="0.2" />
  
  <!-- Base Link -->
  <link name="base_link">
    <visual>
      <geometry>
        <box size="0.5 0.5 0.1"/>
      </geometry>
      <material name="white">
        <color rgba="1 1 1 1"/>
      </material>
    </visual>
  </link>

  <!-- Revolute Joint and Link -->
  <joint name="joint1" type="revolute">
    <parent link="base_link"/>
    <child link="link1"/>
    <origin xyz="0 0 0.05" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
    <limit lower="-3.14" upper="3.14" effort="100" velocity="10"/>
  </joint>

  <link name="link1">
    <visual>
      <geometry>
        <cylinder radius="0.05" length="${length}"/>
      </geometry>
      <material name="red">
        <color rgba="1 0 0 1"/>
      </material>
    </visual>
  </link>

  <!-- ros2_control interface -->
  <ros2_control name="GazeboSystem" type="system">
    <hardware>
      <plugin>gazebo_ros2_control/GazeboSystem</plugin>
    </hardware>
    <joint name="joint1">
      <command_interface name="position">
        <param name="min">-3.14</param>
        <param name="max">3.14</param>
      </command_interface>
      <command_interface name="velocity">
        <param name="min">-10</param>
        <param name="max">10</param>
      </command_interface>
      <state_interface name="position"/>
      <state_interface name="velocity"/>
    </joint>
  </ros2_control>

  <!-- Joint State Broadcaster -->
  <gazebo reference="base_link">
    <plugin filename="libgazebo_ros2_control.so" name="gazebo_ros2_control">
      <parameters>$(find robot_controller_lab)/config/simple_robot_controllers.yaml</parameters>
    </plugin>
  </gazebo>
</robot>
```

### Step 4: Create controller configuration

```yaml
# ~/ros2_ws/src/robot_controller_lab/config/simple_robot_controllers.yaml
controller_manager:
  ros__parameters:
    update_rate: 100  # Hz

    joint_state_broadcaster:
      type: joint_state_broadcaster/JointStateBroadcaster

    joint_trajectory_controller:
      type: joint_trajectory_controller/JointTrajectoryController

joint_trajectory_controller:
  ros__parameters:
    command_interfaces:
      - position
    state_interfaces:
      - position
      - velocity
    joints:
      - joint1
    state_publish_rate: 50.0
    action_monitor_rate: 20.0
    allow_partial_joints_goal: false
    constraints:
      stopped_velocity_tolerance: 0.01
      goal_time: 0.0
      joint1:
        trajectory: 0.05
        goal: 0.01
```

## Lab Exercise 2: Implementing a Joint Trajectory Controller Node

### Step 1: Create the trajectory controller node

```python
# ~/ros2_ws/src/robot_controller_lab/robot_controller_lab/trajectory_controller.py
import rclpy
from rclpy.node import Node
from builtin_interfaces.msg import Duration
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionClient

class SimpleTrajectoryController(Node):
    def __init__(self):
        super().__init__('simple_trajectory_controller')
        
        # Create action client for joint trajectory controller
        self.jtc_action_client = ActionClient(
            self, 
            FollowJointTrajectory,
            '/joint_trajectory_controller/follow_joint_trajectory'
        )
        
        # Wait for the action server to be available
        self.get_logger().info('Waiting for joint trajectory controller action server...')
        self.jtc_action_client.wait_for_server()
        self.get_logger().info('Connected to joint trajectory controller action server')

    def send_joint_trajectory(self, joint_positions, joint_names=['joint1'], duration=5.0):
        """Send a joint trajectory to the controller"""
        
        # Create the goal message
        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory = JointTrajectory()
        goal_msg.trajectory.joint_names = joint_names
        
        # Create trajectory points
        point = JointTrajectoryPoint()
        point.positions = joint_positions
        point.time_from_start = Duration(sec=int(duration), nanosec=(duration % 1) * 1e9)
        
        goal_msg.trajectory.points = [point]
        
        # Send the goal
        self.get_logger().info(f'Sending trajectory goal: positions={joint_positions}')
        return self.jtc_action_client.send_goal_async(goal_msg)


def main(args=None):
    rclpy.init(args=args)
    controller = SimpleTrajectoryController()
    
    # Create a future to wait for the result
    import time
    from rclpy.executors import SingleThreadedExecutor
    
    executor = SingleThreadedExecutor()
    executor.add_node(controller)
    
    # Send a simple trajectory to move the joint
    future = controller.send_joint_trajectory([1.57], duration=3.0)
    
    # Spin until the goal is done
    try:
        rclpy.spin_until_future_complete(controller, future)
        controller.get_logger().info('Trajectory execution completed')
    except KeyboardInterrupt:
        controller.get_logger().info('Trajectory execution interrupted')
    
    controller.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

## Lab Exercise 3: Implementing a Simple Position Controller

### Step 1: Create the position controller node

```python
# ~/ros2_ws/src/robot_controller_lab/robot_controller_lab/position_controller.py
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState

class SimplePositionController(Node):
    def __init__(self):
        super().__init__('simple_position_controller')
        
        # Publisher for joint commands
        self.joint_command_publisher = self.create_publisher(
            Float64MultiArray, 
            '/simple_robot/joint_group_position_controller/commands', 
            10
        )
        
        # Subscriber for joint states
        self.joint_state_subscriber = self.create_subscription(
            JointState,
            '/simple_robot/joint_states',
            self.joint_state_callback,
            10
        )
        
        # Timer to send commands periodically
        self.timer = self.create_timer(0.1, self.control_loop)
        
        self.current_positions = {}
        self.target_positions = {'joint1': 0.0}  # Default target position
        
        self.get_logger().info('Simple Position Controller initialized')

    def joint_state_callback(self, msg):
        """Callback to update current joint positions"""
        for i, name in enumerate(msg.name):
            if name in ['joint1']:
                self.current_positions[name] = msg.position[i]

    def control_loop(self):
        """Main control loop to send position commands"""
        # Create command message
        cmd_msg = Float64MultiArray()
        
        # Set target positions for each joint
        target_values = []
        for joint_name in ['joint1']:
            target_values.append(self.target_positions[joint_name])
        
        cmd_msg.data = target_values
        self.joint_command_publisher.publish(cmd_msg)
        
        # Log current state
        current_pos_str = ", ".join([f"{name}: {pos:.3f}" for name, pos in self.current_positions.items()])
        self.get_logger().info(f'Published commands: {target_values}, Current pos: {current_pos_str}')


def main(args=None):
    rclpy.init(args=args)
    controller = SimplePositionController()
    
    # Set a target position for joint1
    controller.target_positions['joint1'] = 1.57  # Move to 90 degrees
    
    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        controller.get_logger().info('Controller stopped by user')
    
    controller.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

## Lab Exercise 4: Implementing a PID Controller Concept

### Step 1: Create a conceptual PID controller node

```python
# ~/ros2_ws/src/robot_controller_lab/robot_controller_lab/pid_controller.py
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState
import math

class PIDController:
    def __init__(self, kp=1.0, ki=0.0, kd=0.0, dt=0.1):
        self.kp = kp  # Proportional gain
        self.ki = ki  # Integral gain
        self.kd = kd  # Derivative gain
        self.dt = dt  # Time step
        
        self.prev_error = 0.0
        self.integral = 0.0
    
    def compute(self, setpoint, measured_value):
        """Compute control output using PID algorithm"""
        error = setpoint - measured_value
        
        # Proportional term
        p_term = self.kp * error
        
        # Integral term
        self.integral += error * self.dt
        i_term = self.ki * self.integral
        
        # Derivative term
        derivative = (error - self.prev_error) / self.dt
        d_term = self.kd * derivative
        
        # Store error for next iteration
        self.prev_error = error
        
        # Calculate output
        output = p_term + i_term + d_term
        
        return output

class SimplePIDController(Node):
    def __init__(self):
        super().__init__('simple_pid_controller')
        
        # Publisher for joint commands
        self.joint_command_publisher = self.create_publisher(
            Float64MultiArray, 
            '/simple_robot/joint_group_position_controller/commands', 
            10
        )
        
        # Subscriber for joint states
        self.joint_state_subscriber = self.create_subscription(
            JointState,
            '/simple_robot/joint_states',
            self.joint_state_callback,
            10
        )
        
        # Timer to run the control loop
        self.timer = self.create_timer(0.01, self.control_loop)  # 100Hz control loop
        
        # Initialize PID controller for joint1
        self.pid_controller = PIDController(kp=2.0, ki=0.1, kd=0.05, dt=0.01)
        
        self.current_positions = {}
        self.target_positions = {'joint1': 0.0}
        
        self.get_logger().info('Simple PID Controller initialized')

    def joint_state_callback(self, msg):
        """Callback to update current joint positions"""
        for i, name in enumerate(msg.name):
            if name == 'joint1':
                self.current_positions[name] = msg.position[i]

    def control_loop(self):
        """Main PID control loop"""
        if 'joint1' not in self.current_positions:
            return  # Wait for joint state data
            
        # Get current position
        current_pos = self.current_positions['joint1']
        target_pos = self.target_positions['joint1']
        
        # Compute control output using PID
        control_output = self.pid_controller.compute(target_pos, current_pos)
        
        # Create and publish command
        cmd_msg = Float64MultiArray()
        cmd_msg.data = [control_output]
        self.joint_command_publisher.publish(cmd_msg)
        
        # Log control information
        self.get_logger().info(
            f'Joint1: Target={target_pos:.3f}, Current={current_pos:.3f}, '
            f'Error={target_pos-current_pos:.3f}, Output={control_output:.3f}'
        )


def main(args=None):
    rclpy.init(args=args)
    controller = SimplePIDController()
    
    # Set a target position for joint1
    controller.target_positions['joint1'] = 1.57  # Move to 90 degrees
    
    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        controller.get_logger().info('PID controller stopped by user')
    
    controller.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

## Lab Exercise 5: Testing Controllers in Simulation (Conceptual)

### Step 1: Create a launch file for the robot simulation

```python
# ~/ros2_ws/src/robot_controller_lab/robot_controller_lab/launch/simple_robot.launch.py
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Get the package share directory
    pkg_share = get_package_share_directory('robot_controller_lab')
    
    # Declare launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time')
    
    # Robot state publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'robot_description': Command(['xacro ', os.path.join(pkg_share, 'urdf', 'simple_robot.urdf.xacro')])
        }]
    )
    
    # Joint state broadcaster
    joint_state_broadcaster = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster'],
        parameters=[{'use_sim_time': use_sim_time}],
    )
    
    # Joint trajectory controller
    joint_trajectory_controller = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_trajectory_controller'],
        parameters=[{'use_sim_time': use_sim_time}],
    )
    
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation time if true'),
        robot_state_publisher,
        joint_state_broadcaster,
        joint_trajectory_controller,
    ])
```

### Step 2: Run the launch file

```bash
cd ~/ros2_ws
colcon build --packages-select robot_controller_lab
source install/setup.bash

# Launch the robot with controllers
ros2 launch robot_controller_lab simple_robot.launch.py
```

## Lab Exercise 6: Sending Trajectory Commands

### Step 1: Create a trajectory sender node

```python
# ~/ros2_ws/src/robot_controller_lab/robot_controller_lab/trajectory_sender.py
import rclpy
from rclpy.node import Node
from builtin_interfaces.msg import Duration
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionClient
import math

class TrajectorySender(Node):
    def __init__(self):
        super().__init__('trajectory_sender')
        
        # Create action client
        self.jtc_action_client = ActionClient(
            self, 
            FollowJointTrajectory,
            '/joint_trajectory_controller/follow_joint_trajectory'
        )
        
        # Wait for the action server to be available
        self.jtc_action_client.wait_for_server()
        self.get_logger().info('Connected to joint trajectory controller action server')

    def send_sine_trajectory(self, amplitude=1.57, frequency=0.1, duration=10.0):
        """Send a sine wave trajectory"""
        
        # Create the goal message
        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory = JointTrajectory()
        goal_msg.trajectory.joint_names = ['joint1']
        
        # Create trajectory points
        points = []
        dt = 0.1  # Time step
        num_points = int(duration/dt)
        
        for i in range(num_points):
            t = i * dt
            position = amplitude * math.sin(2 * math.pi * frequency * t)
            
            point = JointTrajectoryPoint()
            point.positions = [position]
            point.time_from_start = Duration(sec=int(t), nanosec=(t % 1) * 1e9)
            points.append(point)
        
        goal_msg.trajectory.points = points
        
        # Send the goal
        self.get_logger().info(f'Sending sine trajectory with {len(points)} points')
        return self.jtc_action_client.send_goal_async(goal_msg)

    def send_square_trajectory(self, amplitude=1.57, frequency=0.05, duration=10.0):
        """Send a square wave trajectory"""
        
        # Create the goal message
        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory = JointTrajectory()
        goal_msg.trajectory.joint_names = ['joint1']
        
        # Create trajectory points
        points = []
        dt = 0.1  # Time step
        num_points = int(duration/dt)
        
        for i in range(num_points):
            t = i * dt
            position = amplitude if math.sin(2 * math.pi * frequency * t) >= 0 else -amplitude
            
            point = JointTrajectoryPoint()
            point.positions = [position]
            point.time_from_start = Duration(sec=int(t), nanosec=(t % 1) * 1e9)
            points.append(point)
        
        goal_msg.trajectory.points = points
        
        # Send the goal
        self.get_logger().info(f'Sending square trajectory with {len(points)} points')
        return self.jtc_action_client.send_goal_async(goal_msg)


def main(args=None):
    rclpy.init(args=args)
    sender = TrajectorySender()
    
    # Send a sine wave trajectory
    future = sender.send_sine_trajectory(amplitude=1.57, frequency=0.1, duration=10.0)
    
    # Create an executor and spin until the goal is done
    from rclpy.executors import SingleThreadedExecutor
    executor = SingleThreadedExecutor()
    executor.add_node(sender)
    
    try:
        rclpy.spin_until_future_complete(sender, future)
        sender.get_logger().info('Sine trajectory execution completed')
    except KeyboardInterrupt:
        sender.get_logger().info('Trajectory execution interrupted')
    
    sender.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

## Safety and Best Practices

1. **Always set appropriate limits**: Ensure joint limits are respected to prevent damage
2. **Use proper control rates**: Higher control frequencies generally provide better performance but use more CPU
3. **Test in simulation first**: Always verify controller behavior in simulation before deploying to hardware
4. **Monitor control effort**: Watch for excessive control effort that might indicate tuning issues
5. **Implement safety stops**: Include emergency stop functionality in your controllers

## Exercises

1. Modify the PID controller to control multiple joints simultaneously.
2. Implement a velocity controller in addition to the position controller.
3. Add a state machine to the controller that performs different movement patterns based on commands.

## Quiz

**Question**: Which of the following is NOT a standard interface in ros2_control?
- A) Position
- B) Velocity
- C) Acceleration
- D) Effort

**Answer**: C) Acceleration