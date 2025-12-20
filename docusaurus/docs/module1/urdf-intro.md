---
doc_id: project1_m1_ch05
title: Introduction to URDF (Unified Robot Description Format)
module: module1
estimated_tokens: 1300
embedding_required: true
chunk_hint_tokens: 500
---

<!-- original_metadata: {"doc_id": "project1_m1_ch05", "title": "Introduction to URDF (Unified Robot Description Format)", "module": "module1", "estimated_tokens": 1300, "embedding_required": true, "chunk_hint_tokens": 500} -->
<!-- embedding_placeholder: true -->

<!-- changelog: auto-fixed MDX parse error, converted frontmatter to YAML format -->

# Introduction to URDF (Unified Robot Description Format)

## Objective

This chapter introduces the Unified Robot Description Format (URDF), the standard XML format for representing robot models in ROS, including links, joints, and their physical properties.

## Learning Outcomes

1. Understand the purpose and structure of URDF files
2. Define robot links and joints using URDF
3. Specify visual and collision properties for robot components
4. Create simple robot models using URDF

## What is URDF?

Unified Robot Description Format (URDF) is an XML format used in ROS to describe robot models. URDF defines the physical and visual properties of a robot, including its kinematic structure (how parts move relative to each other), visual appearance, and collision properties. URDF is crucial for robot simulation, visualization, and motion planning.

URDF enables a robot-agnostic approach to robot modeling, allowing different tools and algorithms to work with any robot defined in URDF format. It's extensively used in simulation environments like Gazebo and RViz, as well as in planning frameworks like MoveIt.

## URDF Structure

A URDF file contains several essential elements:

- **Links**: Represent rigid parts of the robot (e.g., chassis, arms, wheels)
- **Joints**: Define how links connect and move relative to each other
- **Visual**: Defines how the link appears in visualization tools
- **Collision**: Defines the collision properties for physics simulation
- **Inertial**: Describes the mass properties of the link

## Basic URDF Elements

### Link
A link represents a rigid body part of the robot. It can have visual, collision, and inertial properties:
- **visual**: How the link looks (shape, material, position)
- **collision**: How the link interacts physically (shape, position)
- **inertial**: Mass, center of mass, and inertia tensor

### Joint
A joint connects two links and defines their relative motion:
- **joint type**: fixed, revolute, continuous, prismatic, floating, planar
- **parent/child**: Links connected by the joint
- **origin**: Transform from parent to child joint frame
- **axis**: Joint axis (for revolute/prismatic joints)

## Basic URDF Example

```xml
<?xml version="1.0"?>
<robot name="simple_robot">
  <!-- Base link -->
  <link name="base_link">
    <visual>
      <geometry>
        <cylinder length="0.6" radius="0.2"/>
      </geometry>
      <material name="blue">
        <color rgba="0 0 0.8 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <cylinder length="0.6" radius="0.2"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="10.0"/>
      <inertia ixx="1.0" ixy="0.0" ixz="0.0" iyy="1.0" iyz="0.0" izz="1.0"/>
    </inertial>
  </link>

  <!-- Wheel links -->
  <link name="wheel_link">
    <visual>
      <geometry>
        <cylinder length="0.1" radius="0.05"/>
      </geometry>
      <origin rpy="0 1.5707 1.5707"/>
      <material name="black">
        <color rgba="0 0 0 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <cylinder length="0.1" radius="0.05"/>
      </geometry>
      <origin rpy="0 1.5707 1.5707"/>
    </collision>
    <inertial>
      <mass value="1.0"/>
      <inertia ixx="0.01" ixy="0.0" ixz="0.0" iyy="0.01" iyz="0.0" izz="0.01"/>
    </inertial>
  </link>

  <!-- Joint connecting base to wheel -->
  <joint name="wheel_joint" type="continuous">
    <parent link="base_link"/>
    <child link="wheel_link"/>
    <origin xyz="0.1 0 0" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
  </joint>
</robot>
```

## URDF Geometry Types

URDF supports multiple geometric shapes:
- **box**: Defined by size="x y z"
- **cylinder**: Defined by radius and length
- **sphere**: Defined by radius
- **mesh**: Defined by filename (e.g., STL, DAE) and scale

## URDF Joint Types

- **fixed**: No movement between parent and child
- **revolute**: Rotational movement with limits
- **continuous**: Rotational movement without limits
- **prismatic**: Linear sliding movement with limits
- **floating**: 6 DOF movement (no constraints)
- **planar**: Movement in a plane

## URDF Materials and Colors

URDF supports material definitions with RGBA color values:
- Red, Green, Blue values range from 0.0 to 1.0
- Alpha (transparency) value range from 0.0 (transparent) to 1.0 (opaque)

## Hands-on Lab

Let's create a simple two-wheeled robot model with URDF:

### Step 1: Create URDF files directory
```bash
mkdir -p ~/ros2_ws/src/simple_robot_description/urdf
```

### Step 2: Create the robot URDF file (simple_robot.urdf)
```xml
<?xml version="1.0"?>
<robot name="simple_robot">
  <!-- Robot base -->
  <link name="base_link">
    <visual>
      <geometry>
        <box size="0.5 0.3 0.1"/>
      </geometry>
      <material name="green">
        <color rgba="0 0.8 0 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <box size="0.5 0.3 0.1"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="1.0"/>
      <inertia ixx="0.1" ixy="0.0" ixz="0.0" iyy="0.1" iyz="0.0" izz="0.1"/>
    </inertial>
  </link>

  <!-- Left wheel -->
  <link name="left_wheel">
    <visual>
      <geometry>
        <cylinder length="0.05" radius="0.1"/>
      </geometry>
      <origin rpy="1.5707 0 0"/>
      <material name="black">
        <color rgba="0 0 0 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <cylinder length="0.05" radius="0.1"/>
      </geometry>
      <origin rpy="1.5707 0 0"/>
    </collision>
    <inertial>
      <mass value="0.2"/>
      <inertia ixx="0.001" ixy="0.0" ixz="0.0" iyy="0.001" iyz="0.0" izz="0.001"/>
    </inertial>
  </link>

  <!-- Right wheel -->
  <link name="right_wheel">
    <visual>
      <geometry>
        <cylinder length="0.05" radius="0.1"/>
      </geometry>
      <origin rpy="1.5707 0 0"/>
      <material name="black">
        <color rgba="0 0 0 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <cylinder length="0.05" radius="0.1"/>
      </geometry>
      <origin rpy="1.5707 0 0"/>
    </collision>
    <inertial>
      <mass value="0.2"/>
      <inertia ixx="0.001" ixy="0.0" ixz="0.0" iyy="0.001" iyz="0.0" izz="0.001"/>
    </inertial>
  </link>

  <!-- Joints -->
  <joint name="left_wheel_joint" type="continuous">
    <parent link="base_link"/>
    <child link="left_wheel"/>
    <origin xyz="0.15 0.15 -0.05" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
  </joint>

  <joint name="right_wheel_joint" type="continuous">
    <parent link="base_link"/>
    <child link="right_wheel"/>
    <origin xyz="0.15 -0.15 -0.05" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
  </joint>
</robot>
```

### Step 3: Visualize the robot in RViz
```bash
# Launch RViz with the robot model
ros2 run rviz2 rviz2 -d /opt/ros/humble/share/rviz_default_plugins/urdf.rviz &
ros2 run robot_state_publisher robot_state_publisher --robot-description-file ~/ros2_ws/src/simple_robot_description/urdf/simple_robot.urdf
```

## URDF Best Practices

1. **Use consistent naming**: Use clear, descriptive names for links and joints
2. **Proper reference frames**: Define coordinate frames following the ROS coordinate conventions (X forward, Y left, Z up)
3. **Realistic inertial properties**: Estimate mass and inertia properties as accurately as possible
4. **Separate visual and collision geometry**: Use simpler geometry for collision detection than for visual appearance
5. **Organize complex models**: Break down complex robots into separate xacro files

## Exercises

1. Create a URDF model for a simple robotic arm with 3 joints: base rotation, shoulder, and elbow.
2. Modify the two-wheeled robot to include a caster wheel at the front for balance.

## Quiz

**Question**: Which URDF joint type allows continuous rotation without limits?
- A) revolute
- B) prismatic
- C) continuous
- D) fixed

**Answer**: C) continuous