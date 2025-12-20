---
doc_id: project1_m1_ch08
title: "Module 1 Quiz: Robotic Nervous System (ROS 2)"
module: module1
estimated_tokens: 800
embedding_required: true
chunk_hint_tokens: 500
---

<!-- original_metadata: {"doc_id": "project1_m1_ch08", "title": "Module 1 Quiz: Robotic Nervous System (ROS 2)", "module": "module1", "estimated_tokens": 800, "embedding_required": true, "chunk_hint_tokens": 500} -->
<!-- embedding_placeholder: true -->

<!-- changelog: auto-fixed MDX parse error, converted frontmatter to YAML format -->

# Module 1 Quiz: Robotic Nervous System (ROS 2)

## Quiz Objective

Test understanding of fundamental ROS 2 concepts, including architecture, communication patterns, client libraries, URDF, and basic control systems.

## Question 1: Architecture
Which middleware does ROS 2 use to enable communication between nodes?
- A) TCP/IP
- B) DDS (Data Distribution Service)
- C) MQTT
- D) HTTP

**Answer**: B) DDS (Data Distribution Service)

## Question 2: Nodes and Communication
In the publisher-subscriber pattern, which statement is true?
- A) Publishers wait for a response from subscribers
- B) Communication is synchronous between publisher and subscriber
- C) Communication is asynchronous, with publishers not knowing if subscribers exist
- D) Only one subscriber can listen to a single topic

**Answer**: C) Communication is asynchronous, with publishers not knowing if subscribers exist

## Question 3: Communication Patterns
Which communication pattern is best suited for broadcasting a robot's current position to multiple subscribers?
- A) Services
- B) Actions
- C) Topics
- D) Parameters

**Answer**: C) Topics

## Question 4: Client Libraries
Which method is used to create a publisher in rclpy (Python client library)?
- A) `self.create_publisher()`
- B) `self.make_publisher()`
- C) `self.init_publisher()`
- D) `self.add_publisher()`

**Answer**: A) `self.create_publisher()`

## Question 5: URDF
Which URDF joint type allows continuous rotation without limits?
- A) revolute
- B) prismatic
- C) continuous
- D) fixed

**Answer**: C) continuous

## Question 6: Quality of Service
Which QoS policy in ROS 2 determines how many messages are kept in the queue?
- A) Reliability
- B) Durability
- C) History
- D) Deadline

**Answer**: C) History

## Question 7: Topics vs Services
What is the main difference between ROS 2 topics and services?
- A) Topics are faster than services
- B) Topics are asynchronous and unidirectional, services are synchronous and bidirectional
- C) Topics can only send string messages, services can send any data type
- D) There is no significant difference between topics and services

**Answer**: B) Topics are asynchronous and unidirectional, services are synchronous and bidirectional

## Question 8: ros2_control
Which of the following is NOT a standard interface in ros2_control?
- A) Position
- B) Velocity
- C) Acceleration
- D) Effort

**Answer**: C) Acceleration

## Question 9: Nodes
In ROS 2, what is a node?
- A) A type of message passed between components
- B) An executable that uses ROS 2 to communicate with other executables
- C) A specific type of network protocol
- D) A configuration file for robot hardware

**Answer**: B) An executable that uses ROS 2 to communicate with other executables

## Question 10: URDF Elements
What does the 'inertial' element in a URDF link define?
- A) How the link appears visually
- B) How the link interacts physically in simulation
- C) The mass properties of the link (mass, center of mass, inertia tensor)
- D) The geometric shape of the link

**Answer**: C) The mass properties of the link (mass, center of mass, inertia tensor)

## Question 11: rclpy Implementation
Which function is used to initialize rclpy in a Python ROS 2 node?
- A) `rclpy.start()`
- B) `rclpy.init()`
- C) `rclpy.setup()`
- D) `rclpy.run()`

**Answer**: B) `rclpy.init()`

## Question 12: Actions
What is the main advantage of using ROS 2 Actions over Services for long-running tasks?
- A) Actions are faster than services
- B) Actions provide feedback during execution and can be preempted
- C) Actions use less memory than services
- D) Actions can only be used with C++, not Python

**Answer**: B) Actions provide feedback during execution and can be preempted

## Question 13: Package Structure
What is the basic unit of software organization in ROS 2?
- A) Workspace
- B) Node
- C) Package
- D) Launch file

**Answer**: C) Package

## Question 14: URDF Geometry
Which of the following is NOT a valid URDF geometry type?
- A) box
- B) cylinder
- C) sphere
- D) pyramid

**Answer**: D) pyramid

## Question 15: Lifecycle Nodes
What is the purpose of lifecycle nodes in ROS 2?
- A) To reduce memory usage
- B) To provide explicit states and transitions for more robust applications
- C) To speed up communication
- D) To replace regular nodes entirely

**Answer**: B) To provide explicit states and transitions for more robust applications

## Answer Key Summary
1. B  2. C  3. C  4. A  5. C  6. C  7. B  8. C  9. B  10. C  11. B  12. B  13. C  14. D  15. B