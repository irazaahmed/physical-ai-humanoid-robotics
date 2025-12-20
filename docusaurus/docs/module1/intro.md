---
doc_id: project1_m1_ch01
title: Introduction to ROS 2
module: module1
estimated_tokens: 1200
embedding_required: true
chunk_hint_tokens: 500
---

<!-- original_metadata: {"doc_id": "project1_m1_ch01", "title": "Introduction to ROS 2", "module": "module1", "estimated_tokens": 1200, "embedding_required": true, "chunk_hint_tokens": 500} -->
<!-- embedding_placeholder: true -->

<!-- changelog: auto-fixed MDX parse error, converted frontmatter to YAML format -->

# Introduction to ROS 2

## Objective

This chapter introduces the fundamental concepts of Robot Operating System 2 (ROS 2) and its role in building robotic applications, establishing a foundation for understanding the robotic nervous system approach.

## Learning Outcomes

1. Understand the basic architecture and components of ROS 2
2. Explain the advantages of using ROS 2 for robotic development
3. Identify key features that distinguish ROS 2 from ROS 1

## Overview of ROS 2

Robot Operating System 2 (ROS 2) is not an actual operating system but rather a flexible framework for writing robot software. It provides services designed specifically for robotic applications, including hardware abstraction, device drivers, libraries, visualizers, message-passing, package management, and more. ROS 2 has been redesigned from the ground up to improve security, real-time performance, and support for commercial products.

ROS 2 is built on DDS (Data Distribution Service), which is an industry standard for real-time, high-performance, scalable communication. This provides better security and real-time performance compared to ROS 1's custom message passing system. The middleware-agnostic design allows for different communication protocols to be used based on the specific application needs.

## Key Concepts and Architecture

ROS 2 follows a distributed architecture where nodes communicate with each other via topics, services, and actions. Nodes are individual processes that perform computation and can be written in any of the supported languages (C++, Python, etc.). These nodes communicate through messages sent over topics (publish-subscribe pattern), services (request-response pattern), or actions (goal-feedback-result pattern).

The ROS 2 ecosystem includes tools for building packages, running nodes, visualizing data, debugging, and testing. These tools are designed to make the development, debugging, and maintenance of robot applications easier.

## Why ROS 2 Matters

ROS 2 has become crucial in the robotics industry for several reasons:

1. **Improved Security**: With built-in security mechanisms, ROS 2 can be used in industrial and commercial applications where security is critical.

2. **Real-time Performance**: Supports real-time systems, making it suitable for applications requiring predictable timing.

3. **Better Architecture**: The middleware-based architecture provides more flexibility and better support for multi-robot systems.

4. **Commercial Viability**: Designed with commercial product development in mind, offering long-term support (LTS) releases.

## Hands-on Lab

Let's get started with a basic ROS 2 installation verification and first run of the classic "Hello World" example.

### Prerequisites:
- A Linux system (Ubuntu 22.04 recommended) or Windows Subsystem for Linux (WSL2)
- ROS 2 Humble Hawksbill installed

### Step 1: Verify ROS 2 Installation
```bash
source /opt/ros/humble/setup.bash
ros2 --version
```

### Step 2: Create a workspace
```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws
colcon build
source install/setup.bash
```

### Step 3: Run a simple talker/listener demo
```bash
# Terminal 1
source ~/ros2_ws/install/setup.bash
ros2 run demo_nodes_cpp talker

# Terminal 2 (in a new terminal)
source ~/ros2_ws/install/setup.bash
ros2 run demo_nodes_py listener
```

You should see messages being passed from the talker to the listener node, demonstrating the core publish-subscribe communication model.

## Exercises

1. Research and write a short paragraph about the differences between ROS 1 and ROS 2 in terms of middleware.
2. Identify three real-world applications where ROS 2 is currently being used and explain why it was chosen for those applications.

## Quiz

**Question**: What middleware does ROS 2 use to enable communication between nodes?
- A) TCP/IP
- B) DDS (Data Distribution Service)
- C) MQTT
- D) HTTP

**Answer**: B) DDS (Data Distribution Service)