---
doc_id: project1_m1_ch02
title: ROS 2 Architecture Overview
module: module1
estimated_tokens: 1500
embedding_required: true
chunk_hint_tokens: 500
---

<!-- original_metadata: {"doc_id": "project1_m1_ch02", "title": "ROS 2 Architecture Overview", "module": "module1", "estimated_tokens": 1500, "embedding_required": true, "chunk_hint_tokens": 500} -->
<!-- embedding_placeholder: true -->

<!-- changelog: auto-fixed MDX parse error, converted frontmatter to YAML format -->

# ROS 2 Architecture Overview

## Objective

This chapter provides a comprehensive overview of the ROS 2 architecture, exploring its components, communication patterns, and the underlying middleware that enables distributed robotic systems.

## Learning Outcomes

1. Describe the core components of the ROS 2 architecture
2. Explain how nodes, packages, and workspaces organize ROS 2 projects
3. Understand the communication patterns: topics, services, and actions

## Core Components of ROS 2

The ROS 2 architecture is built around several core concepts that enable the creation of distributed robotic applications:

- **Nodes**: Individual processes that perform computation. Nodes are the fundamental building blocks of a ROS 2 program. Each node is designed to perform specific, narrowly-focused tasks.

- **Packages**: Reusable modules that contain libraries, launch files, tests, and other resources. Packages are the basic unit of software organization in ROS 2.

- **Workspaces**: Directories where ROS 2 packages are developed and built. A workspace typically contains source, build, and install directories.

- **Launch Files**: Files that allow multiple nodes to be started together with a single command, along with their configuration parameters.

## Communication Patterns

ROS 2 provides three main communication patterns for nodes to interact with each other:

### Topics (Publish/Subscribe)
Topics implement a one-to-many communication pattern where publishers send messages to topics, and subscribers receive messages from topics. This is an asynchronous communication method.

### Services (Request/Response)
Services implement a one-to-one communication pattern where a client sends a request to a server and waits for a response. This is a synchronous communication method.

### Actions (Goal/Feedback/Result)
Actions implement a one-to-one communication pattern with multiple requests and responses over time. They are suitable for long-running tasks that require feedback during execution.

## The DDS Middleware

The Data Distribution Service (DDS) is the foundational technology underlying ROS 2's communication layer. DDS is a vendor-neutral, open standard for real-time systems that provides:

- **Discovery**: Automatic detection of participants in the communication network
- **Quality of Service (QoS)**: Configurable policies for message delivery, reliability, and durability
- **Data-centricity**: The system focuses on the data being communicated rather than the communicating entities
- **Language neutrality**: Support for multiple programming languages

## QoS Profiles

Quality of Service (QoS) profiles allow fine-tuning of the communication behavior between nodes. Key QoS settings include:

- **Reliability**: Reliable vs. best-effort delivery
- **Durability**: Volatile vs. transient-local durability for late-joining subscribers
- **History**: Keep-all vs. keep-last policies for message history
- **Deadline**: Time constraints for message delivery
- **Lifespan**: Maximum time messages are kept in the system

## Hands-on Lab

Let's explore the architecture by creating a simple package and examining its components:

### Step 1: Create a new package
```bash
cd ~/ros2_ws/src
ros2 pkg create --build-type ament_cmake ros2_arch_demo --dependencies rclcpp rclpy
```

### Step 2: Examine the package structure
```bash
cd ros2_arch_demo
ls -la
```

You'll see the typical package structure with CMakeLists.txt, package.xml, and src/ directories.

### Step 3: Create a simple publisher node (in src/publisher_node.cpp)
```cpp
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

class PublisherNode : public rclcpp::Node
{
public:
    PublisherNode() : Node("publisher_node")
    {
        publisher_ = this->create_publisher<std_msgs::msg::String>("topic", 10);
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(500),
            std::bind(&PublisherNode::publish_message, this));
        
        RCLCPP_INFO(this->get_logger(), "Publisher node started");
    }

private:
    void publish_message()
    {
        auto message = std_msgs::msg::String();
        message.data = "Hello ROS 2 World";
        RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
        publisher_->publish(message);
    }

    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PublisherNode>());
    rclcpp::shutdown();
    return 0;
}
```

### Step 4: Build and run the package
```bash
cd ~/ros2_ws
colcon build --packages-select ros2_arch_demo
source install/setup.bash
ros2 run ros2_arch_demo publisher_node
```

## Exercises

1. Design a minimal ROS 2 system architecture for a simple robot that has a camera, wheels, and a distance sensor. Identify what nodes you would create and what topics or services they would use to communicate.
2. Explain how QoS profiles could be used differently for a video stream vs. a safety-critical sensor reading in a robot application.

## Quiz

**Question**: Which of the following is NOT a communication pattern in ROS 2?
- A) Topics (publish/subscribe)
- B) Services (request/response)
- C) Methods (call/return)
- D) Actions (goal/feedback/result)

**Answer**: C) Methods (call/return)