---
doc_id: project1_m1_ch06
title: "Lab 1: Publisher-Subscriber Pattern Implementation"
module: module1
estimated_tokens: 1800
embedding_required: true
chunk_hint_tokens: 500
---

<!-- original_metadata: {"doc_id": "project1_m1_ch06", "title": "Lab 1: Publisher-Subscriber Pattern Implementation", "module": "module1", "estimated_tokens": 1800, "embedding_required": true, "chunk_hint_tokens": 500} -->
<!-- embedding_placeholder: true -->

<!-- changelog: auto-fixed MDX parse error, converted frontmatter to YAML format -->

# Lab 1: Publisher-Subscriber Pattern Implementation

## Objective

Implement and test the publisher-subscriber communication pattern in ROS 2 using both C++ and Python, demonstrating real-time data exchange between nodes.

## Learning Outcomes

1. Create publisher and subscriber nodes in both C++ and Python
2. Implement safe, minimal ROS 2 publisher-subscriber examples
3. Understand Quality of Service (QoS) configuration for real-time communication
4. Test and validate node communication in a simulated ROS 2 environment

## Introduction

The publisher-subscriber pattern is one of the fundamental communication paradigms in ROS 2. This lab demonstrates how nodes can asynchronously exchange data through topics. Publishers send messages to topics, and subscribers receive messages from topics, enabling decoupled communication between nodes.

## Prerequisites

- ROS 2 Humble Hawksbill installed
- Basic understanding of ROS 2 concepts
- Development environment with C++ and Python tools

## Lab Exercise 1: C++ Publisher and Subscriber

### Step 1: Create a new package

```bash
cd ~/ros2_ws/src
ros2 pkg create --build-type ament_cmake pubsub_exercise_cpp --dependencies rclcpp rclpy std_msgs
```

### Step 2: Create the publisher node

```cpp
// ~/ros2_ws/src/pubsub_exercise_cpp/src/talker.cpp
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

class MinimalPublisher : public rclcpp::Node
{
public:
    MinimalPublisher()
    : Node("minimal_publisher"), count_(0)
    {
        publisher_ = this->create_publisher<std_msgs::msg::String>("topic", 10);
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(500),
            std::bind(&MinimalPublisher::timer_callback, this));
    }

private:
    void timer_callback()
    {
        auto message = std_msgs::msg::String();
        message.data = "Hello, world! " + std::to_string(count_++);
        RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
        publisher_->publish(message);
    }
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    size_t count_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MinimalPublisher>());
    rclcpp::shutdown();
    return 0;
}
```

### Step 3: Create the subscriber node

```cpp
// ~/ros2_ws/src/pubsub_exercise_cpp/src/listener.cpp
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

class MinimalSubscriber : public rclcpp::Node
{
public:
    MinimalSubscriber()
    : Node("minimal_subscriber")
    {
        subscription_ = this->create_subscription<std_msgs::msg::String>(
            "topic", 10,
            [this](const std_msgs::msg::String::SharedPtr msg) {
                RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg->data.c_str());
            });
    }

private:
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MinimalSubscriber>());
    rclcpp::shutdown();
    return 0;
}
```

### Step 4: Update CMakeLists.txt

```cmake
# ~/ros2_ws/src/pubsub_exercise_cpp/CMakeLists.txt
cmake_minimum_required(VERSION 3.8)
project(pubsub_exercise_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

# Create the talker executable
add_executable(talker src/talker.cpp)
ament_target_dependencies(talker rclcpp std_msgs)

# Create the listener executable
add_executable(listener src/listener.cpp)
ament_target_dependencies(listener rclcpp std_msgs)

install(TARGETS
  talker
  listener
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_flake8_FOUND TRUE)
  set(ament_cmake_pep257_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
```

### Step 5: Update package.xml

```xml
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>pubsub_exercise_cpp</name>
  <version>0.0.0</version>
  <description>Simple publisher-subscriber example in C++</description>
  <maintainer email="todo@todo.todo">todo</maintainer>
  <license>Apache-2.0</license>

  <buildtool_depend>ament_cmake</buildtool_depend>

  <depend>rclcpp</depend>
  <depend>std_msgs</depend>

  <test_depend>ament_lint_auto</test_depend>
  <test_depend>ament_lint_common</test_depend>

  <export>
    <build_type>ament_cmake</build_type>
  </export>
</package>
```

## Lab Exercise 2: Python Publisher and Subscriber

### Step 1: Create a Python package

```bash
cd ~/ros2_ws/src
ros2 pkg create --build-type ament_python pubsub_exercise_py --dependencies rclpy std_msgs
```

### Step 2: Create the Python publisher

```python
# ~/ros2_ws/src/pubsub_exercise_py/pubsub_exercise_py/talker.py
import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(String, 'topic', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = String()
        msg.data = 'Hello World: %d' % self.i
        self.publisher_.publish(msg)
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

### Step 3: Create the Python subscriber

```python
# ~/ros2_ws/src/pubsub_exercise_py/pubsub_exercise_py/listener.py
import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__('minimal_subscriber')
        self.subscription = self.create_subscription(
            String,
            'topic',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info('I heard: "%s"' % msg.data)


def main(args=None):
    rclpy.init(args=args)
    minimal_subscriber = MinimalSubscriber()
    rclpy.spin(minimal_subscriber)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

### Step 4: Update setup.py

```python
# ~/ros2_ws/src/pubsub_exercise_py/setup.py
from setuptools import find_packages, setup

package_name = 'pubsub_exercise_py'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='todo',
    maintainer_email='todo@todo.todo',
    description='Simple publisher-subscriber example in Python',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'talker = pubsub_exercise_py.talker:main',
            'listener = pubsub_exercise_py.listener:main',
        ],
    },
)
```

## Lab Exercise 3: Quality of Service (QoS) Configuration

### Step 1: Create a QoS example in Python

```python
# ~/ros2_ws/src/pubsub_exercise_py/pubsub_exercise_py/qos_example.py
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSReliabilityPolicy
from std_msgs.msg import String


class QoSPublisher(Node):

    def __init__(self):
        super().__init__('qos_publisher')
        
        # Create a custom QoS profile
        qos_profile = QoSProfile(
            depth=10,
            history=QoSHistoryPolicy.KEEP_LAST,
            reliability=QoSReliabilityPolicy.RELIABLE
        )
        
        self.publisher_ = self.create_publisher(String, 'qos_topic', qos_profile)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = String()
        msg.data = 'QoS Message: %d' % self.i
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing with QoS: "%s"' % msg.data)
        self.i += 1


class QoSSubscriber(Node):

    def __init__(self):
        super().__init__('qos_subscriber')
        
        # Match the QoS profile with the publisher
        qos_profile = QoSProfile(
            depth=10,
            history=QoSHistoryPolicy.KEEP_LAST,
            reliability=QoSReliabilityPolicy.RELIABLE
        )
        
        self.subscription = self.create_subscription(
            String,
            'qos_topic',
            self.listener_callback,
            qos_profile)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info('QoS subscriber heard: "%s"' % msg.data)


def main(args=None):
    rclpy.init(args=args)
    
    qos_publisher = QoSPublisher()
    qos_subscriber = QoSSubscriber()
    
    try:
        # Run both nodes in the same executor
        executor = rclpy.executors.MultiThreadedExecutor()
        executor.add_node(qos_publisher)
        executor.add_node(qos_subscriber)
        
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        qos_publisher.destroy_node()
        qos_subscriber.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

## Lab Exercise 4: Running the Examples

### Step 1: Build the packages

```bash
cd ~/ros2_ws
colcon build --packages-select pubsub_exercise_cpp pubsub_exercise_py
source install/setup.bash
```

### Step 2: Run the C++ publisher and subscriber

Terminal 1:
```bash
source ~/ros2_ws/install/setup.bash
ros2 run pubsub_exercise_cpp talker
```

Terminal 2:
```bash
source ~/ros2_ws/install/setup.bash
ros2 run pubsub_exercise_cpp listener
```

### Step 3: Run the Python publisher and subscriber

Terminal 1:
```bash
source ~/ros2_ws/install/setup.bash
ros2 run pubsub_exercise_py talker
```

Terminal 2:
```bash
source ~/ros2_ws/install/setup.bash
ros2 run pubsub_exercise_py listener
```

### Step 4: Run the QoS example

```bash
source ~/ros2_ws/install/setup.bash
ros2 run pubsub_exercise_py qos_example
```

## Lab Exercise 5: Verification and Testing

### Step 1: Use command line tools to verify topics

```bash
# List all topics
ros2 topic list

# Echo a topic
ros2 topic echo /topic std_msgs/msg/String

# Check topic info
ros2 topic info /topic
```

### Step 2: Use rqt_graph to visualize the node network

```bash
rqt_graph
```

## Safety Notes

- When working with ROS 2 nodes, ensure that each node is properly initialized with `rclpy.init()` or `rclcpp::init()`
- Always clean up resources by calling `rclpy.shutdown()` or `rclcpp::shutdown()`
- Use appropriate QoS settings based on your application requirements
- Ensure proper error handling in production code

## Exercises

1. Modify the publisher to send a custom message type instead of String.
2. Add a parameter to the nodes to control the publishing rate.
3. Implement a filter node that subscribes to one topic and republishes modified data to another topic.

## Quiz

**Question**: In the publisher-subscriber pattern, which statement is true?
- A) Publishers wait for a response from subscribers
- B) Communication is synchronous between publisher and subscriber
- C) Communication is asynchronous, with publishers not knowing if subscribers exist
- D) Only one subscriber can listen to a single topic

**Answer**: C) Communication is asynchronous, with publishers not knowing if subscribers exist