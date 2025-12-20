---
doc_id: project1_m1_ch03
title: Nodes, Topics, and Services in ROS 2
module: module1
estimated_tokens: 1600
embedding_required: true
chunk_hint_tokens: 500
---

<!-- original_metadata: {"doc_id": "project1_m1_ch03", "title": "Nodes, Topics, and Services in ROS 2", "module": "module1", "estimated_tokens": 1600, "embedding_required": true, "chunk_hint_tokens": 500} -->
<!-- embedding_placeholder: true -->

<!-- changelog: auto-fixed MDX parse error, converted frontmatter to YAML format -->

# Nodes, Topics, and Services in ROS 2

## Objective

This chapter explores the fundamental communication mechanisms in ROS 2: nodes, topics, and services, and how they enable distributed robotic applications with real-time communication capabilities.

## Learning Outcomes

1. Understand the roles and implementation of ROS 2 nodes
2. Implement publish-subscribe communication using topics
3. Implement request-response communication using services
4. Compare the use cases for topics vs. services

## Nodes in ROS 2

A node is an executable that uses ROS 2 to communicate with other nodes. Nodes are the fundamental building blocks of a ROS 2 program. Each node contains the business logic for a specific robot function, such as controlling a motor, processing sensor data, or implementing a behavior.

Nodes are instantiated from classes that inherit from the rclcpp::Node class (in C++) or are created using the rclpy.node.Node class (in Python). Each node should be designed to perform a specific, narrowly-focused task. Nodes can be combined to perform more complex robot tasks.

### Node Lifecycle

Nodes in ROS 2 go through a lifecycle that includes:
- Creation and initialization
- Activation and execution
- Deactivation and cleanup

The lifecycle management helps create more robust robotic applications by providing explicit states and transitions.

## Topics and Publish-Subscribe Pattern

Topics implement the publish-subscribe pattern, which enables asynchronous communication between nodes. Publishers send messages to topics, and subscribers receive messages from topics. Multiple publishers can send to the same topic, and multiple subscribers can receive from the same topic.

This pattern is ideal for:
- Sensor data distribution
- Robot state broadcasting
- Continuous data streams like camera feeds
- Non-critical notifications

The publish-subscribe pattern is unidirectional and asynchronous, meaning publishers don't wait for responses from subscribers.

## Services and Request-Response Pattern

Services implement the request-response pattern, which enables synchronous communication. A service client sends a request to a service server, and the server sends back a response. This pattern requires a direct connection between the client and server.

This pattern is ideal for:
- Operations that must complete before continuing
- Configuration changes
- Single-result computations
- Commands that require acknowledgment

The request-response pattern is bidirectional and synchronous, with the client blocking until it receives a response.

## Implementation Examples

### Python Implementation of a Publisher and Subscriber

```python
# publisher_member_function.py
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

### Python Implementation of a Subscriber

```python
# subscriber_member_function.py
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

### Service Implementation

```python
# service_member_function.py
import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts


class MinimalService(Node):

    def __init__(self):
        super().__init__('minimal_service')
        self.srv = self.create_service(AddTwoInts, 'add_two_ints', self.add_two_ints_callback)

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

## Quality of Service (QoS) for Communication

QoS settings are critical when implementing nodes that communicate via topics or services. Different use cases require different reliability and performance characteristics:

- **Reliable vs Best Effort**: Use reliable for critical message delivery, best effort for high-frequency data where some loss is acceptable
- **Durability**: Volatile for real-time systems, transient-local for systems that need to deliver messages to late-joining subscribers
- **History Policy**: Keep-all for applications that need all messages, keep-last for applications that only need recent values
- **Depth**: The length of the publisher queue for outgoing messages or the subscription queue for incoming messages

## Hands-on Lab

Let's implement a complete publisher-subscriber pair with QoS configurations:

### Step 1: Create a package for the lab
```bash
cd ~/ros2_ws/src
ros2 pkg create --build-type ament_python qos_lab --dependencies rclpy std_msgs
```

### Step 2: Create the publisher with custom QoS settings
```python
# qos_lab/qos_lab/qos_publisher.py
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSHistoryPolicy, QoSReliabilityPolicy
from std_msgs.msg import String


class QoSPublisher(Node):

    def __init__(self):
        super().__init__('qos_publisher')
        
        # Create custom QoS profile
        qos_profile = QoSProfile(
            depth=10,
            history=QoSHistoryPolicy.KEEP_LAST,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.VOLATILE
        )
        
        self.publisher_ = self.create_publisher(String, 'qos_chatter', qos_profile)
        timer_period = 1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = String()
        msg.data = f'Hello, QoS World: {self.i}'
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: "{msg.data}" with QoS settings')
        self.i += 1


def main(args=None):
    rclpy.init(args=args)
    qos_publisher = QoSPublisher()
    
    try:
        rclpy.spin(qos_publisher)
    except KeyboardInterrupt:
        pass
    finally:
        qos_publisher.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

### Step 3: Create the subscriber with matching QoS settings
```bash
# Run the publisher and subscriber
source ~/ros2_ws/install/setup.bash
python3 qos_lab/qos_lab/qos_publisher.py

# In another terminal
python3 qos_lab/qos_lab/qos_subscriber.py
```

## Exercises

1. Create a ROS 2 package that includes both a node publishing sensor data and a node subscribing to process the sensor data. Use appropriate QoS settings for sensor data.
2. Design a system where a robot's movement commands are sent via service calls rather than topics. Explain why this might be preferred in certain scenarios.

## Quiz

**Question**: Which communication pattern is best suited for broadcasting a robot's current position to multiple subscribers?
- A) Services
- B) Actions
- C) Topics
- D) Parameters

**Answer**: C) Topics