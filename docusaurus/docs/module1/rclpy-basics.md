---
doc_id: project1_m1_ch04
title: Python Client Library (rclpy) Basics
module: module1
estimated_tokens: 1400
embedding_required: true
chunk_hint_tokens: 500
---

<!-- original_metadata: {"doc_id": "project1_m1_ch04", "title": "Python Client Library (rclpy) Basics", "module": "module1", "estimated_tokens": 1400, "embedding_required": true, "chunk_hint_tokens": 500} -->
<!-- embedding_placeholder: true -->

<!-- changelog: auto-fixed MDX parse error, converted frontmatter to YAML format -->

# Python Client Library (rclpy) Basics

## Objective

This chapter introduces the Python client library (rclpy) for ROS 2, covering fundamental concepts and patterns for developing robotic applications in Python.

## Learning Outcomes

1. Understand the structure and components of rclpy nodes
2. Implement publishers, subscribers, services, and clients using rclpy
3. Create executable ROS 2 nodes in Python
4. Configure Quality of Service (QoS) settings in rclpy

## Introduction to rclpy

rclpy is the Python client library for ROS 2, providing the Python API to ROS 2 concepts such as nodes, publishers, subscribers, services, and parameters. It is built on top of the ROS Client Library (rcl) and the rmw (ROS Middleware) interfaces, which provide the abstraction layer for different middleware implementations.

rclpy follows the same architectural concepts as other ROS client libraries like rclcpp (C++), but provides a more Pythonic interface that's familiar to Python developers while maintaining the same ROS 2 functionality.

## Basic Node Structure in rclpy

Every rclpy node follows a basic structure:

1. Import required modules
2. Create a class that inherits from `Node`
3. Initialize the node in the constructor (`__init__` method)
4. Implement the desired functionality (publishers, subscribers, services, etc.)
5. Create a `main` function to initialize rclpy, create the node, and spin it

## Creating Publishers and Subscribers

### Publisher Example

```python
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

### Subscriber Example

```python
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

## Creating Services and Clients

### Service Server Example

```python
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

### Service Client Example

```python
import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts


class MinimalClientAsync(Node):

    def __init__(self):
        super().__init__('minimal_client_async')
        self.cli = self.create_client(AddTwoInts, 'add_two_ints')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = AddTwoInts.Request()

    def send_request(self, a, b):
        self.req.a = a
        self.req.b = b
        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()


def main(args=None):
    rclpy.init(args=args)
    minimal_client = MinimalClientAsync()
    response = minimal_client.send_request(1, 2)
    minimal_client.get_logger().info(
        'Result of add_two_ints: for %d + %d = %d' % 
        (1, 2, response.sum))
    minimal_client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

## Working with Parameters

Parameters allow nodes to be configured at runtime. Here's how to create and use parameters:

```python
import rclpy
from rclpy.node import Node


class ParameterNode(Node):

    def __init__(self):
        super().__init__('parameter_node')
        
        # Declare parameters with default values
        self.declare_parameter('my_parameter', 'default_value')
        self.declare_parameter('integer_parameter', 42)
        
        # Get parameter value
        my_param = self.get_parameter('my_parameter').value
        int_param = self.get_parameter('integer_parameter').value
        
        self.get_logger().info(f'my_parameter: {my_param}')
        self.get_logger().info(f'integer_parameter: {int_param}')
        
        # Set a callback to be notified when parameters change
        self.add_on_set_parameters_callback(self.parameter_callback)

    def parameter_callback(self, params):
        for param in params:
            self.get_logger().info(f'Parameter {param.name} changed to {param.value}')
        return SetParametersResult(successful=True)


def main(args=None):
    rclpy.init(args=args)
    parameter_node = ParameterNode()
    rclpy.spin(parameter_node)
    parameter_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

## Quality of Service Settings in rclpy

rclpy allows setting custom QoS profiles for publishers and subscribers:

```python
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSReliabilityPolicy
from std_msgs.msg import String


class QoSPublisher(Node):

    def __init__(self):
        super().__init__('qos_publisher')
        
        # Create custom QoS profile
        qos_profile = QoSProfile(
            depth=10,
            history=QoSHistoryPolicy.KEEP_LAST,
            reliability=QoSReliabilityPolicy.RELIABLE
        )
        
        self.publisher_ = self.create_publisher(String, 'qos_topic', qos_profile)
        # ... rest of node implementation


def main(args=None):
    rclpy.init(args=args)
    qos_publisher = QoSPublisher()
    rclpy.spin(qos_publisher)
    qos_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

## Hands-on Lab

Let's create a complete example node that demonstrates multiple rclpy concepts:

### Step 1: Create a package for the lab
```bash
cd ~/ros2_ws/src
ros2 pkg create --build-type ament_python rclpy_basics_lab --dependencies rclpy std_msgs example_interfaces
```

### Step 2: Create a node with publisher, subscriber, service, and parameter
```python
# rclpy_basics_lab/rclpy_basics_lab/demo_node.py
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile
from std_msgs.msg import String
from example_interfaces.srv import AddTwoInts


class RclpyDemoNode(Node):

    def __init__(self):
        super().__init__('rclpy_demo_node')
        
        # Declare parameters
        self.declare_parameter('message_prefix', 'Demo')
        
        # Create publisher
        self.publisher_ = self.create_publisher(String, 'demo_topic', 10)
        
        # Create subscriber
        self.subscription = self.create_subscription(
            String,
            'input_topic',
            self.input_callback,
            10)
        
        # Create service
        self.srv = self.create_service(AddTwoInts, 'demo_add', self.add_callback)
        
        # Set up timer to periodically publish messages
        timer_period = 2  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        
        self.get_logger().info('rclpy Demo Node initialized')

    def timer_callback(self):
        msg = String()
        prefix = self.get_parameter('message_prefix').value
        msg.data = f'{prefix}: Hello from rclpy Demo Node'
        self.publisher_.publish(msg)
        self.get_logger().info(f'Published: "{msg.data}"')

    def input_callback(self, msg):
        self.get_logger().info(f'Received input: "{msg.data}"')
        # Echo the message back
        self.publisher_.publish(msg)

    def add_callback(self, request, response):
        response.sum = request.a + request.b
        self.get_logger().info(f'Adding {request.a} + {request.b} = {response.sum}')
        return response


def main(args=None):
    rclpy.init(args=args)
    demo_node = RclpyDemoNode()
    
    try:
        rclpy.spin(demo_node)
    except KeyboardInterrupt:
        pass
    finally:
        demo_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

## Exercises

1. Create a ROS 2 node in Python that publishes temperature readings and includes a parameter for temperature unit (Celsius or Fahrenheit).
2. Implement a ROS 2 client node that calls the service created in the lab example with various input parameters and logs the results.

## Quiz

**Question**: Which method is used to create a publisher in rclpy?
- A) `self.create_publisher()`
- B) `self.make_publisher()`
- C) `self.init_publisher()`
- D) `self.add_publisher()`

**Answer**: A) `self.create_publisher()`