---
title: Practical Exercises
sidebar_position: 3
---

# Practical Exercises

## Exercise 1: Create Your First Node

### Objective
Create a simple ROS 2 node that publishes messages every second.

### Steps
1. Create a new ROS 2 package
2. Create a Python node that publishes to a topic
3. Create a subscriber node to receive the messages
4. Run both nodes and observe the communication

### Solution Outline
```python
# publisher_node.py
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class PublisherNode(Node):
    def __init__(self):
        super().__init__('publisher_node')
        self.publisher_ = self.create_publisher(String, 'topic', 10)
        self.timer = self.create_timer(1.0, self.timer_callback)
        self.count = 0
    
    def timer_callback(self):
        msg = String()
        msg.data = f'Hello {self.count}'
        self.publisher_.publish(msg)
        self.count += 1
```

## Exercise 2: Service Communication

### Objective
Implement a simple service that adds two numbers.

### Steps
1. Define a service interface
2. Create a service server
3. Create a service client
4. Test the communication

## Exercise 3: Parameter Server

### Objective
Learn to use ROS 2 parameters for configuration.

### Steps
1. Create a node that declares parameters
2. Access parameters during node operation
3. Modify parameters dynamically
4. Observe parameter changes in real-time

## Exercise 4: Multi-Node System

### Objective
Create a system with multiple nodes communicating through topics and services.

### Steps
1. Create a sensor node that publishes data
2. Create a processing node that subscribes and processes
3. Create a control node that sends commands
4. Integrate everything into a coordinated system

## Tips for Success

- Start with simple examples and gradually increase complexity
- Use `rqt` tools to visualize the node graph
- Monitor messages with `ros2 topic echo`
- Debug using `ros2 node list` and `ros2 topic list`

## Common Issues and Solutions

### Issue: Node not connecting
- Check node names and topic names for typos
- Ensure all nodes are using the same ROS domain ID
- Verify network connectivity between nodes

### Issue: Messages not received
- Check subscription QoS settings
- Verify message types match
- Use `ros2 topic pub` to manually test topics

## Next Steps

Apply these exercises in the [Simulating with Gazebo & Unity](../gazebo-unity/simulation-basics) section.: ROS 2

Apply your ROS 2 knowledge with hands-on coding challenges.