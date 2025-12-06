---
title: Working with Nodes
sidebar_position: 2
---

# Working with Nodes

## What is a Node?

A node is an executable that uses ROS 2 to communicate with other nodes. Nodes form the core building blocks of any ROS 2 application.

## Creating a Node

### Python Example

```python
import rclpy
from rclpy.node import Node

class MyNode(Node):
    def __init__(self):
        super().__init__('my_node')
        self.get_logger().info('Node created!')

def main(args=None):
    rclpy.init(args=args)
    node = MyNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### C++ Example

```cpp
#include "rclcpp/rclcpp.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("my_node");
  RCLCPP_INFO(node->get_logger(), "Node created!");
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
```

## Node Lifecycle

1. **Creation**: Instantiate the node object
2. **Initialization**: Set up publishers, subscribers, and services
3. **Spin**: Enter the main loop processing callbacks
4. **Shutdown**: Clean up resources

## Communication Patterns

### Publishers and Subscribers

Nodes can communicate through topics using publishers and subscribers.

### Service Servers and Clients

For request-response patterns, use services.

### Actions

For long-running tasks with feedback, use actions.

## Best Practices

- Keep nodes focused on a single responsibility
- Use meaningful node names
- Log important events and errors
- Handle shutdown gracefully

## Next Steps

Learn about [Practical Exercises](practical-exercises) to apply your node knowledge.

Learn how to create, manage, and communicate with ROS 2 nodes.