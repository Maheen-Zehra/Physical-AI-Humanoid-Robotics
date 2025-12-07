---
sidebar_position: 3
---

# Quickstart Guide

## Overview

This quickstart guide will help you get up and running with the Physical AI & Humanoid Robotics curriculum in under 30 minutes. Follow these steps to create your first ROS 2 node and validate your setup.

## Prerequisites

Ensure you have completed the [setup guide](./setup.md) and have:
- Ubuntu 22.04 LTS
- ROS 2 Humble installed
- Basic Python knowledge

## Step 1: Create Your First ROS 2 Package

Create a new ROS 2 workspace and package:

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
```

Create a new package for your humanoid robot:

```bash
cd src
ros2 pkg create --build-type ament_python humanoid_tutorial --dependencies rclpy std_msgs
```

## Step 2: Create a Simple Publisher Node

Navigate to your package directory and create a simple publisher:

```bash
cd ~/ros2_ws/src/humanoid_tutorial/humanoid_tutorial
```

Create `simple_publisher.py`:

```python
#!/usr/bin/env python3
"""
Simple publisher node for the quickstart guide
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class SimplePublisher(Node):
    def __init__(self):
        super().__init__('simple_publisher')
        self.publisher_ = self.create_publisher(String, 'robot_status', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = String()
        msg.data = f'Robot is operational: {self.i}'
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: "{msg.data}"')
        self.i += 1


def main(args=None):
    rclpy.init(args=args)
    simple_publisher = SimplePublisher()

    try:
        rclpy.spin(simple_publisher)
    except KeyboardInterrupt:
        pass
    finally:
        simple_publisher.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

## Step 3: Make the Node Executable and Build

Make the script executable:

```bash
chmod +x simple_publisher.py
```

Go back to the workspace root and build:

```bash
cd ~/ros2_ws
colcon build --packages-select humanoid_tutorial
```

Source the new package:

```bash
source install/setup.bash
```

## Step 4: Create a Subscriber Node

Create `simple_subscriber.py` in the same directory:

```python
#!/usr/bin/env python3
"""
Simple subscriber node for the quickstart guide
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class SimpleSubscriber(Node):
    def __init__(self):
        super().__init__('simple_subscriber')
        self.subscription = self.create_subscription(
            String,
            'robot_status',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info(f'I heard: "{msg.data}"')


def main(args=None):
    rclpy.init(args=args)
    simple_subscriber = SimpleSubscriber()

    try:
        rclpy.spin(simple_subscriber)
    except KeyboardInterrupt:
        pass
    finally:
        simple_subscriber.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

Make it executable:

```bash
chmod +x simple_subscriber.py
```

Rebuild the package:

```bash
cd ~/ros2_ws
colcon build --packages-select humanoid_tutorial
source install/setup.bash
```

## Step 5: Test the Publisher-Subscriber Communication

Open two terminals:

Terminal 1 (publisher):
```bash
source ~/ros2_ws/install/setup.bash
ros2 run humanoid_tutorial simple_publisher
```

Terminal 2 (subscriber):
```bash
source ~/ros2_ws/install/setup.bash
ros2 run humanoid_tutorial simple_subscriber
```

You should see the publisher sending messages and the subscriber receiving them.

## Step 6: Create a Simple Service

Create `simple_service.py`:

```python
#!/usr/bin/env python3
"""
Simple service node for the quickstart guide
"""

import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts


class SimpleService(Node):
    def __init__(self):
        super().__init__('simple_service')
        self.srv = self.create_service(AddTwoInts, 'add_two_ints', self.add_two_ints_callback)

    def add_two_ints_callback(self, request, response):
        response.sum = request.a + request.b
        self.get_logger().info(f'Incoming request\na: {request.a}, b: {request.b}')
        return response


def main(args=None):
    rclpy.init(args=args)
    simple_service = SimpleService()

    try:
        rclpy.spin(simple_service)
    except KeyboardInterrupt:
        pass
    finally:
        simple_service.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

Make it executable and rebuild:

```bash
chmod +x simple_service.py
cd ~/ros2_ws
colcon build --packages-select humanoid_tutorial
source install/setup.bash
```

## Step 7: Test the Service

Start the service in one terminal:

```bash
source ~/ros2_ws/install/setup.bash
ros2 run humanoid_tutorial simple_service
```

In another terminal, call the service:

```bash
source ~/ros2_ws/install/setup.bash
ros2 service call /add_two_ints example_interfaces/srv/AddTwoInts "{a: 2, b: 3}"
```

You should see the service respond with the sum (5).

## Next Steps

Congratulations! You've successfully created your first ROS 2 nodes and validated your setup. Now you're ready to dive deeper into the curriculum:

1. **Module 1**: Continue with [ROS 2 Fundamentals](./modules/ros2-fundamentals/index.md) to learn more about ROS 2 concepts
2. **Module 2**: Explore [Simulation Environments](./modules/simulation-environments/index.md) to work with robot simulation
3. **Module 3**: Advance to [AI Perception & Navigation](./modules/ai-perception/index.md) to implement perception systems
4. **Module 4**: Complete the curriculum with [VLA Integration](./modules/vla-integration/index.md) to connect voice commands to robot actions

## Common Issues

**Issue**: `ModuleNotFoundError` when running nodes
**Solution**: Make sure you've sourced your workspace: `source ~/ros2_ws/install/setup.bash`

**Issue**: Service/client communication not working
**Solution**: Verify both nodes are running and on the same network/ROS domain

**Issue**: Permission denied when running scripts
**Solution**: Make sure scripts are executable: `chmod +x script_name.py`

## Troubleshooting Tips

- Always source your ROS 2 environment before running commands
- Check that your Python scripts have the correct shebang line (`#!/usr/bin/env python3`)
- Verify that your workspace is properly built with `colcon build`
- Use `ros2 topic list` and `ros2 service list` to verify available topics and services