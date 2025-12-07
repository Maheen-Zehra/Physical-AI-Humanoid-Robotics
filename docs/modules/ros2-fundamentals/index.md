# ROS 2 Fundamentals for Humanoid Robotics

## Learning Objectives

By the end of this module, students will be able to:
- Understand the core concepts of ROS 2 and its role as the "nervous system" of robots
- Create and run basic ROS 2 nodes for communication
- Implement topics, services, and actions for robot control
- Create and work with URDF models for humanoid robots
- Validate sensor streams from robot models

## Key Concepts

- **ROS 2 Nodes**: Independent processes that perform computation
- **Topics**: Unidirectional communication channels for data streams
- **Services**: Bidirectional request/response communication
- **Actions**: Goal-oriented communication with feedback and status
- **URDF**: Unified Robot Description Format for robot modeling
- **Launch Files**: Configuration files to start multiple nodes simultaneously

## Introduction to ROS 2

ROS 2 (Robot Operating System 2) is a flexible framework for writing robot software. It's a collection of tools, libraries, and conventions that aim to simplify the task of creating complex and robust robot behavior across a wide variety of robot platforms and environments.

In the context of humanoid robotics, ROS 2 serves as the "nervous system" of the robot, enabling different components to communicate and coordinate with each other. It provides a standardized way for sensors, actuators, controllers, and perception systems to interact.

### Why ROS 2?

ROS 2 offers several advantages for humanoid robotics:

1. **Modularity**: Components can be developed and tested independently
2. **Scalability**: Systems can be distributed across multiple machines
3. **Real-time capabilities**: With proper configuration, ROS 2 can meet real-time requirements
4. **Rich ecosystem**: Extensive libraries and tools for robotics applications
5. **Industry standard**: Widely adopted in both academia and industry

## Core ROS 2 Concepts

### Nodes

A node is an executable that uses ROS 2 to communicate with other nodes. In a typical humanoid robot system, you might have nodes for:

- Sensor drivers (IMU, cameras, LiDAR)
- Actuator controllers (joint position, velocity, effort)
- Perception systems (object detection, SLAM)
- Behavior managers (walking, grasping, planning)

### Topics and Messages

Topics provide a way for nodes to send and receive data in a unidirectional manner. Nodes that send data are called publishers, and nodes that receive data are called subscribers.

For humanoid robots, common topics include:
- `/joint_states` - Current joint positions, velocities, and efforts
- `/imu/data` - Inertial measurement unit data
- `/camera/image_raw` - Raw camera images
- `/cmd_vel` - Velocity commands for base movement

### Services

Services provide a request/response communication pattern. A service client sends a request to a service server, which processes the request and returns a response.

For humanoid robots, services might include:
- `/set_joint_position` - Request to move joints to specific positions
- `/get_robot_state` - Request current robot state
- `/reset_robot` - Request to reset robot to initial state

### Actions

Actions are similar to services but are designed for long-running tasks. They provide feedback during execution and can be canceled.

For humanoid robots, actions might include:
- `/move_to_pose` - Move to a specific pose with feedback on progress
- `/grasp_object` - Grasp an object with feedback on grasp success
- `/navigate_to_goal` - Navigate to a goal with feedback on progress

## Creating Your First ROS 2 Nodes

Let's walk through creating basic publisher and subscriber nodes for humanoid robot communication.

### Publisher Node

A publisher node sends messages to a topic. Here's a simple example that publishes "Hello World" messages:

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('humanoid_talker')
        self.publisher_ = self.create_publisher(String, 'humanoid_commands', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = String()
        msg.data = f'Hello World: {self.i}'
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: "{msg.data}"')
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

### Subscriber Node

A subscriber node receives messages from a topic:

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__('humanoid_listener')
        self.subscription = self.create_subscription(
            String,
            'humanoid_commands',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info(f'I heard: "{msg.data}"')


def main(args=None):
    rclpy.init(args=args)
    minimal_subscriber = MinimalSubscriber()
    rclpy.spin(minimal_subscriber)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

## Services in ROS 2

Services provide a way to request specific actions from other nodes. Here's an example of a simple service that adds two integers:

### Service Server

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts


class BasicService(Node):

    def __init__(self):
        super().__init__('basic_service')
        self.srv = self.create_service(AddTwoInts, 'add_two_ints', self.add_two_ints_callback)

    def add_two_ints_callback(self, request, response):
        response.sum = request.a + request.b
        self.get_logger().info(f'Returning {request.a} + {request.b} = {response.sum}')
        return response


def main(args=None):
    rclpy.init(args=args)
    basic_service = BasicService()
    rclpy.spin(basic_service)
    basic_service.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

## Actions in ROS 2

Actions are used for long-running tasks that provide feedback. Here's an example that calculates a Fibonacci sequence:

### Action Server

```python
#!/usr/bin/env python3
import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node
from example_interfaces.action import Fibonacci


class BasicAction(Node):

    def __init__(self):
        super().__init__('basic_action')
        self._action_server = ActionServer(
            self,
            Fibonacci,
            'fibonacci',
            self.execute_callback)

    def execute_callback(self, goal_handle):
        self.get_logger().info('Executing goal...')

        feedback_msg = Fibonacci.Feedback()
        feedback_msg.sequence = [0, 1]

        for i in range(1, goal_handle.request.order):
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                self.get_logger().info('Goal canceled')
                return Fibonacci.Result()

            feedback_msg.sequence.append(
                feedback_msg.sequence[i] + feedback_msg.sequence[i-1])
            self.get_logger().info(f'Feedback: {feedback_msg.sequence}')
            goal_handle.publish_feedback(feedback_msg)

        goal_handle.succeed()
        result = Fibonacci.Result()
        result.sequence = feedback_msg.sequence
        self.get_logger().info(f'Result: {result.sequence}')

        return result


def main(args=None):
    rclpy.init(args=args)
    basic_action = BasicAction()
    rclpy.spin(basic_action)
    basic_action.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

## URDF - Unified Robot Description Format

URDF (Unified Robot Description Format) is an XML format for representing a robot model. It defines the physical and visual properties of a robot, including:

- Links: Rigid parts of the robot
- Joints: Connections between links
- Inertial properties: Mass, center of mass, inertia tensor
- Visual properties: Shape, color, and material
- Collision properties: Collision detection geometry

Here's an example URDF for a simple humanoid model:

```xml
<?xml version="1.0"?>
<robot name="simple_humanoid">
  <link name="base_link">
    <visual>
      <geometry>
        <box size="0.5 0.2 0.2"/>
      </geometry>
      <material name="blue">
        <color rgba="0 0 1 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <box size="0.5 0.2 0.2"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="1"/>
      <inertia ixx="0.083" ixy="0" ixz="0" iyy="0.125" iyz="0" izz="0.125"/>
    </inertial>
  </link>

  <link name="torso">
    <visual>
      <geometry>
        <box size="0.3 0.3 0.6"/>
      </geometry>
      <material name="red">
        <color rgba="1 0 0 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <box size="0.3 0.3 0.6"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="2"/>
      <inertia ixx="0.15" ixy="0" ixz="0" iyy="0.15" iyz="0" izz="0.075"/>
    </inertial>
  </link>

  <joint name="base_to_torso" type="fixed">
    <parent link="base_link"/>
    <child link="torso"/>
    <origin xyz="0 0 0.4"/>
  </joint>

  <link name="head">
    <visual>
      <geometry>
        <sphere radius="0.15"/>
      </geometry>
      <material name="white">
        <color rgba="1 1 1 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <sphere radius="0.15"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.5"/>
      <inertia ixx="0.0045" ixy="0" ixz="0" iyy="0.0045" iyz="0" izz="0.0045"/>
    </inertial>
  </link>

  <joint name="torso_to_head" type="fixed">
    <parent link="torso"/>
    <child link="head"/>
    <origin xyz="0 0 0.45"/>
  </joint>
</robot>
```

## Launch Files

Launch files allow you to start multiple nodes with a single command. Here's an example launch file for the basic nodes:

```python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='humanoid_control',
            executable='talker',
            name='humanoid_talker',
            output='screen'
        ),
        Node(
            package='humanoid_control',
            executable='listener',
            name='humanoid_listener',
            output='screen'
        )
    ])
```

## Sensor Validation

For humanoid robots, it's crucial to validate sensor streams to ensure they're providing reliable data. Here's an example of a sensor validation node:

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, CameraInfo, PointCloud2
from std_msgs.msg import Float32


class SensorValidator(Node):

    def __init__(self):
        super().__init__('sensor_validator')

        # Create subscribers for different sensor types
        self.imu_sub = self.create_subscription(
            Imu,
            'imu/data',
            self.imu_callback,
            10)

        self.camera_sub = self.create_subscription(
            CameraInfo,
            'camera_info',
            self.camera_callback,
            10)

        self.lidar_sub = self.create_subscription(
            PointCloud2,
            'lidar/points',
            self.lidar_callback,
            10)

        # Create publisher for validation status
        self.status_pub = self.create_publisher(Float32, 'sensor_validation_status', 10)

        self.get_logger().info('Sensor validator node initialized')

    def imu_callback(self, msg):
        # Validate IMU data
        if abs(msg.linear_acceleration.x) < 50.0 and abs(msg.linear_acceleration.y) < 50.0 and abs(msg.linear_acceleration.z) < 50.0:
            self.get_logger().info(f'Valid IMU data: Accel X={msg.linear_acceleration.x:.2f}, Y={msg.linear_acceleration.y:.2f}, Z={msg.linear_acceleration.z:.2f}')
        else:
            self.get_logger().warn(f'IMU data out of range: Accel X={msg.linear_acceleration.x:.2f}, Y={msg.linear_acceleration.y:.2f}, Z={msg.linear_acceleration.z:.2f}')

    def camera_callback(self, msg):
        # Validate camera data
        self.get_logger().info(f'Camera resolution: {msg.width}x{msg.height}')
        if msg.width > 0 and msg.height > 0:
            self.get_logger().info('Valid camera configuration')
        else:
            self.get_logger().warn('Invalid camera configuration')

    def lidar_callback(self, msg):
        # Validate LiDAR data
        self.get_logger().info(f'LiDAR point count: {msg.data}')
        self.get_logger().info('LiDAR data received and validated')


def main(args=None):
    rclpy.init(args=args)
    sensor_validator = SensorValidator()
    rclpy.spin(sensor_validator)
    sensor_validator.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

## Exercises

1. **Basic Communication**: Create a publisher that publishes joint commands and a subscriber that logs them.

2. **Service Implementation**: Create a service that accepts a joint name and position, and returns whether the command was accepted.

3. **URDF Extension**: Extend the simple humanoid model with arm links and joints.

4. **Launch File**: Create a launch file that starts the talker, listener, and service nodes simultaneously.

## Summary

This module introduced the fundamental concepts of ROS 2 and their application to humanoid robotics. You learned how to create nodes for communication, work with URDF models, and validate sensor streams. These concepts form the foundation for more advanced robotics applications.

## Review Questions

1. What is the difference between a topic, service, and action in ROS 2?
2. Why is URDF important for humanoid robotics?
3. How do launch files simplify robot system management?
4. What are the key components of a URDF robot model?
5. How would you validate sensor data in a humanoid robot system?

## References

- ROS 2 Documentation: https://docs.ros.org/en/humble/
- URDF Tutorials: http://wiki.ros.org/urdf/Tutorials
- ROS 2 Python Client Library: https://docs.ros.org/en/humble/p/launch_ros/