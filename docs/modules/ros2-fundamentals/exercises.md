# Exercises: ROS 2 Fundamentals for Humanoid Robotics

## Exercise 1: Basic Publisher-Subscriber Communication

### Objective
Create a publisher that publishes joint position commands and a subscriber that logs them.

### Requirements
1. Create a publisher node that sends joint position commands to `/joint_commands` topic
2. Create a subscriber node that listens to `/joint_commands` and logs the received commands
3. Use a custom message type or the standard `sensor_msgs/JointState` message
4. The publisher should send commands at 10Hz

### Implementation Steps
1. Define a simple message structure for joint commands
2. Create the publisher node with proper ROS 2 initialization
3. Create the subscriber node that logs received messages
4. Test the communication between nodes

### Validation
- Verify that the subscriber receives messages from the publisher
- Check that messages are published at approximately 10Hz
- Confirm that the content of the messages is preserved during transmission

## Exercise 2: Joint Control Service

### Objective
Create a service that accepts a joint name and target position, and returns whether the command was accepted.

### Requirements
1. Create a service server that listens on `/set_joint_position`
2. The service should accept joint name (string) and target position (float)
3. Return success/failure status and current position
4. Simulate joint movement with a delay

### Implementation Steps
1. Define the service interface with request/response structure
2. Implement the service server with validation logic
3. Add simulated joint movement with timing
4. Test the service with a client

### Validation
- Verify that the service accepts valid joint names
- Confirm that out-of-range positions are rejected
- Test concurrent service calls
- Validate response time and accuracy

## Exercise 3: URDF Model Extension

### Objective
Extend the simple humanoid model with arm links and joints.

### Requirements
1. Add left and right arms to the existing humanoid model
2. Each arm should have shoulder, elbow, and wrist joints
3. Include visual and collision properties for new links
4. Ensure the model is kinematically valid

### Implementation Steps
1. Define new links for upper arm, lower arm, and hand
2. Add appropriate joints connecting the links
3. Set proper inertial properties for each link
4. Validate the URDF model using xacro

### Validation
- Check that the URDF file is well-formed XML
- Verify that all joints have proper parent-child relationships
- Confirm that the model can be visualized in RViz
- Test that the model is kinematically consistent

## Exercise 4: Multi-Node Launch System

### Objective
Create a launch file that starts multiple nodes simultaneously for a basic humanoid control system.

### Requirements
1. Create a launch file that starts at least 3 nodes
2. Include publisher, subscriber, and service nodes
3. Use parameters to configure node behavior
4. Add error handling for node startup failures

### Implementation Steps
1. Design the system architecture with required nodes
2. Create the launch file with proper node definitions
3. Add parameter configuration for each node
4. Test the launch system with various configurations

### Validation
- Verify that all nodes start successfully
- Confirm that nodes can communicate with each other
- Test parameter passing to nodes
- Validate system behavior when individual nodes fail

## Exercise 5: Sensor Validation Node

### Objective
Create a comprehensive sensor validation node that monitors multiple sensor streams.

### Requirements
1. Subscribe to IMU, camera, and LiDAR data
2. Validate data ranges and update rates
3. Publish validation status for each sensor
4. Log warnings when sensors are out of specification

### Implementation Steps
1. Create subscribers for each sensor type
2. Implement validation logic for each sensor
3. Add status publishing with validation results
4. Include logging for debugging and monitoring

### Validation
- Verify that all sensor streams are properly received
- Confirm that validation logic works correctly
- Test with simulated bad data to verify warnings
- Validate that status messages are published consistently

## Exercise 6: Action Server for Joint Trajectory

### Objective
Create an action server that executes joint trajectories with feedback.

### Requirements
1. Create an action server for `/execute_joint_trajectory`
2. Accept a sequence of joint positions and timing
3. Provide feedback on execution progress
4. Support goal cancellation

### Implementation Steps
1. Define the action interface with appropriate message types
2. Implement the action server with trajectory execution
3. Add feedback publishing during execution
4. Include goal cancellation handling

### Validation
- Test trajectory execution with various paths
- Verify feedback messages during execution
- Confirm proper handling of cancellation requests
- Validate that the action server handles multiple concurrent goals

## Solutions

### Exercise 1 Solution

Publisher Node:
```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import math


class JointCommandPublisher(Node):

    def __init__(self):
        super().__init__('joint_command_publisher')
        self.publisher_ = self.create_publisher(JointState, 'joint_commands', 10)
        timer_period = 0.1  # 10Hz
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = JointState()
        msg.name = ['joint1', 'joint2', 'joint3']
        msg.position = [
            math.sin(self.i * 0.1),
            math.cos(self.i * 0.1),
            math.sin(self.i * 0.2)
        ]
        msg.header.stamp = self.get_clock().now().to_msg()
        self.publisher_.publish(msg)
        self.i += 1


def main(args=None):
    rclpy.init(args=args)
    publisher = JointCommandPublisher()
    rclpy.spin(publisher)
    publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

Subscriber Node:
```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


class JointCommandSubscriber(Node):

    def __init__(self):
        super().__init__('joint_command_subscriber')
        self.subscription = self.create_subscription(
            JointState,
            'joint_commands',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info(f'Joint positions: {msg.position}')


def main(args=None):
    rclpy.init(args=args)
    subscriber = JointCommandSubscriber()
    rclpy.spin(subscriber)
    subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

## Assessment Rubric

### Technical Implementation (60%)
- Correct ROS 2 node structure and initialization (15%)
- Proper message/service/action definitions (15%)
- Accurate implementation of functionality (30%)

### Code Quality (25%)
- Clear, well-documented code (10%)
- Proper error handling (10%)
- Following ROS 2 best practices (5%)

### Validation and Testing (15%)
- Comprehensive testing of functionality (10%)
- Proper validation of inputs/outputs (5%)

## Learning Objectives Assessment

After completing these exercises, students should be able to:
- Create and run basic ROS 2 nodes for robot communication
- Implement different communication patterns (topics, services, actions)
- Work with robot description formats (URDF)
- Design launch files for multi-node systems
- Validate sensor data in robot systems