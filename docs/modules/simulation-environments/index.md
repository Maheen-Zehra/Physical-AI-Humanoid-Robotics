# Simulation Environments for Humanoid Robotics

## Learning Objectives

By the end of this module, students will be able to:
- Understand the role of simulation in humanoid robotics development
- Set up and configure Gazebo simulation environments
- Import and configure humanoid models in simulation
- Implement sensor simulation for realistic data generation
- Create human-robot interaction scenarios in simulation
- Connect simulation environments to ROS 2 systems

## Key Concepts

- **Digital Twin**: A virtual representation of a physical system that mirrors its properties, state, and behavior
- **Physics Simulation**: Computational modeling of physical phenomena like gravity, collision, and dynamics
- **Sensor Simulation**: Virtual sensors that generate realistic data similar to physical sensors
- **Gazebo**: An open-source 3D simulation environment for robotics
- **ROS Integration**: Connecting simulation to ROS 2 for seamless development workflows
- **Human-Robot Interaction**: Scenarios where humans and robots interact in simulated environments

## Introduction to Simulation in Robotics

Simulation plays a crucial role in humanoid robotics development. It provides a safe, cost-effective, and efficient environment for testing algorithms, validating control systems, and training robot behaviors before deploying on physical hardware.

### Benefits of Simulation

1. **Safety**: Test dangerous scenarios without risk to hardware or humans
2. **Cost-Effectiveness**: No wear and tear on expensive hardware
3. **Repeatability**: Exact same conditions for consistent testing
4. **Speed**: Run simulations faster than real-time
5. **Debugging**: Detailed insight into robot state and behavior
6. **Scalability**: Test multiple robots simultaneously

### Simulation Fidelity

Simulation fidelity refers to how closely a simulation matches reality. Higher fidelity simulations include:
- Accurate physics modeling
- Realistic sensor noise and limitations
- Detailed environmental conditions
- Proper material properties

However, higher fidelity often means increased computational requirements.

## Gazebo Simulation Environment

Gazebo is a 3D simulation environment that provides realistic physics simulation, high-quality graphics, and convenient programmatic interfaces. It's widely used in the ROS community for robotics simulation.

### Gazebo Architecture

Gazebo consists of several key components:
- **Physics Engine**: Handles collision detection and dynamics (ODE, Bullet, Simbody)
- **Rendering Engine**: Provides 3D visualization (OGRE-based)
- **Sensor Simulation**: Generates realistic sensor data
- **Plugin System**: Extensible architecture for custom functionality

### Creating a Gazebo World

A Gazebo world file defines the environment where robots operate. Here's an example world file:

```xml
<?xml version="1.0" ?>
<sdf version="1.6">
  <world name="humanoid_world">
    <!-- Include a ground plane -->
    <include>
      <uri>model://ground_plane</uri>
    </include>

    <!-- Include sun for lighting -->
    <include>
      <uri>model://sun</uri>
    </include>

    <!-- Physics engine configuration -->
    <physics name="1ms" type="ode">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor>
      <real_time_update_rate>1000.0</real_time_update_rate>
      <gravity>0 0 -9.8</gravity>
    </physics>

    <!-- Import the humanoid model from URDF -->
    <model name="simple_humanoid">
      <pose>0 0 1 0 0 0</pose>
      <include>
        <uri>file://$(find humanoid_control)/models/simple_humanoid.urdf</uri>
      </include>
    </model>
  </world>
</sdf>
```

### Physics Configuration

Physics properties determine how objects behave in the simulation:

```yaml
# Physics properties configuration for humanoid simulation in Gazebo
physics_config:
  gravity:
    x: 0.0
    y: 0.0
    z: -9.8

  ode_config:
    max_step_size: 0.001
    real_time_update_rate: 1000.0
    real_time_factor: 1.0

    # Solver parameters
    solver_type: "quick"
    iters: 100
    sor: 1.3

    # Constraint parameters
    cfm: 0.0
    erp: 0.2

    # Contact parameters
    contact_surface_layer: 0.001
    contact_max_correcting_vel: 100.0

  humanoid_physics:
    mass_scaling_factor: 1.0
    linear_damping: 0.01
    angular_damping: 0.01
    mu1: 500.0
    mu2: 500.0
```

## Sensor Simulation

Accurate sensor simulation is crucial for developing robust robot systems. Sensors in simulation should generate data similar to their real-world counterparts, including appropriate noise and limitations.

### IMU Simulation

An IMU (Inertial Measurement Unit) sensor measures linear acceleration and angular velocity. In Gazebo, IMU sensors can be configured with realistic noise models:

```xml
<xacro:macro name="imu_sensor" params="name parent_link *origin">
  <gazebo reference="${parent_link}">
    <sensor name="${name}_imu" type="imu">
      <always_on>true</always_on>
      <update_rate>100</update_rate>
      <imu>
        <angular_velocity>
          <x>
            <noise type="gaussian">
              <mean>0.0</mean>
              <stddev>0.02</stddev>
            </noise>
          </x>
        </angular_velocity>
        <linear_acceleration>
          <x>
            <noise type="gaussian">
              <mean>0.0</mean>
              <stddev>1.7e-2</stddev>
            </noise>
          </x>
        </linear_acceleration>
      </imu>
      <plugin name="${name}_imu_controller" filename="libgazebo_ros_imu.so">
        <ros>
          <namespace>${name}</namespace>
          <remapping>~/out:=imu/data</remapping>
        </ros>
      </plugin>
    </sensor>
  </gazebo>
</xacro:macro>
```

### Camera Simulation

Camera sensors generate visual data for perception tasks. Configuration includes resolution, field of view, and noise characteristics:

```xml
<xacro:macro name="camera_sensor" params="name parent_link *origin">
  <gazebo reference="${parent_link}">
    <sensor name="${name}_camera" type="camera">
      <always_on>true</always_on>
      <update_rate>30</update_rate>
      <camera name="head">
        <horizontal_fov>1.047</horizontal_fov>
        <image>
          <width>640</width>
          <height>480</height>
          <format>R8G8B8</format>
        </image>
        <clip>
          <near>0.1</near>
          <far>100</far>
        </clip>
      </camera>
      <plugin name="${name}_camera_controller" filename="libgazebo_ros_camera.so">
        <ros>
          <namespace>${name}</namespace>
          <remapping>image_raw:=camera/image_raw</remapping>
          <remapping>camera_info:=camera/camera_info</remapping>
        </ros>
      </plugin>
    </sensor>
  </gazebo>
</xacro:macro>
```

### LiDAR Simulation

LiDAR sensors provide 3D spatial information through laser ranging:

```xml
<xacro:macro name="lidar_sensor" params="name parent_link *origin">
  <gazebo reference="${parent_link}">
    <sensor name="${name}_lidar" type="ray">
      <always_on>true</always_on>
      <update_rate>10</update_rate>
      <ray>
        <scan>
          <horizontal>
            <samples>720</samples>
            <resolution>1</resolution>
            <min_angle>-3.14159</min_angle>
            <max_angle>3.14159</max_angle>
          </horizontal>
        </scan>
        <range>
          <min>0.1</min>
          <max>30.0</max>
          <resolution>0.01</resolution>
        </range>
      </ray>
      <plugin name="${name}_lidar_controller" filename="libgazebo_ros_ray_sensor.so">
        <ros>
          <namespace>${name}</namespace>
          <remapping>~/out:=lidar/points</remapping>
        </ros>
        <output_type>sensor_msgs/LaserScan</output_type>
      </plugin>
    </sensor>
  </gazebo>
</xacro:macro>
```

## Unity Digital Twin Environment

Unity provides high-fidelity visualization and rendering capabilities that complement physics-based simulation. The Unity Robotics package enables connection between Unity and ROS 2 systems.

### Unity ROS Connection

The Unity ROS TCP Connector allows Unity to communicate with ROS 2 systems:

```csharp
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using Unity.Robotics.ROSTCPConnector.MessageTypes.Std;
using Unity.Robotics.ROSTCPConnector.MessageTypes.Sensor;

public class ROSConnection : MonoBehaviour
{
    ROSConnection ros;

    [SerializeField] private string cmdVelTopic = "/cmd_vel";
    [SerializeField] private string jointStatesTopic = "/joint_states";
    [SerializeField] private string imuTopic = "/imu/data";
    [SerializeField] private string cameraTopic = "/camera/image_raw";

    void Start()
    {
        ros = ROSConnection.instance;

        // Subscribe to ROS topics
        ros.Subscribe<Float32Msg>(cmdVelTopic, OnVelocityCommandReceived);
        ros.Subscribe<JointStateMsg>(jointStatesTopic, OnJointStateReceived);

        // Start publishing sensor data
        StartCoroutine(PublishIMUData());
        StartCoroutine(PublishCameraData());
    }

    void OnVelocityCommandReceived(Float32Msg cmdVel)
    {
        // Process velocity command and move humanoid
        float velocity = cmdVel.data;
        transform.Translate(Vector3.forward * velocity * Time.deltaTime);
    }

    IEnumerator PublishIMUData()
    {
        while (true)
        {
            var imuMsg = new ImuMsg();
            imuMsg.header = new std_msgs.Header();
            imuMsg.header.stamp = new builtin_interfaces.Time();
            imuMsg.header.frame_id = "imu_link";

            // Simulate linear acceleration (with some noise)
            imuMsg.linear_acceleration.x = Random.Range(-0.1f, 0.1f);
            imuMsg.linear_acceleration.y = Random.Range(-0.1f, 0.1f);
            imuMsg.linear_acceleration.z = -9.8f + Random.Range(-0.2f, 0.2f);

            ros.Publish(imuTopic, imuMsg);

            yield return new WaitForSeconds(0.01f); // 100Hz
        }
    }
}
```

### Visualization Configuration

Unity allows detailed configuration of visual properties for the digital twin:

```yaml
%YAML 1.1
%TAG !u! tag:unity3d.com,2011:
--- !u!114 &11400000
MonoBehaviour:
  renderingSettings:
    shadowQuality: 2
    antiAliasing: 2
    textureQuality: 1
    anisotropicFiltering: 2
  humanoidModelSettings:
    baseColor: {r: 0.8, g: 0.8, b: 0.8, a: 1}
    highlightColor: {r: 0.9, g: 0.6, b: 0.2, a: 1}
    materialType: 0
  cameraSettings:
    fieldOfView: 60
    clippingPlanes:
      near: 0.3
      far: 1000
    defaultPosition: {x: 0, y: 1, z: -10}
    defaultRotation: {x: 0, y: 0, z: 0, w: 1}
  lightingSettings:
    ambientIntensity: 1
    directionalLightIntensity: 1
    skyboxEnabled: 1
  interactionSettings:
    enableMouseLook: 1
    enableTouchControls: 0
    interactionDistance: 10
    highlightOnHover: 1
```

## Human-Robot Interaction in Simulation

Creating realistic human-robot interaction scenarios is essential for developing socially-aware robots. These scenarios can test perception, planning, and control algorithms in a safe environment.

### Interaction Controller

An interaction controller manages how the robot responds to humans or objects in the environment:

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan, Image, Imu
from std_msgs.msg import Float32
import math


class InteractionController(Node):
    """
    Controller for human-robot interaction scenarios in simulation.
    Handles approach behaviors, obstacle avoidance, and interaction responses.
    """

    def __init__(self):
        super().__init__('interaction_controller')

        # Parameters
        self.declare_parameter('interaction_distance', 2.0)
        self.declare_parameter('object_approach_speed', 0.5)
        self.interaction_distance = self.get_parameter('interaction_distance').value
        self.approach_speed = self.get_parameter('object_approach_speed').value

        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)

        # Subscribers
        self.laser_sub = self.create_subscription(
            LaserScan,
            'scan',
            self.laser_callback,
            10
        )

        # Timer for control loop
        self.timer = self.create_timer(0.1, self.control_loop)  # 10Hz

        # State variables
        self.laser_data = None
        self.near_obstacle = False

        self.get_logger().info('Interaction controller initialized')

    def laser_callback(self, msg):
        """Process laser scan data to detect obstacles and interaction targets."""
        self.laser_data = msg

        # Simple obstacle detection: check if anything is within interaction distance
        min_distance = min(msg.ranges)
        self.near_obstacle = min_distance < self.interaction_distance

        if self.near_obstacle:
            self.get_logger().info(f'Obstacle detected at {min_distance:.2f}m')

    def control_loop(self):
        """Main control loop for interaction behaviors."""
        if self.laser_data is None:
            return

        cmd_vel = Twist()

        # Simple interaction behavior: approach when target detected
        if self.near_obstacle:
            # If obstacle is detected in front, approach it slowly
            front_distance = self.laser_data.ranges[len(self.laser_data.ranges)//2]  # Front reading

            if front_distance > 0.5:  # Don't get too close
                cmd_vel.linear.x = min(self.approach_speed, (front_distance - 0.5) * 0.5)
            else:
                cmd_vel.linear.x = 0.0  # Stop when close enough
        else:
            # Explore: move forward until obstacle detected
            cmd_vel.linear.x = self.approach_speed
            cmd_vel.angular.z = 0.2  # Gentle turn to explore

        # Publish command
        self.cmd_vel_pub.publish(cmd_vel)

    def detect_interaction_target(self):
        """Detect potential interaction targets from sensor data."""
        if self.laser_data:
            min_idx = 0
            min_dist = float('inf')

            for i, dist in enumerate(self.laser_data.ranges):
                if 0 < dist < min_dist and dist < self.interaction_distance:
                    min_dist = dist
                    min_idx = i

            if min_dist < float('inf'):
                angle_to_target = self.laser_data.angle_min + min_idx * self.laser_data.angle_increment
                return True

        return False


def main(args=None):
    rclpy.init(args=args)
    controller = InteractionController()

    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        controller.get_logger().info('Shutting down interaction controller')
    finally:
        # Stop the robot before shutting down
        cmd_vel = Twist()
        controller.cmd_vel_pub.publish(cmd_vel)
        controller.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

## Launch Files for Simulation

Launch files coordinate the startup of multiple simulation components:

```python
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # Declare launch arguments
    world_file_arg = DeclareLaunchArgument(
        'world',
        default_value='humanoid_interaction.world',
        description='Choose one of the world files from `/gazebo_worlds/models`'
    )

    # Get the launch directory
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')

    # Create launch description
    ld = LaunchDescription()

    # Add argument declaration
    ld.add_action(world_file_arg)

    # Launch Gazebo with the interaction world
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gazebo.launch.py')
        ),
        launch_arguments={
            'world': PathJoinSubstitution([get_package_share_directory('gazebo_worlds'),
                                          'models', LaunchConfiguration('world')])
        }.items()
    )

    # Spawn the humanoid model
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', 'simple_humanoid',
            '-file', PathJoinSubstitution([get_package_share_directory('humanoid_control'),
                                          'models', 'simple_humanoid.urdf']),
            '-x', '0', '-y', '0', '-z', '1'
        ],
        output='screen'
    )

    # Launch the interaction controller node
    interaction_controller = Node(
        package='gazebo_worlds',
        executable='interaction_controller',
        name='interaction_controller',
        output='screen',
        parameters=[
            {'interaction_distance': 2.0},
            {'object_approach_speed': 0.5}
        ]
    )

    # Add all actions to launch description
    ld.add_action(gazebo)
    ld.add_action(spawn_entity)
    ld.add_action(interaction_controller)

    return ld
```

## Best Practices for Simulation

### Model Accuracy
- Ensure URDF models accurately represent physical properties
- Include appropriate inertial parameters
- Validate collision geometries match visual geometries

### Sensor Realism
- Add realistic noise models to sensor data
- Include sensor limitations (range, resolution, update rate)
- Consider environmental factors (lighting, weather) where applicable

### Performance Optimization
- Balance simulation fidelity with computational requirements
- Use appropriate update rates for different systems
- Optimize mesh complexity for collision and visual models

### Validation
- Compare simulation results with real-world data when possible
- Test on real hardware to validate simulation assumptions
- Document differences between simulation and reality

## Exercises

1. **World Creation**: Create a new Gazebo world with obstacles and interaction zones.

2. **Sensor Integration**: Add multiple sensor types to your humanoid model and verify data publication.

3. **Interaction Behavior**: Implement a more sophisticated interaction behavior (following, avoidance, etc.).

4. **Unity Visualization**: Create a custom visualization for your robot's state in Unity.

5. **ROS Bridge**: Implement bidirectional communication between Gazebo and Unity environments.

## Summary

This module covered the fundamentals of simulation environments for humanoid robotics, including Gazebo physics simulation, Unity visualization, sensor modeling, and human-robot interaction scenarios. Simulation provides a safe and efficient environment for developing and testing humanoid robot systems before deployment on physical hardware.

## Review Questions

1. What are the main benefits of using simulation in humanoid robotics?
2. How do physics parameters affect robot behavior in simulation?
3. What factors should be considered when creating realistic sensor models?
4. How can Unity complement Gazebo simulation for humanoid robotics?
5. What are important considerations for validating simulation results?

## References

- Gazebo Documentation: http://gazebosim.org/
- ROS 2 with Gazebo: https://github.com/ros-simulation/gazebo_ros_pkgs
- Unity Robotics: https://github.com/Unity-Technologies/ROS-TCP-Connector
- Simulation Best Practices: https://arxiv.org/abs/2008.05443