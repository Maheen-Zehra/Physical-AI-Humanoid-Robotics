# Exercises: Simulation Environments for Humanoid Robotics

## Exercise 1: Gazebo World Creation

### Objective
Create a custom Gazebo world with multiple obstacles and interaction zones for humanoid navigation.

### Requirements
1. Create a world file with at least 3 different obstacle types
2. Include a starting zone and target interaction area
3. Add visual markers to indicate different zones
4. Ensure the world includes proper lighting and physics properties

### Implementation Steps
1. Create a new world file in `src/simulation/gazebo_worlds/models/`
2. Define ground plane, lighting, and physics properties
3. Add obstacle models (boxes, cylinders, etc.) with appropriate poses
4. Create a humanoid spawn point and target area
5. Test the world by launching it in Gazebo

### Validation
- Verify all obstacles are properly positioned
- Confirm physics simulation runs smoothly
- Test that a humanoid model can be spawned in the world
- Ensure collision detection works properly

## Exercise 2: Sensor Integration

### Objective
Integrate multiple sensor types (IMU, camera, LiDAR) into your humanoid model and verify data publication.

### Requirements
1. Add IMU, camera, and LiDAR sensors to your URDF model
2. Configure realistic noise models for each sensor
3. Verify sensor data publication on appropriate ROS topics
4. Create a sensor validation node to monitor data quality

### Implementation Steps
1. Modify your URDF to include sensor definitions
2. Add Gazebo plugins for each sensor type
3. Create launch file to start simulation with sensors
4. Implement sensor validation node
5. Test sensor data publication and quality

### Validation
- Confirm IMU data shows appropriate gravity vector
- Verify camera publishes images at expected rate
- Check LiDAR provides range data within specified limits
- Validate noise characteristics match expected values

## Exercise 3: Interaction Behavior Implementation

### Objective
Implement a more sophisticated interaction behavior that responds to human presence and gestures.

### Requirements
1. Create a behavior that detects humans using sensor data
2. Implement approach, greeting, and response behaviors
3. Add safety checks to maintain appropriate distance
4. Include state management for different interaction phases

### Implementation Steps
1. Extend the interaction controller with human detection
2. Implement finite state machine for interaction states
3. Add safety distance maintenance
4. Create gesture recognition for simple commands
5. Test interaction behavior in simulation

### Validation
- Verify human detection works reliably
- Confirm safe distance maintenance
- Test state transitions work correctly
- Validate response to simple gestures

## Exercise 4: Unity Visualization Enhancement

### Objective
Create a custom visualization in Unity that displays robot state information and sensor data.

### Requirements
1. Add UI elements to display joint angles and sensor readings
2. Create visualization for sensor data (LiDAR points, camera feed)
3. Implement robot state indicators (battery, temperature, etc.)
4. Add interactive controls for simulation parameters

### Implementation Steps
1. Create UI canvas and elements in Unity scene
2. Implement ROS message subscribers for state data
3. Create visualizations for sensor data
4. Add interactive controls for simulation parameters
5. Test visualization with simulated robot data

### Validation
- Confirm all state information displays correctly
- Verify sensor visualizations update in real-time
- Test interactive controls affect simulation
- Validate UI remains responsive during simulation

## Exercise 5: ROS Bridge Implementation

### Objective
Implement bidirectional communication between Gazebo and Unity environments.

### Requirements
1. Create nodes that publish the same data to both environments
2. Implement synchronization between environments
3. Add visualization of Unity data in RViz
4. Create a master controller that coordinates both environments

### Implementation Steps
1. Create ROS nodes that publish to both Gazebo and Unity
2. Implement synchronization mechanisms for timing
3. Create visualization nodes for cross-environment feedback
4. Develop master controller for coordination
5. Test coordinated operation of both environments

### Validation
- Verify both environments receive consistent data
- Confirm timing synchronization works properly
- Test that changes in one environment reflect in the other
- Validate coordinated control operates smoothly

## Exercise 6: Performance Optimization

### Objective
Optimize simulation performance while maintaining adequate fidelity for development.

### Requirements
1. Measure simulation performance in terms of real-time factor
2. Identify bottlenecks in sensor simulation or rendering
3. Implement optimization techniques for identified bottlenecks
4. Validate that optimizations don't compromise simulation quality

### Implementation Steps
1. Profile current simulation performance
2. Identify computational bottlenecks
3. Apply optimization techniques (level of detail, update rates, etc.)
4. Measure performance improvement
5. Validate simulation quality remains acceptable

### Validation
- Confirm real-time factor improves
- Verify simulation quality remains sufficient
- Test that optimizations don't introduce artifacts
- Validate performance improvement is consistent

## Solutions

### Exercise 1 Solution

World file example:
```xml
<?xml version="1.0" ?>
<sdf version="1.6">
  <world name="custom_humanoid_world">
    <!-- Ground plane -->
    <include>
      <uri>model://ground_plane</uri>
    </include>

    <!-- Lighting -->
    <include>
      <uri>model://sun</uri>
    </include>

    <!-- Physics -->
    <physics name="default" type="ode">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1</real_time_factor>
      <gravity>0 0 -9.8</gravity>
    </physics>

    <!-- Obstacle 1: Box -->
    <model name="obstacle_box">
      <pose>2 0 0.5 0 0 0</pose>
      <link name="box_link">
        <visual name="visual">
          <geometry>
            <box><size>1 1 1</size></box>
          </geometry>
          <material>
            <ambient>0.8 0.2 0.2 1</ambient>
            <diffuse>0.8 0.2 0.2 1</diffuse>
          </material>
        </visual>
        <collision name="collision">
          <geometry>
            <box><size>1 1 1</size></box>
          </geometry>
        </collision>
        <inertial>
          <mass>1.0</mass>
          <inertia><ixx>0.166667</ixx><ixy>0</ixy><ixz>0</ixz><iyy>0.166667</iyy><iyz>0</iyz><izz>0.166667</izz></inertia>
        </inertial>
      </link>
    </model>

    <!-- Starting zone marker -->
    <model name="start_marker">
      <pose>-2 0 0.05 0 0 0</pose>
      <link name="marker_link">
        <visual name="visual">
          <geometry>
            <cylinder><radius>0.5</radius><length>0.1</length></cylinder>
          </geometry>
          <material>
            <ambient>0.2 0.8 0.2 1</ambient>
            <diffuse>0.2 0.8 0.2 1</diffuse>
          </material>
        </visual>
        <collision name="collision">
          <geometry>
            <cylinder><radius>0.5</radius><length>0.1</length></cylinder>
          </geometry>
        </collision>
        <inertial>
          <mass>0.1</mass>
          <inertia><ixx>0.00208</ixx><ixy>0</ixy><ixz>0</ixz><iyy>0.00208</iyy><iyz>0</iyz><izz>0.005</izz></inertia>
        </inertial>
      </link>
    </model>
  </world>
</sdf>
```

### Exercise 2 Solution

URDF sensor integration:
```xml
<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="humanoid_with_sensors">
  <!-- Include the base humanoid model -->
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

  <!-- IMU Sensor -->
  <joint name="base_to_imu" type="fixed">
    <parent link="base_link"/>
    <child link="imu_link"/>
    <origin xyz="0 0 0" rpy="0 0 0"/>
  </joint>

  <link name="imu_link">
    <inertial>
      <mass value="0.01"/>
      <inertia ixx="0.000001" ixy="0" ixz="0" iyy="0.000001" iyz="0" izz="0.000001"/>
    </inertial>
  </link>

  <!-- Gazebo plugin for IMU -->
  <gazebo reference="imu_link">
    <sensor name="imu_sensor" type="imu">
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
          <y>
            <noise type="gaussian">
              <mean>0.0</mean>
              <stddev>0.02</stddev>
            </noise>
          </y>
          <z>
            <noise type="gaussian">
              <mean>0.0</mean>
              <stddev>0.02</stddev>
            </noise>
          </z>
        </angular_velocity>
        <linear_acceleration>
          <x>
            <noise type="gaussian">
              <mean>0.0</mean>
              <stddev>1.7e-2</stddev>
            </noise>
          </x>
          <y>
            <noise type="gaussian">
              <mean>0.0</mean>
              <stddev>1.7e-2</stddev>
            </noise>
          </y>
          <z>
            <noise type="gaussian">
              <mean>0.0</mean>
              <stddev>1.7e-2</stddev>
            </noise>
          </z>
        </linear_acceleration>
      </imu>
      <plugin name="imu_plugin" filename="libgazebo_ros_imu.so">
        <ros>
          <namespace>imu</namespace>
          <remapping>~/out:=data</remapping>
        </ros>
        <initial_orientation_as_reference>false</initial_orientation_as_reference>
      </plugin>
    </sensor>
  </gazebo>
</robot>
```

## Assessment Rubric

### Technical Implementation (60%)
- Correct Gazebo world structure and physics (15%)
- Proper sensor integration and configuration (20%)
- Accurate ROS 2 communication setup (15%)
- Performance optimization techniques (10%)

### Code Quality (25%)
- Clear, well-documented code (10%)
- Proper error handling and validation (10%)
- Following ROS 2 and simulation best practices (5%)

### Validation and Testing (15%)
- Comprehensive testing of functionality (10%)
- Proper validation of simulation quality (5%)

## Learning Objectives Assessment

After completing these exercises, students should be able to:
- Create and configure Gazebo simulation environments
- Integrate sensors into robot models with realistic parameters
- Implement human-robot interaction behaviors in simulation
- Connect simulation environments to ROS 2 systems
- Optimize simulation performance for development workflows