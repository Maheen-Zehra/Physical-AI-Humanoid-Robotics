# AI Perception and Navigation for Humanoid Robotics

## Learning Objectives

By the end of this module, students will be able to:
- Implement Visual SLAM (VSLAM) systems for humanoid robot localization and mapping
- Set up and configure Nav2 navigation pipelines for bipedal humanoid robots
- Design perception modules that process synthetic sensor input from simulation
- Integrate perception and navigation systems for autonomous humanoid operation
- Create obstacle avoidance algorithms tailored for humanoid locomotion
- Evaluate and validate perception and navigation performance in simulation

## Key Concepts

- **Visual SLAM (VSLAM)**: Simultaneous Localization and Mapping using visual sensors
- **Bipedal Navigation**: Path planning and control specifically designed for walking robots
- **Synthetic Sensor Processing**: Processing simulated sensor data for perception tasks
- **Isaac ROS**: NVIDIA's robotics software development kit for perception and navigation
- **Obstacle Avoidance**: Algorithms for detecting and avoiding obstacles in humanoid path
- **Humanoid Locomotion**: Walking patterns and gait planning for bipedal robots
- **Perception Pipeline**: Data processing flow from raw sensors to actionable information

## Introduction to AI Perception in Humanoid Robotics

Artificial Intelligence perception in humanoid robotics encompasses the technologies and algorithms that enable robots to understand their environment and navigate safely. Unlike wheeled robots, humanoid robots face unique challenges due to their bipedal nature, including balance constraints, step planning, and complex gait patterns.

### Challenges of Humanoid Perception

1. **Balance Constraints**: Humanoid robots must maintain balance while performing perception tasks
2. **Sensor Height**: Head-mounted sensors provide different perspectives than ground-level sensors
3. **Dynamic Motion**: Walking motion introduces vibrations and movement that affect sensor data
4. **Footstep Planning**: Navigation must consider where feet can be placed
5. **Obstacle Negotiation**: Ability to step over obstacles or navigate around them

### AI Perception Components

A complete AI perception system for humanoid robotics includes:
- **Visual SLAM**: For simultaneous localization and mapping
- **Object Detection**: For identifying and tracking objects in the environment
- **Scene Understanding**: For interpreting the environment's structure
- **Sensor Fusion**: For combining data from multiple sensors
- **Path Planning**: For generating safe and feasible navigation paths

## Visual SLAM (VSLAM) for Humanoid Robots

Visual SLAM enables humanoid robots to build maps of their environment while simultaneously determining their position within that map. This is particularly important for humanoid robots operating in unknown environments.

### VSLAM Architecture

The VSLAM pipeline typically includes:

1. **Feature Detection**: Identifying distinctive features in images
2. **Feature Tracking**: Following features across consecutive frames
3. **Pose Estimation**: Determining camera motion between frames
4. **Mapping**: Building a map of the environment
5. **Loop Closure**: Recognizing previously visited locations

### VSLAM Implementation in Isaac Sim

Isaac Sim provides specialized VSLAM capabilities through Isaac ROS packages:

```python
#!/usr/bin/env python3
"""
VSLAM Pipeline for Humanoid Robot in Isaac Sim Environment
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo, Imu
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped, Point
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import Header
import numpy as np
import tf2_ros
from tf2_ros import TransformBroadcaster
from builtin_interfaces.msg import Time
import message_filters
from cv_bridge import CvBridge
import cv2


class VSLAMPipeline(Node):
    """
    VSLAM (Visual Simultaneous Localization and Mapping) Pipeline
    for humanoid robot localization and mapping in Isaac Sim environment.
    """

    def __init__(self):
        super().__init__('vslam_pipeline')

        # Parameters
        self.declare_parameter('tracking_frame', 'camera_link')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('publish_rate', 30.0)

        self.tracking_frame = self.get_parameter('tracking_frame').value
        self.map_frame = self.get_parameter('map_frame').value
        self.publish_rate = self.get_parameter('publish_rate').value

        # CV Bridge for image processing
        self.cv_bridge = CvBridge()

        # TF broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)

        # Subscribers - synchronized camera and IMU data
        self.image_sub = message_filters.Subscriber(self, Image, '/camera/rgb/image_raw')
        self.info_sub = message_filters.Subscriber(self, CameraInfo, '/camera/rgb/camera_info')
        self.imu_sub = message_filters.Subscriber(self, Imu, '/imu/data')

        # Synchronize image, camera info, and IMU data
        self.sync = message_filters.ApproximateTimeSynchronizer(
            [self.image_sub, self.info_sub, self.imu_sub],
            queue_size=10,
            slop=0.1
        )
        self.sync.registerCallback(self.sync_callback)

        # Publishers
        self.odom_pub = self.create_publisher(Odometry, '/visual_slam/odometry', 10)
        self.path_pub = self.create_publisher(Path, '/visual_slam/path', 10)
        self.feature_pub = self.create_publisher(MarkerArray, '/visual_slam/features', 10)
        self.map_pub = self.create_publisher(MarkerArray, '/visual_slam/map', 10)

        # State variables
        self.previous_features = None
        self.current_pose = np.eye(4)  # 4x4 transformation matrix
        self.path = Path()
        self.path.header.frame_id = self.map_frame

        # Feature detection parameters
        self.feature_detector = cv2.ORB_create(
            nfeatures=1000,
            scaleFactor=1.2,
            nlevels=8,
            edgeThreshold=31,
            patchSize=31
        )

        # Timer for publishing transforms
        self.timer = self.create_timer(1.0/self.publish_rate, self.publish_transforms)

        self.get_logger().info('VSLAM Pipeline initialized')

    def sync_callback(self, image_msg, info_msg, imu_msg):
        """
        Process synchronized image, camera info, and IMU data for VSLAM.
        """
        try:
            # Convert ROS image to OpenCV
            cv_image = self.cv_bridge.imgmsg_to_cv2(image_msg, 'rgb8')

            # Detect features in the image
            keypoints, descriptors = self.feature_detector.detectAndCompute(cv_image, None)

            if keypoints is not None and len(keypoints) > 10:
                # Convert keypoints to numpy array
                current_features = np.array([kp.pt for kp in keypoints])

                if self.previous_features is not None:
                    # Track features between frames
                    pose_update = self.track_features(self.previous_features, current_features)

                    # Update current pose based on feature tracking
                    if pose_update is not None:
                        self.current_pose = self.current_pose @ pose_update

                        # Publish odometry
                        self.publish_odometry(image_msg.header.stamp)

                        # Add pose to path
                        self.add_pose_to_path(image_msg.header.stamp)

                # Store current features for next iteration
                self.previous_features = current_features

                # Publish feature visualization
                self.publish_features(keypoints, image_msg.header.stamp)

        except Exception as e:
            self.get_logger().error(f'Error in VSLAM processing: {str(e)}')

    def track_features(self, prev_features, curr_features):
        """
        Track features between frames to estimate motion.
        """
        try:
            # Use optical flow to track features
            if len(prev_features) >= 4 and len(curr_features) >= 4:
                # Calculate transformation between feature sets
                # This is a simplified approach - real VSLAM would use more sophisticated methods
                if len(prev_features) <= len(curr_features):
                    matched_prev = prev_features
                    matched_curr = curr_features[:len(prev_features)]
                else:
                    matched_prev = prev_features[:len(curr_features)]
                    matched_curr = curr_features

                # Calculate displacement of features
                displacement = np.mean(matched_curr - matched_prev, axis=0)

                # Convert displacement to pose update (simplified)
                pose_update = np.eye(4)
                pose_update[0, 3] = displacement[0] * 0.001  # Scale factor
                pose_update[1, 3] = displacement[1] * 0.001  # Scale factor

                # Include IMU data for rotation
                # This is a simplified approach - real implementation would integrate IMU properly
                return pose_update
        except Exception as e:
            self.get_logger().warn(f'Feature tracking error: {str(e)}')

        return None

    def publish_odometry(self, stamp):
        """
        Publish odometry message with current pose estimate.
        """
        odom_msg = Odometry()
        odom_msg.header.stamp = stamp
        odom_msg.header.frame_id = self.map_frame
        odom_msg.child_frame_id = self.tracking_frame

        # Extract position from transformation matrix
        odom_msg.pose.pose.position.x = self.current_pose[0, 3]
        odom_msg.pose.pose.position.y = self.current_pose[1, 3]
        odom_msg.pose.pose.position.z = self.current_pose[2, 3]

        # Convert rotation matrix to quaternion (simplified)
        # In a real implementation, this would properly convert the rotation matrix
        odom_msg.pose.pose.orientation.w = 1.0  # Simplified - no rotation for now

        # Publish odometry
        self.odom_pub.publish(odom_msg)

    def add_pose_to_path(self, stamp):
        """
        Add current pose to path for visualization.
        """
        pose_stamped = PoseStamped()
        pose_stamped.header.stamp = stamp
        pose_stamped.header.frame_id = self.map_frame
        pose_stamped.pose.position.x = self.current_pose[0, 3]
        pose_stamped.pose.position.y = self.current_pose[1, 3]
        pose_stamped.pose.position.z = self.current_pose[2, 3]
        pose_stamped.pose.orientation.w = 1.0

        self.path.poses.append(pose_stamped)

        # Limit path length to prevent memory issues
        if len(self.path.poses) > 1000:
            self.path.poses.pop(0)

        # Publish updated path
        self.path_pub.publish(self.path)

    def publish_features(self, keypoints, stamp):
        """
        Publish feature visualization markers.
        """
        marker_array = MarkerArray()

        for i, kp in enumerate(keypoints[:50]):  # Limit to 50 features for performance
            marker = Marker()
            marker.header.stamp = stamp
            marker.header.frame_id = self.tracking_frame
            marker.ns = "features"
            marker.id = i
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD

            # Position in camera frame (simplified)
            marker.pose.position.x = kp.pt[0] * 0.001  # Scale down
            marker.pose.position.y = kp.pt[1] * 0.001
            marker.pose.position.z = 0.0
            marker.pose.orientation.w = 1.0

            marker.scale.x = 0.01
            marker.scale.y = 0.01
            marker.scale.z = 0.01

            marker.color.r = 1.0
            marker.color.g = 0.0
            marker.color.b = 0.0
            marker.color.a = 1.0

            marker_array.markers.append(marker)

        self.feature_pub.publish(marker_array)

    def publish_transforms(self):
        """
        Publish TF transforms for the current pose.
        """
        if self.current_pose is not None:
            # Create transform message
            t = tf2_ros.TransformStamped()
            t.header.stamp = self.get_clock().now().to_msg()
            t.header.frame_id = self.map_frame
            t.child_frame_id = self.tracking_frame

            # Set translation
            t.transform.translation.x = self.current_pose[0, 3]
            t.transform.translation.y = self.current_pose[1, 3]
            t.transform.translation.z = self.current_pose[2, 3]

            # Set rotation (simplified)
            t.transform.rotation.w = 1.0

            # Broadcast transform
            self.tf_broadcaster.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    vslam_pipeline = VSLAMPipeline()

    try:
        rclpy.spin(vslam_pipeline)
    except KeyboardInterrupt:
        vslam_pipeline.get_logger().info('Shutting down VSLAM pipeline')
    finally:
        vslam_pipeline.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

### VSLAM Configuration

The VSLAM system is configured with parameters that optimize it for humanoid applications:

```yaml
# Isaac Sim VSLAM Configuration
isaac_ros_config:

  # VSLAM settings
  vslam:
    enable: true
    camera_topics: ["/camera/rgb/image_raw", "/camera/depth/image_raw"]
    imu_topic: "/imu/data"
    output_odometry_topic: "/visual_slam/odometry"
    output_path_topic: "/visual_slam/path"
    tracking_frame: "camera_link"
    map_frame: "map"

    # Feature extraction parameters
    feature_detector:
      max_features: 1000
      min_distance: 20.0
      quality_level: 0.01
      k: 0.04

    # Tracking parameters
    tracker:
      max_level: 3
      max_iterations: 30
      epsilon: 0.01
      min_eigen_threshold: 1e-4

    # Bundle adjustment parameters
    bundle_adjustment:
      enable: true
      max_iterations: 10
      convergence_threshold: 1e-4
```

## Navigation for Bipedal Humanoids

Navigation for bipedal humanoid robots differs significantly from wheeled robot navigation due to the constraints of walking motion and balance requirements.

### Humanoid-Specific Navigation Challenges

1. **Step Planning**: Each foot placement must be carefully planned
2. **Balance Maintenance**: Robot must maintain balance during navigation
3. **Gait Patterns**: Walking patterns must be dynamically adjusted
4. **Obstacle Negotiation**: Ability to step over or around obstacles
5. **Terrain Adaptation**: Adapting to uneven surfaces

### Nav2 Configuration for Humanoid Robots

Nav2 must be configured with humanoid-specific parameters:

```yaml
# Nav2 Configuration for Humanoid Robot
# Parameters for navigation stack in Isaac Sim environment

controller_server:
  ros__parameters:
    use_sim_time: True
    controller_frequency: 20.0
    min_x_velocity_threshold: 0.001
    min_y_velocity_threshold: 0.5
    min_theta_velocity_threshold: 0.001
    progress_checker_plugin: "progress_checker"
    goal_checker_plugin: "goal_checker"
    controller_plugins: ["FollowPath"]

    # Humanoid-specific controller
    FollowPath:
      plugin: "nav2_mppi_controller::MPPIC"
      debug_visualizations: false
      goal_dist_tol: 0.25
      xy_goal_tol: 0.25
      yaw_goal_tol: 0.25
      sim_time: 1.7
      control_time_step: 0.05
      disc_time: 0.2
      max_iteration: 3
      reaction_time: 0.3
      max_robot_vel_x: 0.5  # Slower for stability
      min_robot_vel_x: -0.15
      max_robot_vel_y: 0.5
      max_robot_vel_theta: 0.3
      min_robot_acc_x: -2.5
      max_robot_acc_x: 2.5
      min_robot_acc_y: -2.5
      max_robot_acc_y: 2.5
      min_robot_jerk_x: -5.0
      max_robot_jerk_x: 5.0
      min_robot_jerk_y: -5.0
      max_robot_jerk_y: 5.0
      min_robot_jerk_theta: -5.0
      max_robot_jerk_theta: 5.0
      robot_stuck_check: true
      robot_stuck_distance: 0.05
      robot_stuck_time_threshold: 1.0
      refine_plan: true
      refine_plan_frequency: 5.0
      refine_plan_minX: -1.0
      refine_plan_maxX: 1.0
      refine_plan_minY: -1.0
      refine_plan_maxY: 1.0
      refine_plan_resolution: 0.05
      cost_function_plugins: ["prefer_forward_path"]
      prefer_forward_path:
        plugin: "nav2_mppi_controller::PreferForwardPath"
        weight: 1.0
```

### Humanoid Navigation Node

The humanoid-specific navigation node handles bipedal locomotion:

```python
#!/usr/bin/env python3
"""
Humanoid Navigation Node for Isaac Sim Environment

This module implements navigation specifically for humanoid robots,
addressing bipedal locomotion challenges and obstacle avoidance.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped, Point
from nav_msgs.msg import Path, Odometry
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Header
from visualization_msgs.msg import Marker, MarkerArray
import numpy as np
import math
from scipy.spatial.transform import Rotation as R
import tf2_ros
from tf2_ros import TransformBroadcaster
from builtin_interfaces.msg import Time


class HumanoidNavigation(Node):
    """
    Navigation system specifically designed for humanoid robots in Isaac Sim.
    Handles bipedal locomotion challenges, step planning, and obstacle avoidance.
    """

    def __init__(self):
        super().__init__('humanoid_navigation')

        # Parameters for humanoid-specific navigation
        self.declare_parameter('step_height', 0.15)  # Maximum step height
        self.declare_parameter('max_step_width', 0.3)  # Maximum step width
        self.declare_parameter('foot_separation', 0.2)  # Distance between feet
        self.declare_parameter('base_frame', 'base_footprint')
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('navigation_frequency', 10.0)

        self.step_height = self.get_parameter('step_height').value
        self.max_step_width = self.get_parameter('max_step_width').value
        self.foot_separation = self.get_parameter('foot_separation').value
        self.base_frame = self.get_parameter('base_frame').value
        self.odom_frame = self.get_parameter('odom_frame').value
        self.map_frame = self.get_parameter('map_frame').value
        self.nav_freq = self.get_parameter('navigation_frequency').value

        # Subscribers
        self.goal_sub = self.create_subscription(
            PoseStamped,
            '/goal_pose',
            self.goal_callback,
            10
        )

        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )

        self.scan_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10
        )

        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, '/humanoid/cmd_vel', 10)
        self.local_plan_pub = self.create_publisher(Path, '/local_plan', 10)
        self.global_plan_pub = self.create_publisher(Path, '/global_plan', 10)
        self.footstep_pub = self.create_publisher(MarkerArray, '/footsteps', 10)
        self.collision_risk_pub = self.create_publisher(MarkerArray, '/collision_risk', 10)

        # TF broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)

        # State variables
        self.current_pose = None
        self.current_twist = None
        self.goal_pose = None
        self.scan_data = None
        self.local_plan = []
        self.global_plan = []
        self.footsteps = []
        self.collision_risks = []

        # Navigation state
        self.is_navigating = False
        self.current_step_index = 0

        # Timer for navigation control
        self.nav_timer = self.create_timer(1.0/self.nav_freq, self.navigation_control)

        # Initialize
        self.get_logger().info('Humanoid Navigation Node initialized')

    def goal_callback(self, msg):
        """
        Handle new navigation goal.
        """
        self.goal_pose = msg.pose
        self.get_logger().info(f'New goal received: ({msg.pose.position.x}, {msg.pose.position.y})')

        # Start navigation to new goal
        self.plan_path()
        self.is_navigating = True

    def odom_callback(self, msg):
        """
        Handle odometry updates.
        """
        self.current_pose = msg.pose.pose
        self.current_twist = msg.twist.twist

    def scan_callback(self, msg):
        """
        Handle laser scan data for obstacle detection.
        """
        self.scan_data = msg
        self.detect_obstacles()

    def plan_path(self):
        """
        Plan a global path to the goal considering humanoid constraints.
        """
        if self.current_pose is None or self.goal_pose is None:
            return

        # Simplified path planning - in reality, this would use A* or similar
        start = np.array([self.current_pose.position.x, self.current_pose.position.y])
        goal = np.array([self.goal_pose.position.x, self.goal_pose.position.y])

        # Calculate direction vector
        direction = goal - start
        distance = np.linalg.norm(direction)

        if distance < 0.1:  # Already at goal
            self.is_navigating = False
            return

        # Normalize direction
        direction = direction / distance

        # Create path points
        path_points = []
        step_size = 0.2  # 20cm steps
        num_steps = int(distance / step_size)

        for i in range(num_steps + 1):
            t = i / num_steps if num_steps > 0 else 0
            point = start + t * (goal - start)

            pose_stamped = PoseStamped()
            pose_stamped.header.frame_id = self.map_frame
            pose_stamped.header.stamp = self.get_clock().now().to_msg()
            pose_stamped.pose.position.x = point[0]
            pose_stamped.pose.position.y = point[1]
            pose_stamped.pose.position.z = 0.0
            pose_stamped.pose.orientation.w = 1.0

            path_points.append(pose_stamped)

        self.global_plan = path_points
        self.local_plan = path_points[:20]  # Next 20 points for local planning

        # Publish global plan
        global_path_msg = Path()
        global_path_msg.header.frame_id = self.map_frame
        global_path_msg.header.stamp = self.get_clock().now().to_msg()
        global_path_msg.poses = [p for p in path_points]
        self.global_plan_pub.publish(global_path_msg)

    def detect_obstacles(self):
        """
        Detect obstacles from laser scan data and assess collision risk.
        """
        if self.scan_data is None:
            return

        # Clear previous collision risks
        self.collision_risks = []

        # Check for obstacles in path
        min_distance = float('inf')
        collision_risk_angle = None

        for i, range_val in enumerate(self.scan_data.ranges):
            if not (float('inf') > range_val > 0.1):  # Valid range
                continue

            angle = self.scan_data.angle_min + i * self.scan_data.angle_increment

            # Calculate position of obstacle in robot frame
            obstacle_x = range_val * math.cos(angle)
            obstacle_y = range_val * math.sin(angle)

            # Check if obstacle is in front of robot (within navigation cone)
            if abs(obstacle_y) < 0.5 and obstacle_x > 0.1 and obstacle_x < 2.0:  # In front, within 2m
                if range_val < min_distance:
                    min_distance = range_val
                    collision_risk_angle = angle

                # Add to collision risks for visualization
                risk_point = Point()
                risk_point.x = obstacle_x
                risk_point.y = obstacle_y
                risk_point.z = 0.0
                self.collision_risks.append(risk_point)

        # Publish collision risks for visualization
        self.publish_collision_risks()

    def assess_traversability(self, scan_data, step_direction):
        """
        Assess if a step direction is traversable for humanoid robot.
        """
        if scan_data is None:
            return True

        # Check if the step direction is clear of obstacles
        step_angle = math.atan2(step_direction[1], step_direction[0])

        # Find the corresponding laser beam
        beam_index = int((step_angle - scan_data.angle_min) / scan_data.angle_increment)

        if 0 <= beam_index < len(scan_data.ranges):
            distance = scan_data.ranges[beam_index]
            return distance > 0.5  # Minimum clearance of 50cm

        return True

    def navigation_control(self):
        """
        Main navigation control loop for humanoid robot.
        """
        if not self.is_navigating or self.current_pose is None or self.goal_pose is None:
            # Stop robot if not navigating
            cmd_vel = Twist()
            self.cmd_vel_pub.publish(cmd_vel)
            return

        if self.local_plan and len(self.local_plan) > 0:
            # Get next waypoint in local plan
            next_waypoint = self.local_plan[0]

            # Calculate direction to next waypoint
            current_pos = np.array([self.current_pose.position.x, self.current_pose.position.y])
            waypoint_pos = np.array([next_waypoint.pose.position.x, next_waypoint.pose.position.y])

            direction = waypoint_pos - current_pos
            distance_to_waypoint = np.linalg.norm(direction)

            # Check if we need to replan (obstacle detected)
            if self.scan_data and self.detect_immediate_obstacle():
                self.get_logger().warn('Obstacle detected, replanning...')
                self.plan_path()
                return

            # If close to waypoint, move to next
            if distance_to_waypoint < 0.3 and len(self.local_plan) > 1:
                self.local_plan.pop(0)
                return

            # Calculate velocity command
            cmd_vel = Twist()

            # Calculate angle to waypoint
            angle_to_waypoint = math.atan2(direction[1], direction[0])

            # Get current robot orientation
            q = self.current_pose.orientation
            current_yaw = self.quaternion_to_yaw(q.x, q.y, q.z, q.w)

            # Calculate angle difference
            angle_diff = self.normalize_angle(angle_to_waypoint - current_yaw)

            # Set linear and angular velocities based on angle difference
            if abs(angle_diff) > 0.2:  # Need to turn
                cmd_vel.angular.z = 0.5 * np.sign(angle_diff)  # Turn in correct direction
            else:  # Go forward
                cmd_vel.linear.x = min(0.3, distance_to_waypoint)  # Scale speed with distance

            # Limit velocities for humanoid stability
            cmd_vel.linear.x = max(-0.2, min(0.5, cmd_vel.linear.x))
            cmd_vel.angular.z = max(-0.3, min(0.3, cmd_vel.angular.z))

            # Publish command
            self.cmd_vel_pub.publish(cmd_vel)

            # Publish local plan for visualization
            local_path_msg = Path()
            local_path_msg.header.frame_id = self.map_frame
            local_path_msg.header.stamp = self.get_clock().now().to_msg()
            local_path_msg.poses = [p for p in self.local_plan]
            self.local_plan_pub.publish(local_path_msg)

            # Check if reached goal
            goal_distance = np.linalg.norm(
                np.array([self.goal_pose.position.x, self.goal_pose.position.y]) -
                current_pos
            )

            if goal_distance < 0.5:  # Within 50cm of goal
                self.get_logger().info('Goal reached!')
                self.is_navigating = False
                cmd_vel = Twist()
                self.cmd_vel_pub.publish(cmd_vel)  # Stop robot

    def detect_immediate_obstacle(self):
        """
        Check if there's an immediate obstacle in front of the robot.
        """
        if self.scan_data is None:
            return False

        # Check the front 60 degrees for obstacles
        front_range_start = int((0 - 30 * math.pi/180 - self.scan_data.angle_min) / self.scan_data.angle_increment)
        front_range_end = int((0 + 30 * math.pi/180 - self.scan_data.angle_min) / self.scan_data.angle_increment)

        if front_range_start < 0:
            front_range_start = 0
        if front_range_end >= len(self.scan_data.ranges):
            front_range_end = len(self.scan_data.ranges) - 1

        # Check if any distance in front is less than 0.5m
        for i in range(front_range_start, front_range_end + 1):
            if 0.1 < self.scan_data.ranges[i] < 0.5:  # Obstacle within 50cm
                return True

        return False

    def publish_collision_risks(self):
        """
        Publish collision risks as visualization markers.
        """
        marker_array = MarkerArray()

        for i, risk_point in enumerate(self.collision_risks):
            marker = Marker()
            marker.header.frame_id = self.base_frame
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = "collision_risks"
            marker.id = i
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD

            marker.pose.position = risk_point
            marker.pose.orientation.w = 1.0

            marker.scale.x = 0.1
            marker.scale.y = 0.1
            marker.scale.z = 0.1

            marker.color.r = 1.0
            marker.color.g = 0.0
            marker.color.b = 0.0
            marker.color.a = 0.8

            marker_array.markers.append(marker)

        self.collision_risk_pub.publish(marker_array)

    def quaternion_to_yaw(self, x, y, z, w):
        """
        Convert quaternion to yaw angle.
        """
        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        return math.atan2(siny_cosp, cosy_cosp)

    def normalize_angle(self, angle):
        """
        Normalize angle to [-pi, pi] range.
        """
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle


def main(args=None):
    rclpy.init(args=args)
    humanoid_navigation = HumanoidNavigation()

    try:
        rclpy.spin(humanoid_navigation)
    except KeyboardInterrupt:
        humanoid_navigation.get_logger().info('Shutting down Humanoid Navigation')
    finally:
        # Stop the robot before shutting down
        cmd_vel = Twist()
        humanoid_navigation.cmd_vel_pub.publish(cmd_vel)
        humanoid_navigation.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

## Perception Pipeline for Synthetic Sensors

Processing synthetic sensor data from Isaac Sim requires specialized approaches that account for the differences between simulated and real sensors.

### Synthetic Sensor Characteristics

Synthetic sensors in Isaac Sim have different characteristics than real sensors:

- **Perfect Geometry**: No lens distortion effects
- **Clean Data**: No sensor noise (unless specifically added)
- **High Fidelity**: Accurate physics simulation
- **Synchronized Data**: Perfect temporal alignment

### Perception Pipeline Architecture

The perception pipeline processes multiple sensor modalities:

1. **RGB Processing**: Object detection and classification
2. **Depth Processing**: 3D reconstruction and obstacle detection
3. **LiDAR Processing**: Precise distance measurements
4. **Sensor Fusion**: Combining data from multiple sensors

### Perception Pipeline Implementation

```python
#!/usr/bin/env python3
"""
Perception Pipeline for Humanoid Robot in Isaac Sim

This module implements perception processing for synthetic sensor input
from Isaac Sim environment, including object detection, segmentation,
and scene understanding for humanoid navigation.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo, LaserScan, PointCloud2, Imu
from geometry_msgs.msg import PointStamped, Point
from std_msgs.msg import Header, Float32MultiArray
from visualization_msgs.msg import Marker, MarkerArray
from builtin_interfaces.msg import Time
import numpy as np
import cv2
from cv_bridge import CvBridge
import tf2_ros
from tf2_ros import TransformListener
from tf2_geometry_msgs import do_transform_point
from scipy.spatial.distance import cdist
from sklearn.cluster import DBSCAN
import message_filters


class PerceptionPipeline(Node):
    """
    Perception pipeline for processing synthetic sensor data from Isaac Sim
    to enable humanoid robot to understand its environment.
    """

    def __init__(self):
        super().__init__('perception_pipeline')

        # Parameters
        self.declare_parameter('detection_confidence_threshold', 0.5)
        self.declare_parameter('object_min_distance', 0.3)
        self.declare_parameter('object_max_distance', 10.0)
        self.declare_parameter('clustering_eps', 0.3)
        self.declare_parameter('clustering_min_samples', 5)

        self.confidence_threshold = self.get_parameter('detection_confidence_threshold').value
        self.min_distance = self.get_parameter('object_min_distance').value
        self.max_distance = self.get_parameter('object_max_distance').value
        self.clustering_eps = self.get_parameter('clustering_eps').value
        self.clustering_min_samples = self.get_parameter('clustering_min_samples').value

        # CV Bridge for image processing
        self.cv_bridge = CvBridge()

        # TF listener for coordinate transforms
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Subscribers - using message filters for synchronization
        self.image_sub = message_filters.Subscriber(self, Image, '/camera/rgb/image_raw')
        self.depth_sub = message_filters.Subscriber(self, Image, '/camera/depth/image_raw')
        self.info_sub = message_filters.Subscriber(self, CameraInfo, '/camera/rgb/camera_info')
        self.scan_sub = message_filters.Subscriber(self, LaserScan, '/scan')

        # Synchronize camera and LiDAR data
        self.sync = message_filters.ApproximateTimeSynchronizer(
            [self.image_sub, self.depth_sub, self.info_sub, self.scan_sub],
            queue_size=10,
            slop=0.1
        )
        self.sync.registerCallback(self.sensor_sync_callback)

        # Publishers
        self.object_pub = self.create_publisher(Float32MultiArray, '/perception/objects', 10)
        self.object_markers_pub = self.create_publisher(MarkerArray, '/perception/object_markers', 10)
        self.obstacle_map_pub = self.create_publisher(MarkerArray, '/perception/obstacle_map', 10)
        self.surface_pub = self.create_publisher(MarkerArray, '/perception/surfaces', 10)

        # State variables
        self.camera_intrinsics = None
        self.objects = []
        self.obstacles = []
        self.surfaces = []

        # Timer for processing
        self.process_timer = self.create_timer(0.1, self.process_perception)

        self.get_logger().info('Perception Pipeline initialized')

    def sensor_sync_callback(self, image_msg, depth_msg, info_msg, scan_msg):
        """
        Process synchronized sensor data from Isaac Sim.
        """
        try:
            # Update camera intrinsics if changed
            if self.camera_intrinsics is None:
                self.camera_intrinsics = np.array([
                    [info_msg.k[0], info_msg.k[1], info_msg.k[2]],
                    [info_msg.k[3], info_msg.k[4], info_msg.k[5]],
                    [info_msg.k[6], info_msg.k[7], info_msg.k[8]]
                ])

            # Convert images
            rgb_image = self.cv_bridge.imgmsg_to_cv2(image_msg, 'rgb8')
            depth_image = self.cv_bridge.imgmsg_to_cv2(depth_msg, '32FC1')

            # Process perception pipeline
            self.process_vision_perception(rgb_image, depth_image, info_msg)
            self.process_lidar_perception(scan_msg)

        except Exception as e:
            self.get_logger().error(f'Error in sensor sync callback: {str(e)}')

    def process_vision_perception(self, rgb_image, depth_image, camera_info):
        """
        Process RGB and depth images to detect objects and surfaces.
        """
        # Simple color-based object detection (in real implementation, this would use DNN)
        hsv = cv2.cvtColor(rgb_image, cv2.COLOR_RGB2HSV)

        # Define color ranges for common objects (red, blue, green)
        lower_red = np.array([0, 50, 50])
        upper_red = np.array([10, 255, 255])
        lower_blue = np.array([100, 50, 50])
        upper_blue = np.array([130, 255, 255])
        lower_green = np.array([40, 50, 50])
        upper_green = np.array([80, 255, 255])

        # Create masks for different colors
        mask_red = cv2.inRange(hsv, lower_red, upper_red)
        mask_blue = cv2.inRange(hsv, lower_blue, upper_blue)
        mask_green = cv2.inRange(hsv, lower_green, upper_green)

        # Find contours for each color
        contours_red, _ = cv2.findContours(mask_red, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        contours_blue, _ = cv2.findContours(mask_blue, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        contours_green, _ = cv2.findContours(mask_green, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        # Process detected objects
        self.objects = []
        self.process_contours(contours_red, 'red_object', depth_image, camera_info)
        self.process_contours(contours_blue, 'blue_object', depth_image, camera_info)
        self.process_contours(contours_green, 'green_object', depth_image, camera_info)

    def process_contours(self, contours, object_type, depth_image, camera_info):
        """
        Process contours to extract 3D object information.
        """
        for contour in contours:
            if cv2.contourArea(contour) > 100:  # Filter small contours
                # Get bounding box
                x, y, w, h = cv2.boundingRect(contour)

                # Calculate center of contour in image coordinates
                center_x = x + w // 2
                center_y = y + h // 2

                # Get depth at center point
                depth = depth_image[center_y, center_x]

                if depth > 0 and self.min_distance < depth < self.max_distance:
                    # Convert image coordinates to 3D world coordinates
                    point_3d = self.image_to_world(
                        center_x, center_y, depth,
                        self.camera_intrinsics[0, 0],  # fx
                        self.camera_intrinsics[1, 1],  # fy
                        self.camera_intrinsics[0, 2],  # cx
                        self.camera_intrinsics[1, 2]   # cy
                    )

                    if point_3d is not None:
                        obj_info = {
                            'type': object_type,
                            'position': point_3d,
                            'confidence': 0.8,  # Simple confidence
                            'size': (w, h),
                            'bbox': (x, y, w, h)
                        }
                        self.objects.append(obj_info)

    def image_to_world(self, u, v, depth, fx, fy, cx, cy):
        """
        Convert image coordinates to 3D world coordinates.
        """
        try:
            x = (u - cx) * depth / fx
            y = (v - cy) * depth / fy
            z = depth
            return [x, y, z]
        except:
            return None

    def process_lidar_perception(self, scan_msg):
        """
        Process LiDAR scan data for obstacle detection and clustering.
        """
        # Convert scan to Cartesian points
        points = []
        for i, range_val in enumerate(scan_msg.ranges):
            if self.min_distance < range_val < self.max_distance:
                angle = scan_msg.angle_min + i * scan_msg.angle_increment
                x = range_val * np.cos(angle)
                y = range_val * np.sin(angle)
                points.append([x, y])

        if len(points) > 0:
            # Cluster points to identify obstacles
            points = np.array(points)
            clustering = DBSCAN(eps=self.clustering_eps, min_samples=self.clustering_min_samples)
            labels = clustering.fit_predict(points)

            # Group points by cluster
            self.obstacles = []
            unique_labels = set(labels)
            for label in unique_labels:
                if label == -1:  # Noise points
                    continue

                cluster_points = points[labels == label]
                if len(cluster_points) >= self.clustering_min_samples:
                    # Calculate centroid of cluster
                    centroid = np.mean(cluster_points, axis=0)
                    cluster_size = len(cluster_points)

                    obstacle_info = {
                        'position': centroid,
                        'size': cluster_size,
                        'type': 'obstacle'
                    }
                    self.obstacles.append(obstacle_info)

    def process_perception(self):
        """
        Process perception data and publish results.
        """
        # Publish detected objects
        if self.objects:
            # Create Float32MultiArray message with object information
            objects_msg = Float32MultiArray()
            objects_data = []

            for obj in self.objects:
                # Append [x, y, z, confidence] for each object
                objects_data.extend([
                    obj['position'][0], obj['position'][1], obj['position'][2],
                    obj['confidence']
                ])

            objects_msg.data = objects_data
            self.object_pub.publish(objects_msg)

            # Publish object markers for visualization
            self.publish_object_markers()

        # Publish obstacle map
        if self.obstacles:
            self.publish_obstacle_map()

    def publish_object_markers(self):
        """
        Publish object markers for visualization in RViz.
        """
        marker_array = MarkerArray()

        for i, obj in enumerate(self.objects):
            if obj['confidence'] >= self.confidence_threshold:
                marker = Marker()
                marker.header.frame_id = 'base_link'  # Assuming objects are in robot frame
                marker.header.stamp = self.get_clock().now().to_msg()
                marker.ns = "detected_objects"
                marker.id = i
                marker.type = Marker.CYLINDER
                marker.action = Marker.ADD

                # Position
                marker.pose.position.x = obj['position'][0]
                marker.pose.position.y = obj['position'][1]
                marker.pose.position.z = obj['position'][2] + 0.2  # Slightly above ground
                marker.pose.orientation.w = 1.0

                # Scale (size of cylinder)
                marker.scale.x = 0.2  # diameter
                marker.scale.y = 0.2  # diameter
                marker.scale.z = 0.4  # height

                # Color based on object type
                if 'red' in obj['type']:
                    marker.color.r = 1.0
                    marker.color.g = 0.0
                    marker.color.b = 0.0
                elif 'blue' in obj['type']:
                    marker.color.r = 0.0
                    marker.color.g = 0.0
                    marker.color.b = 1.0
                elif 'green' in obj['type']:
                    marker.color.r = 0.0
                    marker.color.g = 1.0
                    marker.color.b = 0.0

                marker.color.a = 0.8

                marker_array.markers.append(marker)

        self.object_markers_pub.publish(marker_array)

    def publish_obstacle_map(self):
        """
        Publish obstacle map as markers for visualization.
        """
        marker_array = MarkerArray()

        for i, obs in enumerate(self.obstacles):
            marker = Marker()
            marker.header.frame_id = 'odom'  # Obstacles in odom frame
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = "obstacles"
            marker.id = i
            marker.type = Marker.CUBE
            marker.action = Marker.ADD

            # Position
            marker.pose.position.x = obs['position'][0]
            marker.pose.position.y = obs['position'][1]
            marker.pose.position.z = 0.5  # Half meter high
            marker.pose.orientation.w = 1.0

            # Scale based on cluster size
            scale_factor = min(0.5, max(0.1, obs['size'] * 0.02))
            marker.scale.x = scale_factor
            marker.scale.y = scale_factor
            marker.scale.z = 1.0

            # Color
            marker.color.r = 1.0
            marker.color.g = 0.5
            marker.color.b = 0.0
            marker.color.a = 0.8

            marker_array.markers.append(marker)

        self.obstacle_map_pub.publish(marker_array)

    def get_ground_plane(self, points):
        """
        Simple ground plane estimation using RANSAC-like approach.
        """
        if len(points) < 3:
            return None

        # For now, return a simple ground plane at z=0
        # In real implementation, this would use RANSAC to fit a plane
        return {'normal': [0, 0, 1], 'distance': 0}


def main(args=None):
    rclpy.init(args=args)
    perception_pipeline = PerceptionPipeline()

    try:
        rclpy.spin(perception_pipeline)
    except KeyboardInterrupt:
        perception_pipeline.get_logger().info('Shutting down Perception Pipeline')
    finally:
        perception_pipeline.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

## Obstacle Avoidance Algorithms

Humanoid robots require specialized obstacle avoidance algorithms that consider their bipedal nature:

### Humanoid-Specific Obstacle Avoidance

1. **Step-Aware Planning**: Path planning that considers where feet can be placed
2. **Balance-Preserving Avoidance**: Avoidance maneuvers that maintain robot stability
3. **Dynamic Adjustment**: Continuous replanning as robot moves

## Isaac Sim Integration

Isaac Sim provides specialized tools for humanoid perception and navigation:

### Isaac ROS Packages

- **Isaac ROS Visual SLAM**: For visual-based localization and mapping
- **Isaac ROS Image Pipeline**: For preprocessing camera images
- **Isaac ROS Apriltag**: For fiducial marker detection
- **Isaac ROS DNN Encoder**: For neural network-based perception

### Isaac Sim Configuration

Isaac Sim must be configured to work with the perception and navigation system:

```python
# Isaac Sim Configuration for Humanoid Perception
robot_config:

  # Robot model settings
  model:
    name: "simple_humanoid"
    urdf_path: "/path/to/humanoid.urdf"  # This would be the path to your URDF
    usd_path: "/path/to/humanoid.usd"    # USD representation of the robot
    scale: [1.0, 1.0, 1.0]

  # Physics properties
  physics:
    solver_type: "pgs"  # Options: "pgs", "tgs"
    iterations: 16
    velocity_iterations: 8
    articulation_solver_position_iteration_count: 4
    articulation_solver_velocity_iteration_count: 1

    # Material properties
    material:
      static_friction: 0.5
      dynamic_friction: 0.5
      restitution: 0.01

  # Sensor properties
  sensors:
    camera:
      enable: true
      image_topic: "/camera/rgb/image_raw"
      camera_info_topic: "/camera/rgb/camera_info"
      width: 640
      height: 480
      fov: 1.047  # 60 degrees in radians
      near_clip: 0.1
      far_clip: 100.0
      position: [0.1, 0.0, 0.0]  # Relative to head link
      orientation: [0.0, 0.0, 0.0, 1.0]  # Quaternion (x, y, z, w)

    lidar:
      enable: true
      topic: "/scan"
      position: [0.15, 0.0, 0.0]  # Relative to head link
      orientation: [0.0, 0.0, 0.0, 1.0]
      samples: 720
      resolution: 1.0
      min_range: 0.1
      max_range: 30.0
      fov: 3.14159  # 180 degrees
      rotation_rate: 10.0
```

## Best Practices for AI Perception in Humanoid Robotics

### Performance Optimization
- Use lightweight neural networks optimized for humanoid platforms
- Implement multi-threading for parallel processing
- Optimize sensor data frequency based on task requirements
- Use approximate time synchronization for sensor fusion

### Safety Considerations
- Implement emergency stop mechanisms
- Validate perception outputs before navigation commands
- Monitor robot balance during navigation
- Include obstacle detection in all directions

### Validation and Testing
- Test in diverse simulation environments
- Validate with various humanoid morphologies
- Verify performance under different lighting conditions
- Test with moving obstacles and dynamic environments

## Exercises

1. **VSLAM Implementation**: Implement a simple VSLAM system using ORB features and test in Isaac Sim.

2. **Humanoid Navigation**: Configure Nav2 for a humanoid robot and test navigation in various environments.

3. **Perception Pipeline**: Create a perception pipeline that combines RGB, depth, and LiDAR data.

4. **Obstacle Avoidance**: Implement an obstacle avoidance algorithm specifically for humanoid locomotion.

5. **Isaac Sim Integration**: Integrate the perception and navigation systems with Isaac Sim and test in realistic scenarios.

## Summary

This module covered the fundamentals of AI perception and navigation for humanoid robotics, including VSLAM systems, Nav2 configuration for bipedal robots, perception pipeline development, and Isaac Sim integration. Students learned to implement perception systems that process synthetic sensor input and create navigation systems that account for humanoid-specific constraints.

## Review Questions

1. What are the main differences between navigation for wheeled robots and humanoid robots?
2. How does VSLAM differ from traditional SLAM approaches?
3. What are the key challenges in processing synthetic sensor data from Isaac Sim?
4. How do humanoid-specific constraints affect path planning and obstacle avoidance?
5. What are important considerations for validating perception and navigation systems?

## References

- Isaac ROS Documentation: https://nvidia-isaac-ros.github.io/
- Nav2 Documentation: https://navigation.ros.org/
- Visual SLAM Survey: https://arxiv.org/abs/2001.05804
- Humanoid Robotics: https://ieeexplore.ieee.org/document/9307201
- Perception in Robotics: https://www.springer.com/gp/book/9783030277229