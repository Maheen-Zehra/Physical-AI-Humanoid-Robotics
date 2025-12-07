# Exercises: AI Perception and Navigation for Humanoid Robotics

## Exercise 1: VSLAM Implementation

### Objective
Implement a Visual SLAM system for humanoid robot localization and mapping in Isaac Sim.

### Requirements
1. Create a VSLAM node that processes RGB and depth images
2. Implement feature detection and tracking using ORB
3. Estimate robot pose from tracked features
4. Build a map of the environment from visual observations
5. Visualize the trajectory and map in RViz

### Implementation Steps
1. Create a ROS 2 package for VSLAM functionality
2. Implement feature detection using OpenCV ORB
3. Create a tracking mechanism to match features across frames
4. Estimate motion using feature correspondences
5. Integrate motion to estimate trajectory
6. Create a simple mapping algorithm
7. Publish TF transforms for visualization
8. Test with Isaac Sim synthetic data

### Validation
- Verify feature detection works on Isaac Sim images
- Confirm pose estimation is consistent
- Validate that trajectory matches actual robot movement
- Check that map is built correctly
- Test in various Isaac Sim environments

## Exercise 2: Humanoid Navigation Configuration

### Objective
Configure Nav2 navigation system specifically for humanoid robot with bipedal locomotion constraints.

### Requirements
1. Configure Nav2 controller for humanoid-specific parameters
2. Adjust obstacle avoidance for humanoid step planning
3. Implement path smoothing for humanoid gait patterns
4. Validate navigation performance in simulation
5. Create a humanoid-specific behavior tree

### Implementation Steps
1. Modify Nav2 parameters for humanoid constraints
2. Configure controller with humanoid-specific velocity limits
3. Adjust costmap parameters for humanoid foot placement
4. Create custom recovery behaviors for humanoid
5. Implement step-aware path planning
6. Test navigation in Isaac Sim environment
7. Validate obstacle avoidance effectiveness

### Validation
- Verify navigation follows humanoid-appropriate paths
- Confirm obstacle avoidance works for humanoid
- Test step planning functionality
- Validate recovery behaviors work correctly
- Check navigation performance metrics

## Exercise 3: Perception Pipeline Integration

### Objective
Create a perception pipeline that fuses RGB, depth, and LiDAR data for object detection.

### Requirements
1. Implement synchronized processing of multiple sensor streams
2. Create object detection from RGB images
3. Generate 3D object positions from depth and camera parameters
4. Combine LiDAR data for enhanced obstacle detection
5. Publish unified object detections with confidence scores

### Implementation Steps
1. Set up message synchronization for sensor data
2. Implement color-based object detection
3. Create 3D position calculation from depth
4. Integrate LiDAR clustering for obstacle detection
5. Fuse sensor data for unified object representations
6. Publish detection results with confidence scores
7. Create visualization markers for detected objects
8. Test with Isaac Sim synthetic sensor data

### Validation
- Verify object detection accuracy
- Confirm 3D position estimation is correct
- Validate sensor fusion improves detection
- Test in various lighting conditions
- Check real-time performance requirements

## Exercise 4: Obstacle Negotiation Algorithm

### Objective
Implement an obstacle negotiation algorithm that allows humanoid robot to step over or around obstacles.

### Requirements
1. Detect obstacles that are step-over-able vs. must-be-avoided
2. Plan foot placements for stepping over obstacles
3. Implement balance preservation during obstacle negotiation
4. Handle multiple obstacles in sequence
5. Ensure safe transition between normal walking and obstacle negotiation

### Implementation Steps
1. Create obstacle classification algorithm
2. Implement step-over trajectory planning
3. Design balance control during obstacle negotiation
4. Create smooth transitions between navigation modes
5. Test with various obstacle heights and widths
6. Validate stability during obstacle negotiation
7. Integrate with existing navigation system

### Validation
- Confirm obstacle classification is accurate
- Verify step-over trajectories are safe
- Test balance preservation during negotiation
- Validate smooth transitions between modes
- Check navigation efficiency with obstacles

## Exercise 5: Isaac Sim Integration

### Objective
Integrate the complete perception and navigation system with Isaac Sim environment.

### Requirements
1. Connect perception pipeline to Isaac Sim synthetic sensors
2. Interface navigation system with Isaac Sim physics
3. Validate system performance in various simulation scenarios
4. Test with different humanoid robot models
5. Evaluate system robustness under simulation variations

### Implementation Steps
1. Configure Isaac Sim sensor outputs to match ROS topics
2. Set up Isaac Sim physics for humanoid robot
3. Connect perception pipeline to Isaac Sim data
4. Interface navigation system with Isaac Sim
5. Create test scenarios with various environments
6. Validate system behavior in simulation
7. Optimize for simulation-real transfer

### Validation
- Verify all sensor data is properly received
- Confirm navigation commands affect simulation correctly
- Test system in multiple environments
- Validate performance metrics
- Check simulation-real transfer capabilities

## Exercise 6: Performance Optimization

### Objective
Optimize perception and navigation system for real-time performance on humanoid platform.

### Requirements
1. Measure computational performance of each component
2. Identify bottlenecks in perception pipeline
3. Optimize algorithms for humanoid computational constraints
4. Validate that optimizations don't degrade accuracy
5. Test system under load conditions

### Implementation Steps
1. Profile current system performance
2. Identify computational bottlenecks
3. Apply optimization techniques (algorithmic, parallelization)
4. Implement performance monitoring
5. Test optimized system in simulation
6. Validate accuracy is maintained
7. Document performance improvements

### Validation
- Confirm performance improvements achieved
- Verify accuracy is not compromised
- Test system stability under load
- Validate real-time constraints are met
- Document optimization results

## Solutions

### Exercise 1 Solution

VSLAM Node Implementation:
```python
#!/usr/bin/env python3
"""
VSLAM Implementation for Humanoid Robot
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo, Imu
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped, Point
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import Header
import numpy as np
import cv2
from cv_bridge import CvBridge
import tf2_ros
from tf2_ros import TransformBroadcaster
import message_filters


class VSLAMNode(Node):
    """
    Visual SLAM implementation for humanoid robot localization and mapping.
    """

    def __init__(self):
        super().__init__('vslam_node')

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

        # Subscribers
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

        self.get_logger().info('VSLAM Node initialized')

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
    vslam_node = VSLAMNode()

    try:
        rclpy.spin(vslam_node)
    except KeyboardInterrupt:
        vslam_node.get_logger().info('Shutting down VSLAM node')
    finally:
        vslam_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

### Exercise 2 Solution

Nav2 Configuration for Humanoid:
```yaml
# Nav2 Configuration for Humanoid Robot
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

## Assessment Rubric

### Technical Implementation (60%)
- Correct VSLAM algorithm implementation (20%)
- Proper Nav2 configuration for humanoid (15%)
- Accurate perception pipeline (15%)
- Effective obstacle negotiation (10%)

### Code Quality (25%)
- Clear, well-documented code (10%)
- Proper error handling and validation (10%)
- Following ROS 2 and Isaac Sim best practices (5%)

### Validation and Testing (15%)
- Comprehensive testing of functionality (10%)
- Proper validation of perception and navigation results (5%)

## Learning Objectives Assessment

After completing these exercises, students should be able to:
- Implement VSLAM systems for humanoid robots
- Configure Nav2 for bipedal navigation
- Create perception pipelines for synthetic sensor data
- Design obstacle negotiation algorithms for humanoid robots
- Integrate perception and navigation systems with Isaac Sim