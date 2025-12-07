#!/usr/bin/env python3
"""
Humanoid VSLAM Example for Isaac Sim Environment

This example demonstrates a complete VSLAM pipeline for humanoid robots
using Isaac Sim synthetic sensor data.
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


class HumanoidVSLAMExample(Node):
    """
    Example VSLAM implementation for humanoid robot in Isaac Sim.
    Demonstrates visual SLAM with feature tracking and pose estimation.
    """

    def __init__(self):
        super().__init__('humanoid_vslam_example')

        # Parameters
        self.declare_parameter('tracking_frame', 'camera_link')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('publish_rate', 30.0)
        self.declare_parameter('min_features', 50)
        self.declare_parameter('max_features', 1000)

        self.tracking_frame = self.get_parameter('tracking_frame').value
        self.map_frame = self.get_parameter('map_frame').value
        self.publish_rate = self.get_parameter('publish_rate').value
        self.min_features = self.get_parameter('min_features').value
        self.max_features = self.get_parameter('max_features').value

        # CV Bridge for image processing
        self.cv_bridge = CvBridge()

        # TF broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)

        # Subscribers - synchronized processing
        self.image_sub = message_filters.Subscriber(self, Image, '/camera/rgb/image_raw')
        self.info_sub = message_filters.Subscriber(self, CameraInfo, '/camera/rgb/camera_info')
        self.imu_sub = message_filters.Subscriber(self, Imu, '/imu/data')

        # Synchronize sensor data
        self.sync = message_filters.ApproximateTimeSynchronizer(
            [self.image_sub, self.info_sub, self.imu_sub],
            queue_size=10,
            slop=0.1
        )
        self.sync.registerCallback(self.sync_callback)

        # Publishers
        self.odom_pub = self.create_publisher(Odometry, '/humanoid_vslam/odometry', 10)
        self.path_pub = self.create_publisher(Path, '/humanoid_vslam/path', 10)
        self.feature_pub = self.create_publisher(MarkerArray, '/humanoid_vslam/features', 10)
        self.map_pub = self.create_publisher(MarkerArray, '/humanoid_vslam/map', 10)

        # State variables
        self.previous_features = None
        self.current_pose = np.eye(4)  # 4x4 transformation matrix
        self.path = Path()
        self.path.header.frame_id = self.map_frame

        # Feature detector with humanoid-specific parameters
        self.feature_detector = cv2.ORB_create(
            nfeatures=self.max_features,
            scaleFactor=1.2,
            nlevels=8,
            edgeThreshold=31,
            patchSize=31
        )

        # Timer for publishing transforms
        self.timer = self.create_timer(1.0/self.publish_rate, self.publish_transforms)

        self.get_logger().info('Humanoid VSLAM Example initialized')

    def sync_callback(self, image_msg, info_msg, imu_msg):
        """
        Process synchronized sensor data for VSLAM.
        """
        try:
            # Convert ROS image to OpenCV
            cv_image = self.cv_bridge.imgmsg_to_cv2(image_msg, 'rgb8')

            # Detect features in the image
            keypoints, descriptors = self.feature_detector.detectAndCompute(cv_image, None)

            if keypoints is not None and len(keypoints) >= self.min_features:
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
            # Use simple correspondence tracking
            if len(prev_features) >= self.min_features and len(curr_features) >= self.min_features:
                # Match features based on proximity
                distances = cdist(prev_features, curr_features)

                # Find nearest neighbors
                min_indices = np.argmin(distances, axis=1)

                # Calculate average displacement
                valid_matches = []
                for i, j in enumerate(min_indices):
                    if distances[i, j] < 50:  # Threshold for valid match
                        valid_matches.append((i, j))

                if len(valid_matches) >= self.min_features:
                    displacements = []
                    for i, j in valid_matches:
                        disp = curr_features[j] - prev_features[i]
                        displacements.append(disp)

                    avg_displacement = np.mean(displacements, axis=0)

                    # Convert displacement to pose update (simplified)
                    pose_update = np.eye(4)
                    pose_update[0, 3] = avg_displacement[0] * 0.001  # Scale factor
                    pose_update[1, 3] = avg_displacement[1] * 0.001  # Scale factor

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

        # Simplified orientation (in real implementation, this would convert rotation matrix properly)
        odom_msg.pose.pose.orientation.w = 1.0

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

        # Limit path length
        if len(self.path.poses) > 1000:
            self.path.poses.pop(0)

        # Publish updated path
        self.path_pub.publish(self.path)

    def publish_features(self, keypoints, stamp):
        """
        Publish feature visualization markers.
        """
        marker_array = MarkerArray()

        for i, kp in enumerate(keypoints[:50]):  # Limit for performance
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
    vslam_example = HumanoidVSLAMExample()

    try:
        rclpy.spin(vslam_example)
    except KeyboardInterrupt:
        vslam_example.get_logger().info('Shutting down Humanoid VSLAM Example')
    finally:
        vslam_example.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()