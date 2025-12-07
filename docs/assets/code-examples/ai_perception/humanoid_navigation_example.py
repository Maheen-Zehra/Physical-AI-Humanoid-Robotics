#!/usr/bin/env python3
"""
Humanoid Navigation Example for Isaac Sim Environment

This example demonstrates navigation specifically designed for humanoid robots,
addressing bipedal locomotion challenges and obstacle avoidance in Isaac Sim.
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


class HumanoidNavigationExample(Node):
    """
    Navigation example specifically designed for humanoid robots in Isaac Sim.
    Demonstrates bipedal locomotion challenges, step planning, and obstacle avoidance.
    """

    def __init__(self):
        super().__init__('humanoid_navigation_example')

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
        self.get_logger().info('Humanoid Navigation Example initialized')

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
    humanoid_navigation = HumanoidNavigationExample()

    try:
        rclpy.spin(humanoid_navigation)
    except KeyboardInterrupt:
        humanoid_navigation.get_logger().info('Shutting down Humanoid Navigation Example')
    finally:
        # Stop the robot before shutting down
        cmd_vel = Twist()
        humanoid_navigation.cmd_vel_pub.publish(cmd_vel)
        humanoid_navigation.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()