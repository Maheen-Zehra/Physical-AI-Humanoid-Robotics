#!/usr/bin/env python3
"""
Object Recognition Example for Humanoid Robot

This example demonstrates object recognition using computer vision
for the humanoid robot's perception system in Isaac Sim.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import Point
from std_msgs.msg import Header
from visualization_msgs.msg import Marker, MarkerArray
from cv_bridge import CvBridge
import cv2
import numpy as np
import torch
import torchvision
from torchvision import transforms
from PIL import Image as PILImage
import message_filters
from typing import Dict, List, Optional, Tuple
import json


class ObjectRecognitionExample(Node):
    """
    Example object recognition system using computer vision for humanoid robot.
    """

    def __init__(self):
        super().__init__('object_recognition_example')

        # Parameters
        self.declare_parameter('model_name', 'yolov5s')
        self.declare_parameter('confidence_threshold', 0.5)
        self.declare_parameter('nms_threshold', 0.4)
        self.declare_parameter('enable_3d_estimation', True)
        self.declare_parameter('publish_markers', True)

        self.model_name = self.get_parameter('model_name').value
        self.confidence_threshold = self.get_parameter('confidence_threshold').value
        self.nms_threshold = self.get_parameter('nms_threshold').value
        self.enable_3d_estimation = self.get_parameter('enable_3d_estimation').value
        self.publish_markers = self.get_parameter('publish_markers').value

        # CV Bridge for image processing
        self.cv_bridge = CvBridge()

        # Object detection model (using torchvision's pre-trained model as placeholder)
        try:
            self.model = torch.hub.load('ultralytics/yolov5', 'yolov5s', pretrained=True)
            self.model.eval()
            self.get_logger().info('Loaded YOLOv5 model')
        except Exception as e:
            self.get_logger().warn(f'Could not load YOLOv5 model: {str(e)}, using placeholder')
            self.model = None

        # Class names for COCO dataset (80 classes)
        self.class_names = [
            'person', 'bicycle', 'car', 'motorcycle', 'airplane', 'bus', 'train', 'truck', 'boat',
            'traffic light', 'fire hydrant', 'stop sign', 'parking meter', 'bench', 'bird', 'cat',
            'dog', 'horse', 'sheep', 'cow', 'elephant', 'bear', 'zebra', 'giraffe', 'backpack',
            'umbrella', 'handbag', 'tie', 'suitcase', 'frisbee', 'skis', 'snowboard', 'sports ball',
            'kite', 'baseball bat', 'baseball glove', 'skateboard', 'surfboard', 'tennis racket',
            'bottle', 'wine glass', 'cup', 'fork', 'knife', 'spoon', 'bowl', 'banana', 'apple',
            'sandwich', 'orange', 'broccoli', 'carrot', 'hot dog', 'pizza', 'donut', 'cake', 'chair',
            'couch', 'potted plant', 'bed', 'dining table', 'toilet', 'tv', 'laptop', 'mouse',
            'remote', 'keyboard', 'cell phone', 'microwave', 'oven', 'toaster', 'sink', 'refrigerator',
            'book', 'clock', 'vase', 'scissors', 'teddy bear', 'hair drier', 'toothbrush'
        ]

        # Subscribers - synchronized processing
        self.image_sub = message_filters.Subscriber(self, Image, '/camera/rgb/image_raw')
        self.info_sub = message_filters.Subscriber(self, CameraInfo, '/camera/rgb/camera_info')

        # Synchronize sensor data
        self.sync = message_filters.ApproximateTimeSynchronizer(
            [self.image_sub, self.info_sub],
            queue_size=10,
            slop=0.1
        )
        self.sync.registerCallback(self.sync_callback)

        # Publishers
        self.detection_pub = self.create_publisher(
            MarkerArray, '/object_detections', 10)
        self.detection_info_pub = self.create_publisher(
            MarkerArray, '/object_detection_info', 10)

        # State variables
        self.camera_info = None
        self.detection_results = []

        self.get_logger().info('Object Recognition Example initialized')

    def sync_callback(self, image_msg, info_msg):
        """
        Process synchronized image and camera info data.
        """
        try:
            # Convert ROS image to OpenCV
            cv_image = self.cv_bridge.imgmsg_to_cv2(image_msg, 'rgb8')

            # Store camera info
            self.camera_info = info_msg

            # Perform object detection
            detections = self.detect_objects(cv_image)

            # Process detections
            self.process_detections(detections, image_msg.header)

        except Exception as e:
            self.get_logger().error(f'Error in object recognition: {str(e)}')

    def detect_objects(self, cv_image: np.ndarray) -> List[Dict]:
        """
        Perform object detection on the input image.
        """
        try:
            if self.model is not None:
                # Convert OpenCV image to PIL for model
                pil_image = PILImage.fromarray(cv_image)

                # Perform inference
                results = self.model(pil_image)

                # Convert results to our format
                detections = []
                for detection in results.xyxy[0].numpy():  # x1, y1, x2, y2, conf, cls
                    x1, y1, x2, y2, conf, cls = detection
                    if conf >= self.confidence_threshold:
                        class_name = self.class_names[int(cls)] if int(cls) < len(self.class_names) else f'unknown_{int(cls)}'

                        detection_dict = {
                            'class_name': class_name,
                            'confidence': float(conf),
                            'bbox': {
                                'x1': int(x1),
                                'y1': int(y1),
                                'x2': int(x2),
                                'y2': int(y2)
                            },
                            'center': {
                                'x': int((x1 + x2) / 2),
                                'y': int((y1 + y2) / 2)
                            }
                        }

                        # Calculate 3D position if camera info is available
                        if self.enable_3d_estimation and self.camera_info:
                            detection_dict['position_3d'] = self.estimate_3d_position(
                                detection_dict['center']['x'],
                                detection_dict['center']['y'],
                                self.camera_info
                            )

                        detections.append(detection_dict)

                return detections
            else:
                # Placeholder detection - in a real implementation, you would use
                # a proper object detection model
                return self.placeholder_detection(cv_image)

        except Exception as e:
            self.get_logger().error(f'Error in object detection: {str(e)}')
            return []

    def placeholder_detection(self, cv_image: np.ndarray) -> List[Dict]:
        """
        Placeholder detection function when model is not available.
        """
        # This is a simple color-based detection for demonstration
        # In practice, you would use a proper object detection model
        detections = []

        # Convert to HSV for color detection
        hsv = cv2.cvtColor(cv_image, cv2.COLOR_RGB2HSV)

        # Define color ranges for common objects
        color_ranges = {
            'red': ([0, 50, 50], [10, 255, 255]),
            'blue': ([100, 50, 50], [130, 255, 255]),
            'green': ([40, 50, 50], [80, 255, 255])
        }

        for color_name, (lower, upper) in color_ranges.items():
            # Create mask for color
            mask = cv2.inRange(hsv, np.array(lower), np.array(upper))

            # Find contours
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

            for contour in contours:
                area = cv2.contourArea(contour)
                if area > 500:  # Filter small contours
                    # Get bounding box
                    x, y, w, h = cv2.boundingRect(contour)

                    detection_dict = {
                        'class_name': f'{color_name}_object',
                        'confidence': 0.7,  # Placeholder confidence
                        'bbox': {
                            'x1': x,
                            'y1': y,
                            'x2': x + w,
                            'y2': y + h
                        },
                        'center': {
                            'x': x + w // 2,
                            'y': y + h // 2
                        }
                    }

                    # Calculate 3D position if camera info is available
                    if self.enable_3d_estimation and self.camera_info:
                        detection_dict['position_3d'] = self.estimate_3d_position(
                            detection_dict['center']['x'],
                            detection_dict['center']['y'],
                            self.camera_info
                        )

                    detections.append(detection_dict)

        return detections

    def estimate_3d_position(self, x: int, y: int, camera_info: CameraInfo) -> Optional[Dict[str, float]]:
        """
        Estimate 3D position of object using camera intrinsics.
        """
        try:
            if camera_info is None:
                return None

            # Camera intrinsics
            fx = camera_info.k[0]  # Focal length x
            fy = camera_info.k[4]  # Focal length y
            cx = camera_info.k[2]  # Principal point x
            cy = camera_info.k[5]  # Principal point y

            # For now, return a placeholder based on normalized image coordinates
            # In a real implementation, you would use depth information or stereo vision
            # to get accurate 3D positions
            z = 1.0  # Placeholder depth - in reality, this would come from depth sensor

            # Convert pixel coordinates to 3D
            x_3d = (x - cx) * z / fx
            y_3d = (y - cy) * z / fy

            return {
                'x': x_3d,
                'y': y_3d,
                'z': z
            }

        except Exception as e:
            self.get_logger().error(f'Error estimating 3D position: {str(e)}')
            return None

    def process_detections(self, detections: List[Dict], header: Header):
        """
        Process detection results and publish visualization markers.
        """
        try:
            self.detection_results = detections

            if self.publish_markers:
                # Create visualization markers for detections
                marker_array = MarkerArray()

                for i, detection in enumerate(detections):
                    # Create marker for bounding box
                    bbox_marker = Marker()
                    bbox_marker.header = header
                    bbox_marker.ns = "object_detections"
                    bbox_marker.id = i * 2
                    bbox_marker.type = Marker.LINE_STRIP
                    bbox_marker.action = Marker.ADD

                    # Set position based on 3D estimation if available
                    if 'position_3d' in detection:
                        pos_3d = detection['position_3d']
                        bbox_marker.pose.position.x = pos_3d['x']
                        bbox_marker.pose.position.y = pos_3d['y']
                        bbox_marker.pose.position.z = pos_3d['z']
                    else:
                        # Use image coordinates (projected to z=0)
                        bbox_marker.pose.position.z = 0.0

                    bbox_marker.pose.orientation.w = 1.0

                    # Define the bounding box corners
                    bbox = detection['bbox']
                    center_x = detection['center']['x']
                    center_y = detection['center']['y']

                    # Create points for the bounding box
                    scale_factor = 0.1  # Scale factor for 3D visualization
                    width = (bbox['x2'] - bbox['x1']) * scale_factor
                    height = (bbox['y2'] - bbox['y1']) * scale_factor

                    # Define corner points relative to center
                    corners = [
                        Point(x=-width/2, y=-height/2, z=0.1),  # Bottom-left
                        Point(x=width/2, y=-height/2, z=0.1),   # Bottom-right
                        Point(x=width/2, y=height/2, z=0.1),    # Top-right
                        Point(x=-width/2, y=height/2, z=0.1),   # Top-left
                        Point(x=-width/2, y=-height/2, z=0.1)   # Close the loop
                    ]

                    bbox_marker.points = corners
                    bbox_marker.scale.x = 0.02  # Line width
                    bbox_marker.color.r = 1.0
                    bbox_marker.color.g = 0.0
                    bbox_marker.color.b = 0.0
                    bbox_marker.color.a = 0.8

                    marker_array.markers.append(bbox_marker)

                    # Create marker for label
                    label_marker = Marker()
                    label_marker.header = header
                    label_marker.ns = "object_labels"
                    label_marker.id = i * 2 + 1
                    label_marker.type = Marker.TEXT_VIEW_FACING
                    label_marker.action = Marker.ADD

                    # Position label above the bounding box
                    if 'position_3d' in detection:
                        pos_3d = detection['position_3d']
                        label_marker.pose.position.x = pos_3d['x']
                        label_marker.pose.position.y = pos_3d['y']
                        label_marker.pose.position.z = pos_3d['z'] + 0.2  # Above the object
                    else:
                        label_marker.pose.position.z = 0.2

                    label_marker.pose.orientation.w = 1.0
                    label_marker.text = f"{detection['class_name']}: {detection['confidence']:.2f}"
                    label_marker.scale.z = 0.1
                    label_marker.color.r = 1.0
                    label_marker.color.g = 1.0
                    label_marker.color.b = 1.0
                    label_marker.color.a = 1.0

                    marker_array.markers.append(label_marker)

                # Publish detection markers
                self.detection_pub.publish(marker_array)

            # Log detection results
            if detections:
                for detection in detections:
                    self.get_logger().info(
                        f"Detected {detection['class_name']} with confidence {detection['confidence']:.2f}"
                    )

        except Exception as e:
            self.get_logger().error(f'Error processing detections: {str(e)}')

    def get_detections(self) -> List[Dict]:
        """
        Get the latest detection results.
        """
        return self.detection_results


def main(args=None):
    rclpy.init(args=args)
    object_recognition_example = ObjectRecognitionExample()

    try:
        rclpy.spin(object_recognition_example)
    except KeyboardInterrupt:
        object_recognition_example.get_logger().info('Shutting down Object Recognition Example')
    finally:
        object_recognition_example.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()