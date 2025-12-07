# VLA Integration Code Examples

This directory contains code examples for the Vision-Language-Action (VLA) integration module of the Physical AI & Humanoid Robotics curriculum.

## Overview

The VLA (Vision-Language-Action) integration examples demonstrate how to connect voice commands to robot actions through cognitive planning and perception systems.

## Examples Included

### 1. Voice Processing Example
- **File**: `voice_processing_example.py`
- **Purpose**: Demonstrates speech-to-text conversion using OpenAI Whisper
- **Key Concepts**: Audio input handling, Whisper API integration, ROS 2 message publishing

### 2. Cognitive Planning Example
- **File**: `cognitive_planning_example.py`
- **Purpose**: Shows how to convert natural language commands into multi-step action plans
- **Key Concepts**: Natural language parsing, task decomposition, plan optimization

### 3. Object Recognition Example
- **File**: `object_recognition_example.py`
- **Purpose**: Illustrates computer vision-based object detection and 3D position estimation
- **Key Concepts**: Image processing, object detection, 3D coordinate transformation

## Usage

To run any of these examples:

```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
python3 docs/assets/code-examples/vla_integration/[example_name].py
```

## Prerequisites

- ROS 2 Humble installed
- OpenAI API key configured in environment variables
- Required Python packages: pyaudio, torch, torchvision, opencv-python, cv-bridge
- NVIDIA Isaac Sim (for full functionality)

## Dependencies

All examples depend on:
- ROS 2 Python client library (rclpy)
- Standard message types (std_msgs, sensor_msgs, geometry_msgs)
- Computer vision libraries (OpenCV, PIL)
- Audio processing libraries (PyAudio)

## Learning Objectives

After studying these examples, students should understand:
1. How to implement voice processing pipelines
2. How to create cognitive planning systems for task decomposition
3. How to integrate computer vision for object recognition
4. How to connect all VLA components in a cohesive system