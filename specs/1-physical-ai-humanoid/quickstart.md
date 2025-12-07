# Quickstart Guide: Physical AI & Humanoid Robotics

**Feature**: 1-physical-ai-humanoid
**Date**: 2025-12-06
**Prerequisites**: Ubuntu 22.04 LTS, RTX 4080+ GPU, NVIDIA drivers

## Overview

This quickstart guide provides the essential steps to set up the Physical AI & Humanoid Robotics development environment and run your first simulation. Follow these steps to get started with the curriculum modules.

## Prerequisites

### Hardware Requirements
- **Development Workstation**:
  - CPU: 8+ cores (Intel i7/AMD Ryzen 7 or better)
  - RAM: 32GB or more
  - GPU: NVIDIA RTX 4080 or equivalent (20+ GB VRAM)
  - Storage: 2TB SSD (for Isaac Sim and datasets)
- **Edge Device** (optional for deployment):
  - NVIDIA Jetson Orin NX (100+ TOPS AI performance)

### Software Requirements
- **Operating System**: Ubuntu 22.04 LTS (fully updated)
- **Docker**: Version 20.10 or higher
- **NVIDIA Container Toolkit**: For GPU-accelerated containers
- **Python**: 3.10 or higher
- **ROS 2**: Humble Hawksbill distribution

## Environment Setup

### 1. Install ROS 2 Humble

```bash
# Set locale
locale  # check for UTF-8
sudo locale-gen en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

# Setup sources
sudo apt update && sudo apt install -y curl gnupg lsb-release
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(source /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Install ROS 2 packages
sudo apt update
sudo apt install -y ros-humble-desktop ros-humble-ros-base
sudo apt install -y python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool build-essential
sudo apt install -y ros-humble-gazebo-* ros-humble-navigation2 ros-humble-nav2-bringup

# Initialize rosdep
sudo rosdep init
rosdep update

# Source ROS 2 environment
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

### 2. Install NVIDIA Isaac Sim

```bash
# Download Isaac Sim from NVIDIA Developer portal
# Follow the installation guide at: https://docs.omniverse.nvidia.com/isaacsim/latest/installation-guide/index.html

# Install Isaac ROS dependencies
sudo apt install ros-humble-isaac-ros-* ros-humble-nav2-*

# Verify Isaac Sim installation
cd ~/isaac-sim
./isaac-sim-launch.sh
```

### 3. Set up Development Workspace

```bash
# Create workspace
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws

# Build workspace
colcon build --symlink-install

# Source workspace
source install/setup.bash
echo "source ~/ros2_ws/install/setup.bash" >> ~/.bashrc
```

### 4. Install Additional Dependencies

```bash
# Python packages
pip3 install openai-whisper transformers torch torchvision torchaudio

# Unity (for digital twin)
# Download Unity Hub and Unity 2022.3 LTS from Unity website
# Install ROS TCP Connector package from Unity Asset Store

# Additional tools
sudo apt install ros-humble-rosbridge-suite ros-humble-web-video-server
```

## First Simulation: ROS 2 Basics

### 1. Create a Basic ROS 2 Package

```bash
cd ~/ros2_ws/src
ros2 pkg create --build-type ament_python humanoid_control
cd humanoid_control
```

### 2. Create a Simple Publisher Node

Create `humanoid_control/humanoid_control/talker.py`:

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

### 3. Create a Subscriber Node

Create `humanoid_control/humanoid_control/listener.py`:

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

### 4. Run the Basic Example

```bash
# Terminal 1: Build the package
cd ~/ros2_ws
colcon build --packages-select humanoid_control
source install/setup.bash

# Terminal 2: Run the publisher
ros2 run humanoid_control talker

# Terminal 3: Run the subscriber
ros2 run humanoid_control listener
```

## First Simulation: Gazebo Environment

### 1. Launch Gazebo with a Simple Robot

```bash
# Launch empty world
ros2 launch gazebo_ros empty_world.launch.py

# In another terminal, spawn a simple robot
ros2 run gazebo_ros spawn_entity.py -entity simple_robot -file $(ros2 pkg prefix humanoid_control)/share/humanoid_control/models/simple_robot.urdf
```

### 2. Create a URDF Model

Create `humanoid_control/humanoid_control/models/simple_robot.urdf`:

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
</robot>
```

## First AI Integration: Voice Command Processing

### 1. Test Whisper Installation

```python
# Test script: test_whisper.py
import whisper
import torch

# Check if CUDA is available
device = "cuda" if torch.cuda.is_available() else "cpu"
print(f"Using device: {device}")

# Load model
model = whisper.load_model("base", device=device)
print("Whisper model loaded successfully")
```

### 2. Simple Voice Command Node

Create `humanoid_control/humanoid_control/voice_commander.py`:

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import whisper
import torch
import pyaudio
import wave
import numpy as np


class VoiceCommander(Node):

    def __init__(self):
        super().__init__('voice_commander')
        self.publisher_ = self.create_publisher(String, 'parsed_commands', 10)

        # Initialize Whisper model
        device = "cuda" if torch.cuda.is_available() else "cpu"
        self.model = whisper.load_model("base", device=device)

        # Audio parameters
        self.chunk = 1024
        self.format = pyaudio.paInt16
        self.channels = 1
        self.rate = 44100
        self.record_seconds = 3

        # Start audio processing
        self.get_logger().info('Voice Commander initialized')

    def record_audio(self):
        p = pyaudio.PyAudio()

        stream = p.open(format=self.format,
                        channels=self.channels,
                        rate=self.rate,
                        input=True,
                        frames_per_buffer=self.chunk)

        self.get_logger().info("Recording...")
        frames = []

        for i in range(0, int(self.rate / self.chunk * self.record_seconds)):
            data = stream.read(self.chunk)
            frames.append(data)

        self.get_logger().info("Finished recording")

        stream.stop_stream()
        stream.close()
        p.terminate()

        # Save as WAV file
        wf = wave.open("temp_audio.wav", 'wb')
        wf.setnchannels(self.channels)
        wf.setsampwidth(p.get_sample_size(self.format))
        wf.setframerate(self.rate)
        wf.writeframes(b''.join(frames))
        wf.close()

        return "temp_audio.wav"

    def transcribe_audio(self, audio_file):
        result = self.model.transcribe(audio_file)
        return result["text"]


def main(args=None):
    rclpy.init(args=args)
    voice_commander = VoiceCommander()

    # Record and transcribe
    audio_file = voice_commander.record_audio()
    text = voice_commander.transcribe_audio(audio_file)

    # Publish the recognized text
    msg = String()
    msg.data = text
    voice_commander.publisher_.publish(msg)
    voice_commander.get_logger().info(f'Published: "{text}"')

    voice_commander.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

## Running the Curriculum Modules

### Module 1: The Robotic Nervous System (ROS 2)
```bash
# Navigate to the module directory
cd ~/ros2_ws/src/humanoid_control

# Run the ROS 2 fundamentals examples
source install/setup.bash
# See examples in the humanoid_control package
```

### Module 2: The Digital Twin (Gazebo & Unity)
```bash
# Launch simulation environment
source install/setup.bash
# Use the launch files in the simulation packages
```

### Module 3: The AI-Robot Brain (NVIDIA Isaac)
```bash
# Launch Isaac Sim environment
cd ~/isaac-sim
./isaac-sim-launch.sh
# Follow Isaac Sim documentation for humanoid setup
```

### Module 4: Vision-Language-Action (VLA)
```bash
# Run voice-to-action pipeline
source install/setup.bash
# Use the VLA nodes in the ai_integration packages
```

## Troubleshooting

### Common Issues

1. **ROS 2 Not Found**: Ensure you've sourced the setup.bash file:
   ```bash
   source /opt/ros/humble/setup.bash
   source ~/ros2_ws/install/setup.bash
   ```

2. **GPU Not Detected**: Verify NVIDIA drivers are installed:
   ```bash
   nvidia-smi
   ```

3. **Isaac Sim Not Launching**: Check if you have sufficient GPU memory and correct permissions.

4. **Python Package Issues**: Use virtual environments to avoid conflicts:
   ```bash
   python3 -m venv ~/ros2_env
   source ~/ros2_env/bin/activate
   pip3 install -r requirements.txt
   ```

## Next Steps

1. Complete the ROS 2 fundamentals module exercises
2. Set up your Gazebo simulation environment
3. Explore the Isaac Sim tutorials
4. Integrate voice command processing with robot control
5. Work through the capstone autonomous humanoid project

## Resources

- [ROS 2 Humble Documentation](https://docs.ros.org/en/humble/)
- [Isaac Sim Documentation](https://docs.omniverse.nvidia.com/isaacsim/latest/index.html)
- [Gazebo Documentation](http://gazebosim.org/)
- [Unity Robotics Hub](https://unity.com/products/unity-robotics)
- [Academic Papers](https://ieeexplore.ieee.org/)