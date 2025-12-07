---
sidebar_position: 2
---

# Setup Guide

## Prerequisites

Before starting the Physical AI & Humanoid Robotics curriculum, you'll need to set up your development environment with the required software and hardware.

### Hardware Requirements

- **CPU**: Multi-core processor (Intel i7 or AMD Ryzen equivalent)
- **RAM**: 16GB or more (32GB recommended for Isaac Sim)
- **GPU**: NVIDIA RTX 4070 Ti or higher (RTX 4080+ recommended for Isaac Sim)
- **Storage**: 100GB+ of free disk space
- **OS**: Ubuntu 22.04 LTS (other versions may work but are not tested)

### Software Requirements

- Ubuntu 22.04 LTS
- ROS 2 Humble Hawksbill
- NVIDIA GPU drivers (535 or higher)
- CUDA Toolkit (12.2 or higher)
- Python 3.10 or higher

## Installation Steps

### 1. Install Ubuntu 22.04 LTS

If you don't already have Ubuntu 22.04 LTS installed, download it from [ubuntu.com](https://ubuntu.com/download/desktop) and follow the installation instructions.

### 2. Install NVIDIA GPU Drivers

```bash
sudo apt update
sudo apt install nvidia-driver-535 nvidia-settings
sudo reboot
```

### 3. Install CUDA Toolkit

```bash
wget https://developer.download.nvidia.com/compute/cuda/12.2.0/local_installers/cuda_12.2.0_535.54.03_linux.run
sudo sh cuda_12.2.0_535.54.03_linux.run
```

Follow the installer prompts, ensuring you install the CUDA Toolkit (but skip the driver installation if you already installed it above).

### 4. Install ROS 2 Humble

Set up your computer to accept software from packages.ros.org:

```bash
sudo apt update
sudo apt install software-properties-common
sudo add-apt-repository universe
```

Add the ROS 2 GPG key and repository:

```bash
sudo apt update && sudo apt install curl gnupg lsb-release
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(source /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
```

Install ROS 2 Humble packages:

```bash
sudo apt update
sudo apt install ros-humble-desktop
sudo apt install python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool build-essential
```

### 5. Initialize rosdep

```bash
sudo apt update
sudo apt install python3-rosdep
sudo rosdep init
rosdep update
```

### 6. Install Additional ROS Packages

```bash
sudo apt install ros-humble-gazebo-ros-pkgs ros-humble-navigation2 ros-humble-nav2-bringup
sudo apt install ros-humble-isaac-ros-*  # This may need to be installed separately depending on availability
```

### 7. Install Python Dependencies

```bash
pip3 install pyaudio numpy torch torchvision openai
```

### 8. Set Up ROS 2 Environment

Add the following lines to your `~/.bashrc` file:

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash  # This will be created later in the curriculum
```

Then reload your bash configuration:

```bash
source ~/.bashrc
```

### 9. Install Development Tools

```bash
sudo apt install git vim nano
pip3 install black flake8 pytest
```

## Verification Steps

### 1. Verify ROS 2 Installation

```bash
source /opt/ros/humble/setup.bash
ros2 --version
```

### 2. Create a Test Workspace

```bash
mkdir -p ~/test_ws/src
cd ~/test_ws
colcon build
source install/setup.bash
```

### 3. Test Basic ROS 2 Functionality

Open two terminals:

Terminal 1:
```bash
source ~/test_ws/install/setup.bash
ros2 run demo_nodes_cpp talker
```

Terminal 2:
```bash
source ~/test_ws/install/setup.bash
ros2 run demo_nodes_cpp listener
```

You should see messages being published from the talker and received by the listener.

### 4. Verify Python Dependencies

```bash
python3 -c "import rclpy; print('rclpy imported successfully')"
python3 -c "import torch; print('PyTorch imported successfully')"
python3 -c "import openai; print('OpenAI imported successfully')"
```

## Optional: Install Isaac Sim

For the AI Perception module, you may want to install NVIDIA Isaac Sim:

1. Download Isaac Sim from [NVIDIA Developer](https://developer.nvidia.com/isaac-sim)
2. Follow the installation instructions for your platform
3. Verify installation by launching Isaac Sim

## Troubleshooting

### Common Issues and Solutions

**Issue**: `command 'ros2' not found`
**Solution**: Ensure you've sourced the ROS 2 setup file: `source /opt/ros/humble/setup.bash`

**Issue**: GPU acceleration not working
**Solution**: Verify your NVIDIA drivers are properly installed: `nvidia-smi`

**Issue**: Python packages not found
**Solution**: Ensure you're using the correct Python environment and pip: `python3 -m pip install <package>`

## Next Steps

Once your environment is set up and verified, proceed to [Module 1: ROS 2 Fundamentals](./modules/ros2-fundamentals/index.md) to begin the curriculum.