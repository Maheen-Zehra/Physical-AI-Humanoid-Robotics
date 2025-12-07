# Dependencies for Physical AI & Humanoid Robotics Curriculum

This document outlines all software dependencies, libraries, and tools required to run the Physical AI & Humanoid Robotics curriculum.

## Hardware Requirements

### Minimum System
- **CPU**: Multi-core processor (Intel i7 or AMD Ryzen equivalent)
- **RAM**: 16GB (32GB recommended for Isaac Sim)
- **GPU**: NVIDIA RTX 4070 Ti or higher (RTX 4080+ recommended for Isaac Sim)
- **Storage**: 100GB+ of free disk space
- **OS**: Ubuntu 22.04 LTS

### Recommended System
- **CPU**: Intel i9 or AMD Threadripper with 16+ cores
- **RAM**: 64GB or more
- **GPU**: NVIDIA RTX 4090 for optimal Isaac Sim performance
- **Storage**: 500GB+ SSD storage

## Software Dependencies

### Core System Dependencies
- Ubuntu 22.04 LTS
- NVIDIA GPU drivers (535 or higher)
- CUDA Toolkit (12.2 or higher)
- Python 3.10 or higher

### ROS 2 Dependencies
- **ROS 2 Distribution**: Humble Hawksbill
- **Core Packages**:
  - ros-humble-desktop
  - ros-humble-gazebo-ros-pkgs
  - ros-humble-navigation2
  - ros-humble-nav2-bringup
  - ros-humble-isaac-ros-* (various Isaac ROS packages)

### Python Dependencies
- **Core Libraries**:
  - rclpy (ROS 2 Python client library)
  - numpy
  - torch (PyTorch)
  - torchvision
  - openai (OpenAI API client)
  - pyaudio
  - opencv-python (OpenCV)
  - Pillow (PIL)

- **Development Tools**:
  - colcon (ROS 2 build system)
  - black (code formatter)
  - flake8 (linting tool)
  - pytest (testing framework)

### Simulation Dependencies
- **Gazebo**: Fortress or Garden version
- **Isaac Sim**: Latest version from NVIDIA Developer
- **Unity**: 2022.3 LTS (for digital twin visualization)

### Documentation Dependencies
- **Docusaurus v3**:
  - @docusaurus/core
  - @docusaurus/preset-classic
  - @mdx-js/react
  - clsx
  - prism-react-renderer
  - react and react-dom

### Development Tools
- Git
- Vim/Nano/VS Code
- Node.js (18+)
- npm

## Installation Commands

### System Dependencies
```bash
# NVIDIA GPU drivers
sudo apt update
sudo apt install nvidia-driver-535 nvidia-settings

# CUDA Toolkit
wget https://developer.download.nvidia.com/compute/cuda/12.2.0/local_installers/cuda_12.2.0_535.54.03_linux.run
sudo sh cuda_12.2.0_535.54.03_linux.run

# ROS 2 Humble
sudo apt update
sudo apt install software-properties-common
sudo add-apt-repository universe

sudo apt update && sudo apt install curl gnupg lsb-release
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(source /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

sudo apt update
sudo apt install ros-humble-desktop
sudo apt install python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool build-essential

# Initialize rosdep
sudo rosdep init
rosdep update
```

### Additional ROS Packages
```bash
sudo apt install ros-humble-gazebo-ros-pkgs ros-humble-navigation2 ros-humble-nav2-bringup
sudo apt install python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool build-essential
```

### Python Dependencies
```bash
pip3 install pyaudio numpy torch torchvision openai opencv-python Pillow
pip3 install black flake8 pytest
```

### Documentation Dependencies
```bash
npm install
```

## Verification Commands

### ROS 2 Verification
```bash
source /opt/ros/humble/setup.bash
ros2 --version
```

### Python Dependencies Verification
```bash
python3 -c "import rclpy; print('rclpy imported successfully')"
python3 -c "import torch; print('PyTorch imported successfully')"
python3 -c "import openai; print('OpenAI imported successfully')"
python3 -c "import cv2; print('OpenCV imported successfully')"
```

### Gazebo Verification
```bash
gz sim --version
```

## Optional Dependencies

### Isaac Sim (Optional but Recommended)
- Download from NVIDIA Developer portal
- Follow installation instructions for your platform
- Verify installation by launching Isaac Sim

### Unity (Optional)
- Download Unity Hub from Unity website
- Install Unity 2022.3 LTS
- Install required Unity packages for robotics

## Troubleshooting Common Issues

### GPU Acceleration Not Working
- Verify NVIDIA drivers: `nvidia-smi`
- Check CUDA installation: `nvcc --version`
- Verify GPU compatibility with Isaac Sim

### ROS 2 Commands Not Found
- Ensure ROS 2 is sourced: `source /opt/ros/humble/setup.bash`
- Check ROS 2 installation: `apt list --installed | grep ros-humble`

### Python Package Issues
- Use virtual environments to isolate dependencies
- Install packages with `python3 -m pip install <package>`
- Check Python version: `python3 --version`

## Environment Setup

Add to your `~/.bashrc` file:
```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash  # This will be created during curriculum
```

Then reload: `source ~/.bashrc`

## Curriculum-Specific Dependencies

Each module in the curriculum has additional dependencies:

### Module 1 (ROS 2 Fundamentals)
- Basic ROS 2 packages (already installed)
- No additional dependencies

### Module 2 (Simulation Environments)
- Gazebo simulation packages
- Unity (optional)
- Isaac Sim (optional)

### Module 3 (AI Perception & Navigation)
- Isaac ROS packages
- Computer vision libraries (OpenCV, TorchVision)
- Navigation2 packages

### Module 4 (VLA Integration)
- OpenAI API access
- Speech recognition libraries (PyAudio)
- Large Language Model access

## Development Workflow Dependencies

### Workspace Setup
```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws
colcon build
source install/setup.bash
```

### Testing Commands
```bash
# Test basic ROS 2 functionality
ros2 run demo_nodes_cpp talker &
ros2 run demo_nodes_cpp listener
```

## Documentation Site Dependencies

To run the curriculum documentation site:
```bash
cd /path/to/curriculum/root
npm install
npm start
```

This will serve the documentation at http://localhost:3000

## Version Compatibility

- ROS 2: Humble Hawksbill (required)
- Python: 3.10+ (recommended 3.10)
- Ubuntu: 22.04 LTS (recommended)
- Node.js: 18+ (for documentation site)
- CUDA: 12.2+ (for GPU acceleration)
- Isaac Sim: Latest stable (recommended)