# Learning ROS2 Advanced concepts

## Getting started

```bash
# Create workspace
mkdir ros2_ws

# Clone this project
cd ros2_ws
git clone https://github.com/bulingen/learning-ros2-advanced-concepts.git

# Rename folder to src
mv learning-ros2-advanced-concepts src

# Source ROS
source /opt/ros/$ROS_DISTRO/setup.bash

# Build
colcon build --symlink-install

# Run something
ros2 run ...
```
