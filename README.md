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

## Useful commands

```bash
# List actions with their corresponding types
ros2 action list -t

# List hidden topics
ros2 topic list --include-hidden-topics

# List hidden services
ros2 topic list --include-hidden-services

# Get info about given action
ros2 action info /count_until

# Send goal from terminal
ros2 action send_goal /count_until my_robot_interfaces/action/CountUntil "{target_number: 4, period: 1.3}"

# And also getting feedback
ros2 action send_goal /count_until my_robot_interfaces/action/CountUntil "{target_number: 4, period: 1.3}" --feedback
```

## Goal policies

### Multiple goals in parallel

Running multiple goals in parallel can easily be achieved by just adding this to the server:

```diff python
+from rclpy.callback_groups import ReentrantCallbackGroup

self.my_server_ = ActionServer(
    self,
    MyGoal,
    "my_goal",
    goal_callback=self.goal_callback,
    cancel_callback=self.cancel_callback,
    execute_callback=self.execute_callback,
+   callback_group=ReentrantCallbackGroup(),
)
```

and this:

```diff python
+from rclpy.executors import MultiThreadedExecutor

def main(args=None):
    rclpy.init(args=args)
    node = CountUntilServerNode()
-   rclpy.spin(node)
+   rclpy.spin(node, MultiThreadedExecutor())
    rclpy.shutdown()
```

Try it out by just starting multiple clients. The server will handle all clients individually.
