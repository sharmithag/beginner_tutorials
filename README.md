# beginner_tutorials

## Overview
Basic publisher and subcriber implementation in ROS2 Foxy

## Requirements
- Ubuntu 20.04 LTS and `ROS2 Foxy`
- rosdep

## Building the package

```
source /opt/ros/foxy/setup.bash
```

Clone the repository in src of ros workspace
```
cd <YOUR ROS2 WS FOLDER>/src
git clone https://github.com/sharmithag/beginner_tutorials.git
```
Use colcon to build the package.
```
cd <YOUR ROS2 WS FOLDER>
colcon build --packages-select cpp_pubsub

```
## Implementation
```
. install/setup.bash
ros2 run cpp_pubsub talker

In other terminal(Ctrl+Shift+t)
. install/setup.bash
ros2 run cpp_pubsub listener
```
