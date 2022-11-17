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

git clone https://github.com/sharmithag/beginner_tutorials.git
cd beginner_tutorials/
mv cpp_pubsub/ ~/<YOUR ROS2 WS FOLDER>/src
mv string_srv/ ~/<YOUR ROS2 WS FOLDER>/src
cd ~/<YOUR ROS2 WS FOLDER>/src

```
Use colcon to build the package.
```
cd ..
colcon build --packages-select string_srv
colcon build --packages-select cpp_pubsub

```
## Implementation
```
. install/setup.bash
ros2 run cpp_pubsub talker

In other terminal(Ctrl+Shift+t) - FOR CHANGING BASE STRING
. install/setup.bash
ros2 service call /service_topic string_srv/srv/Change
```
