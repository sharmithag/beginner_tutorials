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
#### default timer rate is 100ms
ros2 launch cpp_publish cpp_publish.yaml freq:=200

In other terminal(Ctrl+Shift+t) - FOR CHANGING BASE STRING

. install/setup.bash
ros2 service call /service_topic string_srv/srv/Change
```
## RQT_CONSOLE
![Screenshot from 2022-11-17 18-37-27](https://user-images.githubusercontent.com/90351952/202582825-96f0edec-98cb-44c6-88de-1a7418c8f503.png)
