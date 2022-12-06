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
#### default timer rate is 100ms
```
. install/setup.bash
ros2 launch cpp_pubsub cpp_pubsub.yaml freq:=200
```
#### In other terminal(Ctrl+Shift+t) - FOR CHANGING BASE STRING
```
. install/setup.bash
ros2 service call /service_topic string_srv/srv/Change
```
#### In other terminal(Ctrl+Shift+t) - FOR CHECKING tf topic
```
. install/setup.bash
ros2 run tf2_ros tf2_echo world talk
```
#### In other terminal(Ctrl+Shift+t) - FOR CHECKING tf topic frames
```
. install/setup.bash
ros2 run tf2_tools view_frames.py

```
#### Launch file for rosbag record
```
. install/setup.bash
ros2 launch cpp_pubsub cpp_pubsub.py
```
## RQT_CONSOLE
![Screenshot from 2022-11-17 18-39-15](https://user-images.githubusercontent.com/90351952/202583023-bf5046e9-4380-4a2a-9ac2-54dd59fda0d3.png)

