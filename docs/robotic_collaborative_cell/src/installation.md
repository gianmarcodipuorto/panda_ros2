# Installation  

## System requirements  

- Linux with PREEMPT_RT patched kernel (Ubuntu 16.04 or later, Ubuntu 22.04 recommended)
    - [Franka robotics guide for RT kernel install](https://frankarobotics.github.io/docs/installation_linux.html#setting-up-the-real-time-kernel)
- [libfranka installed](https://github.com/frankarobotics/libfranka) (tested on version 9.0.2)
- ROS2 (Humble)
- [libfreenect installed](https://github.com/fadlio/kinect_ros2) (required for kinect v1 integration)

## Install  

```shell
git clone <repository link>
cd panda_ros
cd yolo_ros
pip3 install -r requirements.txt
cd ..
rosdep install --from-paths src --ignore-src -r -y
colcon build
```


