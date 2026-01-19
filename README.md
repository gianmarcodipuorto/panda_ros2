# Detailed guide and overview  

A detailed guide and overview of the entire project is available as [mdbook](https://gianlucapandolfi.github.io/panda_ros/)

## System requirements  

- Linux with PREEMPT_RT patched kernel (Ubuntu 16.04 or later, Ubuntu 22.04 recommended)
    - [Franka robotics guide for RT kernel install](https://frankarobotics.github.io/docs/installation_linux.html#setting-up-the-real-time-kernel)
- [libfranka installed](https://github.com/frankarobotics/libfranka) (tested on version 9.0.2)
- ROS2 (Jazzy)
- [libfreenect installed](https://github.com/fadlio/kinect_ros2) (required for kinect v1 integration)

## Dependencies  

```
sudo apt install ros-jazzy-{depth-image-proc,pinocchio,camera-info-manager,realtime-tools}
cd src/yolo_ros
pip install -r requirements.txt
cd ..
rosdep install --from-paths src --ignore-src -r -y
```

## Building  

```shell
colcon build
```

## Configuration (2 machines setup)

```
export ROS_DOMAIN_ID=0
```

## Running the demo  

### Franka Emika panda interface  

The first thing to do is activate the FCI on the panda robot we want to use for the demo. To do this we should connect to the IP of the robot we want to control, open the brakes, and then activate the FCI. 

![FCI activation](./docs/robotic_collaborative_cell/src/images/img-franka-fci-active.jpg)

### Launch files  

The launch files we want to use are located in the:

- *panda_utils* package

- *yolo_bringup* subpackage of the *yolo_ros* package
- *kinect_ros2* package
- *image_processing* package

Once the launch files have been launched, we can launch the demo's gui app with: 
```bash
ros2 run demo_visualization_app color_app "state_color" "human_contact"
```

we can proceed to: 
1. activate the impedance controller
2. activate the demo

### Example of file launching (with FCI)

```bash
ros2 launch yolo_bringup yolov8.launch.py model:=yolov8n-pose.pt max_det:=1\
input_image_topic:=/image_raw input_depth_topic:=/depth/image_raw\
input_depth_info_topic:=/camera_info target_frame:=kinect_rgb\
use_3d:=True use_sim_time:=False

ros2 launch kinect_ros2 stream_kinect.py

ros2 launch image_processing launch_yolo_tracker.py measurement_noise:=0.9\
hallucination_threshold:=0.0 single_keypoint_ma_confidence_threshold:=0.80\
ma_confidence_threshold:=0.80 MA_window_size:=10 use_sim_time:=false\
filter:=true debug:=true predict:=false

ros2 launch panda_utils launch_utils.py controller_kp:=1500.0 controller_kd:=140.0\
controller_md:=5.0 controller_rate:=1000.0 alpha:=30.0 task_gain:=0.0\
use_sim_time:=false controller_kp_rot:=100.0 controller_kd_rot:=5.0\
controller_md_rot:=0.5 robot_ip:=<robot_ip>
```

### Example of file launching (with Gazebo)

```bash
ros2 launch yolo_bringup yolov8.launch.py model:=yolov8n-pose.pt max_det:=1\
input_image_topic:=/image_raw input_depth_topic:=/depth/image_raw\
input_depth_info_topic:=/camera_info target_frame:=kinect_rgb\
use_3d:=True use_sim_time:=True

ros2 launch image_processing launch_yolo_tracker.py measurement_noise:=0.9\
hallucination_threshold:=0.0 single_keypoint_ma_confidence_threshold:=0.80\
ma_confidence_threshold:=0.80 MA_window_size:=10 use_sim_time:=true\
filter:=true debug:=true predict:=false

ros2 launch panda_utils launch_utils.py controller_kp:=1500.0 controller_kd:=140.0\
controller_md:=5.0 controller_rate:=1000.0 alpha:=30.0 task_gain:=0.0\
use_sim_time:=true controller_kp_rot:=100.0 controller_kd_rot:=5.0\
controller_md_rot:=0.5

ros2 launch panda_world launch_sim.py
```
