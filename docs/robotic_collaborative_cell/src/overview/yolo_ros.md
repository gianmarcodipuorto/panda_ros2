# yolo_ros  

The ROS2 wrapper for [Ultralytics](https://github.com/ultralytics/ultralytics) YOLO models. In this project it has been used for 3D human pose detection of the operator in the scene seen by the kinect camera. The pipeline expects an rgb image and eventually a depth image to estimate the keypoint on the human body (17, defined in the [COCO-Pose](https://cocodataset.org/#keypoints-2017) dataset), outputting a _DetectionArray_ message containing all the human detections. 

The pipeline have been used to detect only 1 human in the scene.
The model used is the **yolov8n-pose**.


## Launch files and parameters  

The launch file to use with the model chosen is the **yolov8.launch.py** in the _yolo_bringup_ sub-package.
The parameters used are the following:
- **model**: YOLO model to use 
- **max_det**: maximum number of detections 
- **input_image_topic**: camera topic of RGB image 
- **input_depth_topic**: camera topic of depth image
- **input_depth_info_topic**: camera topic for info data 
- **target_frame**: frame to transform the 3D keypoints 
- **use_3D**: whether to activate 3D detections 
- **use_sim_time**: whether to consider real or sim time 

Example: 
```shell
ros2 launch yolo_bringup yolov8.launch.py model:=yolov8n-pose.pt max_det:=1\
input_image_topic:=/image_raw input_depth_topic:=/depth/image_raw\
input_depth_info_topic:=/camera_info target_frame:=kinect_rgb\
use_3d:=True use_sim_time:=False
```
