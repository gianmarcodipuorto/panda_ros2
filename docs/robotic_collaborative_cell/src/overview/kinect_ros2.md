# kinect_ros2  

The package defines the depth and rgb camera calibration parameters in the _cfg_ folder used to publish the relative [CameraInfo](https://docs.ros.org/en/noetic/api/sensor_msgs/html/msg/CameraInfo.html) messages.

## Launch files  

The **stream_kinect.py** launch file allows to define:
- the Tfs of the camera wrt the world frame (camera extrinsic parameters)
- the topic on which the [RegisterNode](https://wiki.ros.org/depth_image_proc) subscribes. The RegisterNode reproject the depth map relatively to the rgb camera, allowing to use a unique CameraInfo message for both the images.

