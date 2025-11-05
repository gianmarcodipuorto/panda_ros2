# Overview of the system  

The system consists of several ROS2 nodes, each specialized for its own task and divided into different ROS2 packages within the repository: 

- [_panda_interfaces_](./panda_interfaces.md): consisting of messages, services and actions interfaces used in the other packages.
- [_panda_world_](./panda_world.md): containing the files relative to the simulation and a few nodes bridging the ROS2 controller and keypoint recognition system with the _gazebo_ simulation.
- [_kinect_ros2_](./kinect_ros2.md): the _kinect_ros2_ package, allowing the publication of images and depth maps from the kinect v1 to the ROS2 network.
- [_yolo_ros_](./yolo_ros.md): the [_yolo_ros_](https://github.com/mgonzs13/yolo_ros) ROS2 package, used for the keypoint recognition using the [Ultralytics YOLO](https://docs.ultralytics.com/) models.
- [_image_processing_](./image_processing.md): consisting of the skeleton tracker algorithm that publishes the TFs of the keypoint, along with other messages regarding the skeleton, on the ROS2 network.
- [_panda_utils_](./panda_utils.md): consisting of the nodes used for the control and data publication of the Franka Emika Panda robot used in this work.
- [_demo_visulization_app_](./demo_visulization_app.md): containing a ROS2 node allowing the activation of the robot's controller and launching the demo.


