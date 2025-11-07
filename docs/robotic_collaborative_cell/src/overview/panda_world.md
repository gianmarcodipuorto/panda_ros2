# _panda_world_  

The package contains all the files needed for the gazebo simulation containing the Franka Emika Panda robot, the RGB-D camera and a human actor. The robot and camera models are in the _models_ folder, with the respective meshes in the _meshes_ folder. The _config_ folder contains the config file for the [_ros_gz_](https://github.com/gazebosim/ros_gz?tab=readme-ov-file) bridge.

## Nodes  

### *image_converter*  

The *image_converter* node converts the gazebo depth map image in 16UC1 format for the YOLO nodes.

### *joint_state_publisher_effort_patcher*  

The *joint_state_publisher_effort_patcher* node republishes the _JointState_ message coming from the bridge to the same ROS2 network updating the torque field with the torque commanded by the controller. This node is required because the _JointStatePublisher_ plugin in gazebo have an [open issue](https://github.com/gazebosim/gz-sim/issues/883) regarding the torque exterted by the joints.

### *torque_sensor_publisher*  

The *torque_sensor_publisher* node republishes the external torques acting on the joints as a _WrenchStamped_ message.

## Launch files  

The package contains one launch file that spawns several nodes along with the gazebo simulation and an rviz2 instance. The nodes publish the Tf2 of the robot link's frame, the robot's joints state, the rgb and depth images.

## ROS2 network  

![ROS2 network of panda_world package](../images/panda_world.svg)
