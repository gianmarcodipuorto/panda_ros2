# panda_utils  

The *panda_utils* package contains all the nodes used to control the robot, **whether real or simulated**. The main node is the *impedance_controller* node, implementing the impedance law and other useful modules to bridge the _libfranka_ library, used to communicate with the **Franka Emika Panda** robot, and the rest of the ROS2 network when using the physical robot.  
Moreover, the package contains all the nodes used to generate **trajectories** for the robot's end effector in different situations, nodes used to estimate the distance from the operator keypoints and the robot frames, and the node implementing the entire project's **demo**.  

## Nodes  

### *impedance_controller*  

The *impedance_controller* node allows to control the robot with a cartesian impedance law. The node has 2 possible mode of operation: 
1. `sim`: the node expects a simulation running, sending and receiving robot commands and state throught the _gz_bridge_. 
2. `franka`: the node expects a *robot_ip* to be declared on which the communication has to be established. Upon **activation** of the node, the main thread is spawned starting the control loop of the robot. Along with this one, there are other thread publishing robot state on the network.  

There are many parameters that can be chosen to customize the behaviour of the controller: 
- controller gains (Kp, Kd, Md, translation and rotational)
- controller frequency
- safe range values (max joint speed, max effort value, ...)
- mode of operation

A more in depth analysis of the node is proposed in [robot_controller](../usage/robot_controller.md)  

### *frame_publisher*  

The _frame_publisher_ node publishes the frames relative to the robot when the `franka` mode is used: differently from the simulation mode, the `franka` mode doesn't have a ROS2 integration when the _libfranka_ library is used as is. Because of that, we use the `Pose` of each robot's frame to publish the corresponding Tfs in the network and build the kinematic chain representation. The node offers a ROS2 **service** that can be enabled to start the publishing of the frames based on a `PoseArray` message.

### *send_joints_effort_cmd*  

This node simply unpacks the `JointsEffort` message, sending the corresponding effort value to each of the robot's joints in the simulation environment, through the _gz_bridge_.

### *cart_traj*  

The _cart_traj_ node implements the  `CartTraj` action defined in the [*panda_interfaces*](./panda_interfaces.md) package. The trajectory is generated using a quintic polynomial, ensuring the continuity of position, velocity and accelerations.
<div class="warning">
Differently from the action, the node doesn't use the initial pose defined in the request. Instead, the node uses the current pose of the robot. 
</div>  

If the action is canceled the node generates a trajectory that brings the velocity to 0, according to the one used in the [*exponential_cart_traj* node](#exponential_traj)

### *loop_cart_traj*  

The _loop_cart_traj_ node implements the `LoopCartTraj` action defined in the [*panda_interfaces*](./panda_interfaces.md) package. The action accepts an array of `Pose` and cycle through them generating segment trajectories between two subsequent poses. The first pose is used as a check: if the robot is not in the first position the action is aborted.  

### *exponential_cart_traj* {#exponential_traj}

The _exponential_cart_traj_ node implements the `StopTraj` action in the [*panda_interfaces*](./panda_interfaces.md) package. The trajectory generated brings the velocity of the EE to 0 with an exponential law. The acceleration and the position are respectively derived and integrated from the velocity expression being: 
$$
v_i(t) = \alpha e^{-\lambda t} + \beta e^{-\gamma t}\\, , i = {x, y, z}
$$

### *human_presence*  

The _human_presence_ node computes the distances between the robot's frames and the operator's keypoints, indicating when the operator enters the virtual robotic cell and which frame is in contact with any of the operator's wrists. The node uses the Tf2 system to retrieve all the robot and operator Tfs and then compute the distances as 
$$
d = ||kp_i - frame_j||\\,, i = {1,...,17},\\, j={1,...,7}.
$$ 
The node then compare each of the distances with a threshold: if lower than the threshold value the operator is considered inside the cell. After a certain interval in which the condition is true, the operator is considered to be **inside the cell**.  
If the operator is inside the cell then the distance between the wrists and the robot's frame is computed: if the smallest of these distances happens to be under a certain threshold, the pair of wrist and frame is considered in contact.  
The node allows to define the **contact** and **no_contact** threshold with ROS2 parameters. The 2 thresholds are used to prevent that the frame in contact is changed frequently, degrading the control: the **no_contact** threshold has to be chosen greater than the **contact** one, so that the operator has to move away the hand from the frame in contact and only then touch another frame.  

### *demo*

The _demo_ node implements a complete demo of the project, featuring the robot controller, the trajectory generators, the keypoint estimation and the compliance mode. 5 states are used to take decisions in the demo: 
```cpp
enum class SceneState {
  // The robot has to be moved in home pose
  no_state,
  // The robot is doing whichever task has to do
  task,
  // The human enters the robot area and the robot has to enter compliance
  // mode
  transition_human,
  // The robot is in compliance mode: it can be freely moved by the human
  compliance,
  // The human leaves the robot area, allowing the robot to return in home pose and resume task
  transition_leave_human,
};

```

The task accomplished by the robot is an infinite cartesian trajectory describing a triangle in a fixed plane in space, with a constant orientation. The demo can be monitored with the node described in the [next section](./demo_visulization_app.md).

## Launch files and parameters  

The package contains one launch file that launches the various nodes used to control and command the robot with or without an operator inside the camera view. The parameters of the file are the following: 

- **use_sim_time**: whether to consider real or sim time 
- **controller_kp**, **controller_kd**, **controller_md**, **controller_kp_rot**, **controller_kd_rot**, **controller_md_rot**: controller gains
- **control_rate**: controller rate
- **use_robot**, **robot_ip**: whether or not to use real robot and associated IP on the subnet
- **world_base_link**: robot's base frame to world transform as [x, y, z, w, x, y, z]
- **safe_joint_speed**: maximum joint speed
- **safe_effort_perc**: maximum percentage of maximum control torque allowed
- **wrist_estimation**: whether or not estimate the wrist-frame contact
- **contact_threshold**, **no_contact_threshold**
- **home_pose**: translational part for the home pose of the robot in the demo
