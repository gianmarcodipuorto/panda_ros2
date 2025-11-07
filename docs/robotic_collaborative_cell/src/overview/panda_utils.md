# panda_utils  

The *panda_utils* package contains all the nodes used to control the robot, **whether real or simulated**. The main node is the *impedance_controller* node, implementing the impedance law and other useful modules to bridge the _libfranka_ library, used to communicate with the **Franka Emika Panda** robot, and the rest of the ROS2 network when using the physical robot.  
Moreover, the package contains all the nodes used to generate **trajectories** for the robot's end effector in different situations, nodes used to estimate the distance from the operator keypoints and the robot frames, and the node implementing the entire project's **demo**.  

## Nodes  

### *impedance_controller*  

### *frame_publisher*  

### *send_joints_effort_cmd*  

### *cart_traj*  

### *loop_cart_traj*  

### *exponential_cart_traj*  

### *impedance_controller*  

### *demo*

