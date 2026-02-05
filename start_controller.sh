#! /bin/bash

ros2 lifecycle set /panda_joint_velocities_bridge configure
sleep 2
ros2 lifecycle set /panda_joint_velocities_bridge activate