#! /bin/bash

ros2 lifecycle set /impedance_controller configure
sleep 2
ros2 lifecycle set /impedance_controller activate
