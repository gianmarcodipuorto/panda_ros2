#! /bin/bash

echo "Choose controller"
echo "1) (joint space) inverse dynamics controller"
echo "2) Impedance controller"
echo "0) Exit"

read -r number

case "$number" in
  1)
    echo "Activating inverse dynamics controller"
    ros2 lifecycle set /inverse_dynamics_controller configure
    echo "Configured"
    sleep 1
    ros2 lifecycle set /inverse_dynamics_controller activate
    echo "Activated"
    echo ""
    ;;
  2)
    echo "Activating impedance controller"
    ros2 lifecycle set /impedance_controller configure
    echo "Configured"
    sleep 1
    ros2 lifecycle set /impedance_controller activate
    echo "Activated"
    echo ""
    ;;
  0)
    echo "Exiting"
    exit 0
    ;;
  *)
    echo "Unknown number: $number"
    ;;
esac

read -p "Activate CLIK? (y/n): " answer
case "$answer" in
  [Yy]* )
    echo "Activating CLIK";
    ros2 lifecycle set /clik_cmd_pub configure;
    echo "Configured";
    sleep 1
    ros2 lifecycle set /clik_cmd_pub activate;
    echo "Activated";;
  [Nn]* )
    exit 1;;
  * )
    echo "Invalid input. Please answer y or n."; exit 1;;
esac
