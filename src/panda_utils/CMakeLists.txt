cmake_minimum_required(VERSION 3.8)
project(panda_utils)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

add_compile_options(-Wall -Wextra)


# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(panda_interfaces REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(pinocchio REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)

include_directories(
  include
  /opt/ros/jazzy/include
  /opt/ros/jazzy/include/pinocchio
  ~/libfranka/include/
)

link_directories(
  ~/libfranka/build/
)

add_library(robot_model
  src/robot_model.cpp
)

target_link_libraries(robot_model
  ${pinocchio_LIBRARIES}
)

ament_target_dependencies(robot_model
  ament_index_cpp
  tf2_eigen
)

add_executable(send_joints_cmd_server ./src/send_joints_pos_cmd.cpp)
ament_target_dependencies(send_joints_cmd_server rclcpp panda_interfaces std_msgs tf2_eigen)

add_executable(send_joints_cmd_simple_client ./examples/send_joints_pos_cmd_client.cpp)
ament_target_dependencies(send_joints_cmd_simple_client rclcpp panda_interfaces std_msgs tf2_eigen)

add_executable(send_joints_effort_server ./src/send_joints_effort_cmd.cpp)
ament_target_dependencies(send_joints_effort_server rclcpp panda_interfaces std_msgs tf2_eigen)
target_link_libraries(send_joints_effort_server franka)

add_executable(send_joints_effort_simple_client ./examples/send_joints_effort_cmd_client.cpp)
ament_target_dependencies(send_joints_effort_simple_client rclcpp panda_interfaces std_msgs tf2_eigen)

add_executable(joint_traj ./src/joint_traj.cpp)
ament_target_dependencies(joint_traj rclcpp panda_interfaces std_msgs tf2_eigen tf2 tf2_msgs sensor_msgs)

add_executable(cart_traj ./src/cart_traj.cpp)
ament_target_dependencies(cart_traj rclcpp panda_interfaces std_msgs tf2_eigen tf2 tf2_msgs sensor_msgs)

add_executable(clik_cmd_pub ./src/clik_cmd_publisher.cpp)
ament_target_dependencies(clik_cmd_pub rclcpp panda_interfaces std_msgs tf2_eigen tf2 tf2_msgs sensor_msgs rclcpp_lifecycle ament_index_cpp)
target_link_libraries(clik_cmd_pub robot_model)

add_executable(demo_node ./examples/demo_node.cpp)
ament_target_dependencies(demo_node rclcpp panda_interfaces std_msgs tf2_eigen tf2 tf2_msgs sensor_msgs)

add_executable(pinocchio_example ./examples/pinocchio_example.cpp)
ament_target_dependencies(pinocchio_example rclcpp panda_interfaces std_msgs tf2_eigen tf2 tf2_msgs sensor_msgs pinocchio ament_index_cpp)
target_link_libraries(pinocchio_example robot_model)

add_executable(pd_grav_controller ./src/pd_grav_controller.cpp)
ament_target_dependencies(pd_grav_controller rclcpp panda_interfaces std_msgs tf2_eigen tf2 tf2_msgs sensor_msgs pinocchio ament_index_cpp rclcpp_lifecycle)
target_link_libraries(pd_grav_controller robot_model)

add_executable(inverse_dynamics_controller ./src/inverse_dynamics_controller.cpp)
ament_target_dependencies(inverse_dynamics_controller rclcpp panda_interfaces std_msgs tf2_eigen tf2 tf2_msgs sensor_msgs pinocchio ament_index_cpp rclcpp_lifecycle)
target_link_libraries(inverse_dynamics_controller robot_model franka)

add_executable(pd_grav_controller_demo ./examples/demo_pd_grav.cpp)
ament_target_dependencies(pd_grav_controller_demo rclcpp panda_interfaces std_msgs tf2_eigen tf2 tf2_msgs sensor_msgs pinocchio ament_index_cpp rclcpp_lifecycle)
target_link_libraries(pd_grav_controller_demo robot_model)

add_executable(controller_manager ./src/controller_manager.cpp)
ament_target_dependencies(controller_manager rclcpp panda_interfaces std_msgs tf2_eigen tf2 tf2_msgs sensor_msgs pinocchio ament_index_cpp rclcpp_lifecycle)
target_link_libraries(controller_manager robot_model)

add_executable(cart_traj_example ./examples/cart_traj_example.cpp)
ament_target_dependencies(cart_traj_example rclcpp panda_interfaces std_msgs tf2_eigen tf2 tf2_msgs sensor_msgs pinocchio ament_index_cpp rclcpp_lifecycle)
target_link_libraries(cart_traj_example robot_model)

add_executable(joint_traj_example ./examples/joint_traj_example.cpp)
ament_target_dependencies(joint_traj_example rclcpp panda_interfaces std_msgs tf2_eigen tf2 tf2_msgs sensor_msgs pinocchio ament_index_cpp rclcpp_lifecycle)

add_executable(impedance_controller ./src/impedance_controller.cpp)
ament_target_dependencies(impedance_controller rclcpp panda_interfaces std_msgs tf2_eigen tf2 tf2_msgs sensor_msgs pinocchio ament_index_cpp rclcpp_lifecycle)
target_link_libraries(impedance_controller robot_model franka)

install(TARGETS
  send_joints_cmd_server
  send_joints_cmd_simple_client
  send_joints_effort_server
  send_joints_effort_simple_client
  joint_traj
  cart_traj
  clik_cmd_pub
  demo_node
  pinocchio_example
  pd_grav_controller
  pd_grav_controller_demo
  inverse_dynamics_controller
  controller_manager
  cart_traj_example
  joint_traj_example
  impedance_controller
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY
    launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

ament_export_include_directories(include)
ament_export_dependencies(tf2_eigen)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
