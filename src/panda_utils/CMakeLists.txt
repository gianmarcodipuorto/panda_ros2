cmake_minimum_required(VERSION 3.8)
project(panda_utils)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

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

include_directories(
  include
  /opt/ros/jazzy/include
  /opt/ros/jazzy/include/pinocchio
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

add_executable(send_joints_cmd_simple_client ./src/send_joints_pos_cmd_client.cpp)
ament_target_dependencies(send_joints_cmd_simple_client rclcpp panda_interfaces std_msgs tf2_eigen)

add_executable(send_joints_effort_server ./src/send_joints_effort_cmd.cpp)
ament_target_dependencies(send_joints_effort_server rclcpp panda_interfaces std_msgs tf2_eigen)

add_executable(send_joints_effort_simple_client ./src/send_joints_effort_cmd_client.cpp)
ament_target_dependencies(send_joints_effort_simple_client rclcpp panda_interfaces std_msgs tf2_eigen)

add_executable(calc_jacobian_server ./src/jacobian_server.cpp)
ament_target_dependencies(calc_jacobian_server rclcpp panda_interfaces std_msgs tf2_eigen tf2 tf2_msgs)

add_executable(calc_jacobian_client ./src/jacobian_client.cpp)
ament_target_dependencies(calc_jacobian_client rclcpp panda_interfaces std_msgs tf2_eigen tf2 tf2_msgs)

add_executable(calc_forward_kine_server ./src/forward_kine_server.cpp)
ament_target_dependencies(calc_forward_kine_server rclcpp panda_interfaces std_msgs tf2_eigen tf2 tf2_msgs)

add_executable(calc_forward_kine_client ./src/forward_kine_client.cpp)
ament_target_dependencies(calc_forward_kine_client rclcpp panda_interfaces std_msgs tf2_eigen tf2 tf2_msgs)

add_executable(calc_clik_server ./src/clik_server.cpp)
ament_target_dependencies(calc_clik_server rclcpp panda_interfaces std_msgs tf2_eigen tf2 tf2_msgs)

add_executable(calc_clik_client ./src/clik_client.cpp)
ament_target_dependencies(calc_clik_client rclcpp panda_interfaces std_msgs tf2_eigen tf2 tf2_msgs)

add_executable(joint_traj ./src/joint_traj.cpp)
ament_target_dependencies(joint_traj rclcpp panda_interfaces std_msgs tf2_eigen tf2 tf2_msgs sensor_msgs)

add_executable(cart_traj ./src/cart_traj.cpp)
ament_target_dependencies(cart_traj rclcpp panda_interfaces std_msgs tf2_eigen tf2 tf2_msgs sensor_msgs)

add_executable(clik_cmd_pub ./src/clik_cmd_publisher.cpp)
ament_target_dependencies(clik_cmd_pub rclcpp panda_interfaces std_msgs tf2_eigen tf2 tf2_msgs sensor_msgs)

add_executable(demo_node ./src/demo_node.cpp)
ament_target_dependencies(demo_node rclcpp panda_interfaces std_msgs tf2_eigen tf2 tf2_msgs sensor_msgs)

add_executable(pinocchio_example ./src/pinocchio_example.cpp)
ament_target_dependencies(pinocchio_example rclcpp panda_interfaces std_msgs tf2_eigen tf2 tf2_msgs sensor_msgs pinocchio ament_index_cpp)
target_link_libraries(pinocchio_example robot_model)

install(TARGETS
  send_joints_cmd_server
  send_joints_cmd_simple_client
  send_joints_effort_server
  send_joints_effort_simple_client
  calc_jacobian_server
  calc_jacobian_client
  calc_forward_kine_server
  calc_forward_kine_client
  calc_clik_server
  calc_clik_client
  joint_traj
  cart_traj
  clik_cmd_pub
  demo_node
  pinocchio_example
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
