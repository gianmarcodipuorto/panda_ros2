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

include_directories(
  include
)

# add_library(robot ./include/panda_utils/robot.hpp)
# ament_target_dependencies(robot rclcpp panda_interfaces std_msgs tf2_eigen tf2 tf2_msgs)

# add_library(constants ./include/panda_utils/constants.hpp)
# ament_target_dependencies(constants tf2_eigen tf2 tf2_msgs)
# target_link_libraries(constants robot)

add_executable(send_joints_cmd_server ./src/send_joints_pos_cmd.cpp)
ament_target_dependencies(send_joints_cmd_server rclcpp panda_interfaces std_msgs tf2_eigen)
# target_link_libraries(send_joints_cmd_server constants)

add_executable(send_joints_cmd_simple_client ./src/send_joints_pos_cmd_client.cpp)
ament_target_dependencies(send_joints_cmd_simple_client rclcpp panda_interfaces std_msgs tf2_eigen)
# target_link_libraries(send_joints_cmd_simple_client constants)

add_executable(calc_jacobian_server ./src/jacobian_server.cpp)
ament_target_dependencies(calc_jacobian_server rclcpp panda_interfaces std_msgs tf2_eigen tf2 tf2_msgs)
# target_link_libraries(calc_jacobian_server constants)

add_executable(calc_jacobian_client ./src/jacobian_client.cpp)
ament_target_dependencies(calc_jacobian_client rclcpp panda_interfaces std_msgs tf2_eigen tf2 tf2_msgs)
# target_link_libraries(calc_jacobian_client constants)

add_executable(calc_forward_kine_server ./src/forward_kine_server.cpp)
ament_target_dependencies(calc_forward_kine_server rclcpp panda_interfaces std_msgs tf2_eigen tf2 tf2_msgs)
# target_link_libraries(calc_forward_kine_server robot)

add_executable(calc_forward_kine_client ./src/forward_kine_client.cpp)
ament_target_dependencies(calc_forward_kine_client rclcpp panda_interfaces std_msgs tf2_eigen tf2 tf2_msgs)
# target_link_libraries(calc_forward_kine_client constants)

add_executable(calc_clik_server ./src/clik_server.cpp)
ament_target_dependencies(calc_clik_server rclcpp panda_interfaces std_msgs tf2_eigen tf2 tf2_msgs)
# target_link_libraries(calc_clik_server constants)

add_executable(calc_clik_client ./src/clik_client.cpp)
ament_target_dependencies(calc_clik_client rclcpp panda_interfaces std_msgs tf2_eigen tf2 tf2_msgs)
# target_link_libraries(calc_clik_client constants)

install(TARGETS
  send_joints_cmd_server
  send_joints_cmd_simple_client
  calc_jacobian_server
  calc_jacobian_client
  calc_forward_kine_server
  calc_forward_kine_client
  calc_clik_server
  calc_clik_client
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
