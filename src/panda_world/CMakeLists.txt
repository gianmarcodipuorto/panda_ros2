cmake_minimum_required(VERSION 3.8)
project(panda_world)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(panda_interfaces REQUIRED)

add_executable(joint_state_publisher_effort_patcher ./src/joint_state_publisher_effort_patcher.cpp)
ament_target_dependencies(joint_state_publisher_effort_patcher rclcpp sensor_msgs panda_interfaces)

add_executable(force_torque_sensor_publisher ./src/force_torque_sensors_publisher.cpp)
ament_target_dependencies(force_torque_sensor_publisher rclcpp sensor_msgs panda_interfaces)

install(TARGETS
  joint_state_publisher_effort_patcher
  force_torque_sensor_publisher
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY
    launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

install(
  DIRECTORY
    meshes/
  DESTINATION share/${PROJECT_NAME}/meshes
)

install(
  DIRECTORY
    config/
  DESTINATION share/${PROJECT_NAME}/config
)

install(
  DIRECTORY
    models/
  DESTINATION share/${PROJECT_NAME}/models
)

install(
  DIRECTORY
    worlds/
  DESTINATION share/${PROJECT_NAME}/worlds
)

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
