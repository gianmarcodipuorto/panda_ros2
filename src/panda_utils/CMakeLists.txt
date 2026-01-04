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
find_package(std_srvs REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(pinocchio REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(realtime_tools REQUIRED)
find_package(image_processing REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

include_directories(
  include
  /opt/ros/jazzy/include
  /opt/ros/jazzy/include/pinocchio
  ~/libfranka/include/
  ${CMAKE_CURRENT_SOURCE_DIR}/../image_processing/include
)


link_directories(
  ~/libfranka/build/
)

add_library(robot_model
  src/robot_model.cpp
)

add_library(debug_publisher
  src/debug_publisher.cpp
)

target_link_libraries(robot_model
  ${pinocchio_LIBRARIES}
)

ament_target_dependencies(robot_model
  ament_index_cpp
  tf2_eigen
)

ament_target_dependencies(debug_publisher
  rclcpp_lifecycle
  rclcpp
  sensor_msgs
  panda_interfaces
  geometry_msgs
  tf2_eigen
)

target_link_libraries(debug_publisher
  franka
)

add_executable(send_joints_effort_server ./src/send_joints_effort_cmd.cpp)
ament_target_dependencies(send_joints_effort_server rclcpp panda_interfaces std_msgs tf2_eigen)
target_link_libraries(send_joints_effort_server franka)

add_executable(send_joints_effort_simple_client ./examples/send_joints_effort_cmd_client.cpp)
ament_target_dependencies(send_joints_effort_simple_client rclcpp panda_interfaces std_msgs tf2_eigen)

add_executable(joint_traj ./src/joint_traj.cpp)
ament_target_dependencies(joint_traj rclcpp panda_interfaces std_msgs tf2_eigen tf2 tf2_msgs sensor_msgs realtime_tools)

add_executable(cart_traj ./src/cart_traj.cpp)
ament_target_dependencies(cart_traj rclcpp panda_interfaces std_msgs tf2_eigen tf2 tf2_msgs sensor_msgs realtime_tools)

add_executable(loop_cart_traj ./src/loop_cart_traj.cpp)
ament_target_dependencies(loop_cart_traj rclcpp panda_interfaces std_msgs tf2_eigen tf2 tf2_msgs sensor_msgs realtime_tools)

add_executable(stop_traj ./src/exponential_cart_traj.cpp)
ament_target_dependencies(stop_traj rclcpp panda_interfaces std_msgs tf2_eigen tf2 tf2_msgs sensor_msgs realtime_tools)

add_executable(human_presence ./src/human_presence.cpp ./src/constants.cpp)
ament_target_dependencies(human_presence rclcpp panda_interfaces std_msgs std_srvs tf2_eigen tf2 tf2_msgs sensor_msgs realtime_tools image_processing)

add_executable(frame_publisher ./src/frame_publisher.cpp ./src/constants.cpp)
ament_target_dependencies(frame_publisher rclcpp panda_interfaces std_msgs std_srvs tf2_eigen tf2 tf2_msgs sensor_msgs realtime_tools image_processing)

add_executable(demo ./examples/demo.cpp ./src/constants.cpp)
ament_target_dependencies(demo rclcpp panda_interfaces std_msgs tf2_eigen tf2 tf2_msgs sensor_msgs image_processing std_srvs)
target_link_libraries(demo robot_model)

add_executable(inverse_dynamics_controller ./src/inverse_dynamics_controller.cpp)
ament_target_dependencies(inverse_dynamics_controller rclcpp panda_interfaces std_msgs tf2_eigen tf2 tf2_msgs sensor_msgs pinocchio ament_index_cpp rclcpp_lifecycle realtime_tools)
target_link_libraries(inverse_dynamics_controller robot_model franka)

add_executable(cart_traj_example ./examples/cart_traj_example.cpp)
ament_target_dependencies(cart_traj_example rclcpp panda_interfaces std_msgs tf2_eigen tf2 tf2_msgs sensor_msgs pinocchio ament_index_cpp rclcpp_lifecycle)
target_link_libraries(cart_traj_example robot_model)

add_executable(loop_cart_traj_example ./examples/loop_cart_traj_example.cpp)
ament_target_dependencies(loop_cart_traj_example rclcpp panda_interfaces std_msgs tf2_eigen tf2 tf2_msgs sensor_msgs pinocchio ament_index_cpp rclcpp_lifecycle)
target_link_libraries(loop_cart_traj_example robot_model)

add_executable(joint_traj_example ./examples/joint_traj_example.cpp)
ament_target_dependencies(joint_traj_example rclcpp panda_interfaces std_msgs tf2_eigen tf2 tf2_msgs sensor_msgs pinocchio ament_index_cpp rclcpp_lifecycle)

add_executable(impedance_controller ./src/impedance_controller.cpp ./src/constants.cpp)
ament_target_dependencies(impedance_controller rclcpp panda_interfaces std_msgs tf2_eigen tf2 tf2_msgs sensor_msgs pinocchio ament_index_cpp rclcpp_lifecycle realtime_tools tf2_geometry_msgs std_srvs)
target_link_libraries(impedance_controller robot_model franka debug_publisher)

install(TARGETS
  send_joints_effort_server
  send_joints_effort_simple_client
  joint_traj
  cart_traj
  stop_traj
  loop_cart_traj
  demo
  inverse_dynamics_controller
  cart_traj_example
  joint_traj_example
  impedance_controller
  loop_cart_traj_example
  human_presence
  frame_publisher
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
