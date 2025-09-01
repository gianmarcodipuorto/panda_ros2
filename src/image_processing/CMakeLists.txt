cmake_minimum_required(VERSION 3.8)
project(image_processing)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(OpenCV REQUIRED)
find_package(message_filters REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(image_geometry REQUIRED)
find_package(realtime_tools REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(panda_interfaces REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(yolo_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)


include_directories(
  include
)








add_executable(skeleton_track_yolo ./src/skeleton_tracker_yolo.cpp)
ament_target_dependencies(skeleton_track_yolo OpenCV ament_index_cpp rclcpp sensor_msgs cv_bridge image_geometry realtime_tools tf2 tf2_ros tf2_eigen panda_interfaces visualization_msgs yolo_msgs tf2_geometry_msgs)
target_link_libraries(skeleton_track_yolo
)

add_executable(image_saver ./examples/image_saver.cpp)
ament_target_dependencies(image_saver rclcpp std_msgs sensor_msgs cv_bridge image_transport OpenCV message_filters)


install(TARGETS
  image_saver
  skeleton_track_yolo
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY
    launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

install(
  DIRECTORY
    include/
  DESTINATION share/${PROJECT_NAME}/include
)

ament_export_include_directories(include)

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
