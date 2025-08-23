cmake_minimum_required(VERSION 3.5)
project(demo_visualization_app)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rqt_gui_cpp REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Widgets)

include_directories(include ${Qt5Widgets_INCLUDE_DIRS})

add_executable(color_app ./src/color_node_app.cpp)
target_link_libraries(color_app Qt5::Widgets)
ament_target_dependencies(color_app rclcpp std_msgs)

install(TARGETS color_app
  RUNTIME DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

ament_package()

