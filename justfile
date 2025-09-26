[no-cd]
build:
	colcon build

[no-cd]
release:
	colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release -DCMAKE_CXX_FLAGS_RELEASE="-O1 -DNDEBUG"

[no-cd]
init: 
  build

[no-cd]
completion: 
	colcon build --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON --cmake-clean-cache

[no-cd]
completion-release: 
	colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release -DCMAKE_CXX_FLAGS_RELEASE="-O1 -DNDEBUG" -DCMAKE_EXPORT_COMPILE_COMMANDS=ON --cmake-clean-cache

[no-cd]
ros-pkg pkg-name:
  ros2 pkg create --build-type ament_cmake --license Apache-2.0 {{pkg-name}}

[no-cd]
build-pkg pkg-name:
	colcon build --packages-select {{pkg-name}}

