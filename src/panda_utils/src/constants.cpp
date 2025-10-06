#include "panda_utils/constants.hpp"

// This time should be at 1/30 Hz, but for connection delays reasons it's greater
const rclcpp::Duration max_tf_age = rclcpp::Duration::from_seconds(0.5);
const double robot_radius_area = 0.5; // meters
