
#include "panda_interfaces/msg/joints_commanded_velocities.hpp"
#include "panda_utils/constants.hpp"
#include "rclcpp/rclcpp.hpp"
#include <cmath>
#include <cstddef>
#include <cstdlib>
#include <memory>
#include <thread>

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("simple_velocity_example_node");

  auto pub = node->create_publisher<panda_interfaces::msg::JointsCommandedVelocities>(
      "/panda/joint_velocity_command", panda_interface_names::DEFAULT_TOPIC_QOS());
      

  int iteration = 0;

  rclcpp::Rate loop_rate(0.2); // 0.1 Hz
  while (rclcpp::ok()) {
    auto msg = panda_interfaces::msg::JointsCommandedVelocities();
    if(iteration%2==0)
      msg.velocities = {0.0, 0.0, 0.0, 0.0, 0.0, 0.2, 0.0}; // Stop
    else
      msg.velocities = {0.0, 0.0, 0.0, 0.0, 0.0, -0.2, 0.0}; // Example velocities
    
    iteration++;
    pub->publish(msg);
    RCLCPP_INFO(node->get_logger(), "Published joint velocities command.");
    
    loop_rate.sleep();
  }
  rclcpp::spin_some(node);
  rclcpp::shutdown();
  return 0;
}