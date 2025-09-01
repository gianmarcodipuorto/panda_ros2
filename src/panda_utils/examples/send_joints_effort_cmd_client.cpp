#include "panda_interfaces/msg/joints_effort.hpp"
#include "panda_interfaces/msg/joints_pos.hpp"
#include "panda_interfaces/srv/send_joints_pos_cmd.hpp"
#include "panda_utils/constants.hpp"
#include "std_msgs/msg/float64.hpp"
#include <chrono>
#include <cstddef>
#include <cstdlib>
#include <memory>
#include <rclcpp/executors.hpp>
#include <rclcpp/future_return_code.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/service.hpp>
#include <rclcpp/subscription_base.hpp>
#include <rclcpp/timer.hpp>
#include <rclcpp/utilities.hpp>
#include <string>
#include <vector>

using namespace std::chrono_literals;

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  if (argc < 8) {
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"),
                 "Need all 7 joints values to send");
  }

  using panda_interfaces::msg::JointsEffort;

  std::shared_ptr<rclcpp::Node> node =
      rclcpp::Node::make_shared("client_joints_cmd_effort");
  std::shared_ptr<rclcpp::Publisher<panda_interfaces::msg::JointsEffort>> pub =
      node->create_publisher<panda_interfaces::msg::JointsEffort>(
          panda_interface_names::panda_effort_cmd_topic_name,
          panda_interface_names::DEFAULT_TOPIC_QOS());

  JointsEffort mess;

  for (std::size_t i = 0; i < mess.effort_values.size(); i++) {
    RCLCPP_INFO(node->get_logger(), "Argument %zu is %s", i + 1, argv[i + 1]);
    mess.effort_values[i] = atof(argv[i + 1]);
  }

  pub->publish(mess);

  rclcpp::shutdown();
  return 0;
}
