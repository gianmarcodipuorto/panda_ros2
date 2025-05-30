#include "panda_interfaces/msg/joints_effort.hpp"
#include "panda_utils/constants.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"
#include <memory>
#include <rclcpp/create_timer.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/service.hpp>
#include <rclcpp/subscription.hpp>
#include <rclcpp/timer.hpp>
#include <rclcpp/utilities.hpp>
#include <vector>

using namespace std::chrono_literals;

class SendJointsEffortCmd : public rclcpp::Node {
public:
  SendJointsEffortCmd() : Node("send_joints_cmd_effort") {

    for (const auto &name :
         panda_interface_names::bridge_effort_cmd_topic_names) {
      auto pub = this->create_publisher<std_msgs::msg::Float64>(
          name, panda_interface_names::DEFAULT_TOPIC_QOS);
      joints_effort_pubs.push_back(pub);
    }

    auto save_joints_effort = [this](panda_interfaces::msg::JointsEffort mess) {
      for (size_t i = 0;
           i < std::min(mess.effort_values.size(), joints_effort_pubs.size());
           ++i) {
        this->efforts[i] = mess.effort_values[i];
      }
    };

    auto send_joints_effort = [this]() {
      for (size_t i = 0; i < joints_effort_pubs.size(); ++i) {
        std_msgs::msg::Float64 effort;
        auto pub = joints_effort_pubs[i];
        effort.data = efforts[i];
        pub->publish(effort);
      }
      RCLCPP_INFO(this->get_logger(), "Sent effort command");
    };

    joints_effort_sub =
        this->create_subscription<panda_interfaces::msg::JointsEffort>(
            panda_interface_names::panda_effort_cmd_topic_name,
            panda_interface_names::DEFAULT_TOPIC_QOS, save_joints_effort);

    RCLCPP_DEBUG_STREAM(
        this->get_logger(),
        "Created subscriber to "
            << panda_interface_names::panda_effort_cmd_topic_name);

    timer = rclcpp::create_timer(this, this->get_clock(), 10ns, send_joints_effort);
  }

private:
  std::vector<rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr>
      joints_effort_pubs;
  rclcpp::Subscription<panda_interfaces::msg::JointsEffort>::SharedPtr
      joints_effort_sub;
  rclcpp::TimerBase::SharedPtr timer;
  double efforts[7]{};
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SendJointsEffortCmd>());
  rclcpp::shutdown();
  return 0;
}
