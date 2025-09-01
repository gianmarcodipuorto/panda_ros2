#include "panda_interfaces/srv/send_joints_pos_cmd.hpp"
#include "panda_interfaces/msg/joints_pos.hpp"
#include "panda_utils/constants.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"
#include <memory>
#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/service.hpp>
#include <rclcpp/timer.hpp>
#include <rclcpp/utilities.hpp>
#include <vector>

class SendJointsCmdPosServer : public rclcpp::Node {
public:
  SendJointsCmdPosServer() : Node("send_joints_cmd_pos") {

    for (const auto &name : panda_interface_names::bridge_pos_cmd_topic_names) {
      auto pub = this->create_publisher<std_msgs::msg::Float64>(
          name, panda_interface_names::DEFAULT_TOPIC_QOS());
      joints_cmd_pubs.push_back(pub);
    }

    // Create service for publication
    auto send_joints_pos = [this](panda_interfaces::msg::JointsPos mess) {
      for (size_t i = 0;
           i < std::min(mess.joint_values.size(), joints_cmd_pubs.size());
           ++i) {

        double joint_val = mess.joint_values[i];
        auto pub = this->joints_cmd_pubs[i];
        std_msgs::msg::Float64 pos;
        pos.data = joint_val;
        pub->publish(pos);
      }
    };

    joints_pos_sub =
        this->create_subscription<panda_interfaces::msg::JointsPos>(
            panda_interface_names::panda_pos_cmd_topic_name,
            panda_interface_names::DEFAULT_TOPIC_QOS(), send_joints_pos);

    RCLCPP_INFO_STREAM(this->get_logger(),
                       "Created subscriber to "
                           << panda_interface_names::panda_pos_cmd_topic_name);
  }

private:
  std::vector<rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr>
      joints_cmd_pubs;
  rclcpp::Subscription<panda_interfaces::msg::JointsPos>::SharedPtr
      joints_pos_sub;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SendJointsCmdPosServer>());
  rclcpp::shutdown();
  return 0;
}
