#include "panda_interfaces/msg/joints_effort.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include <rclcpp/create_timer.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/timer.hpp>
#include <string>

using namespace std::chrono_literals;

class JointStateEffortPatcher : public rclcpp::Node {
public:
  JointStateEffortPatcher() : Node("joint_state_effort_patcher") {
    effort_sub_ =
        this->create_subscription<panda_interfaces::msg::JointsEffort>(
            "/panda/cmd/effort", 10,
            std::bind(&JointStateEffortPatcher::effort_cmd_callback, this,
                      std::placeholders::_1));

    joint_states_pub_ = this->create_publisher<sensor_msgs::msg::JointState>(
        "/joint_states", 10);

    joint_states_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
        "/joint_states_no_effort", 10,
        std::bind(&JointStateEffortPatcher::joint_states_callback, this,
                  std::placeholders::_1));
    auto publish_joint_states = [this]() {
      joint_states_pub_->publish(this->last_mess);
    };
    timer = rclcpp::create_timer(this, this->get_clock(), 1ms, publish_joint_states);
  }

private:
  void effort_cmd_callback(const panda_interfaces::msg::JointsEffort msg) {
    for (size_t i = 0; i < msg.effort_values.size(); ++i) {
      this->efforts[i] = msg.effort_values[i];
    }
    RCLCPP_DEBUG_STREAM(this->get_logger(),
                        "Efforts: " << efforts[0] << ", " << efforts[1] << ", "
                                    << efforts[2] << ", " << efforts[3] << ", "
                                    << efforts[4] << ", " << efforts[5] << ", "
                                    << efforts[6]);
  }

  void
  joint_states_callback(const sensor_msgs::msg::JointState::SharedPtr msg) {
    // Costruisce un nuovo messaggio copiando posizione e velocità
    auto fixed_msg = sensor_msgs::msg::JointState();
    fixed_msg.header = msg->header;
    fixed_msg.name = msg->name;
    fixed_msg.position = msg->position;
    fixed_msg.velocity = msg->velocity;
    fixed_msg.effort.resize(msg->name.size());

    for (size_t i = 0; i < msg->name.size(); ++i) {
      fixed_msg.effort[i] = this->efforts[i];
    }

    last_mess = fixed_msg;
    joint_states_pub_->publish(fixed_msg);
  }

  rclcpp::Subscription<panda_interfaces::msg::JointsEffort>::SharedPtr
      effort_sub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr
      joint_states_sub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_states_pub_;
  rclcpp::TimerBase::SharedPtr timer;
  double efforts[7]{};
  sensor_msgs::msg::JointState last_mess{};
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<JointStateEffortPatcher>());
  rclcpp::shutdown();
  return 0;
}
