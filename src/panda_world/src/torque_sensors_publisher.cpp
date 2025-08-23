#include "geometry_msgs/msg/wrench.hpp"
#include "geometry_msgs/msg/wrench_stamped.hpp"
#include "panda_interfaces/msg/joint_torque_measure_stamped.hpp"
#include "panda_interfaces/msg/joints_effort.hpp"
#include "panda_interfaces/msg/wrench_array.hpp"
#include <rclcpp/create_timer.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/timer.hpp>
#include <string>
#include <vector>

using namespace std::chrono_literals;

class TorqueSensorsPublisher : public rclcpp::Node {
public:
  TorqueSensorsPublisher() : Node("torque_sensors_publisher") {

    torque_measures_pub = this->create_publisher<
        panda_interfaces::msg::JointTorqueMeasureStamped>(
        torque_sensor_topic_name, 10);

    wrenches.resize(sensor_topic_names.size());
    gz_wrench_stamped_sub.resize(sensor_topic_names.size());

    for (size_t i = 0; i < sensor_topic_names.size(); i++) {
      auto wrench_sub_cb = [this,
                            i](const geometry_msgs::msg::WrenchStamped msg) {
        this->wrenches[i] = msg;
      };

      gz_wrench_stamped_sub[i] =
          this->create_subscription<geometry_msgs::msg::WrenchStamped>(
              sensor_topic_names[i], 10, wrench_sub_cb);
    }

    auto joints_effort_cb =
        [this](const panda_interfaces::msg::JointsEffort msg) {
          joints_effort = msg;
        };

    joints_effort_sub =
        this->create_subscription<panda_interfaces::msg::JointsEffort>(
            panda_effort_cmd_topic_name, 10, joints_effort_cb);

    auto wrench_array_pub_cb = [this]() {
      panda_interfaces::msg::JointTorqueMeasureStamped torque_stamped;
      for (size_t i = 0; i < torque_stamped.measures.torque.size(); i++) {
        torque_stamped.measures.torque[i] =
            wrenches[i].wrench.torque.z - joints_effort.effort_values[i];
      }
      torque_stamped.header.stamp = this->get_clock()->now();
      this->torque_measures_pub->publish(torque_stamped);
    };

    timer =
        rclcpp::create_timer(this, this->get_clock(), 1ms, wrench_array_pub_cb);
    RCLCPP_INFO_STREAM(this->get_logger(),
                       "Started timer for node " << this->get_name());
  }

private:
  rclcpp::Publisher<panda_interfaces::msg::JointTorqueMeasureStamped>::SharedPtr
      torque_measures_pub;

  std::vector<
      rclcpp::Subscription<geometry_msgs::msg::WrenchStamped>::SharedPtr>
      gz_wrench_stamped_sub;

  rclcpp::Subscription<panda_interfaces::msg::JointsEffort>::SharedPtr
      joints_effort_sub;

  std::vector<geometry_msgs::msg::WrenchStamped> wrenches;
  panda_interfaces::msg::JointsEffort joints_effort;
  rclcpp::TimerBase::SharedPtr timer;

  const std::vector<std::string> sensor_topic_names = {
      "/joint1/force_torque_sensor", "/joint2/force_torque_sensor",
      "/joint3/force_torque_sensor", "/joint4/force_torque_sensor",
      "/joint5/force_torque_sensor", "/joint6/force_torque_sensor",
      "/joint7/force_torque_sensor",
  };

  const std::string torque_sensor_topic_name{"/tau_sensors"};
  const std::string panda_effort_cmd_topic_name{"/panda/cmd/effort"};
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TorqueSensorsPublisher>());
  rclcpp::shutdown();
  return 0;
}
