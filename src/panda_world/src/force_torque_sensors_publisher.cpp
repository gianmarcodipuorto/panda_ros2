#include "geometry_msgs/msg/wrench.hpp"
#include "geometry_msgs/msg/wrench_stamped.hpp"
#include "panda_interfaces/msg/wrench_array.hpp"
#include <rclcpp/create_timer.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/timer.hpp>
#include <string>
#include <vector>

using namespace std::chrono_literals;

class ForceTorqueSensorsPublisher : public rclcpp::Node {
public:
  ForceTorqueSensorsPublisher() : Node("force_torque_sensors_publisher") {

    wrench_array_pub =
        this->create_publisher<panda_interfaces::msg::WrenchArray>(
            force_torque_sensor_topic_name, 10);

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

    auto wrench_array_pub_cb = [this]() {
      panda_interfaces::msg::WrenchArray wrench_array;
      wrench_array.wrenches = wrenches;
      wrench_array_pub->publish(wrench_array);
    };

    timer =
        rclcpp::create_timer(this, this->get_clock(), 1ms, wrench_array_pub_cb);
    RCLCPP_INFO_STREAM(this->get_logger(),
                       "Started timer for node " << this->get_name());
  }

private:
  std::vector<
      rclcpp::Subscription<geometry_msgs::msg::WrenchStamped>::SharedPtr>
      gz_wrench_stamped_sub;
  rclcpp::Publisher<panda_interfaces::msg::WrenchArray>::SharedPtr
      wrench_array_pub;

  std::vector<geometry_msgs::msg::WrenchStamped> wrenches;
  rclcpp::TimerBase::SharedPtr timer;

  const std::vector<std::string> sensor_topic_names = {
      "/joint1/force_torque_sensor", "/joint2/force_torque_sensor",
      "/joint3/force_torque_sensor", "/joint4/force_torque_sensor",
      "/joint5/force_torque_sensor", "/joint6/force_torque_sensor",
      "/joint7/force_torque_sensor",
  };

  const std::string force_torque_sensor_topic_name{"/ft_sensors"};
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ForceTorqueSensorsPublisher>());
  rclcpp::shutdown();
  return 0;
}
