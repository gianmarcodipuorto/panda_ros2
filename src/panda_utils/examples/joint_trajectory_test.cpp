#include "panda_interfaces/msg/joints_command.hpp"
#include "panda_utils/constants.hpp"
#include <math.h>
#include <rclcpp/node.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/timer.hpp>
#include <rclcpp/utilities.hpp>

double qintic(double q_i, double q_f, double t, double t_f) {
  if (t <= 0)
    return q_i;
  if (t >= t_f)
    return q_f;

  double tau = t / t_f;
  double q_cap =
      6 * std::pow(tau, 5) - 15 * std::pow(tau, 4) + 10 * std::pow(tau, 3);
  return q_i + (q_f - q_i) * q_cap;
}

double qintic_velocity(double q_i, double q_f, double t, double t_f) {
  if (t <= 0)
    return 0;
  if (t >= t_f)
    return 0;

  double tau = t / t_f;
  double q_cap =
      30 * std::pow(tau, 4) - 60 * std::pow(tau, 3) + 30 * std::pow(tau, 2);
  return (q_f - q_i) * q_cap / t_f;
}

double qintic_accel(double q_i, double q_f, double t, double t_f) {
  if (t <= 0)
    return 0;
  if (t >= t_f)
    return 0;

  double tau = t / t_f;
  double q_cap = 120 * std::pow(tau, 3) - 180 * std::pow(tau, 2) + 60 * tau;
  return (q_f - q_i) * q_cap / std::pow(t_f, 2);
}

int main(int argc, char *argv[]) {

  rclcpp::init(argc, argv);
  auto node = rclcpp::Node("joint_traj_test_node");

  rclcpp::Publisher<panda_interfaces::msg::JointsCommand>::SharedPtr
      joint_cmd_pub =
          node.create_publisher<panda_interfaces::msg::JointsCommand>(
              "/test/joint_traj", panda_interface_names::DEFAULT_TOPIC_QOS);

  rclcpp::Time t0 = node.get_clock()->now();
  rclcpp::Duration t = rclcpp::Duration(0, 0);
  panda_interfaces::msg::JointsCommand initial_config;
  initial_config.positions = std::array<double, 7>{
      0.0, -M_PI_4, 0.0, -M_PI_2, 0.0, 3.0 * M_PI / 4.0, M_PI_4};

  panda_interfaces::msg::JointsCommand desired_config;
  desired_config.positions = std::array<double, 7>{
      0.0, 0.0, 0.0, -3.0 / 4.0 * M_PI, 0.0, 3.0 / 4.0 * M_PI, M_PI_4};

  double total_time = 5.0;
  rclcpp::Duration traj_duration = rclcpp::Duration(total_time, 0);
  rclcpp::Rate loop_rate(100.0);

  while (rclcpp::ok() && t < traj_duration) {
    t = node.get_clock()->now() - t0;

    // Get next JointState
    panda_interfaces::msg::JointsCommand cmd;

    for (size_t i = 0; i < 7; i++) {

      cmd.positions[i] =
          qintic(initial_config.positions[i], desired_config.positions[i],
                 t.seconds(), total_time);

      cmd.velocities[i] =
          qintic_velocity(initial_config.positions[i],
                          desired_config.positions[i], t.seconds(), total_time);

      cmd.accelerations[i] =
          qintic_accel(initial_config.positions[i], desired_config.positions[i],
                       t.seconds(), total_time);
    }

    cmd.header.stamp = node.now();

    joint_cmd_pub->publish(cmd);

    loop_rate.sleep();
  }

  std::cin.ignore();

  rclcpp::shutdown();
  return 0;
}
