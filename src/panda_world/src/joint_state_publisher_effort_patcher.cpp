#include "franka/robot_state.h"
#include "panda_interfaces/msg/joints_effort.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include <franka/robot.h>
#include <random>
#include <rclcpp/create_timer.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/timer.hpp>
#include <string>

using namespace std::chrono_literals;

double mean = 0.0;
double variance = 1.2e-4;
double std_dev = sqrt(variance);

class GaussianNoiseGenerator {
public:
  GaussianNoiseGenerator(double mean, double stddev)
      : generator(std::random_device{}()), distribution(mean, stddev) {}

  double generate() { return distribution(generator); }
  double operator()() { return generate(); }

private:
  std::mt19937 generator;

  std::normal_distribution<> distribution;
};

class JointStateEffortPatcher : public rclcpp::Node {
public:
  JointStateEffortPatcher()
      : Node("joint_state_effort_patcher"), noise_gen(mean, std_dev) {

    this->declare_parameter<std::string>("robot_ip", "192.168.1.0");
    this->declare_parameter<bool>("use_robot", false);

    use_robot = this->get_parameter("use_robot").as_bool();

    if (use_robot) {
      robot_ = franka::Robot(this->get_parameter("robot_ip").as_string());
    }

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

    auto publish_joint_states_robot = [this]() {
      try {
        franka::RobotState state = robot_.value().readOnce();
        auto msg = sensor_msgs::msg::JointState{};
        msg.position =
            std::vector<double>(std::begin(state.q), std::end(state.q));
        msg.velocity =
            std::vector<double>(std::begin(state.dq), std::end(state.dq));

        msg.effort =
            std::vector<double>(std::begin(state.tau_J), std::end(state.tau_J));
        joint_states_pub_->publish(msg);
      } catch (const std::exception &e) {
        RCLCPP_WARN(this->get_logger(), "Error reading state: %s", e.what());
      }
    };

    if (use_robot) {
      timer = rclcpp::create_timer(this, this->get_clock(), 10us,
                                   publish_joint_states_robot);
      RCLCPP_INFO_STREAM(this->get_logger(),
                         "Created node "
                             << this->get_name() << " with clock "
                             << (this->get_clock()->ros_time_is_active()
                                     ? "simulation clock"
                                     : "system clock")
                             << " using real panda robot");
    } else {
      timer = rclcpp::create_timer(this, this->get_clock(), 10us,
                                   publish_joint_states);
      RCLCPP_INFO_STREAM(this->get_logger(),
                         "Created node "
                             << this->get_name() << " with clock "
                             << (this->get_clock()->ros_time_is_active()
                                     ? "simulation clock"
                                     : "system clock"));
    }
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

    // Add noise
    for (size_t i = 0; i < fixed_msg.velocity.size(); i++) {
      fixed_msg.velocity[i] += noise_gen.generate();
    }

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

  GaussianNoiseGenerator noise_gen;

  bool use_robot;
  std::optional<franka::Robot> robot_{};
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<JointStateEffortPatcher>());
  rclcpp::shutdown();
  return 0;
}
