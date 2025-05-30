#include "geometry_msgs/msg/pose.hpp"
#include "panda_interfaces/action/cart_traj.hpp"
#include "panda_interfaces/msg/joints_pos.hpp"
#include "panda_utils/constants.hpp"
#include "panda_utils/robot.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/bool.hpp"
#include <Eigen/src/Core/Matrix.h>
#include <Eigen/src/Geometry/Quaternion.h>
#include <chrono>
#include <random>
#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/rate.hpp>
#include <rclcpp/timer.hpp>
#include <rclcpp/utilities.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <rclcpp_action/server_goal_handle.hpp>
#include <std_msgs/msg/bool.hpp>
#include <string>
#include <tf2_eigen/tf2_eigen.hpp>

using CartTraj = panda_interfaces::action::CartTraj;
using GoalHandleCart = rclcpp_action::ServerGoalHandle<CartTraj>;
using namespace std::chrono_literals;
using Pose = geometry_msgs::msg::Pose;

class CLIKPublisher : public rclcpp::Node {

public:
  CLIKPublisher()
      : rclcpp::Node("clik_cmd_publisher_node"),
        panda(PANDA_DH_PARAMETERS, PANDA_JOINT_TYPES, UNIT_QUATERNION) {

    auto set_pose_cb = [this](const Pose msg) {
      desired_pose.position = msg.position;
      Eigen::Quaterniond quaternion_des{msg.orientation.w, msg.orientation.x,
                                        msg.orientation.y, msg.orientation.z};
      quaternion_des = quaternionContinuity(quaternion_des, old_quaternion);
      desired_pose.orientation.x = quaternion_des.x();
      desired_pose.orientation.y = quaternion_des.y();
      desired_pose.orientation.z = quaternion_des.z();
    };

    desired_pose_sub = this->create_subscription<Pose>(
        panda_interface_names::panda_pose_cmd_topic_name,
        panda_interface_names::DEFAULT_TOPIC_QOS, set_pose_cb);

    auto start_stop_clik = [this](const std_msgs::msg::Bool start) {
      if (!start.data) {
        RCLCPP_INFO(this->get_logger(), "Clik stopped");
        timer->cancel();
      } else {
        RCLCPP_INFO(this->get_logger(),
                    "Assigning desired pose to the current one");
        double joints[7];
        for (size_t i = 0; i < 7; i++) {
          joints[i] = joint_state.position[i];
        }
        desired_pose = panda.pose(joints);
        this->joint_state_sub = nullptr;
        RCLCPP_INFO(this->get_logger(), "Clik started");
        timer->reset();
      }
    };

    start_clik_sub = this->create_subscription<std_msgs::msg::Bool>(
        panda_interface_names::start_and_stop_clik_topic_name,
        panda_interface_names::DEFAULT_TOPIC_QOS, start_stop_clik);

    auto set_joint_state = [this](const sensor_msgs::msg::JointState msg) {
      joint_state = msg;
    };

    joint_state_sub = this->create_subscription<sensor_msgs::msg::JointState>(
        panda_interface_names::joint_state_topic_name,
        panda_interface_names::DEFAULT_TOPIC_QOS, set_joint_state);

    auto control_cycle = [this]() {
      Eigen::Vector<double, 7> joints;
      for (size_t i = 0; i < 7; i++) {
        joints[i] = joint_state.position[i];
      }

      panda.clik_one_step(desired_pose, joints, ts, gamma,
                          Eigen::Vector<double, 6>::Zero());
      RCLCPP_INFO(this->get_logger(), "CLIK step done");
      // sensor_msgs::msg::JointState cmd;
      // cmd.position.resize(7);
      // cmd.name = joint_names_;
      // cmd.header.stamp = this->now();

      panda_interfaces::msg::JointsPos cmd;
      RCLCPP_INFO(this->get_logger(), "Assigning joint values to cmd");
      for (size_t i = 0; i < 7; i++) {
        // cmd.position[i] = joints[i];
        cmd.joint_values[i] = joints[i];
      }
      cmd_pos_pub->publish(cmd);
      if (!rclcpp::ok()) {
        RCLCPP_INFO(this->get_logger(), "Requested shutdown");
        rclcpp::shutdown();
      }
    };
    timer = this->create_wall_timer(
        std::chrono::milliseconds((int)(ts / 1000.0)), control_cycle);
    timer->cancel();
    cmd_pos_pub = this->create_publisher<panda_interfaces::msg::JointsPos>(
        panda_interface_names::panda_pos_cmd_topic_name,
        panda_interface_names::DEFAULT_TOPIC_QOS);
    RCLCPP_INFO_STREAM(this->get_logger(), "Clik publisher ready with ts: "
                                               << ts
                                               << ", gamma: " << gamma * ts);
  }

private:
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub;
  rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr desired_pose_sub;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr start_clik_sub;
  rclcpp::Publisher<panda_interfaces::msg::JointsPos>::SharedPtr cmd_pos_pub;
  sensor_msgs::msg::JointState joint_state{};
  Robot<7> panda;
  rclcpp::TimerBase::SharedPtr timer;
  Pose desired_pose;
  Eigen::Quaterniond old_quaternion{};
  double ts = 0.01;
  double gamma = 0.5 / ts;
  std::vector<std::string> joint_names_ = {
      "panda_joint1", "panda_joint2", "panda_joint3", "panda_joint4",
      "panda_joint5", "panda_joint6", "panda_joint7"};
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<CLIKPublisher>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
