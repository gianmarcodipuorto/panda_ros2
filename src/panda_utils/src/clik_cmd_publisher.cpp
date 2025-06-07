#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "panda_interfaces/action/cart_traj.hpp"
#include "panda_interfaces/msg/joints_command.hpp"
#include "panda_interfaces/msg/joints_pos.hpp"
#include "panda_utils/constants.hpp"
#include "panda_utils/robot.hpp"
#include "panda_utils/robot_model.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/bool.hpp"
#include <Eigen/src/Core/Matrix.h>
#include <Eigen/src/Geometry/Quaternion.h>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <chrono>
#include <cmath>
#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/rate.hpp>
#include <rclcpp/timer.hpp>
#include <rclcpp/utilities.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <rclcpp_action/server_goal_handle.hpp>
#include <rclcpp_lifecycle/rclcpp_lifecycle/lifecycle_node.hpp>
#include <std_msgs/msg/bool.hpp>
#include <string>
#include <tf2_eigen/tf2_eigen.hpp>
#include <thread>

using CartTraj = panda_interfaces::action::CartTraj;
using GoalHandleCart = rclcpp_action::ServerGoalHandle<CartTraj>;
using namespace std::chrono_literals;
using Pose = geometry_msgs::msg::Pose;

auto DEFAULT_URDF_PATH =
    ament_index_cpp::get_package_share_directory("panda_world") +
    panda_constants::panda_model_effort;

class CLIKPublisher : public rclcpp_lifecycle::LifecycleNode {

public:
  CLIKPublisher(const std::string urdf_robot_path = DEFAULT_URDF_PATH)
      : rclcpp_lifecycle::LifecycleNode(panda_interface_names::clik_node_name),
        panda(urdf_robot_path, true) {

    this->declare_parameter<double>("ts", 0.01);
    this->declare_parameter<double>("gamma", 0.5);

    auto set_pose_cb = [this](const Pose msg) {
      desired_pose.position = msg.position;
      Eigen::Quaterniond quaternion_des{msg.orientation.w, msg.orientation.x,
                                        msg.orientation.y, msg.orientation.z};
      quaternion_des = quaternionContinuity(quaternion_des, old_quaternion);
      desired_pose.orientation.x = quaternion_des.x();
      desired_pose.orientation.y = quaternion_des.y();
      desired_pose.orientation.z = quaternion_des.z();
    };

    auto set_twist_cb = [this](const geometry_msgs::msg::Twist msg) {
      desired_twist[0] = msg.linear.x;
      desired_twist[1] = msg.linear.y;
      desired_twist[2] = msg.linear.z;
      desired_twist[3] = msg.angular.x;
      desired_twist[4] = msg.angular.y;
      desired_twist[5] = msg.angular.z;
    };

    desired_pose_sub = this->create_subscription<Pose>(
        panda_interface_names::panda_pose_cmd_topic_name,
        panda_interface_names::DEFAULT_TOPIC_QOS, set_pose_cb);

    desired_twist_sub = this->create_subscription<geometry_msgs::msg::Twist>(
        panda_interface_names::panda_speed_cmd_topic_name,
        panda_interface_names::DEFAULT_TOPIC_QOS, set_twist_cb);

    auto control_cycle = [this]() {
      clik_one_step();

      panda_interfaces::msg::JointsCommand cmd;
      for (size_t i = 0; i < 7; i++) {
        cmd.positions[i] = clik_joint_state[i];
      }
      cmd_pub->publish(cmd);

      if (!rclcpp::ok()) {
        RCLCPP_INFO(this->get_logger(), "Requested shutdown");
        rclcpp::shutdown();
      }
    };
    timer = rclcpp::create_timer(this, this->get_clock(),
                                 std::chrono::milliseconds((int)(ts / 1000.0)),
                                 control_cycle);
    timer->cancel();
    cmd_pub = this->create_publisher<panda_interfaces::msg::JointsCommand>(
        panda_interface_names::panda_joint_cmd_topic_name,
        panda_interface_names::DEFAULT_TOPIC_QOS);

    RCLCPP_INFO_STREAM(this->get_logger(), "Clik publisher ready with ts: "
                                               << ts
                                               << ", gamma: " << gamma * ts);
  }

  CallbackReturn on_configure(const rclcpp_lifecycle::State &) override {
    RCLCPP_INFO(get_logger(), "Configuring...");
    ts = this->get_parameter("ts").as_double();
    gamma = this->get_parameter("gamma").as_double() / ts;

    auto set_joint_state =
        [this](const sensor_msgs::msg::JointState::SharedPtr msg) {
          joint_state = msg;
        };

    joint_state_sub = this->create_subscription<sensor_msgs::msg::JointState>(
        panda_interface_names::joint_state_topic_name,
        panda_interface_names::DEFAULT_TOPIC_QOS, set_joint_state);

    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_activate(const rclcpp_lifecycle::State &) override {
    RCLCPP_INFO(get_logger(), "Activating...");
    auto current_config_thread = [this]() {
      RCLCPP_INFO(this->get_logger(),
                  "Assigning desired pose to the current one");
      joint_state = nullptr;
      while (!joint_state) {
        if (rclcpp::ok()) {
          std::this_thread::sleep_for(500ns);
        } else {
          rclcpp::shutdown();
        }
      }
      clik_joint_state.resize(panda.getModel().nq);
      for (size_t i = 0; i < 7; i++) {
        clik_joint_state[i] = joint_state->position[i];
      }
      panda.computeForwardKinematics(clik_joint_state);
      desired_pose = panda.getPose(frame_id_name);
      this->joint_state_sub = nullptr;
      RCLCPP_INFO_STREAM(this->get_logger(),
                         "Clik started with ts = " << ts
                                                   << " and gamma = " << gamma);
      timer->reset();
    };
    std::thread{current_config_thread}.detach();
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_deactivate(const rclcpp_lifecycle::State &) override {
    RCLCPP_INFO(get_logger(), "Deactivating...");
    RCLCPP_INFO(this->get_logger(), "Clik stopped");
    timer->cancel();
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_cleanup(const rclcpp_lifecycle::State &) override {
    RCLCPP_INFO(get_logger(), "Cleaning up...");
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_shutdown(const rclcpp_lifecycle::State &) override {
    RCLCPP_INFO(get_logger(), "Shutting down...");

    return CallbackReturn::SUCCESS;
  }

private:
  // Gets:
  // - Joints state
  // - Desired reachable pose
  // - Start & stop command for publishing
  //
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub;
  rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr desired_pose_sub;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr desired_twist_sub;

  // Sends:
  // - Desired joints position
  rclcpp::Publisher<panda_interfaces::msg::JointsCommand>::SharedPtr cmd_pub;

  sensor_msgs::msg::JointState::SharedPtr joint_state{};
  Eigen::VectorXd clik_joint_state{};

  panda::RobotModel panda;
  rclcpp::TimerBase::SharedPtr timer;
  Pose desired_pose;
  Vector6 desired_twist{};
  Eigen::Quaterniond old_quaternion{};
  double ts = 0.01;
  double gamma = 0.5 / ts;
  const std::string frame_id_name{"fr3_hand_tcp"};

  void clik_one_step();
};

void CLIKPublisher::clik_one_step() {
  using geometry_msgs::msg::Pose;

  Eigen::Quaterniond current_quat{};
  Eigen::Vector<double, 6> error;
  Eigen::Vector3d error_quat{};
  Eigen::Quaterniond final_quat{
      desired_pose.orientation.w, desired_pose.orientation.x,
      desired_pose.orientation.y, desired_pose.orientation.z};
  final_quat.normalize();

  // Get current infos
  Pose current_pose = panda.getPose(frame_id_name);

  Eigen::MatrixXd jacobian = panda.getGeometricalJacobian(frame_id_name);
  auto jacob_pinv = jacobian.completeOrthogonalDecomposition();
  current_quat.w() = current_pose.orientation.w;
  current_quat.x() = current_pose.orientation.x;
  current_quat.y() = current_pose.orientation.y;
  current_quat.z() = current_pose.orientation.z;
  current_quat.normalize();

  current_quat = quaternionContinuity(current_quat, this->old_quaternion);
  this->old_quaternion = current_quat;

  // Construct errors
  error[0] = desired_pose.position.x - current_pose.position.x;
  error[1] = desired_pose.position.y - current_pose.position.y;
  error[2] = desired_pose.position.z - current_pose.position.z;
  error_quat = (final_quat * current_quat.inverse()).vec();
  error.block<3, 1>(3, 0) = error_quat;

  RCLCPP_INFO_STREAM(this->get_logger(),
                     "Error: [" << error[0] << ", " << error[1] << ", "
                                << error[2] << ", " << error[3] << ", "
                                << error[4] << ", " << error[5] << "]"
                                << std::endl);

  Eigen::Vector<double, 6> q_dot = desired_twist + gamma * error;

  // Run algorithm
  clik_joint_state = clik_joint_state + ts * jacob_pinv.solve(q_dot);
}

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<CLIKPublisher>();
  rclcpp::spin(node->get_node_base_interface());
  rclcpp::shutdown();
  return 0;
}
