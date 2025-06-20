#include "ament_index_cpp/ament_index_cpp/get_package_share_directory.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "panda_interfaces/action/cart_traj.hpp"
#include "panda_interfaces/action/joint_traj.hpp"
#include "panda_interfaces/msg/joints_command.hpp"
#include "panda_utils/constants.hpp"
#include "panda_utils/robot.hpp"
#include "panda_utils/robot_model.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include <Eigen/src/Core/Matrix.h>
#include <array>
#include <cmath>
#include <cstdlib>
#include <memory>
#include <rclcpp/logging.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/utilities.hpp>
#include <rclcpp/wait_for_message.hpp>
#include <rclcpp_action/client.hpp>
#include <rclcpp_action/create_client.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

using panda_interfaces::action::JointTraj;
using sensor_msgs::msg::JointState;
using namespace std::chrono_literals;
using GoalHandleJointTraj = rclcpp_action::ClientGoalHandle<JointTraj>;

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  double total_time = 5.0;
  if (argc >= 2) {
    total_time = atoi(argv[1]);
  }

  auto node = std::make_shared<rclcpp::Node>("joint_traj_example_node");

  auto joint_cmd_pub =
      node->create_publisher<panda_interfaces::msg::JointsCommand>(
          panda_interface_names::panda_joint_cmd_topic_name,
          panda_interface_names::DEFAULT_TOPIC_QOS);

  auto joint_traj_action_client = rclcpp_action::create_client<JointTraj>(
      node, panda_interface_names::panda_traj_move_action_name);

  auto DEFAULT_URDF_PATH =
      ament_index_cpp::get_package_share_directory("panda_world") +
      panda_constants::panda_model_effort_no_table;
  Robot<7> panda_mine(PANDA_DH_PARAMETERS, PANDA_JOINT_TYPES,
                      OrientationConfiguration::UNIT_QUATERNION);

  panda_interfaces::msg::JointsCommand initial_config;
  initial_config.positions = std::array<double, 7>{
      0.0, -M_PI_4, 0.0, -M_PI_2, 0.0, 3.0 * M_PI / 4.0, M_PI_4};

  panda_interfaces::msg::JointsCommand desired_config;
  desired_config.positions = std::array<double, 7>{
      0.0, 0.0, 0.0, -3.0 / 4.0 * M_PI, 0.0, 3.0 / 4.0 * M_PI, M_PI_4};
  desired_config.positions =
      std::array<double, 7>{M_PI_2, -M_PI_4,           M_PI_4, -3.0 / 4.0 * M_PI,
                            M_PI_2, 3.0 / 4.0 * M_PI, M_PI_4};
  // desired_config.positions =
  //     std::array<double, 7>{0.0, 0.0, 0.0, -3.0 / 4.0 * M_PI, 0.0, 0.0, 0.0};

  RCLCPP_INFO(node->get_logger(), "Waiting for servers...");
  joint_traj_action_client->wait_for_action_server();
  RCLCPP_INFO(node->get_logger(), "Servers UP");

  // Send robot initial config
  // {
  //   RCLCPP_INFO_STREAM(node->get_logger(),
  //                      "Sending initial joint configuration to robot: ["
  //                          << initial_config.positions[0] << ", "
  //                          << initial_config.positions[1] << ", "
  //                          << initial_config.positions[2] << ", "
  //                          << initial_config.positions[3] << ", "
  //                          << initial_config.positions[4] << ", "
  //                          << initial_config.positions[5] << ", "
  //                          << initial_config.positions[6] << ", "
  //                          << "], press ENTER");
  //   std::cin.ignore();
  //
  //   initial_config.header.stamp = node->now();
  //   joint_cmd_pub->publish(initial_config);
  // }

  // Calling action server
  {
    RCLCPP_INFO_STREAM(
        node->get_logger(),
        "Ready to call action server with desired configuration: ["
            << desired_config.positions[0] << ", "
            << desired_config.positions[1] << ", "
            << desired_config.positions[2] << ", "
            << desired_config.positions[3] << ", "
            << desired_config.positions[4] << ", "
            << desired_config.positions[5] << ", "
            << desired_config.positions[6] << ", "
            << "], press ENTER");
    std::cin.ignore();

    panda_interfaces::action::JointTraj_Goal joint_traj_goal;

    joint_traj_goal.desired_joint_cmd = desired_config;
    joint_traj_goal.total_time = total_time;

    rclcpp_action::Client<JointTraj>::SendGoalOptions goal_options;
    goal_options.feedback_callback =
        [node](GoalHandleJointTraj::SharedPtr,
               const std::shared_ptr<
                   const panda_interfaces::action::JointTraj_Feedback>
                   feedback) {
          RCLCPP_INFO_STREAM(node->get_logger(),
                             "Time left: " << feedback->time_left << "[s].");
        };
    // chiamo l'azione e aspetto che termini
    auto future_goal_handle = joint_traj_action_client->async_send_goal(
        joint_traj_goal, goal_options);
    if (rclcpp::spin_until_future_complete(node, future_goal_handle) !=
        rclcpp::FutureReturnCode::SUCCESS) {
      RCLCPP_ERROR_STREAM(node->get_logger(), "ERROR, GOAL NOT SENT");
      return -1;
    }
    auto future_result =
        joint_traj_action_client->async_get_result(future_goal_handle.get());
    if (rclcpp::spin_until_future_complete(node, future_result) !=
        rclcpp::FutureReturnCode::SUCCESS) {
      RCLCPP_ERROR_STREAM(node->get_logger(), "ERROR, NO RESULT");
      return -1;
    }

    // check dello stato dell'azione, se non ho errori lo stato deve essere
    // SUCCEEDED
    if (future_result.get().code != rclcpp_action::ResultCode::SUCCEEDED) {
      RCLCPP_ERROR_STREAM(node->get_logger(),
                          "ERROR, CART TRAJECTORY NOT SUCCEEDED");
      return -1;
    }
  }
}
