#include "ament_index_cpp/ament_index_cpp/get_package_share_directory.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "panda_interfaces/action/cart_traj.hpp"
#include "panda_interfaces/action/joint_traj.hpp"
#include "panda_utils/constants.hpp"
#include "panda_utils/robot.hpp"
#include "panda_utils/robot_model.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include <Eigen/src/Core/Matrix.h>
#include <memory>
#include <rclcpp/logging.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/wait_for_message.hpp>
#include <rclcpp_action/client.hpp>
#include <rclcpp_action/create_client.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

using panda_interfaces::action::CartTraj;
using panda_interfaces::action::JointTraj;
using sensor_msgs::msg::JointState;
using namespace std::chrono_literals;
using TrajMove = panda_interfaces::action::JointTraj;
using GoalHandleTrajMove = rclcpp_action::ClientGoalHandle<TrajMove>;

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  auto node = std::make_shared<rclcpp::Node>("task_node");

  auto cart_traj_action_client = rclcpp_action::create_client<CartTraj>(
      node, panda_interface_names::panda_cart_move_action_name);

  auto DEFAULT_URDF_PATH =
      ament_index_cpp::get_package_share_directory("panda_world") +
      panda_constants::panda_model_effort;
  panda::RobotModel panda{DEFAULT_URDF_PATH};
  geometry_msgs::msg::Pose initial_pose;
  geometry_msgs::msg::Pose desired_pose;
  double total_time = 10.0;
  const std::string frame_id_name{"fr3_hand_tcp"};

  RCLCPP_INFO(node->get_logger(), "Waiting for servers...");
  cart_traj_action_client->wait_for_action_server();
  RCLCPP_INFO(node->get_logger(), "Servers UP");

  // // Get initial pose
  {
    RCLCPP_INFO_STREAM(node->get_logger(),
                       "Getting current robot pose, press ENTER");
    std::cin.ignore();

    sensor_msgs::msg::JointState joints;
    RCLCPP_INFO_STREAM(node->get_logger(), "Acquiring joint states");
    rclcpp::wait_for_message<sensor_msgs::msg::JointState>(
        joints, node, panda_interface_names::joint_state_topic_name, 10s);

    Eigen::VectorXd joints_doub{};
    joints_doub.resize(7);
    for (size_t i = 0; i < 7; i++) {
      joints_doub[i] = joints.position[i];
    }
    panda.computeAll(joints_doub, joints_doub);
    initial_pose = panda.getPose(frame_id_name);
  }
  //
  // // Calling action server
  {
    RCLCPP_INFO_STREAM(node->get_logger(),
                       "Ready to call action server, press ENTER");
    std::cin.ignore();

    panda_interfaces::action::CartTraj_Goal cart_goal;
    cart_goal.initial_pose = initial_pose;
    geometry_msgs::msg::Pose desired_pose;
    desired_pose = initial_pose;
    desired_pose.position.x -= 0.1;
    desired_pose.position.y += 0.3;
    desired_pose.position.z += 0.5;
    cart_goal.desired_pose = desired_pose;
    cart_goal.total_time = total_time;

    // chiamo l'azione e aspetto che termini
    auto future_goal_handle =
        cart_traj_action_client->async_send_goal(cart_goal);
    if (rclcpp::spin_until_future_complete(node, future_goal_handle) !=
        rclcpp::FutureReturnCode::SUCCESS) {
      RCLCPP_ERROR_STREAM(node->get_logger(), "ERROR, GOAL NOT SENT");
      return -1;
    }
    auto future_result =
        cart_traj_action_client->async_get_result(future_goal_handle.get());
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
