#include "geometry_msgs/msg/pose.hpp"
#include "panda_interfaces/action/cart_traj.hpp"
#include "panda_interfaces/action/joint_traj.hpp"
#include "panda_utils/constants.hpp"
#include "panda_utils/robot.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
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

  // Initializing action clients
  auto joint_traj_action_client = rclcpp_action::create_client<JointTraj>(
      node, panda_interface_names::panda_traj_move_action_name);

  auto cart_traj_action_client = rclcpp_action::create_client<CartTraj>(
      node, panda_interface_names::panda_cart_move_action_name);

  Robot<7> panda{PANDA_DH_PARAMETERS, PANDA_JOINT_TYPES, UNIT_QUATERNION};
  geometry_msgs::msg::Pose initial_pose;
  geometry_msgs::msg::Pose desired_pose;
  double total_time = 10.0;

  RCLCPP_INFO(node->get_logger(), "Waiting for servers...");
  joint_traj_action_client->wait_for_action_server();
  RCLCPP_INFO(node->get_logger(), "Servers UP");

  {
    RCLCPP_INFO_STREAM(node->get_logger(), "Joint trajectory -- Press ENTER");
    std::cin.ignore();

    JointTraj::Goal joint_goal;
    joint_goal.total_time = total_time; // 3 seconds
    joint_goal.desired_joint_pos = {
        0.0,   0.0, 0.0, -3.0 / 4.0 * M_PI, 0.0, 3.0 / 4.0 * M_PI,
        M_PI_4}; // final desired config
    //
    auto goal_options = rclcpp_action::Client<JointTraj>::SendGoalOptions();
    auto feedback_joint_traj =
        [node](GoalHandleTrajMove::SharedPtr goal_handle,
               const std::shared_ptr<const TrajMove::Feedback> feedback) {
          RCLCPP_INFO_STREAM(node->get_logger(), "Still "
                                                     << feedback->time_left
                                                     << "s till termination.");
        };
    goal_options.feedback_callback = feedback_joint_traj;

    // Calling action
    auto future_goal_handle =
        joint_traj_action_client->async_send_goal(joint_goal, goal_options);
    if (rclcpp::spin_until_future_complete(node, future_goal_handle) !=
        rclcpp::FutureReturnCode::SUCCESS) {
      RCLCPP_ERROR_STREAM(node->get_logger(), "ERROR, GOAL NOT SENT");
      return -1;
    }

    // Action called, waiting for termination
    auto future_result =
        joint_traj_action_client->async_get_result(future_goal_handle.get());
    if (rclcpp::spin_until_future_complete(node, future_result) !=
        rclcpp::FutureReturnCode::SUCCESS) {
      RCLCPP_ERROR_STREAM(node->get_logger(), "ERROR, NO RESULT");
      return -1;
    }

    // se arrivo quì l'azione è terminata, controllo se è terminata con successo
    RCLCPP_INFO_STREAM(node->get_logger(),
                       "Action terminated with 'completed':"
                           << future_result.get().result->completed);

    // check dello stato dell'azione, se non ho errori lo stato deve essere
    // SUCCEEDED
    if (future_result.get().code != rclcpp_action::ResultCode::SUCCEEDED) {
      RCLCPP_ERROR_STREAM(node->get_logger(),
                          "ERROR, JOINT TRAJECTORY NOT SUCCEEDED");
      return -1;
    }
  }

  // // Get initial pose
  {
    RCLCPP_INFO_STREAM(node->get_logger(),
                       "Getting current robot pose, press ENTER");
    std::cin.ignore();

    sensor_msgs::msg::JointState joints;
    RCLCPP_INFO_STREAM(node->get_logger(), "Acquiring joint states");
    rclcpp::wait_for_message<sensor_msgs::msg::JointState>(
        joints, node, panda_interface_names::joint_state_topic_name, 10s);

    double joints_doub[7];
    for (size_t i = 0; i < 7; i++) {
      joints_doub[i] = joints.position[i];
    }
    initial_pose = panda.pose(joints_doub);
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
    desired_pose.position.z += 0.1;
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
