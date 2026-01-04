#include "ament_index_cpp/ament_index_cpp/get_package_share_directory.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "panda_interfaces/action/cart_traj.hpp"
#include "panda_interfaces/action/joint_traj.hpp"
#include "panda_interfaces/msg/joints_command.hpp"
#include "panda_utils/constants.hpp"
#include "panda_utils/robot_model.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include <Eigen/src/Core/Matrix.h>
#include <algorithm>
#include <cstdlib>
#include <memory>
#include <rclcpp/executor.hpp>
#include <rclcpp/executors.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/utilities.hpp>
#include <rclcpp/wait_for_message.hpp>
#include <rclcpp_action/client.hpp>
#include <rclcpp_action/create_client.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <string>
#include <thread>
#include <unsupported/Eigen/CXX11/src/Tensor/TensorFFT.h>
#include <vector>

using panda_interfaces::action::CartTraj;
using panda_interfaces::action::JointTraj;
using sensor_msgs::msg::JointState;
using namespace std::chrono_literals;

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("cart_traj_example_node");
  geometry_msgs::msg::Pose delta_pose;
  geometry_msgs::msg::Pose desired_pose;
  delta_pose.orientation.w = 0.0;
  double total_time = 5.0;
  std::vector<std::string> modes{"delta", "pose"};
  std::string mode;

  switch (argc) {

  case 6: {
    auto val = std::find(modes.begin(), modes.end(), std::string{argv[2]});
    if (val == modes.end()) {
      RCLCPP_INFO_STREAM(node->get_logger(),
                         "Mode " << argv[2] << " is not valid");
      rclcpp::shutdown();
      return 0;
    }

    mode = *val;
    if (mode == std::string{"delta"}) {
      total_time = atoi(argv[1]);
      delta_pose.position.x = atof(argv[3]);
      delta_pose.position.y = atof(argv[4]);
      delta_pose.position.z = atof(argv[5]);
      RCLCPP_INFO_STREAM(
          node->get_logger(),
          "Choosing time ("
              << total_time
              << ") and desired pose with delta on initial pose (only "
                 "translation) for cartesian trajectory; Delta pose position: ["
              << delta_pose.position.x << ", " << delta_pose.position.y << ", "
              << delta_pose.position.z << "]");
    } else {
      total_time = atoi(argv[1]);
      desired_pose.position.x = atof(argv[3]);
      desired_pose.position.y = atof(argv[4]);
      desired_pose.position.z = atof(argv[5]);
      RCLCPP_INFO_STREAM(
          node->get_logger(),
          "Choosing time ("
              << total_time
              << ") and desired pose (only "
                 "translation) for cartesian trajectory; position: ["
              << desired_pose.position.x << ", " << desired_pose.position.y
              << ", " << desired_pose.position.z << "]");
    }
    break;
  }

  case 10: {
    auto val = std::find(modes.begin(), modes.end(), std::string{argv[2]});
    if (val == modes.end()) {
      RCLCPP_INFO_STREAM(node->get_logger(),
                         "Mode " << argv[2] << " is not valid");
      rclcpp::shutdown();
      return 0;
    }

    mode = *val;
    if (mode == std::string{"delta"}) {
      total_time = atoi(argv[1]);

      delta_pose.position.x = atof(argv[3]);
      delta_pose.position.y = atof(argv[4]);
      delta_pose.position.z = atof(argv[5]);

      delta_pose.orientation.w = atof(argv[6]);
      delta_pose.orientation.x = atof(argv[7]);
      delta_pose.orientation.y = atof(argv[8]);
      delta_pose.orientation.z = atof(argv[9]);
      RCLCPP_INFO_STREAM(
          node->get_logger(),
          "Choosing time ("
              << total_time
              << ") and desired pose with delta on initial pose for cartesian "
                 "trajectory; Delta pose position: ["
              << delta_pose.position.x << ", " << delta_pose.position.y << ", "
              << delta_pose.position.z << "]. Delta pose orientaion: ["
              << delta_pose.orientation.w << ", " << delta_pose.orientation.x
              << ", " << delta_pose.orientation.y << ", "
              << delta_pose.orientation.z << "].");
    } else {
      total_time = atoi(argv[1]);

      desired_pose.position.x = atof(argv[3]);
      desired_pose.position.y = atof(argv[4]);
      desired_pose.position.z = atof(argv[5]);

      desired_pose.orientation.w = atof(argv[6]);
      desired_pose.orientation.x = atof(argv[7]);
      desired_pose.orientation.y = atof(argv[8]);
      desired_pose.orientation.z = atof(argv[9]);
      RCLCPP_INFO_STREAM(
          node->get_logger(),
          "Choosing time ("
              << total_time
              << ") and desired pose for cartesian trajectory; pose position: ["
              << desired_pose.position.x << ", " << desired_pose.position.y
              << ", " << desired_pose.position.z << "]. pose orientaion: ["
              << desired_pose.orientation.w << ", "
              << desired_pose.orientation.x << ", "
              << desired_pose.orientation.y << ", "
              << desired_pose.orientation.z << "].");
    }
    break;
  }
  default: {

    RCLCPP_INFO_STREAM(node->get_logger(),
                       "Usage: <desired_time> 'delta' <delta_x> <delta_y> "
                       "<delta_z> <delta_w> <delta_x> <delta_y> <delta_z>");
    RCLCPP_INFO_STREAM(node->get_logger(),
                       "Usage: <desired_time> 'pose' <x> <y> "
                       "<z> <w> <x> <y> <z>");
    RCLCPP_ERROR(node->get_logger(),
                 "Wrong number of arguments, shutting down");
    rclcpp::shutdown();
    return 0;
  }
  }

  auto cart_traj_action_client = rclcpp_action::create_client<CartTraj>(
      node, panda_interface_names::panda_cart_move_action_name);

  auto sub_node = std::make_shared<rclcpp::Node>("sub_node");
  geometry_msgs::msg::PoseStamped::SharedPtr initial_pose;
  auto pose_sub =
      sub_node->create_subscription<geometry_msgs::msg::PoseStamped>(
          panda_interface_names::panda_pose_state_topic_name,
          panda_interface_names::CONTROLLER_PUBLISHER_QOS(),
          [&initial_pose](
              const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
            initial_pose = msg;
          });

  geometry_msgs::msg::PoseStamped current_desired_pose;

  RCLCPP_INFO(node->get_logger(), "Waiting for servers...");
  cart_traj_action_client->wait_for_action_server();
  RCLCPP_INFO(node->get_logger(), "Servers UP");

  // Get initial pose
  {
    RCLCPP_INFO_STREAM(node->get_logger(), "Getting initial pose, press ENTER");
    std::cin.ignore();

    initial_pose = nullptr;

    std::thread{[sub_node]() {
      rclcpp::spin(sub_node);
      rclcpp::shutdown();
    }}.detach();

    while (!initial_pose && rclcpp::ok()) {
      RCLCPP_INFO_STREAM(node->get_logger(), "Waiting pose state");
      rclcpp::sleep_for(1s);
    }
    if (!rclcpp::ok()) {
      rclcpp::shutdown();
      return -1;
    }

    RCLCPP_INFO_STREAM(node->get_logger(),
                       "Initial pose: ["
                           << initial_pose->pose.position.x << ", "
                           << initial_pose->pose.position.y << ", "
                           << initial_pose->pose.position.z
                           << "], Orientation (w, x, y, z): ["
                           << initial_pose->pose.orientation.w << ", "
                           << initial_pose->pose.orientation.x << ", "
                           << initial_pose->pose.orientation.y << ", "
                           << initial_pose->pose.orientation.z << "]");
  }

  // Calling action server for the true trajectory commanded
  {

    RCLCPP_INFO_STREAM(node->get_logger(),
                       "Ready to call action server for the true desired "
                       "trajectory, press ENTER");
    std::cin.ignore();

    panda_interfaces::action::CartTraj_Goal cart_goal;
    cart_goal.initial_pose = initial_pose->pose;
    if (mode == std::string{"delta"}) {

      desired_pose = initial_pose->pose;

      desired_pose.position.x = desired_pose.position.x + delta_pose.position.x;
      desired_pose.position.y = desired_pose.position.y + delta_pose.position.y;
      desired_pose.position.z = desired_pose.position.z + delta_pose.position.z;

      desired_pose.orientation.w =
          desired_pose.orientation.w + delta_pose.orientation.w;
      desired_pose.orientation.x =
          desired_pose.orientation.x + delta_pose.orientation.x;
      desired_pose.orientation.y =
          desired_pose.orientation.y + delta_pose.orientation.y;
      desired_pose.orientation.z =
          desired_pose.orientation.z + delta_pose.orientation.z;
    }
    cart_goal.desired_pose = desired_pose;
    cart_goal.total_time = total_time;

    RCLCPP_INFO_STREAM(
        node->get_logger(),
        "Objective: Desired pose ["
            << desired_pose.position.x << ", " << desired_pose.position.y
            << ", " << desired_pose.position.z
            << "], Orientation (w, x, y, z): [" << desired_pose.orientation.w
            << ", " << desired_pose.orientation.x << ", "
            << desired_pose.orientation.y << ", " << desired_pose.orientation.z
            << "] in " << total_time << "[s]");
    // chiamo l'azione e aspetto che termini
    rclcpp_action::Client<CartTraj>::SendGoalOptions goal_options;
    // goal_options.feedback_callback =
    //     [node](rclcpp_action::ClientGoalHandle<CartTraj>::SharedPtr,
    //            const std::shared_ptr<
    //                const panda_interfaces::action::CartTraj_Feedback>
    //                feedback) {
    //       RCLCPP_INFO_STREAM(node->get_logger(),
    //                          "Time left: " << feedback->time_left << "[s].");
    //     };

    using namespace std::chrono_literals;
    RCLCPP_INFO_STREAM(node->get_logger(),
                       "Sleeping for 2s and then launching action");
    std::this_thread::sleep_for(2s);
    RCLCPP_INFO_STREAM(node->get_logger(), "LAUNCHING ACTION");

    auto future_goal_handle =
        cart_traj_action_client->async_send_goal(cart_goal, goal_options);
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
