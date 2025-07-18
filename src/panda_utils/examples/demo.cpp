#include "geometry_msgs/msg/pose.hpp"
#include "multibody/joint/joint-generic.hpp"
#include "panda_interfaces/action/cart_traj.hpp"
#include "panda_interfaces/action/loop_cart_traj.hpp"
#include "panda_interfaces/msg/human_detected.hpp"
#include "panda_interfaces/srv/set_compliance_mode.hpp"
#include "panda_utils/constants.hpp"
#include "panda_utils/robot.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include <Eigen/src/Geometry/Quaternion.h>
#include <array>
#include <atomic>
#include <cstddef>
#include <future>
#include <memory>
#include <optional>
#include <panda_interfaces/msg/human_detected.hpp>
#include <rclcpp/client.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/subscription.hpp>
#include <rclcpp/utilities.hpp>
#include <rclcpp/wait_for_message.hpp>
#include <rclcpp_action/client.hpp>
#include <rclcpp_action/create_client.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <tf2/buffer_core.hpp>
#include <tf2/time.hpp>
#include <tf2_ros/transform_listener.h>
#include <thread>

using geometry_msgs::msg::Pose;
using panda_interfaces::action::CartTraj;
using panda_interfaces::action::LoopCartTraj;
using sensor_msgs::msg::JointState;
using namespace std::chrono_literals;

enum class SceneState {
  // The robot has to be configured
  no_state,
  // The robot is doing whichever task has to do
  task,
  // The human enters the robot area and the robot has to enter compliance mode
  transition_human,
  // The robot is in compliance mode: it can be freely moved by the human
  compliance,
  // The human leaves the robot area, allowing the robot to resume task
  transition_leave_human
};

void fill_pose_orientation(geometry_msgs::msg::Pose &pose,
                           const Eigen::Quaterniond &orientation) {

  pose.orientation.w = orientation.w();
  pose.orientation.x = orientation.x();
  pose.orientation.y = orientation.y();
  pose.orientation.z = orientation.z();
}

void fill_pose_position(geometry_msgs::msg::Pose &pose, const double &x,
                        const double &y, const double &z) {

  pose.position.x = x;
  pose.position.y = y;
  pose.position.z = z;
}

panda_interfaces::action::LoopCartTraj_Goal
generate_triangle_task(double x, double y, double z, double h, double l,
                       Eigen::Quaterniond orientation, double total_time) {

  using geometry_msgs::msg::Pose;
  panda_interfaces::action::LoopCartTraj_Goal goal;
  Pose initial;
  Pose right_corner;
  Pose left_corner;

  fill_pose_orientation(initial, orientation);
  fill_pose_orientation(right_corner, orientation);
  fill_pose_orientation(left_corner, orientation);

  fill_pose_position(initial, x, y, z);
  fill_pose_position(initial, x, y + l / 2, z - h);
  fill_pose_position(initial, x, y - l / 2, z - h);

  goal.desired_poses = std::vector{initial, right_corner, left_corner};
  goal.total_time = total_time;
  return goal;
}

void go_to_pose(
    const rclcpp::Node &node, rclcpp_action::Client<CartTraj> &client,
    const rclcpp_action::Client<CartTraj>::SendGoalOptions &cart_traj_options,
    const panda_interfaces::action::CartTraj_Goal &pose_goal) {}

struct LastRobotState {
  geometry_msgs::msg::Pose pose;
  std::array<double, 7> joint_config;
};

std::array<double, 7> home_joint_config;
Pose home_pose;
Pose initial_task_pose;
double triangle_height; // h
double triangle_base;   // l
double total_task_time;
const std::string robot_base_frame_name{"fr3_link0"};
const std::string robot_end_affector_frame{"fr3_joint7"};
const std::string world_frame{"world"};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  auto main_node = std::make_shared<rclcpp::Node>("task_node");
  auto sub_node = std::make_shared<rclcpp::Node>("task_sub_node");
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(sub_node);

  geometry_msgs::msg::Pose initial_pose;
  geometry_msgs::msg::Pose desired_pose;
  tf2::BufferCore tf_buffer;
  SceneState state{SceneState::no_state};
  LastRobotState last_state;
  JointState joint_states;
  // The goal handles for the 2 possible actions of robot, these needs to be
  // stopped in any error or shutdown state
  std::optional<rclcpp_action::ClientGoalHandle<CartTraj>::SharedPtr>
      cartesian_traj_handle;
  std::optional<rclcpp_action::ClientGoalHandle<LoopCartTraj>::SharedPtr>
      loop_cartesian_traj_handle;
  // Goal options
  rclcpp_action::Client<CartTraj>::SendGoalOptions cart_traj_options;
  rclcpp_action::Client<LoopCartTraj>::SendGoalOptions loop_cart_traj_options;
  // Flag variable that indicates someone entered in the working area of the
  // robot
  std::atomic<bool> person_in_proximity{false};
  double total_time = 1.0;

  RCLCPP_INFO(main_node->get_logger(), "Defining Home goal");
  // Home goal definition
  panda_interfaces::action::CartTraj_Goal home_goal;
  home_goal.desired_pose = home_pose;
  home_goal.total_time = 5.0;

  RCLCPP_INFO(main_node->get_logger(), "Defining Task goal: triangle");
  // Task goal definition: triangle
  Eigen::Quaterniond triangle_orient{
      home_pose.orientation.w, home_pose.orientation.x, home_pose.orientation.y,
      home_pose.orientation.z};
  panda_interfaces::action::LoopCartTraj_Goal triangle_task_goal =
      generate_triangle_task(0, 0.3, 0.5, 0.1, 0.2, triangle_orient, 5.0);

  auto joint_states_cb = [&joint_states](const JointState msg) {
    joint_states = msg;
  };

  // Topic to read for scene infos
  rclcpp::Subscription<JointState>::SharedPtr joint_states_sub =
      sub_node->create_subscription<JointState>(
          panda_interface_names::joint_state_topic_name,
          panda_interface_names::DEFAULT_TOPIC_QOS, joint_states_cb);

  std::unique_ptr<tf2_ros::TransformListener> tf_listener =
      std::make_unique<tf2_ros::TransformListener>(tf_buffer, sub_node);

  // Compliance mode service client
  rclcpp::Client<panda_interfaces::srv::SetComplianceMode>::SharedPtr
      compliance_mode_client =
          main_node->create_client<panda_interfaces::srv::SetComplianceMode>(
              panda_interface_names::set_compliance_mode_service_name);

  // Action clients for trajectories (actions)
  rclcpp_action::Client<CartTraj>::SharedPtr cart_traj_action_client =
      rclcpp_action::create_client<CartTraj>(
          main_node, panda_interface_names::panda_cart_move_action_name);

  rclcpp_action::Client<LoopCartTraj>::SharedPtr loop_traj_action_client =
      rclcpp_action::create_client<LoopCartTraj>(
          main_node, panda_interface_names::panda_cart_loop_action_name);

  // Management of action nodes for the 2 types of trajectories

  auto cancel_actions = [&cartesian_traj_handle, &loop_cartesian_traj_handle,
                         &cart_traj_action_client, &loop_traj_action_client,
                         &main_node]() {
    if (cartesian_traj_handle.has_value()) {
      cart_traj_action_client->async_cancel_goal(cartesian_traj_handle.value());
      RCLCPP_INFO(main_node->get_logger(),
                  "Requested cancel of cartesian trajectory action");
    }

    if (loop_cartesian_traj_handle.has_value()) {
      loop_traj_action_client->async_cancel_goal(
          loop_cartesian_traj_handle.value());
      RCLCPP_INFO(main_node->get_logger(),
                  "Requested cancel of loop cartesian trajectory action");
    }
  };

  // Definition of SendGoalOptions struct
  {
    auto cart_result_callback =
        [&cartesian_traj_handle](
            const rclcpp_action::ClientGoalHandle<CartTraj>::WrappedResult &) {
          cartesian_traj_handle = std::nullopt;
        };
    cart_traj_options.result_callback = cart_result_callback;

    auto cart_goal_response_callback =
        [&cartesian_traj_handle](
            rclcpp_action::ClientGoalHandle<CartTraj>::SharedPtr goal_handle) {
          cartesian_traj_handle = goal_handle;
        };
    cart_traj_options.goal_response_callback = cart_goal_response_callback;

    auto loop_cart_goal_response_callback =
        [&loop_cartesian_traj_handle](
            rclcpp_action::ClientGoalHandle<LoopCartTraj>::SharedPtr
                goal_handle) { loop_cartesian_traj_handle = goal_handle; };
    loop_cart_traj_options.goal_response_callback =
        loop_cart_goal_response_callback;

    auto loop_result_callback =
        [&loop_cartesian_traj_handle](
            const rclcpp_action::ClientGoalHandle<LoopCartTraj>::WrappedResult
                &) { loop_cartesian_traj_handle = std::nullopt; };
    loop_cart_traj_options.result_callback = loop_result_callback;
  }

  RCLCPP_INFO(main_node->get_logger(), "Waiting for servers...");

  RCLCPP_INFO(main_node->get_logger(), "Waiting for cartesian trajectory");
  cart_traj_action_client->wait_for_action_server(10s);

  RCLCPP_INFO(main_node->get_logger(), "Waiting for loop cartesian trajectory");
  loop_traj_action_client->wait_for_action_server(10s);

  RCLCPP_INFO(main_node->get_logger(), "Waiting for compliance mode server");
  compliance_mode_client->wait_for_service(10s);

  RCLCPP_INFO(main_node->get_logger(), "Servers UP");

  // Go to initial pose
  // Begin task: cancel when a human enters the area
  // Enter in compliance mode: exit when human leaves the area, staying in
  // current pose Go back to last task state Resume task

  std::thread info_thread([&executor]() { executor.spin(); });
  std::thread safe_keeper_thread{
      [cancel_actions, &person_in_proximity, &state]() {
        if (person_in_proximity && state != SceneState::transition_human) {
          cancel_actions();
          state = SceneState::transition_human;
        }
      }};
  std::thread calculate_proximity{[&tf_buffer] {
    // TODO: handle 2 things:
    // - Proximty value as bool, depending on distance from working area
    // - Minimum distance between each joint of the robot and the human wrist,
    // if they are in scene
  }};

  while (rclcpp::ok()) {
    switch (state) {
    case SceneState::no_state: {
      // Ensure the robot is up and running
      // Going to pose
      RCLCPP_INFO(main_node->get_logger(), "Robot has no known state");
      if (!cartesian_traj_handle.has_value()) {
        RCLCPP_INFO(main_node->get_logger(), "Sending robot home pose");
        cart_traj_action_client->async_send_goal(home_goal, cart_traj_options);
        std::this_thread::sleep_for(1s);
        while (cartesian_traj_handle.has_value()) {
          std::this_thread::sleep_for(500ms);
          RCLCPP_INFO(main_node->get_logger(),
                      "Waiting robot to reach home pose");
        }

        RCLCPP_INFO(main_node->get_logger(),
                    "Robot reached home pose, starting task");
        loop_traj_action_client->async_send_goal(triangle_task_goal);
        state = SceneState::task;
      }

      RCLCPP_INFO(main_node->get_logger(),
                  "Still performing another action in 'no_state' state");
      // Activate controller and publish to pose topic the home pose, start the
      // task -> go to task
    }
    case SceneState::task: {
      // Read the human_state and handle the task accordingly; when human enter
      // -> go to transition_human
    }
    case SceneState::transition_human: {
      // Stop the task, save the state and enter compliance mode -> go to
      // compliance
    }
    case SceneState::compliance: {
      // Read the human_state and handle the compliance mode accordingly; when
      // human leaves -> go to transition_leave_human
    }
    case SceneState::transition_leave_human: {
      // Exit from compliance mode staying in current pose, return to last state
      // pose, resume the task -> go to task
    } break;
    }
  }

  executor.cancel();
  info_thread.join();

  rclcpp::shutdown();

  // Calling action server
  // {
  //   RCLCPP_INFO_STREAM(main_node->get_logger(),
  //                      "Ready to call action server, press ENTER");
  //   std::cin.ignore();
  //
  //   panda_interfaces::action::CartTraj_Goal cart_goal;
  //   cart_goal.initial_pose = initial_pose;
  //   geometry_msgs::msg::Pose desired_pose;
  //   desired_pose = initial_pose;
  //   desired_pose.position.x -= 0.1;
  //   desired_pose.position.y += 0.3;
  //   desired_pose.position.z += 0.5;
  //   cart_goal.desired_pose = desired_pose;
  //   cart_goal.total_time = total_time;
  //
  //   rclcpp_action::Client<LoopCartTraj>::SendGoalOptions goal_options;
  //
  //   // Republish feedback to caller via the server
  //   goal_options.feedback_callback =
  //       [](rclcpp_action::ClientGoalHandle<LoopCartTraj>::SharedPtr,
  //          const std::shared_ptr<const LoopCartTraj::Feedback> feedback) {};
  //
  //   goal_options.result_callback = [](const auto &result) {};
  //
  //   auto future_goal_handle =
  //       cart_traj_action_client->async_send_goal(cart_goal);
  //   if (rclcpp::spin_until_future_complete(main_node, future_goal_handle) !=
  //       rclcpp::FutureReturnCode::SUCCESS) {
  //     RCLCPP_ERROR_STREAM(main_node->get_logger(), "ERROR, GOAL NOT SENT");
  //     return -1;
  //   }
  //   auto future_result =
  //       cart_traj_action_client->async_get_result(future_goal_handle.get());
  //   if (rclcpp::spin_until_future_complete(main_node, future_result) !=
  //       rclcpp::FutureReturnCode::SUCCESS) {
  //     RCLCPP_ERROR_STREAM(main_node->get_logger(), "ERROR, NO RESULT");
  //     return -1;
  //   }
  //
  //   // check dello stato dell'azione, se non ho errori lo stato deve essere
  //   // SUCCEEDED
  //   if (future_result.get().code != rclcpp_action::ResultCode::SUCCEEDED) {
  //     RCLCPP_ERROR_STREAM(main_node->get_logger(),
  //                         "ERROR, CART TRAJECTORY NOT SUCCEEDED");
  //     return -1;
  //   }
  // }
}
