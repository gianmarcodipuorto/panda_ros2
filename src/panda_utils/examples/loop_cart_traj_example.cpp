#include "geometry_msgs/msg/accel.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "image_processing/constants.hpp"
#include "panda_interfaces/action/cart_traj.hpp"
#include "panda_interfaces/action/loop_cart_traj.hpp"
#include "panda_interfaces/action/stop_traj.hpp"
#include "panda_interfaces/msg/human_detected.hpp"
#include "panda_interfaces/srv/set_compliance_mode.hpp"
#include "panda_interfaces/srv/wrist_contact.hpp"
#include "panda_utils/constants.hpp"
#include "panda_utils/utils_func.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/color_rgba.hpp"
#include "std_msgs/msg/u_int8.hpp"
#include <Eigen/src/Core/PartialReduxEvaluator.h>
#include <Eigen/src/Geometry/Quaternion.h>
#include <array>
#include <atomic>
#include <cstddef>
#include <future>
#include <memory>
#include <mutex>
#include <optional>
#include <panda_interfaces/msg/human_detected.hpp>
#include <rclcpp/client.hpp>
#include <rclcpp/duration.hpp>
#include <rclcpp/executors.hpp>
#include <rclcpp/executors/multi_threaded_executor.hpp>
#include <rclcpp/future_return_code.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/parameter.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/subscription.hpp>
#include <rclcpp/utilities.hpp>
#include <rclcpp/wait_for_message.hpp>
#include <rclcpp_action/client.hpp>
#include <rclcpp_action/create_client.hpp>
#include <rclcpp_action/exceptions.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <shared_mutex>
#include <string>
#include <tf2/buffer_core.hpp>
#include <tf2/exceptions.hpp>
#include <tf2/time.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <thread>

using geometry_msgs::msg::Accel;
using geometry_msgs::msg::Pose;
using geometry_msgs::msg::PoseStamped;
using geometry_msgs::msg::Twist;
using panda_interfaces::action::CartTraj;
using panda_interfaces::action::LoopCartTraj;
using panda_interfaces::action::StopTraj;
using sensor_msgs::msg::JointState;
using namespace std::chrono_literals;
template <typename ActionT>
using GoalOptions = typename rclcpp_action::Client<ActionT>::SendGoalOptions;
template <typename ActionT>
using OptionalGoalHandle = typename std::optional<
    typename rclcpp_action::ClientGoalHandle<ActionT>::SharedPtr>;

enum class SceneState {
  // The robot has to be configured
  no_state,
  // The robot is doing whichever task has to do
  task,
  // The human enters the robot area and the robot has to enter compliance
  // mode
  transition_human,
  // The robot is in compliance mode: it can be freely moved by the human
  compliance,
  // The human leaves the robot area, allowing the robot to resume task
  transition_leave_human,

};

void publish_state(
    const rclcpp::Publisher<std_msgs::msg::ColorRGBA>::SharedPtr pub,
    const SceneState &state) {

  std_msgs::msg::ColorRGBA color;

  switch (state) {

  case SceneState::no_state: {
    color.r = 0.0;
    color.g = 0.0;
    color.b = 0.0;
    break;
  }
  case SceneState::task: {
    color.r = 0.0;
    color.g = 255.0;
    color.b = 0.0;
    break;
  }
  case SceneState::transition_human: {
    color.r = 255.0;
    color.g = 255.0;
    color.b = 0.0;
    break;
  }
  case SceneState::compliance: {
    color.r = 255.0;
    color.g = 0.0;
    color.b = 0.0;
    break;
  }
  case SceneState::transition_leave_human: {
    color.r = 0.0;
    color.g = 255.0;
    color.b = 255.0;
    break;
  } break;
  }

  pub->publish(color);
}


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
  fill_pose_position(right_corner, x, y + l / 2, z - h);
  fill_pose_position(left_corner, x, y - l / 2, z - h);

  goal.desired_poses = std::vector{initial, right_corner, left_corner};
  goal.total_time = total_time;
  return goal;
}

double triangle_height; // h
double triangle_base;   // l
double total_task_time;

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  auto main_node = std::make_shared<rclcpp::Node>("loop_cart_example");

  // Choosing home pose

  RCLCPP_INFO(main_node->get_logger(), "Defining Task goal: triangle");
  geometry_msgs::msg::PoseStamped initial_pose;
  rclcpp::wait_for_message<geometry_msgs::msg::PoseStamped>(
      initial_pose, main_node,
      panda_interface_names::panda_pose_state_topic_name, 10s,
      panda_interface_names::CONTROLLER_PUBLISHER_QOS());

  // Task goal definition: triangle

  Eigen::Quaterniond triangle_orient{
      initial_pose.pose.orientation.w, initial_pose.pose.orientation.x,
      initial_pose.pose.orientation.y, initial_pose.pose.orientation.z};

  panda_interfaces::action::LoopCartTraj_Goal triangle_task_goal =
      generate_triangle_task(initial_pose.pose.position.x,
                             initial_pose.pose.position.y,
                             initial_pose.pose.position.z, 0.1, 0.1,
                             triangle_orient.normalized(), 15.0);

  for (size_t i = 0; i < triangle_task_goal.desired_poses.size(); i++) {
    RCLCPP_INFO_STREAM(main_node->get_logger(),
                       "Desired pose "
                           << i + 1 << ": ["
                           << triangle_task_goal.desired_poses[i].position.x
                           << ", "
                           << triangle_task_goal.desired_poses[i].position.y
                           << ", "
                           << triangle_task_goal.desired_poses[i].position.z
                           << "]");
  }

  rclcpp_action::Client<LoopCartTraj>::SharedPtr loop_traj_action_client =
      rclcpp_action::create_client<LoopCartTraj>(
          main_node, panda_interface_names::panda_cart_loop_action_name);

  RCLCPP_INFO(main_node->get_logger(), "Waiting for servers...");

  RCLCPP_INFO(main_node->get_logger(), "Waiting for loop cartesian trajectory");
  loop_traj_action_client->wait_for_action_server(10s);

  RCLCPP_INFO(main_node->get_logger(), "Servers UP");

  using namespace std::chrono_literals;
  RCLCPP_INFO_STREAM(main_node->get_logger(),
                     "Sleeping for 5s and then launching action");
  std::this_thread::sleep_for(5s);
  if (!rclcpp::ok()) {
    rclcpp::shutdown();
    return -1;
  }
  RCLCPP_INFO_STREAM(main_node->get_logger(), "LAUNCHING ACTION");

  auto future_goal_handle =
      loop_traj_action_client->async_send_goal(triangle_task_goal);
  if (rclcpp::spin_until_future_complete(main_node, future_goal_handle) !=
          rclcpp::FutureReturnCode::SUCCESS &&
      rclcpp::ok()) {
    if (!rclcpp::ok()) {
      loop_traj_action_client->async_cancel_goal(future_goal_handle.get());
      rclcpp::shutdown();
    }
    RCLCPP_ERROR_STREAM(main_node->get_logger(), "ERROR, GOAL NOT SENT");
    return -1;
  }
  auto future_result =
      loop_traj_action_client->async_get_result(future_goal_handle.get());
  if (rclcpp::spin_until_future_complete(main_node, future_result) !=
          rclcpp::FutureReturnCode::SUCCESS &&
      rclcpp::ok()) {
    if (!rclcpp::ok()) {
      loop_traj_action_client->async_cancel_goal(future_goal_handle.get());
      rclcpp::shutdown();
    }
    RCLCPP_ERROR_STREAM(main_node->get_logger(), "ERROR, NO RESULT");
    return -1;
  }

  // check dello stato dell'azione, se non ho errori lo stato deve essere
  // SUCCEEDED
  if (future_result.get().code != rclcpp_action::ResultCode::SUCCEEDED &&
      rclcpp::ok()) {
    if (!rclcpp::ok()) {
      loop_traj_action_client->async_cancel_goal(future_goal_handle.get());
      rclcpp::shutdown();
    }
    RCLCPP_ERROR_STREAM(main_node->get_logger(),
                        "ERROR, CART TRAJECTORY NOT SUCCEEDED");
    return -1;
  }

  rclcpp::shutdown();
}
