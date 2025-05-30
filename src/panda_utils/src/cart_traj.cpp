#include "panda_interfaces/action/cart_traj.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "panda_utils/constants.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "std_msgs/msg/bool.hpp"
#include <cmath>
#include <rclcpp/duration.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/subscription.hpp>
#include <rclcpp/subscription_wait_set_mask.hpp>
#include <rclcpp/utilities.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <rclcpp_action/types.hpp>
#include <thread>

using CartTraj = panda_interfaces::action::CartTraj;
using GoalHandleCartMove = rclcpp_action::ServerGoalHandle<CartTraj>;
using namespace std::chrono_literals;

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

class CartTrajectory : public rclcpp::Node {

public:
  CartTrajectory(const rclcpp::NodeOptions opt = rclcpp::NodeOptions())
      : Node("cart_trajectory", opt) {

    auto handle_goal = [this](const rclcpp_action::GoalUUID uuid,
                              std::shared_ptr<const CartTraj::Goal> goal) {
      RCLCPP_INFO(this->get_logger(), "Received goal request");
      (void)uuid;
      RCLCPP_INFO_STREAM(this->get_logger(),
                         "Desired pose is: ["
                             << goal->desired_pose.position.x << ", "
                             << goal->desired_pose.position.y << ", "
                             << goal->desired_pose.position.z << ", "
                             << goal->desired_pose.orientation.x << ", "
                             << goal->desired_pose.orientation.y << ", "
                             << goal->desired_pose.orientation.z << "]");
      RCLCPP_INFO(this->get_logger(), "Started CLIK");
      std_msgs::msg::Bool start;
      start.data = true;
      clik_start_pub->publish(start);
      return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    };

    auto handle_cancel =
        [this](const std::shared_ptr<rclcpp_action::ServerGoalHandle<CartTraj>>
                   goal_handle) {
          RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
          (void)goal_handle;
          return rclcpp_action::CancelResponse::ACCEPT;
        };

    auto handle_accepted =
        [this](const std::shared_ptr<GoalHandleCartMove> goal_handle) {
          using namespace std::placeholders;

          std::thread{std::bind(&CartTrajectory::execute, this, _1),
                      goal_handle}
              .detach();
        };

    this->action_traj_server = rclcpp_action::create_server<CartTraj>(
        this, panda_interface_names::panda_cart_move_action_name, handle_goal,
        handle_cancel, handle_accepted);

    clik_start_pub = this->create_publisher<std_msgs::msg::Bool>(
        panda_interface_names::start_and_stop_clik_topic_name,
        panda_interface_names::DEFAULT_TOPIC_QOS);

    cmd_pose_clik_pub = this->create_publisher<geometry_msgs::msg::Pose>(
        panda_interface_names::panda_pose_cmd_topic_name,
        panda_interface_names::DEFAULT_TOPIC_QOS);
  }

private:
  rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr cmd_pose_clik_pub;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr clik_start_pub;
  rclcpp_action::Server<CartTraj>::SharedPtr action_traj_server;

  void execute(const std::shared_ptr<GoalHandleCartMove> goal_handle) {
    RCLCPP_INFO(this->get_logger(), "Executing goal");
    rclcpp::Rate loop_rate(100.0);
    const auto goal = goal_handle->get_goal();
    auto feedback = std::make_shared<CartTraj::Feedback>();
    auto result = std::make_shared<CartTraj::Result>();

    rclcpp::Time t0 = this->now();
    rclcpp::Duration t = rclcpp::Duration(0, 0);

    rclcpp::Duration traj_duration = rclcpp::Duration(goal->total_time, 0);

    while (rclcpp::ok() && t < traj_duration) {
      t = this->now() - t0;

      geometry_msgs::msg::Pose cmd_pose;
      RCLCPP_INFO(this->get_logger(), "Uploading next pose");

      cmd_pose.position.x =
          qintic(goal->initial_pose.position.x, goal->desired_pose.position.x,
                 t.seconds(), goal->total_time);
      cmd_pose.position.y =
          qintic(goal->initial_pose.position.y, goal->desired_pose.position.y,
                 t.seconds(), goal->total_time);
      cmd_pose.position.z =
          qintic(goal->initial_pose.position.z, goal->desired_pose.position.z,
                 t.seconds(), goal->total_time);

      cmd_pose.orientation = goal->initial_pose.orientation;

      RCLCPP_INFO(this->get_logger(), "Assigning time left");
      feedback->time_left = (traj_duration - t).seconds();

      RCLCPP_INFO(this->get_logger(), "Publishing feedback");
      goal_handle->publish_feedback(feedback);

      RCLCPP_INFO(this->get_logger(), "Publish command");
      cmd_pose_clik_pub->publish(cmd_pose);

      // Sleep
      //
      RCLCPP_INFO(this->get_logger(), "Sleep");
      loop_rate.sleep();
    }

    // Check if goal is done
    if (rclcpp::ok()) {
      goal_handle->succeed(result);
      RCLCPP_INFO(this->get_logger(), "Goal succeeded");
    }
  }
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<CartTrajectory>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
