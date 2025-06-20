#include "panda_interfaces/action/cart_traj.hpp"
#include "geometry_msgs/msg/accel.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "panda_utils/constants.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "realtime_tools/realtime_tools/realtime_publisher.hpp"
#include "std_msgs/msg/bool.hpp"
#include <Eigen/src/Core/Matrix.h>
#include <Eigen/src/Geometry/AngleAxis.h>
#include <Eigen/src/Geometry/Quaternion.h>
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
#include <string>
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

double qintic_velocity(double q_i, double q_f, double t, double t_f) {
  if (t <= 0)
    return 0;
  if (t >= t_f)
    return 0;

  double tau = t / t_f;
  double q_cap =
      30 * std::pow(tau, 4) - 60 * std::pow(tau, 3) + 30 * std::pow(tau, 2);
  return (q_f - q_i) * q_cap / t_f;
}

double qintic_accel(double q_i, double q_f, double t, double t_f) {
  if (t <= 0)
    return 0;
  if (t >= t_f)
    return 0;

  double tau = t / t_f;
  double q_cap = 120 * std::pow(tau, 3) - 180 * std::pow(tau, 2) + 60 * tau;
  return (q_f - q_i) * q_cap / std::pow(t_f, 2);
}

class CartTrajectory : public rclcpp::Node {

public:
  CartTrajectory(const rclcpp::NodeOptions opt = rclcpp::NodeOptions())
      : Node(panda_interface_names::cart_traj_node_name, opt) {

    this->declare_parameter<double>("loop_rate_freq", 1000.0);

    loop_rate_freq = this->get_parameter("loop_rate_freq").as_double();

    auto handle_goal = [this](const rclcpp_action::GoalUUID uuid,
                              std::shared_ptr<const CartTraj::Goal> goal) {
      RCLCPP_INFO(this->get_logger(), "Received goal request");
      (void)uuid;
      RCLCPP_INFO_STREAM(this->get_logger(),
                         "Initial pose: Position: ["
                             << goal->initial_pose.position.x << ", "
                             << goal->initial_pose.position.y << ", "
                             << goal->initial_pose.position.z
                             << "] Orientation(w, x, y, z): ["
                             << goal->initial_pose.orientation.w << ", "
                             << goal->initial_pose.orientation.x << ", "
                             << goal->initial_pose.orientation.y << ", "
                             << goal->initial_pose.orientation.z << "]");
      RCLCPP_INFO_STREAM(this->get_logger(),
                         "Final pose: Position: ["
                             << goal->desired_pose.position.x << ", "
                             << goal->desired_pose.position.y << ", "
                             << goal->desired_pose.position.z
                             << "] Orientation(w, x, y, z): ["
                             << goal->desired_pose.orientation.w << ", "
                             << goal->desired_pose.orientation.x << ", "
                             << goal->desired_pose.orientation.y << ", "
                             << goal->desired_pose.orientation.z << "]");
      RCLCPP_INFO_STREAM(this->get_logger(), "In " << goal->total_time << "s");
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

    cmd_pose_pub = std::make_shared<
        realtime_tools::RealtimePublisher<geometry_msgs::msg::Pose>>(
        this->create_publisher<geometry_msgs::msg::Pose>(
            panda_interface_names::panda_pose_cmd_topic_name,
            panda_interface_names::DEFAULT_TOPIC_QOS));

    cmd_twist_pub = std::make_shared<
        realtime_tools::RealtimePublisher<geometry_msgs::msg::Twist>>(
        this->create_publisher<geometry_msgs::msg::Twist>(
            panda_interface_names::panda_twist_cmd_topic_name,
            panda_interface_names::DEFAULT_TOPIC_QOS));

    cmd_accel_pub = std::make_shared<
        realtime_tools::RealtimePublisher<geometry_msgs::msg::Accel>>(
        this->create_publisher<geometry_msgs::msg::Accel>(
            panda_interface_names::panda_accel_cmd_topic_name,
            panda_interface_names::DEFAULT_TOPIC_QOS));
  }

private:
  realtime_tools::RealtimePublisherSharedPtr<geometry_msgs::msg::Pose>
      cmd_pose_pub;
  realtime_tools::RealtimePublisherSharedPtr<geometry_msgs::msg::Twist>
      cmd_twist_pub;
  realtime_tools::RealtimePublisherSharedPtr<geometry_msgs::msg::Accel>
      cmd_accel_pub;

  rclcpp_action::Server<CartTraj>::SharedPtr action_traj_server;

  double loop_rate_freq{};

  void execute(const std::shared_ptr<GoalHandleCartMove> goal_handle) {
    RCLCPP_INFO(this->get_logger(), "Executing goal");
    rclcpp::Rate loop_rate(loop_rate_freq, this->get_clock());

    RCLCPP_DEBUG(this->get_logger(), "Getting goal");
    const auto goal = goal_handle->get_goal();

    // Desired orientation
    Eigen::Quaterniond desired_quat{
        goal->desired_pose.orientation.w, goal->desired_pose.orientation.x,
        goal->desired_pose.orientation.y, goal->desired_pose.orientation.z};
    desired_quat.normalize();
    Eigen::AngleAxisd angle_axis_desired = Eigen::AngleAxisd{desired_quat};
    double theta_f = angle_axis_desired.angle();
    Eigen::Vector3d axis = angle_axis_desired.axis();

    // Initial orientation
    Eigen::Quaterniond initial_quat{
        goal->initial_pose.orientation.w, goal->initial_pose.orientation.x,
        goal->initial_pose.orientation.y, goal->initial_pose.orientation.z};
    initial_quat.normalize();
    Eigen::Matrix3d initial_rot{initial_quat};
    Eigen::Vector3d initial_quat_vec = initial_quat.vec();

    auto feedback = std::make_shared<CartTraj::Feedback>();
    auto result = std::make_shared<CartTraj::Result>();

    rclcpp::Time t0 = this->get_clock()->now();
    rclcpp::Duration t = rclcpp::Duration(0, 0);

    rclcpp::Duration traj_duration = rclcpp::Duration(goal->total_time, 0);
    geometry_msgs::msg::Pose cmd_pose;
    geometry_msgs::msg::Twist cmd_twist;
    geometry_msgs::msg::Accel cmd_accel;

    RCLCPP_DEBUG(this->get_logger(), "Entering while");
    while (rclcpp::ok() && t < traj_duration) {
      t = this->get_clock()->now() - t0;
      RCLCPP_DEBUG(this->get_logger(), "Time now %f", t.seconds());

      RCLCPP_DEBUG(this->get_logger(), "Uploading next pose");

      // Calculating position
      cmd_pose.position.x =
          qintic(goal->initial_pose.position.x, goal->desired_pose.position.x,
                 t.seconds(), goal->total_time);
      cmd_pose.position.y =
          qintic(goal->initial_pose.position.y, goal->desired_pose.position.y,
                 t.seconds(), goal->total_time);
      cmd_pose.position.z =
          qintic(goal->initial_pose.position.z, goal->desired_pose.position.z,
                 t.seconds(), goal->total_time);

      cmd_twist.linear.x = qintic_velocity(goal->initial_pose.position.x,
                                           goal->desired_pose.position.x,
                                           t.seconds(), goal->total_time);
      cmd_twist.linear.y = qintic_velocity(goal->initial_pose.position.y,
                                           goal->desired_pose.position.y,
                                           t.seconds(), goal->total_time);
      cmd_twist.linear.z = qintic_velocity(goal->initial_pose.position.z,
                                           goal->desired_pose.position.z,
                                           t.seconds(), goal->total_time);
      cmd_accel.linear.x = qintic_accel(goal->initial_pose.position.x,
                                        goal->desired_pose.position.x,
                                        t.seconds(), goal->total_time);
      cmd_accel.linear.y = qintic_accel(goal->initial_pose.position.y,
                                        goal->desired_pose.position.y,
                                        t.seconds(), goal->total_time);
      cmd_accel.linear.z = qintic_accel(goal->initial_pose.position.z,
                                        goal->desired_pose.position.z,
                                        t.seconds(), goal->total_time);

      // Calculating orientation
      double theta = qintic(0, theta_f, t.seconds(), goal->total_time);
      double theta_dot =
          qintic_velocity(0, theta_f, t.seconds(), goal->total_time);
      double theta_ddot =
          qintic_accel(0, theta_f, t.seconds(), goal->total_time);

      Eigen::Vector3d epsilon = axis * sin(theta / 2);
      Eigen::Quaterniond current_quat{cos(theta / 2), epsilon[0], epsilon[1],
                                      epsilon[2]};
      current_quat.normalize();
      // Orientation
      current_quat = initial_quat * current_quat;
      cmd_pose.orientation.w = current_quat.w();
      cmd_pose.orientation.x = current_quat.x();
      cmd_pose.orientation.y = current_quat.y();
      cmd_pose.orientation.z = current_quat.z();

      // Angular velocity
      Eigen::Vector3d angular_vel = initial_rot * axis * theta_dot;
      cmd_twist.angular.x = angular_vel.x();
      cmd_twist.angular.y = angular_vel.y();
      cmd_twist.angular.z = angular_vel.z();

      // Angular acceleration
      Eigen::Vector3d angular_accel = initial_rot * axis * theta_ddot;
      cmd_accel.angular.x = angular_accel.x();
      cmd_accel.angular.y = angular_accel.y();
      cmd_accel.angular.z = angular_accel.z();

      RCLCPP_DEBUG_ONCE(this->get_logger(), "Assigning time left");
      feedback->time_left = (traj_duration - t).seconds();

      RCLCPP_DEBUG_ONCE(this->get_logger(), "Publishing feedback");
      goal_handle->publish_feedback(feedback);

      RCLCPP_DEBUG_ONCE(this->get_logger(), "Publish command");
      cmd_pose_pub->tryPublish(cmd_pose);
      cmd_twist_pub->tryPublish(cmd_twist);
      cmd_accel_pub->tryPublish(cmd_accel);

      // Sleep
      //
      RCLCPP_DEBUG_ONCE(this->get_logger(), "Sleep");
      loop_rate.sleep();
    }

    // Check if goal is done
    if (rclcpp::ok()) {
      goal_handle->succeed(result);
      RCLCPP_INFO(this->get_logger(), "Goal succeeded");
    } else {
      rclcpp::shutdown();
    }
  }
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<CartTrajectory>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
